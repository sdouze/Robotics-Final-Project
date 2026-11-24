"""AX-12A arm driver (Dynamixel SDK, Protocol 1.0) and smooth motion.

Smoothness comes from two things:
  1. All four joints are commanded together with ONE GroupSyncWrite packet
     that writes Goal Position (addr 30) and Moving Speed (addr 32) at once.
  2. Motions are streamed as small set-points at CONTROL_HZ. Each set-point
     carries the speed needed to reach it in exactly one period, so the servos
     move continuously instead of accelerating to full speed and stopping.
     Point-to-point moves use a minimum-jerk (quintic) profile in joint space;
     Cartesian moves use a trapezoidal / minimum-jerk profile along the path.
"""

import math
import sys
import time

import numpy as np

import config as C
import kinematics as kin

if C.SDK_SRC_DIR not in sys.path:
    sys.path.insert(0, C.SDK_SRC_DIR)
import dynamixel_sdk as dxl  # noqa: E402

# AX-12A control table (Protocol 1.0)
ADDR_MODEL_NUMBER = 0
ADDR_CW_ANGLE_LIMIT = 6       # EEPROM, 2 bytes: the servo refuses goals below this
ADDR_CCW_ANGLE_LIMIT = 8      # EEPROM, 2 bytes: ... and above this
ADDR_TORQUE_ENABLE = 24
ADDR_CW_COMPLIANCE_MARGIN = 26
ADDR_CCW_COMPLIANCE_MARGIN = 27
ADDR_CW_COMPLIANCE_SLOPE = 28
ADDR_CCW_COMPLIANCE_SLOPE = 29
ADDR_GOAL_POSITION = 30
ADDR_MOVING_SPEED = 32
ADDR_TORQUE_LIMIT = 34
ADDR_PRESENT_POSITION = 36
ADDR_MOVING = 46
ADDR_PUNCH = 48
LEN_GOAL_AND_SPEED = 4        # goal position (2 bytes) + moving speed (2 bytes)

RETRIES = 5                   # per single read/write
MAX_FAILED_READS = 25         # consecutive failed position reads of one servo before giving up


class UserAbort(Exception):
    """Raised when the user presses q / Esc or Ctrl+C."""


class JointBlocked(RuntimeError):
    """A joint stopped far from its goal: it is pushing against an obstacle."""


class ServoError(IOError):
    """A servo answered correctly but reports a hardware error (overload, ...).

    Unlike a corrupted packet this is never retried or tolerated: the servo has
    switched its torque off and the cause must be fixed first."""


# Error bits of the AX-12A status packet and what to do about them.
SERVO_ERROR_HELP = {
    32: "Overload: the joint was blocked (mechanical stop, table, cable or the robot itself) "
        "and the servo switched its torque off. Remove the cause, then switch the robot "
        "power OFF and ON to clear it. Run --check-joints before moving again: a wrong "
        "JOINT_DIRECTION/JOINT_OFFSET_DEG or a pose the wrist cannot reach is the usual cause.",
    4: "Overheating: let the servo cool down, then power-cycle the robot.",
    1: "Input voltage error: check the power supply of the robot.",
    2: "Angle limit error: the goal is outside the servo's CW/CCW angle limits "
       "(check the limits in Dynamixel Wizard and JOINT_LIMITS_DEG).",
}


# ---------------------------------------------------------------------------
# Low level: Dynamixel SDK
# ---------------------------------------------------------------------------
class DxlArm:
    def __init__(self, port_name=C.PORT, baudrate=C.BAUDRATE, ids=C.DXL_IDS):
        self.ids = tuple(ids)
        self.baudrate = baudrate
        self.port = dxl.PortHandler(port_name)
        self.packet = dxl.PacketHandler(C.PROTOCOL_VERSION)
        self.sync_write = dxl.GroupSyncWrite(self.port, self.packet,
                                             ADDR_GOAL_POSITION, LEN_GOAL_AND_SPEED)
        self.is_open = False
        # Tolerance for a noisy bus: the last good position of every servo is
        # reused when a status packet arrives corrupted (seen on these robots
        # while the motors draw current).
        self.last_ticks = None
        self.failed_in_a_row = {i: 0 for i in self.ids}
        self.failed_total = 0

    # -- helpers ------------------------------------------------------------
    def _check(self, dxl_id, what, result, error):
        if result != dxl.COMM_SUCCESS:
            raise IOError(f"ID {dxl_id} {what}: {self.packet.getTxRxResult(result)}")
        if error != 0:
            hints = [h for bit, h in SERVO_ERROR_HELP.items() if error & bit]
            raise ServoError(f"servo {dxl_id} ({what}): {self.packet.getRxPacketError(error)}"
                             + "".join(f"\n  -> {h}" for h in hints))

    def _discard_input(self):
        """Drop stale bytes of a corrupted status packet before retrying.

        PortHandler.clearPort() only flushes the OUTPUT buffer, so the
        pyserial object behind it is used directly."""
        ser = getattr(self.port, "ser", None)
        if ser is not None:
            try:
                ser.reset_input_buffer()
            except Exception:
                pass

    def _retry(self, func, dxl_id, what, *args):
        for attempt in range(RETRIES):
            out = func(self.port, dxl_id, *args)
            result, error = out[-2], out[-1]
            if result == dxl.COMM_SUCCESS and error == 0:
                return out
            if result == dxl.COMM_SUCCESS:          # valid packet with an error flag:
                self._check(dxl_id, what, result, error)   # a real servo fault, no retry
            if attempt + 1 < RETRIES:
                time.sleep(0.005 + 0.01 * attempt)      # let the rest of a bad packet arrive ...
                self._discard_input()                  # ... then throw it away
        self._check(dxl_id, what, result, error)

    def write1(self, dxl_id, addr, value):
        self._retry(self.packet.write1ByteTxRx, dxl_id, f"write addr {addr}", addr, int(value))

    def write2(self, dxl_id, addr, value):
        self._retry(self.packet.write2ByteTxRx, dxl_id, f"write addr {addr}", addr, int(value))

    def read1(self, dxl_id, addr):
        return self._retry(self.packet.read1ByteTxRx, dxl_id, f"read addr {addr}", addr)[0]

    def read2(self, dxl_id, addr):
        return self._retry(self.packet.read2ByteTxRx, dxl_id, f"read addr {addr}", addr)[0]

    # -- life cycle -----------------------------------------------------------
    def open(self):
        if not self.port.openPort():
            raise IOError(f"Failed to open port {self.port.getPortName()}")
        self.is_open = True
        if not self.port.setBaudRate(self.baudrate):
            raise IOError(f"Failed to set baud rate {self.baudrate}")

        for dxl_id in self.ids:
            for attempt in range(RETRIES):              # a garbled reply is retried,
                model, result, error = self.packet.ping(self.port, dxl_id)
                if result == dxl.COMM_SUCCESS:          # an error flag is not
                    break
                time.sleep(0.005 + 0.01 * attempt)
                self._discard_input()
            self._check(dxl_id, "ping", result, error)
            if model != C.AX12A_MODEL_NUMBER:
                print(f"  warning: ID {dxl_id} has model number {model} (AX-12A is 12)")

        # Hold the CURRENT pose before torque is enabled, so nothing jumps.
        # (The Annex C template instead commands a fixed pose first; writing
        # the present position as the goal is the safer order.) This read must
        # be real for every servo, so it is strict.
        ticks = self.read_ticks(strict=True)
        self.write_goal_speed(ticks, [40] * len(self.ids))
        for dxl_id in self.ids:
            self.write1(dxl_id, ADDR_CW_COMPLIANCE_MARGIN, C.COMPLIANCE_MARGIN)
            self.write1(dxl_id, ADDR_CCW_COMPLIANCE_MARGIN, C.COMPLIANCE_MARGIN)
            self.write1(dxl_id, ADDR_CW_COMPLIANCE_SLOPE, C.COMPLIANCE_SLOPE)
            self.write1(dxl_id, ADDR_CCW_COMPLIANCE_SLOPE, C.COMPLIANCE_SLOPE)
            self.write2(dxl_id, ADDR_PUNCH, C.PUNCH)
            self.write2(dxl_id, ADDR_TORQUE_LIMIT, C.TORQUE_LIMIT)
        self.set_torque(True)

    def close(self, release=False):
        if not self.is_open:
            return
        if release:
            try:
                self.set_torque(False)
            except IOError as e:
                print(f"  warning: could not disable torque: {e}")
        if self.failed_total:
            print(f"  note: {self.failed_total} corrupted/missing status packets were tolerated "
                  "(check the cables and the power supply if this number is large)")
        self.port.closePort()
        self.is_open = False

    # -- I/O ----------------------------------------------------------------
    def set_torque(self, on):
        for dxl_id in self.ids:
            self.write1(dxl_id, ADDR_TORQUE_ENABLE, 1 if on else 0)

    def read_ticks(self, strict=False):
        """Present position of all servos [ticks].

        strict=False (normal use): if one servo still answers with a corrupted
        packet after all retries, its last good value is used for this cycle.
        Only after MAX_FAILED_READS failures in a row is that treated as a real
        fault (cable / power / servo) and an error raised."""
        ticks = []
        self.last_read_fresh = True               # False if any value is a reused old one
        for k, dxl_id in enumerate(self.ids):
            try:
                ticks.append(self.read2(dxl_id, ADDR_PRESENT_POSITION))
                self.failed_in_a_row[dxl_id] = 0
            except ServoError:
                raise                                   # overload etc.: never tolerated
            except IOError as e:
                self.failed_total += 1
                self.failed_in_a_row[dxl_id] += 1
                if strict or self.last_ticks is None:
                    raise IOError(f"{e}\n  -> servo {dxl_id} does not answer reliably. Check its "
                                  "cable/connector, the power supply, and that the Dynamixel "
                                  "Wizard is closed.") from None
                if self.failed_in_a_row[dxl_id] >= MAX_FAILED_READS:
                    raise IOError(f"{e}\n  -> servo {dxl_id} failed {MAX_FAILED_READS} reads in a row. "
                                  "Check its cable/connector and the power supply.") from None
                ticks.append(self.last_ticks[k])
                self.last_read_fresh = False
        self.last_ticks = np.array(ticks)
        return self.last_ticks.copy()

    def read_angle_limits(self):
        """CW/CCW angle limits stored in every servo [ticks], as (n, 2) array.

        A goal outside these limits is ignored by the servo. Because goals are
        sent by sync write (no status reply), the servo's 'Angle Limit Error'
        would never be seen, so the limits are read once and respected."""
        return np.array([[self.read2(i, ADDR_CW_ANGLE_LIMIT), self.read2(i, ADDR_CCW_ANGLE_LIMIT)]
                         for i in self.ids])

    def is_moving(self):
        """True if any servo reports 'moving'. Servos that do not answer are skipped
        (the position tolerance in wait_settled still applies)."""
        for dxl_id in self.ids:
            try:
                if self.read1(dxl_id, ADDR_MOVING):
                    return True
            except ServoError:
                raise
            except IOError:
                self.failed_total += 1
        return False

    def write_goal_speed(self, goals, speeds):
        """One sync-write packet: goal position + moving speed for all servos.

        Addresses 30-33 are contiguous (goal L/H, speed L/H), so one
        GroupSyncWrite starting at 30 with 4 data bytes covers both. The
        packet goes to the broadcast ID and the SDK does not wait for a
        status packet, so this call returns in well under a millisecond."""
        try:
            for dxl_id, g, s in zip(self.ids, goals, speeds):
                g = int(min(1023, max(0, g)))
                s = int(min(1023, max(1, s)))     # never 0 (= uncontrolled max speed)
                ok = self.sync_write.addParam(dxl_id, [dxl.DXL_LOBYTE(g), dxl.DXL_HIBYTE(g),
                                                       dxl.DXL_LOBYTE(s), dxl.DXL_HIBYTE(s)])
                if not ok:
                    raise IOError(f"sync write: addParam failed for ID {dxl_id}")
            result = self.sync_write.txPacket()
        finally:
            self.sync_write.clearParam()          # also after an exception
        if result != dxl.COMM_SUCCESS:
            raise IOError(f"sync write: {self.packet.getTxRxResult(result)}")


# ---------------------------------------------------------------------------
# High level: joint angles, smooth trajectories
# ---------------------------------------------------------------------------
class Robot:
    """Works with DxlArm or with sim.SimArm (same tick-level interface)."""

    def __init__(self, arm, poll=None, realtime=True):
        self.arm = arm
        self.poll = poll or (lambda: None)   # called every control period
        self.realtime = realtime
        self.dir = np.array(C.JOINT_DIRECTION, float)
        self.off = np.array(C.JOINT_OFFSET_DEG, float)
        self.q_cmd = None

    # -- conversions ----------------------------------------------------------
    def q_to_ticks(self, q):
        deg = np.degrees(q)
        return np.round(C.TICKS_CENTER + self.dir * (deg + self.off) / C.DEG_PER_TICK).astype(int)

    def ticks_to_q(self, ticks):
        deg = (np.asarray(ticks, float) - C.TICKS_CENTER) * C.DEG_PER_TICK / self.dir - self.off
        return np.radians(deg)

    def read_q(self):
        return self.ticks_to_q(self.arm.read_ticks())

    def apply_servo_angle_limits(self, margin_deg=2.0):
        """Shrink the software joint limits to the angle limits stored in the
        servos (EEPROM), so no trajectory ever asks for an angle the servo
        would silently refuse. Prints both ranges."""
        if not hasattr(self.arm, "read_angle_limits"):
            return
        lim_ticks = self.arm.read_angle_limits()
        new = np.radians(np.array(C.JOINT_LIMITS_DEG, float))
        print("  servo angle limits (EEPROM) -> joint range used:")
        for j, (cw, ccw) in enumerate(lim_ticks):
            if cw == 0 and ccw == 0:
                print(f"    joint {j + 1}: servo in WHEEL mode (CW = CCW = 0) - check it in Dynamixel Wizard!")
                continue
            a, b = np.radians((np.array([cw, ccw], float) - C.TICKS_CENTER) * C.DEG_PER_TICK
                              / self.dir[j] - self.off[j])
            lo, hi = min(a, b) + math.radians(margin_deg), max(a, b) - math.radians(margin_deg)
            new[j, 0], new[j, 1] = max(new[j, 0], lo), min(new[j, 1], hi)
            tag = "" if (cw, ccw) == (0, 1023) else "   <- limited by the servo"
            print(f"    joint {j + 1}: servo {math.degrees(min(a, b)):7.1f} .. {math.degrees(max(a, b)):6.1f} deg"
                  f"   used {math.degrees(new[j, 0]):7.1f} .. {math.degrees(new[j, 1]):6.1f} deg{tag}")
        kin.JOINT_LIMITS = new
        C.JOINT_LIMITS_DEG = tuple(tuple(round(float(v), 1) for v in row) for row in np.degrees(new))

    # -- commands ---------------------------------------------------------------
    def command(self, q, qdot):
        """Send one set-point with per-joint speeds (rad/s).

        Joint limits are checked by the trajectory generators (ik_tool,
        move_joints), not here: a move that starts slightly outside the
        limits must still be allowed to bring the arm back inside."""
        units = np.degrees(np.abs(qdot)) * C.SPEED_HEADROOM / C.SPEED_UNIT_DEG_S
        units = np.clip(np.round(units), max(1, round(C.MIN_SERVO_SPEED_DEG_S / C.SPEED_UNIT_DEG_S)), 1023)
        self.arm.write_goal_speed(self.q_to_ticks(q), units)
        self.q_cmd = np.array(q, float)

    def stream(self, q_samples, dt=1.0 / C.CONTROL_HZ, on_step=None):
        """Play a dense joint trajectory, one sample every dt seconds."""
        if self.q_cmd is None:
            self.q_cmd = self.read_q()
        prev = self.q_cmd
        t_next = time.perf_counter()
        for i, q in enumerate(q_samples):
            step = np.abs(q - prev)
            if np.degrees(step.max()) > C.MAX_JOINT_STEP_DEG:
                raise RuntimeError(f"joint jump of {np.degrees(step.max()):.1f} deg in one "
                                   "sample - trajectory rejected")
            self.command(q, step / dt)
            prev = q
            if on_step is not None:
                on_step(i)
            self.poll()
            if self.realtime:
                t_next += dt
                time.sleep(max(0.0, t_next - time.perf_counter()))
        # last set-point once more, at a brisk speed: the final samples of a
        # smooth move are tiny, and a lagging joint would otherwise crawl to its
        # goal at ~1 deg/s and jitter under the load of the arm and stylus
        if len(q_samples):
            self.command(self.q_cmd, np.full(len(self.q_cmd), np.radians(C.SETTLE_SPEED_DEG_S)) / C.SPEED_HEADROOM)

    def wait_settled(self, tol_deg=C.SETTLE_TOL_DEG, timeout=C.SETTLE_TIMEOUT_S, check_moving=True):
        """Wait until every servo reports 'not moving' and is within tol_deg.

        check_moving=False: only the position tolerance counts. Used while the
        stylus presses on the paper - the goal is then slightly below the surface,
        the servo can never reach it exactly and keeps its 'Moving' flag set.

        Blocked joint detection: if a joint is still far from its goal but has
        not moved for STUCK_TIME_S, it is pushing against something (end stop,
        table, link). All servos are then told to hold their present position,
        so the servo stops pushing BEFORE its overload protection trips, and a
        JointBlocked error explains which joint and where."""
        t0 = time.perf_counter()
        q_prev, t_prev = None, t0
        while True:
            q = self.read_q()
            err = np.degrees(np.abs(q - self.q_cmd))
            if err.max() < tol_deg and (not check_moving or not self.arm.is_moving()):
                return True
            now = time.perf_counter()
            if q_prev is None or np.degrees(np.abs(q - q_prev)).max() > 0.5:
                q_prev, t_prev = q, now                   # still making progress
            elif now - t_prev > C.STUCK_TIME_S and err.max() > C.STUCK_ERROR_DEG:
                j = int(np.argmax(err))
                goal = self.q_cmd.copy()
                self.hold()
                raise JointBlocked(
                    f"joint {j + 1} is blocked: it stays at {np.degrees(q[j]):.1f} deg but was "
                    f"commanded to {np.degrees(goal[j]):.1f} deg (all joints now hold their position).\n"
                    f"  measured q = {np.round(np.degrees(q), 1)} deg\n"
                    f"  goal     q = {np.round(np.degrees(goal), 1)} deg\n"
                    f"  -> run --check-joints: check the direction of joint {j + 1} (JOINT_DIRECTION) "
                    f"and how far it can really move, then set JOINT_LIMITS_DEG[{j}] to that range.")
            if now - t0 > timeout:
                print(f"  warning: not settled, joint errors {np.round(err, 1)} deg")
                return False
            self.poll()
            time.sleep(0.03)

    def hold(self):
        """Stop where we are: command the present position at low speed."""
        try:
            q = self.read_q()
        except Exception:                      # bus trouble: freeze at the last command
            q = self.q_cmd
        if q is None:
            return
        try:
            self.arm.write_goal_speed(self.q_to_ticks(q), [60] * 4)
            self.q_cmd = np.array(q, float)
        except Exception as e:  # best effort during shutdown
            print(f"  warning: hold failed: {e}")

    # -- motion primitives ----------------------------------------------------
    @staticmethod
    def min_jerk(n):
        tau = np.linspace(0.0, 1.0, n + 1)[1:]
        return 10 * tau ** 3 - 15 * tau ** 4 + 6 * tau ** 5

    def move_joints(self, q_goal, speed_deg_s=C.JOINT_SPEED_DEG_S, settle=True):
        """Minimum-jerk joint-space move (peak speed = 1.875 * distance / T)."""
        kin.check_limits(q_goal)
        if self.q_cmd is None:
            self.q_cmd = self.read_q()
        q0 = self.q_cmd.copy()
        try:
            kin.check_limits(q0)
        except kin.IKError as e:
            print(f"  warning: start pose outside the software limits ({e}); moving back inside")
        dist = np.degrees(np.abs(q_goal - q0)).max()
        T = max(C.MIN_MOVE_TIME_S, 1.875 * dist / speed_deg_s)
        n = max(1, int(math.ceil(T * C.CONTROL_HZ)))
        s = self.min_jerk(n)
        self.stream([q0 + si * (q_goal - q0) for si in s])
        if settle:
            self.wait_settled()

    def move_linear(self, p_from, p_to, pitch, speed=C.LINEAR_SPEED_MM_S, on_paper=False):
        """Straight Cartesian line for the STYLUS TIP (min-jerk along the line);
        pitch = stylus angle above the horizontal.

        on_paper=True: the end point presses into the paper, so only the joint
        angles are checked when settling (see wait_settled)."""
        p_from, p_to = np.asarray(p_from, float), np.asarray(p_to, float)
        L = np.linalg.norm(p_to - p_from)
        T = max(C.MIN_MOVE_TIME_S, 1.875 * L / speed)
        n = max(1, int(math.ceil(T * C.CONTROL_HZ)))
        qs = [kin.ik_tool(p_from + s * (p_to - p_from), pitch) for s in self.min_jerk(n)]
        if self.q_cmd is not None and np.degrees(np.abs(qs[0] - self.q_cmd).max()) > 0.5 * C.MAX_JOINT_STEP_DEG:
            self.move_joints(qs[0])                     # start pose differs (e.g. stylus angle): get there smoothly
        self.stream(qs)
        self.wait_settled(check_moving=not on_paper)


def time_parametrize(path, v_max, a_max, hz=C.CONTROL_HZ):
    """Trapezoidal speed profile along a polyline -> points sampled at hz."""
    path = np.asarray(path, float)
    seg = np.linalg.norm(np.diff(path, axis=0), axis=1)
    s_knots = np.concatenate(([0.0], np.cumsum(seg)))
    L = s_knots[-1]
    if L < 1e-6:
        return path[-1:].copy(), 1.0 / hz
    t_acc = v_max / a_max
    if a_max * t_acc ** 2 > L:                   # triangular profile
        t_acc = math.sqrt(L / a_max)
        v_max = a_max * t_acc
    d_acc = 0.5 * a_max * t_acc ** 2
    T = 2 * t_acc + (L - 2 * d_acc) / v_max
    t = np.arange(1, int(math.ceil(T * hz)) + 1) / hz
    t = np.minimum(t, T)
    s = np.where(t < t_acc, 0.5 * a_max * t ** 2,
                 np.where(t < T - t_acc, d_acc + v_max * (t - t_acc),
                          L - 0.5 * a_max * (T - t) ** 2))
    pts = np.column_stack([np.interp(s, s_knots, path[:, k]) for k in range(path.shape[1])])
    return pts, T
