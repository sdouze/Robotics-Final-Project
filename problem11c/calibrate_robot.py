"""Guided calibration of the robot - the arm moves by itself, you only answer.

    python calibrate_robot.py               (add --camera 1 if needed)

What it does (about 3 minutes):
  1. Moves the arm SLOWLY to straight up (all servos at their centre).
     If a link is not straight you correct it with small jogs, e.g. "3 +5".
  2. Moves each joint a little and asks one letter: which way did it move?
     -> JOINT_DIRECTION (sign of every joint) and JOINT_OFFSET_DEG.
  3. Asks which way the camera lens looks and where the table/front appears in
     the live camera image -> CAMERA_OPTICAL_AXIS_IN_F5, CAMERA_IMAGE_RIGHT/DOWN.
  4. Shows the result and writes it into config.py (old file kept as backup).

Definitions used in the questions:
  FRONT = the side of the robot where the paper will lie.
  LEFT / RIGHT = as seen standing BEHIND the robot, looking towards its front.
Keep a hand near the power switch. Ctrl+C stops (the arm then holds still).
"""

import argparse
import os
import re
import shutil
import time

import numpy as np

import config as C
import kinematics as kin
from robot import DxlArm, JointBlocked, Robot, ServoError

SPEED = 15.0                        # deg/s - slow on purpose
TESTS = (                           # joint, test move [deg], question, answer expected if sign is right
    (0, +20.0, "Did the arm turn to the robot's LEFT or RIGHT?", "lr", "l"),
    (1, -15.0, "Did the arm lean FORWARD (towards the paper side) or BACKWARD?", "fb", "f"),
    (2, -15.0, "Did the upper part of the arm (from the elbow) lean FORWARD or BACKWARD?", "fb", "f"),
    (3, +10.0, "Did the stylus tip tilt FORWARD or BACKWARD?", "fb", "b"),
)
# Directions at q = 0 expressed in frame 4/5 (x4 = up, y4 = backward, z4 = robot's right)
LENS_DIRS = {"u": (1, 0, 0), "d": (-1, 0, 0), "b": (0, 1, 0), "f": (0, -1, 0),
             "r": (0, 0, 1), "l": (0, 0, -1)}
WORLD_DOWN_F5 = np.array([-1.0, 0.0, 0.0])
WORLD_FRONT_F5 = np.array([0.0, -1.0, 0.0])


def ask(question, options):
    while True:
        a = input(f"  {question} [{'/'.join(options)}]: ").strip().lower()
        if a in options:
            return a
        print(f"  please answer one of: {', '.join(options)}")


def ask_in_window(camera, question, options):
    """Show the live camera image and wait for one of the option keys in that window."""
    import cv2
    print(f"  {question}\n  -> click the camera window and press one of: {', '.join(options)}")
    while True:
        img = camera.read().copy()
        cv2.putText(img, question[:70], (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
        cv2.putText(img, "keys: " + " / ".join(options), (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 255, 255), 2)
        cv2.imshow("camera", img)
        key = cv2.waitKey(30) & 0xFF
        if key in (27,):
            raise KeyboardInterrupt
        if chr(key) in options:
            return chr(key)


def move(robot, q_deg, label=""):
    if label:
        print(f"  moving {label} ...")
    robot.move_joints(np.radians(q_deg), speed_deg_s=SPEED)


def write_config(values):
    """Replace the assignment lines in config.py; keep a timestamped backup."""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.py")
    backup = path.replace(".py", time.strftime("_backup_%Y%m%d_%H%M%S.py"))
    shutil.copyfile(path, backup)
    text = open(path, encoding="utf-8").read()
    for name, val in values.items():
        new, n = re.subn(rf"^{name} = \([^)]*\)", f"{name} = {val}", text, flags=re.M)
        if n != 1:
            raise RuntimeError(f"could not find '{name} = (...)' in config.py")
        text = new
    open(path, "w", encoding="utf-8").write(text)
    return path, backup


def fmt(v):
    return "(" + ", ".join(f"{x:g}" for x in v) + ")"


# ---------------------------------------------------------------------------
# Tool measurement: where are the stylus tip and the camera lens?
# ---------------------------------------------------------------------------
TOOL_HELP = """
TOOL MEASUREMENT - the arm stands straight up (as in Figure 1). Use a ruler.

  WRIST AXIS = the centre of the shaft of the LAST servo (joint 4), i.e. the
  joint the camera/stylus part rotates about.

  For the camera lens and for the stylus tip you give three distances, all
  measured from the wrist axis, with the arm straight up:
    UP      along the arm, upwards                        (mm)
    FORWARD towards the paper side (negative = backwards) (mm)
    RIGHT   to the robot's right, seen from behind it     (mm, Enter = 0)

        side view            UP
                              ^      lens  o----   (looks forward)
                              |      stylus ======>  tip
        robot back   <--------+--------->  FORWARD (paper side)
                         wrist axis (joint 4)
"""


def ask_mm(question, default=None):
    while True:
        a = input(f"  {question}{'' if default is None else f' [Enter = {default:g}]'}: ").strip()
        if not a and default is not None:
            return float(default)
        try:
            return float(a.replace(",", "."))
        except ValueError:
            print("  please type a number in mm, e.g. 62 or -10")


def tool_values(lens, tip, direction):
    """Measurements (up, forward, right from the wrist axis, arm straight up)
    -> config values in frame 4: x4 = up - L4, y4 = -forward, z4 = right."""
    to_f4 = lambda u, f, r: (round(u - C.L4, 1), round(-f, 1), round(r, 1))    # noqa: E731
    dirs = {"u": (1, 0, 0), "f": (0, -1, 0), "b": (0, 1, 0)}
    return {"CAMERA_OFFSET_F4": fmt(to_f4(*lens)),
            "TOOL_TIP_IN_F4": fmt(to_f4(*tip)),
            "TOOL_DIR_IN_F4": fmt(dirs[direction])}


def measure_tool(args):
    if args.sim:
        import sim
        arm = sim.SimArm()
    else:
        arm = DxlArm(args.port)
    robot = Robot(arm, realtime=not args.sim)
    try:
        arm.open()
        robot.apply_servo_angle_limits()
        print(TOOL_HELP)
        input("  The arm will move SLOWLY to straight up (with the calibrated joints). Press Enter...")
        q = np.degrees(robot.read_q())
        move(robot, [q[0], 0.0, q[2], q[3]], "shoulder (joint 2) up")
        move(robot, [q[0], 0.0, 0.0, 0.0], "elbow and wrist (joints 3, 4) straight")
        move(robot, [0.0, 0.0, 0.0, 0.0], "base (joint 1) to centre")

        print("\n  CAMERA LENS (centre of the lens):")
        lens = (ask_mm("UP from the wrist axis"), ask_mm("FORWARD from the wrist axis"),
                ask_mm("RIGHT of the arm's centre line", 0.0))
        print("\n  STYLUS:")
        direction = ask("which way does the stylus point?  u = up (along the arm, Figure 1)  "
                        "f = forward (same way as the lens)  b = backward", "ufb")
        tip = (ask_mm("TIP: UP from the wrist axis"), ask_mm("TIP: FORWARD from the wrist axis"),
               ask_mm("TIP: RIGHT of the arm's centre line", 0.0))

        # the stylus tip must be the first thing that touches the paper
        if direction == "f" and tip[1] < lens[1] + 5.0:
            print(f"\n  WARNING: the stylus tip is only {tip[1] - lens[1]:.0f} mm in front of the lens. "
                  "It must stick out further than the camera (at least 5 mm, better 20-40 mm),\n"
                  "  otherwise the camera touches the paper first.")
        if direction == "u" and tip[0] < lens[0] + 5.0:
            print(f"\n  WARNING: the stylus tip is only {tip[0] - lens[0]:.0f} mm above the lens - "
                  "the camera would touch the paper first.")
        values = tool_values(lens, tip, direction)
        print("\nRESULT:")
        for k, v in values.items():
            print(f"  {k} = {v}")
        if ask("Write these values into config.py?", "yn") == "y":
            path, backup = write_config(values)
            print(f"  written to {path}\n  old version saved as {backup}")
            print("\nNext: python line_follower.py --check-camera   (click a mark, press t: "
                  "the STYLUS TIP should stop 3 mm above it)")
        else:
            print("  nothing written.")
    except (KeyboardInterrupt, EOFError):
        print("\nSTOPPED - holding the current position.")
        robot.hold()
    except (JointBlocked, ServoError) as e:
        robot.hold()
        print(f"\nSTOPPED: {e}")
    finally:
        arm.close()


def calibrate(robot, camera):
    # Work in RAW servo angles: q = 0 is the servo centre (tick 512), direction +1.
    robot.dir = np.ones(4)
    robot.off = np.zeros(4)
    kin.JOINT_LIMITS = np.radians(np.array([[-145.0, 145.0]] * 4))   # blocked-joint detection still active

    q = np.degrees(robot.read_q())
    print(f"\nSTEP 1 - straight up.  Servo angles now (from centre): {np.round(q, 1)} deg")
    input("  The arm will move SLOWLY to straight up (joint 2 first, so it lifts off the table).\n"
          "  Press Enter to start...")
    move(robot, [q[0], 0.0, q[2], q[3]], "shoulder (joint 2) up")
    move(robot, [q[0], 0.0, 0.0, 0.0], "elbow and wrist (joints 3, 4) straight")
    move(robot, [0.0, 0.0, 0.0, 0.0], "base (joint 1) to centre")

    jog = np.zeros(4)
    print("\n  Now compare with Figure 1: all links in ONE vertical line, stylus pointing up,\n"
          "  and the arm should bend towards the FRONT (paper side), i.e. the servo shafts of\n"
          "  joints 2-4 point to the left/right.\n"
          "  If something is off, jog it: type '<joint> <degrees>', e.g. '3 +5' or '1 -90'.\n"
          "  Press Enter (empty line) when it is straight.")
    while True:
        cmd = input("  jog> ").strip()
        if not cmd:
            break
        m = re.fullmatch(r"([1-4])\s*([+-]?\d+(\.\d+)?)", cmd)
        if not m:
            print("  format: joint and degrees, e.g. '3 +5'")
            continue
        jog[int(m.group(1)) - 1] += float(m.group(2))
        move(robot, jog)
    straight_ticks = robot.q_to_ticks(np.radians(jog))
    print(f"  straight pose = servo ticks {straight_ticks.tolist()}")

    print("\nSTEP 2 - directions. Each joint moves a little and comes back.")
    direction = np.ones(4, int)
    for j, delta, question, options, expected in TESTS:
        test = jog.copy()
        test[j] += delta
        move(robot, test, f"joint {j + 1} by {delta:+.0f} deg")
        answer = ask(question, options)
        direction[j] = 1 if answer == expected else -1
        print(f"  -> joint {j + 1}: {'OK, same direction as the model' if direction[j] == 1 else 'REVERSED'}")
        move(robot, jog)

    # offset: the straight pose must become q = 0 with the new directions
    offset = np.round(direction * (straight_ticks - C.TICKS_CENTER) * C.DEG_PER_TICK, 2)

    print("\nSTEP 3 - camera (the arm stays straight up).")
    print("  Which way does the camera LENS look right now?\n"
          "    u = up (same way as the stylus)   d = down\n"
          "    f = forward (paper side)          b = backward\n"
          "    l = left                          r = right")
    lens = ask("lens looks", list(LENS_DIRS))
    z_c = np.array(LENS_DIRS[lens], float)
    if lens in "ud":
        world, what = WORLD_FRONT_F5, ("Hold your hand on the FRONT (paper) side, above the camera. "
                                       "Where does it appear in the image?")
    else:
        world, what = WORLD_DOWN_F5, "Where is the TABLE / floor in the camera image?"
    q2 = f"{what} b=bottom t=top l=left r=right"
    side = ask_in_window(camera, q2, "btlr") if camera is not None else ask(q2, "btlr")
    if side in "bt":
        y_c = world if side == "b" else -world
        x_c = np.cross(y_c, z_c)
    else:
        x_c = world if side == "r" else -world
        y_c = np.cross(z_c, x_c)

    values = {
        "JOINT_DIRECTION": fmt(direction),
        "JOINT_OFFSET_DEG": fmt(offset),
        "CAMERA_OPTICAL_AXIS_IN_F5": fmt(z_c),
        "CAMERA_IMAGE_RIGHT_IN_F5": fmt(x_c),
        "CAMERA_IMAGE_DOWN_IN_F5": fmt(y_c),
    }
    print("\nRESULT:")
    for k, v in values.items():
        print(f"  {k} = {v}")
    if lens in "lr":
        print("  WARNING: a lens looking sideways cannot be used for the camera aiming in\n"
              "  line_follower.py - check the answer, the lens normally looks along or across the stylus.")
    return values


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=C.PORT)
    ap.add_argument("--camera", type=int, default=C.CAMERA_INDEX)
    ap.add_argument("--no-camera", action="store_true", help="answer the camera question in the terminal")
    ap.add_argument("--sim", action="store_true", help="test the script with the simulator")
    ap.add_argument("--tool", action="store_true",
                    help="only measure the stylus and camera position with a ruler (arm straight up)")
    args = ap.parse_args()
    if args.tool:
        measure_tool(args)
        return
    print(__doc__)

    if args.sim:
        import sim
        arm = sim.SimArm()
    else:
        arm = DxlArm(args.port)
    robot = Robot(arm, realtime=not args.sim)
    camera = None
    try:
        arm.open()
        if not (args.no_camera or args.sim):
            from line_follower import Camera
            camera = Camera(args.camera)
        values = calibrate(robot, camera)
        if ask("Write these values into config.py?", "yn") == "y":
            path, backup = write_config(values)
            print(f"  written to {path}\n  old version saved as {backup}")
            print("\nNext: python line_follower.py --check-camera")
        else:
            print("  nothing written.")
    except (KeyboardInterrupt, EOFError):
        print("\nSTOPPED - holding the current position.")
        robot.hold()
    except (JointBlocked, ServoError) as e:
        robot.hold()
        print(f"\nSTOPPED: {e}")
    finally:
        if camera is not None:
            camera.release()
            import cv2
            cv2.destroyAllWindows()
        arm.close()


if __name__ == "__main__":
    main()
