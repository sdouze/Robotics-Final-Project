"""Problem 11c - follow a black line on an A4 sheet with the stylus.

Sequence (state machine in run_task):
  1. HOME    smooth move to an observation pose (camera looking down)
  2. SEARCH  sweep joint 1 at two radii until a white A4 sheet is in view
  3. LOCK    re-centre the camera over the visible paper until it converges
  4. MAP     visit a grid of camera poses over the paper; every image is
             projected onto the table plane (frame 0) into one top-down map
  5. DETECT  find the start dot and the black line, order the line from the
             dot to its end; extra views are taken if the line leaves the map
  6. PLAN    inverse kinematics for every path point (steepest feasible
             stylus pitch), trapezoidal speed profile along the line
  7. FOLLOW  hover over the dot -> touch down -> trace the line -> lift -> home

Usage (run from this folder, Python with numpy + opencv-python + pyserial):
  python line_follower.py --sim            full test with the simulator
  python line_follower.py --camera-only    live paper/ink detection, no robot
  python line_follower.py --check-joints   torque OFF, print q and FK while
                                           you move the arm by hand
  python line_follower.py --check-camera   verify camera model + PAPER_Z:
                                           click a point, press t -> stylus
                                           goes to it; b -> back
  python line_follower.py                  run the task on the robot
Options: --port COM8 --camera 1 --auto (no confirmation) --release (torque
off at the end) --no-gui --fast (simulator without real-time waits).
Press q or Esc in any window to stop the robot.
"""

import argparse
import glob
import json
import math
import os
import threading
import time

import cv2
import numpy as np

import config as C
import kinematics as kin
import vision as vis
from robot import DxlArm, JointBlocked, Robot, ServoError, UserAbort, time_parametrize


# ---------------------------------------------------------------------------
# Camera and UI
# ---------------------------------------------------------------------------
class Camera:
    """USB camera read by a background thread.

    The thread keeps grabbing frames, so the driver buffer never fills up with
    old images and read() returns the newest frame at once (smooth live view).
    read(fresh=True) waits until FRAMES_TO_FLUSH new frames have arrived, i.e.
    frames that were captured after the arm stopped moving."""

    def __init__(self, index):
        backend = cv2.CAP_DSHOW if os.name == "nt" else cv2.CAP_ANY
        self.cap = cv2.VideoCapture(index, backend)
        if not self.cap.isOpened():
            raise RuntimeError(f"cannot open camera {index} (try --camera 0)")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, C.FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, C.FRAME_HEIGHT)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        ok, frame = self.cap.read()
        if not ok:
            raise RuntimeError("camera gives no images")
        self._lock = threading.Lock()
        self._frame, self._count = frame, 1
        self._running = True
        self._thread = threading.Thread(target=self._grab_loop, daemon=True)
        self._thread.start()
        h, w = frame.shape[:2]
        self.dist = None
        if os.path.exists(C.CALIBRATION_FILE):
            data = np.load(C.CALIBRATION_FILE)
            if tuple(data["size"]) != (w, h):
                raise RuntimeError(f"calibration was made for {tuple(data['size'])}, camera gives {(w, h)}")
            self.model = kin.CameraModel(data["K"], w, h)
            self.dist = data["dist"]
            self.model.set_undistortion(self.dist)
            print(f"Camera: {w}x{h}, calibration loaded from {C.CALIBRATION_FILE}")
        else:
            self.model = kin.CameraModel.from_fov(w, h, C.CAMERA_HFOV_DEG)
            print(f"Camera: {w}x{h}, no calibration file -> HFOV {C.CAMERA_HFOV_DEG} deg assumed")

    def _grab_loop(self):
        while self._running:
            ok, frame = self.cap.read()
            if ok and frame is not None:
                with self._lock:
                    self._frame, self._count = frame, self._count + 1
            else:
                time.sleep(0.01)

    def read(self, fresh=False):
        """Newest frame. fresh=True: wait for frames captured after this call."""
        with self._lock:
            start = self._count
        if fresh:
            deadline = time.perf_counter() + 3.0
            while True:
                with self._lock:
                    if self._count >= start + C.FRAMES_TO_FLUSH:
                        break
                if time.perf_counter() > deadline:
                    raise RuntimeError("the camera stopped delivering images")
                time.sleep(0.005)
        with self._lock:
            frame = self._frame.copy()
        if self.dist is not None:
            frame = cv2.undistort(frame, self.model.K, self.dist)
        return frame

    def release(self):
        self._running = False
        self._thread.join(timeout=1.0)
        self.cap.release()


class UI:
    LIVE_WINDOW = "camera (live)"

    def __init__(self, enabled):
        self.enabled = enabled
        self.live = None            # a Camera: shown live in its own window, also while moving
        self._t_live = 0.0

    def show(self, name, img):
        if self.enabled:
            cv2.imshow(name, img)
            self.poll()

    def poll(self):
        if self.enabled:
            if self.live is not None and time.perf_counter() - self._t_live > 0.08:
                self._t_live = time.perf_counter()
                try:
                    cv2.imshow(self.LIVE_WINDOW, self.live.read())
                except RuntimeError:
                    pass
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                raise UserAbort("stopped by user")
            return key
        return -1

    def confirm(self, text):
        print(text)
        if not self.enabled:
            return input("Type 'g' + Enter to go, anything else aborts: ").strip().lower() == "g"
        print("  -> press 'g' in the image window to go, 'q' to abort")
        while True:
            try:
                key = self.poll()
            except UserAbort:
                return False
            if key == ord("g"):
                return True
            time.sleep(0.03)

    def wait_retry(self, text):
        """Wait while the user arranges the sheet (live view keeps running).
        True = try again, False = quit."""
        print(text)
        if not self.enabled:
            return input("Press Enter to try again, 'q' + Enter to quit: ").strip().lower() != "q"
        print("  -> press 'r' in an image window to try again, 'q' to quit")
        while True:
            try:
                key = self.poll()
            except UserAbort:
                return False
            if key in (ord("r"), ord("g"), 13, 32):
                return True
            time.sleep(0.03)

    def close(self):
        if self.enabled:
            cv2.destroyAllWindows()


def annotate(frame, text, paper=None, ink=None):
    img = frame.copy()
    if paper is not None:
        cs, _ = cv2.findContours(paper, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(img, cs, -1, (0, 255, 0), 2)
    if ink is not None:
        img[ink > 0] = (0, 0, 255)
    cv2.putText(img, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    return img


# ---------------------------------------------------------------------------
# The task
# ---------------------------------------------------------------------------
class LineFollower:
    def __init__(self, robot, camera, ui, realtime=True, debug_subdir="debug"):
        self.robot = robot
        self.camera = camera
        self.cam = camera.model
        self.ui = ui
        self.realtime = realtime
        self.mosaic = vis.Mosaic(self.cam)
        self.pitch_obs = math.radians(C.OBSERVE_PITCH_DEG)
        # every image the decision is based on is saved here (overwritten each run;
        # simulator runs use their own folder so real images are never lost)
        self.debug_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), debug_subdir)
        os.makedirs(self.debug_dir, exist_ok=True)
        for f in glob.glob(os.path.join(self.debug_dir, "*.png")) + \
                glob.glob(os.path.join(self.debug_dir, "views.json")):
            os.remove(f)
        self.n_views = 0
        self.partial = False        # --partial: follow only the reachable part of a too long line
        self.dry = False            # --dry: no stylus mounted -> trace above the paper, no touching
        self.last_view_map = None
        self.view_log = []
        self.line_map = self.mosaic
        self.teach_mode = False     # --teach: measure the camera error with the stylus
        self.correction = None      # camera -> stylus correction (from --teach)
        self.correction_file = None
        self.paper_probed = False

    # -- helpers ------------------------------------------------------------
    LOOK_TOLERANCE_MM = 5.0      # the image centre must land this close to the target
    STYLUS_CLEARANCE_MM = 15.0   # keep the stylus tip this far above the paper while looking

    def camera_q(self, xy):
        """Joint angles (all four joints) that make the camera LOOK AT table point xy.

        The optical axis (x5 = stylus direction) lies in the vertical plane of
        the arm, which also contains the target. So for a viewing angle theta
        below the horizontal and a viewing distance d along the axis, the camera
        position is known exactly:
            r_cam = r_target - d cos(theta),   z_cam = PAPER_Z + d sin(theta)
        and ik_camera() gives all four joint angles. The search keeps, among
        all (theta, d) whose pose respects the joint limits and keeps the
        stylus clear of the paper, the one with the HIGHEST camera (largest
        field of view), preferring views at least 45 deg down."""
        target = np.asarray(xy, float)
        r0, az = math.hypot(*target), math.atan2(target[1], target[0])
        u_dir = np.array([math.cos(az), math.sin(az)])
        # Points close to the base cannot be aimed at when the wrist is limited;
        # then aim a bit farther out on the same line (the point itself is still
        # in the lower part of the image).
        for extra in np.arange(0.0, 100.1, 10.0):
            q = self._look_at(u_dir * (r0 + extra), u_dir, r0 + extra)
            if q is not None:
                if extra > 0 and not getattr(self, "_told_far", False):
                    print(f"  note: points closer than ~{r0 + extra:.0f} mm cannot be aimed at with the "
                          "wrist limit; the camera aims slightly farther out")
                    self._told_far = True
                return q
        raise kin.IKError(f"no camera pose inside the joint limits can look at {np.round(target, 0)} mm")

    def _look_at(self, target, u_dir, r_t):
        """Best pose looking exactly at target (see camera_q), or None.

        theta is the angle of the OPTICAL AXIS below the horizontal. The stylus
        angle passed to ik_camera is theta minus the fixed angle between lens
        and stylus (kin.CAMERA_AXIS_OFFSET, 0 if the lens looks along the stylus)."""
        if kin.CAMERA_AXIS_OFFSET is None:
            raise kin.IKError("the camera looks sideways (CAMERA_OPTICAL_AXIS_IN_F5 along z5); "
                              "camera aiming only supports a lens looking within the arm plane")
        pp = [[self.cam.K[0, 2], self.cam.K[1, 2]]]              # principal point = optical axis
        best = {True: None, False: None}                        # steep enough? -> (h, q, theta)
        for theta_deg in np.arange(-90.0, -29.9, 2.5):
            th = math.radians(theta_deg)
            stylus_pitch = th - kin.CAMERA_AXIS_OFFSET
            for d in np.arange(40.0, 260.1, 5.0):
                h = -d * math.sin(th)                           # camera height above the paper
                if h > C.OBSERVE_HEIGHT or h < 30.0:
                    continue
                r_cam = r_t - d * math.cos(th)
                if r_cam < 20.0:
                    continue
                xy_cam = u_dir * r_cam
                try:
                    q = kin.ik_camera((xy_cam[0], xy_cam[1], C.PAPER_Z + h), stylus_pitch)
                except kin.IKError:
                    continue
                if kin.fk_tool(q)[0][2] < C.PAPER_Z + self.STYLUS_CLEARANCE_MM:
                    continue
                hit = self.cam.pixel_to_plane(kin.camera_pose(q), pp)[0][:2]
                if not np.all(np.isfinite(hit)) or np.linalg.norm(hit - target) > self.LOOK_TOLERANCE_MM:
                    continue
                steep = theta_deg <= -45.0
                if best[steep] is None or h > best[steep][0]:
                    best[steep] = (h, q, th)
        choice = best[True] or best[False]
        if choice is None:
            return None
        self._report_tilt(choice[2])
        return choice[1]

    def _report_tilt(self, pitch):
        if abs(pitch - self.pitch_obs) > 1e-6 and not getattr(self, "_told_pitch", False):
            print(f"  note: camera tilted to {math.degrees(pitch):.0f} deg instead of "
                  f"{math.degrees(self.pitch_obs):.0f} deg to stay inside JOINT_LIMITS_DEG")
            self._told_pitch = True

    def look(self, label):
        """Settle, grab a fresh image, return (frame, camera pose from MEASURED q).
        The raw image and the joint angles are saved in debug/ (raw_XX.png,
        views.json), so the detection can be re-run offline on real images."""
        if self.realtime:
            t0 = time.perf_counter()
            while time.perf_counter() - t0 < C.SETTLE_TIME_S:
                self.ui.poll()
                time.sleep(0.02)
        # the sharpest of a few fresh frames: right after a move the image can be
        # blurred (arm still shaking, webcam autofocus hunting)
        best, frame = -1.0, None
        for _ in range(max(1, C.LOOK_FRAMES)):
            f = self.camera.read(fresh=True)
            sharp = float(cv2.Laplacian(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY), cv2.CV_64F).var())
            if sharp > best:
                best, frame = sharp, f
        q = self.robot.read_q()
        T0c = kin.camera_pose(q)
        paper, frac = vis.paper_mask(frame)
        ink = vis.dark_mask(frame, paper)
        shown = annotate(frame, f"{label}  paper {100 * frac:.0f}%", paper, ink)
        self.n_views += 1
        cv2.imwrite(os.path.join(self.debug_dir, f"view_{self.n_views:02d}.png"), shown)
        cv2.imwrite(os.path.join(self.debug_dir, f"raw_{self.n_views:02d}.png"), frame)
        self.view_log.append(dict(view=self.n_views, label=label, q_deg=np.degrees(q).round(3).tolist(),
                                  paper_z=C.PAPER_Z, K=np.asarray(self.cam.K).round(3).tolist()))
        with open(os.path.join(self.debug_dir, "views.json"), "w", encoding="utf-8") as f:
            json.dump(self.view_log, f, indent=1)
        self.ui.show("detection", shown)
        return frame, T0c, paper, frac

    def save_map(self, line=None, mosaic=None):
        m = mosaic if mosaic is not None else self.mosaic
        img = m.render(line["path"], line["dot"]) if line is not None else m.render()
        cv2.imwrite(os.path.join(self.debug_dir, "map.png"), img)
        self.ui.show("map", img)

    def go_camera(self, xy):
        self.robot.move_joints(self.camera_q(xy))

    # -- 1 + 2: home and search ---------------------------------------------
    def search(self):
        print("[SEARCH] looking for the A4 sheet")
        self.go_camera(C.HOME_CAMERA_XY)
        frame, T0c, paper, frac = self.look("home")
        if frac >= C.PAPER_MIN_AREA_FRACTION:
            return frame, T0c, paper
        for k, r in enumerate(C.SEARCH_CAMERA_RADII):
            angles = C.SEARCH_Q1_DEG if k % 2 == 0 else C.SEARCH_Q1_DEG[::-1]
            for a in angles:
                xy = (r * math.cos(math.radians(a)), r * math.sin(math.radians(a)))
                try:
                    q = self.camera_q(xy)
                except kin.IKError:
                    continue
                self.robot.move_joints(q)
                frame, T0c, paper, frac = self.look(f"search r={r:.0f} q1={a}")
                if frac >= C.PAPER_MIN_AREA_FRACTION:
                    print(f"  paper seen at r={r:.0f} mm, q1={a} deg ({100 * frac:.0f}% of image)")
                    return frame, T0c, paper
        raise vis.VisionError("A4 sheet not found in the search area")

    # -- 3: lock the line -------------------------------------------------------
    def paper_centroid(self, T0c, paper):
        pts = np.argwhere(paper > 0)[::25][:, ::-1]           # (u, v)
        xyz = self.cam.pixel_to_plane(T0c, pts)
        return np.nanmean(xyz[:, :2], axis=0)

    def check_newest_view(self):
        """Look for the line in the NEWEST view ON ITS OWN.

        Views are not stitched together for this: small calibration errors make
        different views place the same line a few mm apart, and stitching would
        smear it. Returns (line, None) if this view shows the whole line and
        the dot, else (None, the partial line or None). The reason is printed."""
        frame, T0c = self.mosaic.frames[-1]
        m = vis.Mosaic(self.cam)
        m.add_view(frame, T0c)
        try:
            line = vis.extract_line(m)
        except vis.VisionError as e:
            self.save_map(None, m)
            if not e.retry:
                raise
            print(f"  not locked yet: {e}")
            return None, None
        self.save_map(line, m)
        self.last_view_map = m
        if line["complete"]:
            self.line_map = m
            return line, None
        print(f"  not locked yet: {line['why']}")
        return None, line

    def stylus_z(self):
        """Height of the stylus tip while tracing (on the paper, or above it with --dry)."""
        return C.PAPER_Z + C.DRY_HOVER_MM if self.dry else C.PAPER_Z - C.PEN_DEPTH

    def stylus_can_reach(self, xy):
        for pd in C.FOLLOW_PITCHES_DEG:
            try:
                kin.ik_tool((xy[0], xy[1], self.stylus_z()), math.radians(pd))
                return True
            except kin.IKError:
                continue
        return False

    def reach_band(self, xy):
        """(nearest, farthest) distance from the base, in the direction of xy,
        at which the stylus can reach the paper (joint limits included)."""
        a = math.atan2(xy[1], xy[0])
        ok = [r for r in range(40, 330, 2) if self.stylus_can_reach((r * math.cos(a), r * math.sin(a)))]
        return (ok[0], ok[-1]) if ok else (None, None)

    def reach_message(self, xy, what):
        r = float(np.hypot(xy[0], xy[1]))
        lo, hi = self.reach_band(xy)
        if lo is None:
            return f"{what} at ({xy[0]:.0f}, {xy[1]:.0f}) mm cannot be reached in that direction at all"
        if lo <= r <= hi:
            return (f"{what} at ({xy[0]:.0f}, {xy[1]:.0f}) mm is {r:.0f} mm from the base, inside the reach "
                    f"({lo}-{hi} mm in that direction) but at a spot the joint limits exclude")
        where = (f"{lo - r:.0f} mm too CLOSE to the robot" if r < lo else f"{r - hi:.0f} mm too FAR from the robot")
        return (f"{what} at ({xy[0]:.0f}, {xy[1]:.0f}) mm is {r:.0f} mm from the base - {where}. "
                f"The stylus reaches the paper only between {lo} and {hi} mm from the base")

    def plan_view(self, P):
        """Camera pose that shows ALL table points P (N, 2) at once.

        Searches the same family of poses as camera_q (optical axis in the arm
        plane, aimed at a table point, viewing angle theta, distance d), for a
        few arm directions and aim points around the points' centre, and keeps
        the pose whose image shows every point farthest from the image border.
        Among poses with enough margin (LOCK_VIEW_MARGIN_PX) a steeper view is
        preferred (less sensitive to calibration errors).
        Returns (q, margin in pixels) - margin < 0 means some points stay outside."""
        if kin.CAMERA_AXIS_OFFSET is None:
            raise kin.IKError("camera view planning needs a lens looking within the arm plane")
        P = np.asarray(P, float)
        P3 = np.column_stack((P, np.full(len(P), C.PAPER_Z)))
        c = P.mean(axis=0)
        r_c, az_c = math.hypot(*c), math.atan2(c[1], c[0])
        W, H = self.cam.width, self.cam.height
        best_score, best_q, best_m = -np.inf, None, -np.inf
        for daz in (0.0, -6.0, 6.0, -12.0, 12.0):
            az = az_c + math.radians(daz)
            u = np.array([math.cos(az), math.sin(az)])
            for r_t in np.arange(max(40.0, r_c - 60.0), r_c + 60.1, 10.0):
                for th_deg in np.arange(-90.0, -29.9, 2.5):
                    th = math.radians(th_deg)
                    for d in np.arange(60.0, 300.1, 5.0):
                        h = -d * math.sin(th)
                        r_cam = r_t - d * math.cos(th)
                        if h > C.OBSERVE_HEIGHT or h < 30.0 or r_cam < 20.0:
                            continue
                        try:
                            q = kin.ik_camera((*(u * r_cam), C.PAPER_Z + h), th - kin.CAMERA_AXIS_OFFSET)
                        except kin.IKError:
                            continue
                        if kin.fk_tool(q)[0][2] < C.PAPER_Z + self.STYLUS_CLEARANCE_MM:
                            continue
                        uv, depth = self.cam.project(kin.camera_pose(q), P3)
                        if not (np.all(np.isfinite(uv)) and np.all(depth > 1.0)):
                            continue
                        m = min(uv[:, 0].min(), W - 1 - uv[:, 0].max(), uv[:, 1].min(), H - 1 - uv[:, 1].max())
                        score = min(m, C.LOCK_VIEW_MARGIN_PX + 15.0) + 0.2 * (-th_deg)
                        if score > best_score:
                            best_score, best_q, best_m = score, q, m
        return best_q, best_m

    def acquire_line(self):
        """Find the paper and LOCK the line from a single camera view.

        If the whole line and the dot are in the first view: locked, nothing
        else is looked at. If the line runs out of the picture, the camera does
        NOT just turn towards the cut end (that loses the other end): every line
        point seen so far, plus a guessed continuation beyond each cut end, is
        given to plan_view(), which finds an arm pose (usually farther away and
        flatter) whose single image contains all of them. This repeats with the
        new information until one view shows the whole line and the dot.
        Only if no single view can hold the line are the views combined."""
        frame, T0c, paper = self.search()
        self.mosaic.add_view(frame, T0c)
        line, part = self.check_newest_view()
        seen_pts = []
        visited = [self.robot.read_q()]
        tight_tried = False
        tries = 0
        while line is None and tries < C.LOCK_MAX_VIEWS:
            tries += 1
            if part is not None and part.get("points_xy") is not None:
                seen_pts.append(part["points_xy"])
                dot = part.get("dot")
                if dot is not None and dot[2] > 0 and not part.get("open_is_dot"):
                    # the whole DISC must be inside, not only its centre
                    a = np.linspace(0.0, 2.0 * math.pi, 16, endpoint=False)
                    seen_pts.append(dot[:2] + (dot[2] + 3.0) * np.column_stack((np.cos(a), np.sin(a))))
                q, margin = None, -np.inf
                for ext in (C.LOCK_EXTEND_MM, 0.5 * C.LOCK_EXTEND_MM, 8.0):
                    guess = [e + dvec * s for e, dvec in part["cut_ends"] for s in np.arange(4.0, ext + 0.1, 4.0)]
                    P = np.vstack(seen_pts + ([np.array(guess)] if guess else []))
                    q, margin = self.plan_view(P)
                    if q is not None and margin >= C.LOCK_VIEW_MARGIN_PX:
                        break
                if q is None or margin < 0:
                    print("  no single camera view can show the whole line")
                    break
                if margin < C.LOCK_VIEW_MARGIN_PX:              # only a tight fit: try it once
                    if tight_tried:
                        print("  no single camera view shows the whole line with enough margin")
                        break
                    tight_tried = True
                if min(np.max(np.abs(np.degrees(q - v))) for v in visited) < 1.5:
                    print("  the best view was already tried")
                    break
                print(f"  -> moving the camera to see the whole line at once "
                      f"(planned: all of it at least {margin:.0f} px inside the image)")
            else:
                target = self.paper_centroid(T0c, paper)
                print(f"  -> no line yet, looking at the middle of the paper ({target[0]:.0f}, {target[1]:.0f}) mm")
                try:
                    q = self.camera_q(target)
                except kin.IKError as e:
                    print(f"  cannot look there: {e}")
                    break
            self.robot.move_joints(q)
            visited.append(q)
            frame, T0c, paper, _ = self.look(f"view {self.n_views + 1}")
            self.mosaic.add_view(frame, T0c)
            line, part = self.check_newest_view()
        if line is None:
            print("  no single view shows the whole line -> combining the views, each one lined up "
                  "on the line seen in the others")
            combined = vis.combine_aligned(self.cam, self.mosaic.frames, C.PAPER_Z)
            line = vis.extract_line(combined)
            self.line_map = self.mosaic = combined
            if line["path"] is None:
                raise vis.VisionError(f"{line['why']} - even after {self.n_views} camera views. Put the "
                                      "sheet so that the whole line is about 10-19 cm in front of the robot.",
                                      retry=False)
            self.save_map(line, self.mosaic)
            if not line["complete"]:
                print(f"  WARNING: {line['why']}; following the visible part")
        print(f"[LOCK] line and start dot locked (camera views used: {self.n_views})")
        if C.ESTIMATE_PAPER_Z:
            self.mosaic = self.line_map                 # estimate the height on the locked view(s)
            self.refine_paper_height()
            try:
                line = vis.extract_line(self.line_map)
            except vis.VisionError as e:
                print(f"  (keeping the line found before the height correction: {e})")
        d = line["dot"]
        print(f"  start dot ({d[0]:.1f}, {d[1]:.1f}) mm, radius {d[2]:.1f} mm; "
              f"line {line['length_mm']:.0f} mm long, {2 * line['half_width_mm']:.1f} mm wide")
        return line

    @staticmethod
    def a4_scale(info):
        """true / measured size from the sheet sides that were fully observed, or None."""
        (w, h), (vw, vh) = info["size"], info["valid"]
        if vw and vh:
            return 0.5 * (C.A4_SIZE_MM[0] / w + C.A4_SIZE_MM[1] / h), f"{w:.0f} x {h:.0f} mm"
        if vw or vh:                      # one side truncated: classify the other one
            m = w if vw else h
            true = C.A4_SIZE_MM[0] if m < 0.5 * sum(C.A4_SIZE_MM) else C.A4_SIZE_MM[1]
            return true / m, f"{m:.0f} mm side (other side not fully seen)"
        return None, None

    def refine_paper_height(self):
        """Depth from the known A4 size (assignment Hint 2).

        The map is a back-projection of the images onto the plane z = PAPER_Z.
        If the sheet really lies at a different height, it appears scaled:
        with the camera h_assumed above the assumed plane and h_true above the
        real one, s_measured / s_true = h_assumed / h_true. Solving for the
        real height gives a corrected PAPER_Z, and the stored images are
        re-projected onto it (no extra robot motion needed).

        The map is stitched from many views, each distorting the sheet only
        around its own camera position, so the single-camera formula above
        under-corrects. It is therefore used only for the first guess; after
        that the measured size is treated as a function of the assumed height
        and its root (size error = 0) is found with the secant method."""
        z0 = C.PAPER_Z
        history = []                                            # (z, measured/true - 1)
        for it in range(6):
            info = self.mosaic.paper_info()
            scale, measured = self.a4_scale(info) if info is not None else (None, None)
            if scale is None:
                print("  paper height not checked: no sheet side was observed completely")
                return
            f = 1.0 / scale - 1.0
            history.append((C.PAPER_Z, f))
            if len(history) == 1:
                cam_z = float(np.mean([T[2, 3] for _, T in self.mosaic.frames]))
                z_new = cam_z - (cam_z - C.PAPER_Z) * scale
            else:
                (za, fa), (zb, fb) = history[-2], history[-1]
                z_new = zb - fb * (zb - za) / (fb - fa) if abs(fb - fa) > 1e-9 else zb
            delta = z_new - C.PAPER_Z
            if abs(delta) < C.PAPER_Z_TOLERANCE_MM:
                print(f"  paper height {'confirmed' if it == 0 else 'converged'}: PAPER_Z = "
                      f"{C.PAPER_Z:.1f} mm (A4 measures {measured})")
                return
            if abs(z_new - z0) > C.PAPER_Z_MAX_CORRECTION_MM:
                print(f"  WARNING: A4 size suggests PAPER_Z = {z_new:.0f} mm, more than "
                      f"{C.PAPER_Z_MAX_CORRECTION_MM:.0f} mm from the configured {z0:.0f} mm; "
                      "keeping the configured value - check the camera calibration and L4")
                C.PAPER_Z = z0
                self.mosaic.rebuild(z0)
                return
            print(f"  A4 measures {measured} -> paper is {abs(delta):.1f} mm "
                  f"{'higher' if delta > 0 else 'lower'} than assumed; PAPER_Z {C.PAPER_Z:.1f} -> {z_new:.1f} mm")
            C.PAPER_Z = z_new                         # used by IK targets and pixel_to_plane
            self.mosaic.rebuild(z_new)
            self.ui.show("map", self.mosaic.render())
        print(f"  paper height: stopped after {len(history)} iterations at PAPER_Z = {C.PAPER_Z:.1f} mm")

    # -- touch the paper once to measure its height ----------------------------
    def can_probe(self, xy, pitch, quiet=False):
        """True if the stylus can be lowered at xy from the hover height to a few
        mm below PAPER_Z without hitting a joint limit on the way."""
        top = min(C.PAPER_Z + C.HOVER_HEIGHT, float(kin.fk_tool(self.robot.q_cmd)[0][2]))             if self.robot.q_cmd is not None else C.PAPER_Z + C.HOVER_HEIGHT
        for z in np.arange(top, C.PAPER_Z - 3.0 - 1e-9, -1.0):
            try:
                kin.ik_tool((xy[0], xy[1], z), pitch)
            except kin.IKError:
                if not quiet:
                    print(f"  WARNING: the stylus cannot be lowered straight down at ({xy[0]:.0f}, {xy[1]:.0f}) "
                          f"mm (joint limits at z = {z:.0f} mm) - paper height NOT measured, "
                          f"using PAPER_Z = {C.PAPER_Z:.1f} mm")
                return False
        return True

    def probe_surface(self, xy, pitch):
        """Lower the stylus slowly onto the paper and return the paper height.

        While the stylus moves freely, the MEASURED tip height (forward kinematics
        of the measured joint angles) follows the commanded height with a nearly
        constant offset (servo lag, gravity sag). When the tip touches the paper
        it stops, the command keeps going down, and the offset grows by more than
        PROBE_TRIGGER_MM: that is the contact. The arm holds immediately, so it
        only presses lightly, and the measured tip height is the paper height.
        This needs no camera, so it also corrects a wrong PAPER_Z or camera model."""
        x, y = float(xy[0]), float(xy[1])
        # start where the tip is now (above xy) and come back there: near the edge of
        # the reach the usual HOVER_HEIGHT above the paper may be impossible
        z_top = min(C.PAPER_Z + C.HOVER_HEIGHT, float(kin.fk_tool(self.robot.q_cmd)[0][2]))
        # lowest height reachable WITHOUT a gap from z_top down (with a long
        # stylus the wrist can run into its limit half way down)
        z_bot = z_top
        while z_bot - 1.0 >= C.PAPER_Z - C.PROBE_RANGE_MM:
            try:
                kin.ik_tool((x, y, z_bot - 1.0), pitch)
            except kin.IKError:
                break
            z_bot -= 1.0
        if z_bot > C.PAPER_Z - 3.0:
            raise kin.IKError(f"cannot lower the stylus at ({x:.0f}, {y:.0f}) mm with the stylus at "
                              f"{math.degrees(pitch):.0f} deg (joint limits)")
        print(f"[PROBE] lowering the stylus onto the paper at ({x:.0f}, {y:.0f}) mm to measure its height")
        dt = 1.0 / C.CONTROL_HZ
        z_cmd, baseline, hits, at_bottom = z_top, [], 0, 0
        t_last = time.perf_counter()
        while True:
            now = time.perf_counter()
            step = (now - t_last) if self.realtime else dt
            t_last = now
            z_cmd = max(z_bot, z_cmd - C.PROBE_SPEED_MM_S * min(max(step, 1e-3), 0.2))
            q = kin.ik_tool((x, y, z_cmd), pitch)
            q_dn = kin.ik_tool((x, y, max(z_bot, z_cmd - 1.0)), pitch)
            self.robot.command(q, np.abs(q_dn - q) * C.PROBE_SPEED_MM_S)   # rad/mm * mm/s
            z_meas = kin.fk_tool(self.robot.read_q())[0][2]
            lag = z_meas - z_cmd
            self.ui.poll()
            if getattr(self.robot.arm, "last_read_fresh", True):  # ignore reused (stale) values
                if len(baseline) < 6:
                    baseline.append(lag)
                else:
                    hits = hits + 1 if lag - float(np.median(baseline)) > C.PROBE_TRIGGER_MM else 0
                    if hits >= 2:
                        self.robot.hold()                      # stop pressing at once
                        print(f"  contact: paper surface at z = {z_meas:.1f} mm")
                        # Lift straight up keeping the angle the stylus REALLY has now:
                        # pressing bends the wrist (long stylus = long lever), so the
                        # planned angle would mean a sudden joint jump while touching.
                        q_now = self.robot.q_cmd
                        tip = kin.fk_tool(q_now)[0]
                        up = (tip[0], tip[1], z_meas + (z_top - C.PAPER_Z))
                        try:
                            self.robot.move_linear(tip, up, kin.tool_pitch_of(q_now))
                        except (kin.IKError, RuntimeError):
                            self.robot.move_joints(kin.ik_tool((x, y, up[2]), pitch))
                        return z_meas
            if z_cmd <= z_bot:
                at_bottom += 1
                if at_bottom > 15:
                    self.robot.hold()
                    raise RuntimeError(f"no contact with the paper down to z = {z_bot:.0f} mm "
                                       f"(PAPER_Z = {C.PAPER_Z:.0f}). Is the paper there? "
                                       "Set PAPER_Z / --paper-z closer to the real height.")
            if self.realtime:
                time.sleep(max(0.0, dt - (time.perf_counter() - now)))

    # -- 6: plan --------------------------------------------------------------
    def plan(self, path_xy):
        path_xy = self.correct(path_xy)                  # camera estimate -> where the stylus goes
        path = np.column_stack((path_xy, np.full(len(path_xy), self.stylus_z())))
        # approach / retreat height above the line ends: as high as the arm allows
        # there (near the edge of the reach a 25 mm lift may be impossible)
        lifts = [np.array([0.0, 0.0, h + C.PEN_DEPTH]) for h in (C.HOVER_HEIGHT, 15.0, 8.0)]
        lift = lifts[0]

        def reachable(p, pitch):
            try:
                kin.ik_tool(p, pitch)
                return True
            except kin.IKError:
                return False

        def vertical_ok(p, lf, pitch):
            """Every height from the paper up to p + lf (1 mm steps): with a long
            stylus the wrist can leave its range half way up and come back."""
            return all(reachable(p + np.array([0.0, 0.0, h]), pitch) for h in np.arange(0.0, lf[2] + 0.01, 1.0))

        def lift_for(pitch, a_pt, b_pt):
            return next((lf for lf in lifts if vertical_ok(a_pt, lf, pitch) and vertical_ok(b_pt, lf, pitch)), None)

        def length(pts):
            return float(np.linalg.norm(np.diff(pts[:, :2], axis=0), axis=1).sum()) if len(pts) > 1 else 0.0

        chosen = None                                           # (pitch_deg, first index, end index)
        for pitch_deg in C.FOLLOW_PITCHES_DEG:                 # whole line, steepest stylus first
            pitch = math.radians(pitch_deg)
            if all(reachable(p, pitch) for p in path):
                lf = lift_for(pitch, path[0], path[-1])
                if lf is not None:
                    chosen, lift = (pitch_deg, 0, len(path)), lf
                    break
        if chosen is None and self.partial:
            # --partial (test mode): the LONGEST piece of the line the stylus can reach
            best_len = 0.0
            for pitch_deg in C.FOLLOW_PITCHES_DEG:
                pitch = math.radians(pitch_deg)
                ok = np.array([reachable(p, pitch) for p in path])
                i = 0
                while i < len(path):
                    if not ok[i]:
                        i += 1
                        continue
                    j = i
                    while j < len(path) and ok[j]:
                        j += 1
                    a, b = i, j                                  # shrink until the lift poses work too
                    while a < b and not vertical_ok(path[a], lifts[-1], pitch):
                        a += 1
                    while b > a and not vertical_ok(path[b - 1], lifts[-1], pitch):
                        b -= 1
                    if length(path[a:b]) > best_len + 1e-6:
                        best_len, chosen = length(path[a:b]), (pitch_deg, a, b)
                    i = j
            if chosen is not None and best_len < 20.0:
                chosen = None
            if chosen is not None:
                lift = lift_for(math.radians(chosen[0]), path[chosen[1]], path[chosen[2] - 1])
                start = length(path[:chosen[1] + 1])
                print(f"  PARTIAL: following {best_len:.0f} mm of the {length(path):.0f} mm line, starting "
                      f"{start:.0f} mm after the dot (the rest is out of reach of the stylus)")
        if chosen is None:
            bad = next((p for p in path if not self.stylus_can_reach(p[:2])), None)
            if bad is None:
                raise kin.IKError(
                    "every point of the line can be reached, but not all with ONE stylus angle (tried "
                    f"{', '.join(str(p) for p in C.FOLLOW_PITCHES_DEG)} deg) plus room to lift the stylus at "
                    "the ends. Move the sheet about 1 cm closer to the robot - or test with --partial")
            raise kin.IKError(self.reach_message(bad[:2], "part of the line") +
                              ". Move the sheet (or redraw the line) so that the whole line lies in that "
                              "ring - or test the motion with --partial")
        pitch_deg, a, b = chosen
        pitch = math.radians(pitch_deg)
        path = path[a:b]
        samples, T = time_parametrize(path, C.FOLLOW_SPEED_MM_S, C.FOLLOW_ACCEL_MM_S2)
        qs = [kin.ik_tool(p, pitch) for p in samples]
        jump = max(np.degrees(np.abs(np.diff(qs, axis=0))).max(initial=0), 0)
        print(f"[PLAN] stylus pitch {pitch_deg} deg, {len(qs)} set-points, {T:.1f} s, "
              f"largest joint step {jump:.2f} deg")
        return dict(path=path, pitch=pitch, qs=qs, samples=samples, lift=lift, T=T)

    # -- 7: follow ------------------------------------------------------------
    def follow(self, plan, line):
        path, pitch, lift = plan["path"], plan["pitch"], plan["lift"]
        print("[FOLLOW] approaching the start dot")
        self.robot.move_joints(kin.ik_tool(path[0] + lift, pitch))
        if self.dry:
            print(f"  --dry: no touching; the line is traced with the stylus tip {C.DRY_HOVER_MM:.0f} mm "
                  "above the paper")
        elif self.paper_probed:
            print(f"  paper height measured while teaching: z = {C.PAPER_Z:.1f} mm")
        elif C.PROBE_PAPER and self.can_probe(path[0][:2], pitch):
            z = self.probe_surface(path[0][:2], pitch)
            if abs(z - C.PAPER_Z) >= C.PAPER_Z_TOLERANCE_MM:
                print(f"  paper is {abs(z - C.PAPER_Z):.1f} mm {'higher' if z > C.PAPER_Z else 'lower'} "
                      f"than PAPER_Z = {C.PAPER_Z:.1f} -> re-projecting the images onto z = {z:.1f} mm")
                C.PAPER_Z = z
                self.line_map.rebuild(z)             # the locked view(s), exact paper height
                try:
                    new_line = vis.extract_line(self.line_map)
                    if new_line["path"] is not None:
                        line = new_line
                except vis.VisionError as e:
                    print(f"  (keeping the previous line: {e})")
                plan = self.plan(line["path"])
                path, pitch, lift = plan["path"], plan["pitch"], plan["lift"]
                self.robot.move_joints(kin.ik_tool(path[0] + lift, pitch))
            else:
                print(f"  paper height confirmed: z = {z:.1f} mm")
                self.robot.move_joints(kin.ik_tool(path[0] + lift, pitch))   # back to the planned stylus angle
        base = self.line_map.render(line["path"], line["dot"])
        self.ui.show("map", base)
        self.robot.move_linear(path[0] + lift, path[0], pitch, speed=C.LINEAR_SPEED_MM_S / 2,
                               on_paper=True)
        print("[FOLLOW] tracing the line")

        def progress(i):
            if i % 5 == 0:
                self.ui.show("map", self.line_map.draw_progress(base, plan["samples"][i][:2]))

        self.robot.stream(plan["qs"], on_step=progress)
        self.robot.wait_settled(check_moving=False)
        tip = kin.fk_tool(self.robot.read_q())[0]
        print(f"  end reached, tip at ({tip[0]:.1f}, {tip[1]:.1f}, {tip[2]:.1f}) mm "
              f"(target {np.round(path[-1], 1)})")
        self.robot.move_linear(path[-1], path[-1] + lift, pitch)
        self.go_camera(C.HOME_CAMERA_XY)
        return line, plan

    def acquire_with_retry(self, auto):
        """acquire_line(), but if the line cannot be locked the arm goes back to
        the home view and WAITS: the user can move the sheet into the live view
        and try again, instead of the program quitting."""
        while True:
            try:
                return self.acquire_line()
            except (vis.VisionError, kin.IKError) as e:
                if auto:
                    raise
                print()
                print(f"  NOT LOCKED: {e}")
                self.go_camera(C.HOME_CAMERA_XY)
                if not self.ui.wait_retry("  Move / straighten the sheet so the line and the big dot are in the "
                                          "'camera (live)' window (dot on the paper, not cut off)."):
                    raise
                print()
                print("[RETRY]")
                self.mosaic = vis.Mosaic(self.cam)
                self.line_map = self.mosaic
                self.last_view_map = None

    # -- teach: measure the camera error with the stylus -------------------------
    TEACH_HOVER_MM = 4.0          # tip height above the (probed) paper while teaching
    TEACH_HOVER_DRY_MM = 10.0     # ... with --dry (paper height not measured)
    JOG_KEYS = {ord("w"): (1.0, 0.0), ord("s"): (-1.0, 0.0), ord("a"): (0.0, 1.0), ord("d"): (0.0, -1.0)}

    def correct(self, xy):
        """Camera estimate (mm, frame 0) -> where the stylus has to go, using the
        correction measured by --teach (identity if there is none)."""
        xy = np.asarray(xy, float)
        if self.correction is None:
            return xy
        M = np.asarray(self.correction["matrix"], float)
        return (xy.reshape(-1, 2) @ M[:, :2].T + M[:, 2]).reshape(xy.shape)

    def jog(self, start_xy, z, pitch, what):
        """The user moves the stylus tip (z fixed) exactly over a mark; returns its xy."""
        xy = np.array(start_xy, float)
        self.robot.move_joints(kin.ik_tool((xy[0], xy[1], z + 15.0), pitch))
        self.robot.move_linear((xy[0], xy[1], z + 15.0), (xy[0], xy[1], z), pitch)
        print(f"\n  Put the stylus tip exactly over the {what}.")
        if self.ui.enabled:
            print("  Keys in an image window: w / s = 1 mm away from / towards the robot, "
                  "a / d = 1 mm to the robot's left / right\n"
                  "  (W A S D = 5 mm). 'g' = this is the spot, 'q' = stop.")
        while True:
            if self.ui.enabled:
                key = self.ui.poll()
                if key in (ord("g"), 13, 32):
                    break
                lower = key + 32 if 65 <= key <= 90 else key
                if lower not in self.JOG_KEYS:
                    time.sleep(0.03)
                    continue
                move = np.array(self.JOG_KEYS[lower]) * (5.0 if lower != key else 1.0)
            else:
                cmd = input("  move ('x+2' = 2 mm away from the robot, 'y-1' = 1 mm to its right, "
                            "'x-1 y+3' ...) or 'ok': ").strip().lower()
                if cmd in ("ok", "g", ""):
                    break
                move = np.zeros(2)
                try:
                    for part in cmd.replace(",", " ").split():
                        move["xy".index(part[0])] += float(part[1:])
                except (ValueError, IndexError):
                    print("  ? write e.g. x+2 or y-1.5 or ok")
                    continue
            new = xy + move
            try:
                # quick nudge: 0.2 s straight line, no waiting for the servos to settle
                # (holding a key down keeps the tip moving)
                n = max(5, int(math.ceil(np.linalg.norm(move) / 1.0)))     # samples <= 1 mm apart
                qs = [kin.ik_tool((*(xy + s * move), z), pitch) for s in np.linspace(1.0 / n, 1.0, n)]
                steps = np.abs(np.diff(np.vstack([self.robot.q_cmd] + qs), axis=0))
                if np.degrees(steps.max()) > C.MAX_JOINT_STEP_DEG:      # check BEFORE moving at all
                    raise RuntimeError("the arm would have to jump (edge of the reach)")
                self.robot.stream(qs)
                xy = new
            except (kin.IKError, RuntimeError) as e:
                # out of reach with this stylus angle: tilt the stylus (in the air) to the
                # first angle of FOLLOW_PITCHES_DEG that reaches the new spot, then go on
                new_pitch = self._pitch_reaching(new, z, pitch)
                if new_pitch is None:
                    print(f"  cannot go there with any stylus angle ({e})")
                else:
                    self._change_pitch(xy, new, z, pitch, new_pitch)
                    pitch, xy = new_pitch, new
                    print(f"  (stylus tilted to {math.degrees(pitch):.0f} deg to reach this spot)")
            d = xy - start_xy
            print(f"  tip at ({xy[0]:.1f}, {xy[1]:.1f}) mm = camera estimate {d[0]:+.1f} / {d[1]:+.1f} mm")
        for up in (15.0, 10.0, 5.0):                     # lift off (as high as the wrist allows here)
            try:
                self.robot.move_linear((xy[0], xy[1], z), (xy[0], xy[1], z + up), pitch)
                break
            except kin.IKError:
                continue
        return xy

    def _column_ok(self, xy, z_lo, z_hi, pitch):
        """The stylus can stand at xy at every height z_lo..z_hi (1 mm steps)."""
        try:
            for zz in np.arange(z_lo, z_hi + 0.01, 1.0):
                kin.ik_tool((xy[0], xy[1], zz), pitch)
            return True
        except kin.IKError:
            return False

    def _pitch_reaching(self, xy, z, current):
        """First stylus angle (FOLLOW_PITCHES_DEG) other than `current` that reaches
        xy at the jog height and 5 mm above it, or None."""
        for pd in C.FOLLOW_PITCHES_DEG:
            p = math.radians(pd)
            if abs(p - current) > 1e-6 and self._column_ok(xy, z, z + 5.0, p):
                return p
        return None

    def _change_pitch(self, xy_from, xy_to, z, old, new):
        """Lift (as far as possible at the old angle), turn the stylus to the new
        angle above the new spot in the air, come down again."""
        up = next((dz for dz in (15.0, 10.0, 5.0, 0.0) if self._column_ok(xy_from, z, z + dz, old)
                   and self._column_ok(xy_to, z, z + dz, new)), 0.0)
        if up > 0:
            try:
                self.robot.move_linear((xy_from[0], xy_from[1], z), (xy_from[0], xy_from[1], z + up), old)
            except (kin.IKError, RuntimeError):
                pass                                    # turn without lifting first
        self.robot.move_joints(kin.ik_tool((xy_to[0], xy_to[1], z + up), new))
        if up > 0:
            self.robot.move_linear((xy_to[0], xy_to[1], z + up), (xy_to[0], xy_to[1], z), new)

    @staticmethod
    def correction_from_pairs(pairs):
        """Camera estimate -> stylus position, from the taught point pairs.

        3+ points (dot, top of the curve, end): AFFINE map (shift, rotation,
        scale and shear) - also corrects a camera model that squeezes far points
        more than near ones. 2 points: similarity (shift + rotation + scale).
        Implausible fits fall back to the simpler model, finally a plain shift."""
        V = np.array([p[0] for p in pairs], float)
        Rr = np.array([p[1] for p in pairs], float)
        t = (Rr - V).mean(axis=0)
        M, kind = np.array([[1.0, 0.0, t[0]], [0.0, 1.0, t[1]]]), "shift"
        if len(pairs) >= 3:
            e1, e2 = V[1] - V[0], V[2] - V[0]
            area = abs(e1[0] * e2[1] - e1[1] * e2[0]) / 2.0
            if area > 300.0:                                       # the points span a real triangle
                X = np.linalg.lstsq(np.column_stack((V, np.ones(len(V)))), Rr, rcond=None)[0].T   # 2 x 3
                U, sv, Wt = np.linalg.svd(X[:, :2])
                rot = math.degrees(math.atan2((U @ Wt)[1, 0], (U @ Wt)[0, 0]))
                if 0.7 < sv.min() and sv.max() < 1.45 and abs(rot) < 15.0:
                    return X, f"affine (scale {sv.min():.3f}..{sv.max():.3f}, rotation {rot:+.1f} deg)"
        if len(pairs) >= 2:
            d = np.linalg.norm(V[:, None] - V[None], axis=2)
            i0, i1 = np.unravel_index(np.argmax(d), d.shape)        # the two points farthest apart
            if d[i0, i1] > 30.0:
                v = V[[i0, i1], 0] + 1j * V[[i0, i1], 1]
                r = Rr[[i0, i1], 0] + 1j * Rr[[i0, i1], 1]
                a = (r[1] - r[0]) / (v[1] - v[0])
                b = r[0] - a * v[0]
                if 0.75 < abs(a) < 1.35 and abs(math.degrees(np.angle(a))) < 12.0:
                    M = np.array([[a.real, -a.imag, b.real], [a.imag, a.real, b.imag]])
                    kind = f"shift + rotation {math.degrees(np.angle(a)):+.1f} deg + scale {abs(a):.3f}"
        return M, kind

    def teach(self, line):
        """Measure where the stylus really has to go for the start dot and the
        end of the line (the user moves the tip there), and store the
        correction camera -> stylus in CORRECTION_FILE for this and later runs."""
        print("\n[TEACH] measuring the camera error with the stylus")
        dot, end = line["path"][0], line["path"][-1]
        # 3rd point: the point of the line farthest from the straight dot-end chord
        # (the top of the curve). With 3 points the correction also undoes a camera
        # model that squeezes far points more than near ones.
        chord = end - dot
        off = np.abs(chord[0] * (line["path"][:, 1] - dot[1]) - chord[1] * (line["path"][:, 0] - dot[0])) / max(np.linalg.norm(chord), 1e-6)
        mid = line["path"][int(np.argmax(off))]
        pitch = None
        heights = np.arange(C.PAPER_Z + self.TEACH_HOVER_MM, C.PAPER_Z + self.TEACH_HOVER_DRY_MM + 15.0 + 0.01, 1.0)
        for pd in C.FOLLOW_PITCHES_DEG:                      # every height the jog will use, all points
            try:
                for p in (dot, mid, end):
                    for z in heights:
                        kin.ik_tool((*self.correct(p), z), math.radians(pd))
                pitch = math.radians(pd)
                break
            except kin.IKError:
                continue
        if pitch is None:
            raise kin.IKError("the stylus cannot reach the start dot and the end of the line to teach them")
        if not self.dry and C.PROBE_PAPER:
            probe_pitch = next((math.radians(pd) for pd in C.FOLLOW_PITCHES_DEG
                                if self.can_probe(self.correct(dot), math.radians(pd), quiet=True)), None)
        if not self.dry and C.PROBE_PAPER and probe_pitch is not None:
            p = self.correct(dot)
            self.robot.move_joints(kin.ik_tool((p[0], p[1], C.PAPER_Z + C.HOVER_HEIGHT), probe_pitch))
            z = self.probe_surface(p, probe_pitch)
            self.paper_probed = True
            if abs(z - C.PAPER_Z) >= C.PAPER_Z_TOLERANCE_MM:
                C.PAPER_Z = z
                self.line_map.rebuild(z)
                try:
                    new_line = vis.extract_line(self.line_map)
                    if new_line["path"] is not None:
                        line = new_line
                        dot, end = line["path"][0], line["path"][-1]
                        chord = end - dot
                        off = np.abs(chord[0] * (line["path"][:, 1] - dot[1]) - chord[1] * (line["path"][:, 0] - dot[0])) / max(np.linalg.norm(chord), 1e-6)
                        mid = line["path"][int(np.argmax(off))]
                except vis.VisionError as e:
                    print(f"  (keeping the previous line: {e})")
            hover = C.PAPER_Z + self.TEACH_HOVER_MM
        else:
            hover = C.PAPER_Z + self.TEACH_HOVER_DRY_MM
        names = ["start dot"]
        pairs = [(dot, self.jog(self.correct(dot), hover, pitch, "CENTRE of the start dot"))]
        if off.max() > 10.0:                              # a curved line: also its top
            names.append("top of curve")
            pairs.append((mid, self.jog(self.correct(mid), hover, pitch,
                                        "TOP of the curve (move straight onto the centre of the line, "
                                        "to its nearest point)")))
        names.append("line end")
        pairs.append((end, self.jog(self.correct(end), hover, pitch,
                                    "END of the line (centre of the line, 2-3 mm before its tip)")))
        M, kind = self.correction_from_pairs(pairs)
        self.correction = dict(matrix=M.tolist(), kind=kind,
                               pairs=[[*np.round(v, 2), *np.round(r, 2)] for v, r in pairs],
                               taught=time.strftime("%Y-%m-%d %H:%M"))
        for (v, r), name in zip(pairs, names):
            print(f"  {name}: camera said ({v[0]:.1f}, {v[1]:.1f}), stylus needed ({r[0]:.1f}, {r[1]:.1f}) "
                  f"-> error {r[0] - v[0]:+.1f} / {r[1] - v[1]:+.1f} mm")
        print(f"  correction: {kind}")
        if self.correction_file:
            with open(self.correction_file, "w", encoding="utf-8") as f:
                json.dump(self.correction, f, indent=1)
            print(f"  saved in {self.correction_file} (used automatically from now on; "
                  "delete the file or --teach again to redo)")
        return line

    def run(self, auto=False):
        line = self.acquire_with_retry(auto)
        if self.teach_mode:
            if auto:
                raise RuntimeError("--teach needs you at the keyboard; do not combine it with --auto")
            line = self.teach(line)
        elif self.correction is not None:
            print(f"  using the taught camera correction ({self.correction.get('kind', '?')}, "
                  f"{self.correction.get('taught', '?')})")
        plan = self.plan(line["path"])
        self.ui.show("map", self.line_map.render(line["path"], line["dot"]))
        if not auto and not self.ui.confirm("Planned path is shown in the 'map' window "
                                            "(green = path, blue = start dot)."):
            print("Aborted before touching the paper.")
            self.go_camera(C.HOME_CAMERA_XY)
            return None
        line, plan = self.follow(plan, line)
        print("DONE.")
        return line, plan


# ---------------------------------------------------------------------------
# Calibration / test modes
# ---------------------------------------------------------------------------
def describe_direction(v):
    """Words for a direction vector in frame 0, e.g. 'forward 40 deg down'."""
    horiz = math.hypot(v[0], v[1])
    elev = math.degrees(math.atan2(v[2], horiz))
    if horiz < 0.17:                                     # within ~10 deg of vertical
        return "straight DOWN" if v[2] < 0 else "straight UP"
    az = math.degrees(math.atan2(v[1], v[0]))
    side = ("FORWARD (away from base)" if abs(az) < 45 else
            "BACKWARD (towards/behind base)" if abs(az) > 135 else
            "to the robot's LEFT" if az > 0 else "to the robot's RIGHT")
    return f"{side}, {abs(elev):.0f} deg {'down' if elev < 0 else 'up'}"


def check_joints(arm, robot):
    print("Torque will be DISABLED. Hold the arm, then move it by hand.\n"
          "Compare what is printed with the real robot:\n"
          "  1. arm straight up (Figure 1) must read q = 0 0 0 0 and tip = (0, 0, 286);\n"
          "  2. tilt each joint FORWARD/DOWN a bit: q2, q3, q4 must become MORE NEGATIVE,\n"
          "     q1 turning to the robot's left must become POSITIVE.\n"
          "     A joint with the wrong sign -> flip it in JOINT_DIRECTION;\n"
          "  3. 'stylus' and 'camera' tell where the program THINKS they point. If the\n"
          "     camera line is wrong while the stylus line is right, the camera axes in\n"
          "     config.py (CAMERA_IMAGE_RIGHT/DOWN_IN_F5, optical axis = stylus) are wrong.\n"
          "Ctrl+C to stop.")
    input("Press Enter when you hold the arm...")
    arm.set_torque(False)
    try:
        while True:
            q = robot.read_q()
            tip, direction = kin.fk_tool(q)
            Tc = kin.camera_pose(q)
            print(f"q = {np.round(np.degrees(q), 1)} deg   stylus tip = {np.round(tip, 0)} mm\n"
                  f"   stylus points {describe_direction(direction)}\n"
                  f"   camera looks  {describe_direction(Tc[:3, 2])}\n")
            time.sleep(1.0)
    except KeyboardInterrupt:
        print()


def check_camera(task):
    print("Camera check: the yellow dots are a 20 mm grid ON THE TABLE (frame 0),\n"
          "the arrow points towards the robot base.\n"
          "  p          = find the 4 CORNERS of the sheet and print where they are (mm)\n"
          "  click, or move the red cross with i/j/k/l (I/J/K/L = finer) and press ENTER\n"
          "             = print where that point is (mm)\n"
          "  t = put the stylus 3 mm above the point, b = back to the home view, q = quit\n"
          "Measure the same points with a ruler from the CENTRE of the base:\n"
          "  x = straight forward, y = to the robot's LEFT.")
    cam, robot = task.cam, task.robot
    task.ui.live = None                 # this mode shows its own live window
    clicked = {"uv": None}
    T0c = None

    def report(uv, name="point"):
        P = cam.pixel_to_plane(T0c, [uv])[0]
        if np.all(np.isfinite(P)):
            print(f"  {name} at pixel ({uv[0]:.0f}, {uv[1]:.0f}) -> table x = {P[0]:.1f} mm, y = {P[1]:.1f} mm "
                  f"({math.hypot(P[0], P[1]):.0f} mm from the base)")
        else:
            print(f"  {name} at pixel ({uv[0]:.0f}, {uv[1]:.0f}) does not look at the table")

    def on_mouse(event, x, y, *_):
        if event == cv2.EVENT_LBUTTONDOWN:
            clicked["uv"] = (x, y)
            report(clicked["uv"])

    def sheet_corners(img):
        """The corners of the sheet in the image (paper mask -> polygon)."""
        m, frac = vis.paper_mask(img)
        if frac < C.PAPER_MIN_AREA_FRACTION:
            return []
        cnt = max(cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0], key=cv2.contourArea)
        for eps in (0.01, 0.02, 0.03, 0.05, 0.08):
            poly = cv2.approxPolyDP(cnt, eps * cv2.arcLength(cnt, True), True)[:, 0]
            if len(poly) <= 6:
                break
        return [((int(u), int(v)), u <= 2 or v <= 2 or u >= cam.width - 3 or v >= cam.height - 3)
                for u, v in poly]

    g = np.arange(-300, 301, 20.0)
    GX, GY = np.meshgrid(g, g)
    grid = np.column_stack((GX.ravel(), GY.ravel(), np.full(GX.size, C.PAPER_Z)))

    def overlay_for_current_pose():
        """Grid + arrow drawn once per arm pose (not every frame -> fast live view)."""
        T = kin.camera_pose(robot.read_q())
        layer = np.zeros((cam.height, cam.width, 3), np.uint8)
        uv, z = cam.project(T, grid)
        for (u, v), zz, P in zip(uv, z, grid):
            if zz > 0 and 0 <= u < cam.width and 0 <= v < cam.height:
                cv2.circle(layer, (int(u), int(v)), 2, (0, 255, 255), -1)
                if P[0] % 40 == 0 and P[1] % 40 == 0:
                    cv2.putText(layer, f"{P[0]:.0f},{P[1]:.0f}", (int(u) + 3, int(v) - 3),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 255), 1)
        c = np.array([cam.width / 2, cam.height / 2])
        b, _ = cam.project(T, [[0.0, 0.0, C.PAPER_Z]])
        d = b[0] - c
        if np.all(np.isfinite(d)) and np.linalg.norm(d) > 1:
            tip = c + 80 * d / np.linalg.norm(d)
            cv2.arrowedLine(layer, tuple(c.astype(int)), tuple(tip.astype(int)), (255, 0, 255), 2)
        return T, layer, layer.any(axis=2)

    task.go_camera(C.HOME_CAMERA_XY)
    T0c, layer, drawn = overlay_for_current_pose()
    cv2.namedWindow("camera", cv2.WINDOW_AUTOSIZE)
    cv2.imshow("camera", task.camera.read())
    cv2.waitKey(1)
    cv2.setMouseCallback("camera", on_mouse)            # after the window really exists
    clicked["uv"] = (cam.width // 2, cam.height // 2)
    marks = []                                          # sheet corners found with 'p'
    touch = None                                        # stylus over a clicked point (t)
    pairs = []                                          # stored (pixel, real position) pairs (g)
    moves = {ord("i"): (0, -1), ord("k"): (0, 1), ord("j"): (-1, 0), ord("l"): (1, 0)}
    while True:
        img = task.camera.read().copy()
        img[drawn] = layer[drawn]
        for (uv, at_border), name in zip(marks, "ABCDEFGH"):
            cv2.circle(img, uv, 7, (128, 128, 128) if at_border else (255, 0, 0), 2)
            cv2.putText(img, name, (uv[0] + 6, uv[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        P = cam.pixel_to_plane(T0c, [clicked["uv"]])[0]
        cv2.drawMarker(img, tuple(int(c) for c in clicked["uv"]), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
        cv2.putText(img, f"({P[0]:.1f}, {P[1]:.1f}) mm", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.imshow("camera", img)
        key = cv2.waitKey(15) & 0xFF
        if key in (27, ord("q")):
            return
        low = key + 32 if 65 <= key <= 90 else key
        if low in moves:
            step = 10 if low == key else 1
            clicked["uv"] = (int(np.clip(clicked["uv"][0] + moves[low][0] * step, 0, cam.width - 1)),
                             int(np.clip(clicked["uv"][1] + moves[low][1] * step, 0, cam.height - 1)))
        if key in (13, 10):
            report(clicked["uv"])
        if key == ord("p"):
            marks = sheet_corners(task.camera.read())
            if not marks:
                print("  no sheet found in the image")
            else:
                print("  sheet corners (grey circle = where the sheet leaves the image, not a real corner):")
            for (uv, at_border), name in zip(marks, "ABCDEFGH"):
                report(uv, f"corner {name}{' (image border)' if at_border else ''}")
        if key == ord("t") and clicked["uv"] is not None:
            P = cam.pixel_to_plane(T0c, [clicked["uv"]])[0]
            if not np.all(np.isfinite(P)):
                print("that pixel does not look at the table - click on the paper")
                continue
            q = q_hover = None
            for pd in C.FOLLOW_PITCHES_DEG:          # one stylus angle for BOTH poses
                try:
                    q_hover = kin.ik_tool((P[0], P[1], C.PAPER_Z + 25.0), math.radians(pd))
                    q = kin.ik_tool((P[0], P[1], C.PAPER_Z + 3.0), math.radians(pd))
                    break
                except kin.IKError:
                    q = q_hover = None
            if q is None:
                print(f"point ({P[0]:.0f}, {P[1]:.0f}) mm is not reachable with the stylus "
                      "(joint limits) - click a point about 100-190 mm in front of the base")
                continue
            touch = dict(uv=clicked["uv"], T0c=T0c.copy(), est=P[:2].copy(), xy=P[:2].copy(),
                         pitch=math.radians(pd), z=C.PAPER_Z + 3.0)
            robot.move_joints(q_hover)
            robot.move_joints(q)
            T0c, layer, drawn = overlay_for_current_pose()
            print(f"stylus 3 mm above ({P[0]:.1f}, {P[1]:.1f}) mm (stylus angle {pd} deg).")
            print("  Not on your mark? Move it with w / s (away from / towards the robot), a / d (robot's "
                  "left / right), 1 mm per key (W A S D = 5 mm),")
            print("  then press g to store this point (or b to go back without storing).")
        low2 = key + 32 if 65 <= key <= 90 else key
        if touch is not None and low2 in task.JOG_KEYS:
            move = np.array(task.JOG_KEYS[low2]) * (5.0 if low2 != key else 1.0)
            try:
                qs = [kin.ik_tool((*(touch["xy"] + s_ * move), touch["z"]), touch["pitch"])
                      for s_ in np.linspace(0.2, 1.0, 5)]
                robot.stream(qs)
                touch["xy"] = touch["xy"] + move
                d = touch["xy"] - touch["est"]
                print(f"  tip at ({touch['xy'][0]:.1f}, {touch['xy'][1]:.1f}) mm = camera estimate "
                      f"{d[0]:+.1f} / {d[1]:+.1f} mm")
            except (kin.IKError, RuntimeError) as e:
                print(f"  cannot go there: {e}")
        if key == ord("g") and touch is not None:
            pairs.append(dict(uv=touch["uv"], T0c=touch["T0c"], true=touch["xy"].copy()))
            d = touch["xy"] - touch["est"]
            print(f"  point {len(pairs)} stored: camera said ({touch['est'][0]:.1f}, {touch['est'][1]:.1f}), "
                  f"real ({touch['xy'][0]:.1f}, {touch['xy'][1]:.1f}) -> error {d[0]:+.1f} / {d[1]:+.1f} mm. "
                  f"{'Take 3-4 points spread over the sheet, then press f.' if len(pairs) < 3 else 'Press f to fit.'}")
            touch = None
            key = ord("b")
        if key == ord("b"):
            touch = None
            task.go_camera(C.HOME_CAMERA_XY)
            T0c, layer, drawn = overlay_for_current_pose()
        if key == ord("f"):
            if len(pairs) < 2:
                print("  store at least 2 points first (3-4 spread over the sheet is better)")
                continue
            fix = fit_camera_fix(cam, pairs)
            if fix is not None:
                T0c, layer, drawn = overlay_for_current_pose()


def fit_camera_fix(cam, pairs):
    """Small rotation of the camera on its mount that makes the clicked pixels
    land where the stylus really had to go (least squares, Gauss-Newton). The
    result is applied at once and written to CAMERA_FIX_DEG in config.py."""
    def err(r):
        R = kin.rotvec_matrix(np.radians(r))
        out = []
        for p in pairs:
            T = p["T0c"].copy()
            T[:3, :3] = T[:3, :3] @ R
            P = cam.pixel_to_plane(T, [p["uv"]])[0][:2]
            out.append(np.nan_to_num(P - p["true"], nan=1e3))
        return np.concatenate(out)
    # only tilt about the image right / down axes: a turn about the optical axis
    # (image roll) is hard to tell from errors of the stylus itself and gave
    # wrong results (sheet seen turned by 15 deg)
    r = np.zeros(3)
    e0 = err(r)
    for _ in range(30):
        e = err(r)
        J = np.column_stack([(err(r + np.eye(3)[i] * 0.05) - e) / 0.05 for i in range(2)])
        dr = np.linalg.lstsq(J.T @ J + 1e-3 * np.eye(2), -J.T @ e, rcond=None)[0]
        r = r + np.append(dr, 0.0)
        if np.abs(dr).max() < 1e-4:
            break
    e1 = err(r)
    rms = lambda v: float(np.sqrt(np.mean(v.reshape(-1, 2) ** 2) * 2))     # noqa: E731
    print(f"  camera tilt correction ({r[0]:+.2f}, {r[1]:+.2f}) deg: error {rms(e0):.1f} mm -> "
          f"{rms(e1):.1f} mm over {len(pairs)} points")
    if np.abs(r).max() > 25.0 or rms(e1) > 0.6 * rms(e0) + 2.0:
        print("  this does not fit well (wrong point stored?) - NOT applied. Press b, store the points again.")
        return None
    R_new = kin.T5C[:3, :3] @ kin.rotvec_matrix(np.radians(r))
    R_nom = kin._camera_rotation_in_f5()
    fix_vec, _ = cv2.Rodrigues(R_nom.T @ R_new)
    fix_deg = np.degrees(fix_vec.ravel())
    kin.T5C[:3, :3] = R_new                                    # use it right away
    ax = kin.T5C[:3, 2]
    kin.CAMERA_AXIS_OFFSET = None if abs(ax[2]) > 0.5 else math.atan2(ax[1], ax[0])
    C.CAMERA_FIX_DEG = tuple(float(v) for v in fix_deg)
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.py")
    with open(path, encoding="utf-8") as f:
        text = f.read()
    import re
    text, n = re.subn(r"^CAMERA_FIX_DEG = \([^)]*\)", "CAMERA_FIX_DEG = (%.2f, %.2f, %.2f)" % tuple(fix_deg),
                      text, flags=re.M)
    if n:
        with open(path, "w", encoding="utf-8") as f:
            f.write(text)
        print(f"  saved CAMERA_FIX_DEG = ({fix_deg[0]:.2f}, {fix_deg[1]:.2f}, {fix_deg[2]:.2f}) in config.py "
              "(applied now). Click + t again to check; delete vision_correction.json and teach again.")
    pairs.clear()
    return fix_deg


def camera_only(camera, ui):
    print("Live detection only (no robot). q to quit.")
    while True:
        frame = camera.read()
        paper, frac = vis.paper_mask(frame)
        ink = vis.dark_mask(frame, paper)
        ui.show("camera", annotate(frame, f"paper {100 * frac:.0f}% of image", paper, ink))


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=C.PORT)
    ap.add_argument("--camera", type=int, default=C.CAMERA_INDEX)
    ap.add_argument("--sim", action="store_true", help="simulated robot + camera")
    ap.add_argument("--fast", action="store_true", help="simulator without real-time waits")
    ap.add_argument("--camera-only", action="store_true")
    ap.add_argument("--check-joints", action="store_true")
    ap.add_argument("--check-camera", action="store_true")
    ap.add_argument("--auto", action="store_true", help="no confirmation before following")
    ap.add_argument("--release", action="store_true", help="disable torque at the end")
    ap.add_argument("--no-gui", action="store_true")
    ap.add_argument("--paper-z", type=float, default=None,
                    help=f"height of the paper in frame 0 [mm] (default {C.PAPER_Z})")
    ap.add_argument("--dry", action="store_true",
                    help="no stylus mounted: trace the line with the (imagined) stylus tip "
                         f"{C.DRY_HOVER_MM:.0f} mm above the paper, never touch it")
    ap.add_argument("--partial", action="store_true",
                    help="TEST MODE: if the line is longer than the stylus can reach, follow the "
                         "reachable part from the dot instead of stopping")
    ap.add_argument("--teach", action="store_true",
                    help="after locking the line, move the stylus tip onto the start dot and the line end "
                         "with the keyboard; the camera error measured this way is saved and corrected "
                         "in every later run")
    args = ap.parse_args()
    if args.paper_z is not None:
        C.PAPER_Z = args.paper_z

    ui = UI(not args.no_gui)
    if args.camera_only:
        camera = Camera(args.camera)
        try:
            camera_only(camera, ui)
        except UserAbort:
            pass
        finally:
            camera.release()
            ui.close()
        return

    realtime = not (args.sim and args.fast)
    if args.sim:
        import sim
        arm = sim.SimArm()
    else:
        arm = DxlArm(args.port)
    robot = Robot(arm, poll=ui.poll, realtime=realtime)
    camera = None
    try:
        print(f"Connecting to the servos {C.DXL_IDS} on {'SIMULATOR' if args.sim else args.port} ...")
        arm.open()
        robot.apply_servo_angle_limits()
        q_now = robot.read_q()
        print(f"  joint angles now: {np.round(np.degrees(q_now), 1)} deg, "
              f"stylus tip at {np.round(kin.fk_tool(q_now)[0], 0)} mm")
        try:
            kin.check_limits(q_now)
        except kin.IKError as e:
            print(f"  WARNING: {e}. If the arm is NOT in that pose, JOINT_DIRECTION / "
                  "JOINT_OFFSET_DEG are wrong: run --check-joints first.")
        if args.check_joints:
            check_joints(arm, robot)
            return
        if args.sim:
            cam_model = kin.CameraModel.from_fov(C.FRAME_WIDTH, C.FRAME_HEIGHT, C.CAMERA_HFOV_DEG)
            camera = sim.SimCamera(robot, cam_model)
            camera.model = cam_model
        else:
            camera = Camera(args.camera)
            ui.live = camera                     # live view, also while the arm moves
        task = LineFollower(robot, camera, ui, realtime, "debug_sim" if args.sim else "debug")
        task.partial = args.partial
        task.dry = args.dry
        task.teach_mode = args.teach
        if not args.sim:                     # the simulator never uses / overwrites the real correction
            task.correction_file = C.CORRECTION_FILE
            if os.path.exists(C.CORRECTION_FILE):
                with open(C.CORRECTION_FILE, encoding="utf-8") as f:
                    task.correction = json.load(f)
        print(f"  images used for every decision are saved in {task.debug_dir}")
        if args.check_camera:
            check_camera(task)
        else:
            result = task.run(auto=args.auto)
            if args.sim and result is not None:
                truth = camera.truth
                path = result[1]["path"][:, :2]
                err = [np.min(np.linalg.norm(truth - p, axis=1)) for p in path]
                print(f"[SIM] path vs. true line: mean {np.mean(err):.2f} mm, max {np.max(err):.2f} mm; "
                      f"start error {np.linalg.norm(path[0] - truth[0]):.2f} mm, "
                      f"end error {np.linalg.norm(path[-1] - truth[-1]):.2f} mm")
    except (UserAbort, KeyboardInterrupt):
        print("\nSTOP requested - holding the current position.")
        robot.hold()
    except (JointBlocked, ServoError, kin.IKError, vis.VisionError) as e:
        robot.hold()
        print(f"\nSTOPPED: {e}")
    except Exception:
        robot.hold()
        raise
    finally:
        if camera is not None:
            camera.release()
        arm.close(release=args.release)
        ui.close()


if __name__ == "__main__":
    main()
