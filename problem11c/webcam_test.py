"""Webcam test for Problem 11c - no robot needed.

Opens the laptop camera, detects the A4 sheet, the black line and the start
dot with the same vision code as the robot program, and SIMULATES the
following: the sheet is placed virtually in front of the robot, the stylus
path is planned with the same inverse kinematics / speed profile, and the
stylus is animated along the line together with the joint angles.

How it works
  1. paper_mask() + 4 corners -> homography -> the sheet is warped to a
     top-down 210 x 297 mm image (0.5 mm per pixel). This replaces the
     robot's multi-view mosaic, so a hand-held camera works too.
  2. vision.extract_line() finds the dot and orders the line (in mm).
  3. The sheet is put at PAPER_CENTER in the robot base frame, every path
     point goes through kinematics.ik_tool(), and the stylus is animated in
     real time at FOLLOW_SPEED_MM_S.

Usage (from this folder):
  python webcam_test.py                  laptop camera (index 0)
  python webcam_test.py --camera 1       another camera
  python webcam_test.py --image foto.jpg test on a photo
Keys:  SPACE  lock the detected path and start the simulated following
       r      unlock / detect again
       w s a d move the virtual sheet in front of the robot (+-10 mm)
       o      landscape / portrait placement of the virtual sheet
       q/Esc  quit
"""

import argparse
import math
import time

import cv2
import numpy as np

import config as C
import kinematics as kin
import vision as vis
from robot import time_parametrize

RES = 0.5                                    # mm per pixel of the rectified sheet
W_MM, H_MM = C.A4_SIZE_MM                    # 210 x 297 (portrait)
WPX, HPX = int(round(W_MM / RES)), int(round(H_MM / RES))
EDGE_MARGIN_MM = 4.0                         # ignore the sheet border (shadows, corner error)


# ---------------------------------------------------------------------------
# Sheet detection and rectification
# ---------------------------------------------------------------------------
def find_corners(frame):
    """Four corners of the sheet in the image (ordered), or None."""
    paper, frac = vis.paper_mask(frame)
    if frac < 0.05:
        return None, paper, frac
    contours, _ = cv2.findContours(paper, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, paper, frac
    c = max(contours, key=cv2.contourArea)
    peri = cv2.arcLength(c, True)
    for eps in (0.02, 0.03, 0.04, 0.05, 0.06):
        poly = cv2.approxPolyDP(c, eps * peri, True)
        if len(poly) == 4:
            break
    else:
        return None, paper, frac
    pts = poly.reshape(4, 2).astype(np.float32)
    s, d = pts.sum(1), pts[:, 1] - pts[:, 0]
    tl, br = pts[np.argmin(s)], pts[np.argmax(s)]
    tr, bl = pts[np.argmin(d)], pts[np.argmax(d)]
    corners = np.array([tl, tr, br, bl], np.float32)
    # the long side of the sheet must become the vertical side of the warp
    if np.linalg.norm(tr - tl) > np.linalg.norm(bl - tl):
        corners = np.array([tr, br, bl, tl], np.float32)
    return corners, paper, frac


def homography(corners):
    dst = np.array([[0, 0], [WPX - 1, 0], [WPX - 1, HPX - 1], [0, HPX - 1]], np.float32)
    return cv2.getPerspectiveTransform(corners, dst)


class SheetMap:
    """Same interface as vision.Mosaic, so vision.extract_line() can be reused."""

    def __init__(self, warped):
        self.res = RES
        self.shape = warped.shape[:2]
        mask = np.zeros(self.shape, np.uint8)
        e = int(round(EDGE_MARGIN_MM / RES))
        mask[e:-e, e:-e] = 255
        dark = vis.dark_mask(warped, mask)
        dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
        n, labels, stats, _ = cv2.connectedComponentsWithStats(dark, 8)
        keep = np.zeros(n, bool)
        keep[1:] = stats[1:, cv2.CC_STAT_AREA] >= C.MIN_LINE_AREA_MM2 / RES ** 2
        self._ink = np.where(keep[labels], 255, 0).astype(np.uint8)

    def ink(self, paper=None):
        return self._ink

    def to_world(self, rc):                  # sheet coordinates in mm: x right, y down
        rc = np.asarray(rc, float)
        return np.column_stack((rc[:, 1] * RES, rc[:, 0] * RES))

    def seen_mask(self):
        return np.ones(self.shape, bool)


# ---------------------------------------------------------------------------
# Virtual placement in front of the robot and planning
# ---------------------------------------------------------------------------
LANDSCAPE = [True]          # sheet orientation in front of the robot (key 'o' toggles)


def sheet_to_robot(xy, center):
    """Sheet mm (x right, y down, seen from above) -> robot frame 0.

    Landscape: the long side runs across the robot (along y0), the short side
    points away from it. Portrait: the long side points away from the robot.
    Both maps have determinant -1 because image y points down while robot y
    points left (the image is seen from above, z into the table)."""
    xy = np.atleast_2d(xy)
    if LANDSCAPE[0]:
        return np.column_stack((center[0] + (xy[:, 0] - W_MM / 2),
                                center[1] + (H_MM / 2 - xy[:, 1])))
    return np.column_stack((center[0] + (H_MM / 2 - xy[:, 1]),
                            center[1] + (W_MM / 2 - xy[:, 0])))


def plan(path_sheet, center):
    """Time-parametrised path + IK. Returns a dict (qs is None if unreachable)."""
    samples, T = time_parametrize(path_sheet, C.FOLLOW_SPEED_MM_S, C.FOLLOW_ACCEL_MM_S2)
    samples = np.vstack([path_sheet[:1], samples])
    rob = sheet_to_robot(samples, center)
    z = C.PAPER_Z - C.PEN_DEPTH
    for pd in C.FOLLOW_PITCHES_DEG:
        try:
            qs = np.array([kin.ik_tool((x, y, z), math.radians(pd)) for x, y in rob])
            return dict(samples=samples, rob=rob, qs=qs, pitch=pd, T=T)
        except kin.IKError:
            continue
    return dict(samples=samples, rob=rob, qs=None, pitch=None, T=T)


# ---------------------------------------------------------------------------
# Drawing
# ---------------------------------------------------------------------------
def draw_camera(frame, corners, H, line, marker_xy, text):
    img = frame.copy()
    if corners is not None:
        cv2.polylines(img, [corners.astype(np.int32).reshape(-1, 1, 2)], True, (0, 255, 0), 2)
    if line is not None and H is not None:
        Hinv = np.linalg.inv(H)
        to_img = lambda xy: cv2.perspectiveTransform(                       # noqa: E731
            (np.atleast_2d(xy) / RES).reshape(-1, 1, 2).astype(np.float32), Hinv).reshape(-1, 2)
        cv2.polylines(img, [to_img(line["path"]).astype(np.int32).reshape(-1, 1, 2)], False, (0, 200, 0), 2)
        d = to_img(line["dot"][:2])[0]
        cv2.circle(img, tuple(d.astype(int)), 10, (255, 120, 0), 2)
        if marker_xy is not None:
            cv2.circle(img, tuple(to_img(marker_xy)[0].astype(int)), 7, (255, 0, 255), -1)
    y = 25
    for t in text:
        cv2.putText(img, t, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
        y += 22
    return img


def draw_sheet(warped, smap, line, marker_xy):
    img = warped.copy()
    if smap is not None:
        img[smap.ink() > 0] = (0, 0, 255)
    if line is not None:
        cv2.polylines(img, [(line["path"] / RES).astype(np.int32).reshape(-1, 1, 2)], False, (0, 200, 0), 2)
        cv2.circle(img, tuple((line["dot"][:2] / RES).astype(int)), int(line["dot"][2] / RES), (255, 120, 0), 2)
        if marker_xy is not None:
            cv2.circle(img, tuple((np.asarray(marker_xy) / RES).astype(int)), 6, (255, 0, 255), -1)
    return cv2.resize(img, (WPX * 3 // 4, HPX * 3 // 4))


def draw_robot(center, line, pl, k):
    """Top view (x up, y left) and side view (r right, z up) of the simulated robot."""
    S = 360
    top = np.full((S, S, 3), 245, np.uint8)
    side = np.full((S, S, 3), 245, np.uint8)
    tp = lambda x, y: (int(S / 2 - y), int(S - 20 - x))                    # noqa: E731
    sp = lambda r, z: (int(20 + r), int(S - 60 - z))                       # noqa: E731

    cv2.circle(top, tp(0, 0), 186, (210, 210, 210), 1)                     # max reach
    cv2.circle(top, tp(0, 0), 65, (210, 210, 210), 1)                      # min reach (stylus down)
    corners = sheet_to_robot(np.array([[0, 0], [W_MM, 0], [W_MM, H_MM], [0, H_MM]]), center)
    cv2.polylines(top, [np.array([tp(*c) for c in corners]).reshape(-1, 1, 2)], True, (150, 150, 150), 1)
    cv2.line(side, sp(-20, C.PAPER_Z), sp(330, C.PAPER_Z), (150, 150, 150), 1)

    if pl is not None:
        col = (0, 170, 0) if pl["qs"] is not None else (0, 0, 220)
        cv2.polylines(top, [np.array([tp(*p) for p in pl["rob"]]).reshape(-1, 1, 2)], False, col, 2)
        cv2.circle(top, tp(*sheet_to_robot(line["dot"][:2], center)[0]), 5, (255, 120, 0), 2)
        if pl["qs"] is not None:
            frames = kin.fk_frames(pl["qs"][k])
            pts = np.array([[0, 0, 0]] + [T[:3, 3] for T in frames])
            cv2.polylines(top, [np.array([tp(p[0], p[1]) for p in pts]).reshape(-1, 1, 2)],
                          False, (60, 60, 60), 3)
            r = np.hypot(pts[:, 0], pts[:, 1])
            cv2.polylines(side, [np.array([sp(ri, zi) for ri, zi in zip(r, pts[:, 2])]).reshape(-1, 1, 2)],
                          False, (60, 60, 60), 3)
            for ri, zi in zip(r, pts[:, 2]):
                cv2.circle(side, sp(ri, zi), 4, (0, 120, 255), -1)
            cv2.circle(top, tp(*pl["rob"][k]), 5, (255, 0, 255), -1)
    cv2.drawMarker(top, tp(0, 0), (0, 0, 0), cv2.MARKER_TILTED_CROSS, 12, 2)
    cv2.putText(top, "top view (base = X)", (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)
    cv2.putText(side, "side view (arm plane)", (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)
    if pl is not None and pl["qs"] is not None:
        q = np.degrees(pl["qs"][k])
        for i, v in enumerate(q):
            cv2.putText(side, f"q{i + 1} = {v:7.1f} deg", (8, 40 + 18 * i),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)
        cv2.putText(side, f"stylus pitch {pl['pitch']} deg", (8, 40 + 18 * 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)
    elif pl is not None:
        cv2.putText(side, "line NOT reachable here", (8, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 220), 2)
        cv2.putText(side, "move the sheet with w/s/a/d/o", (8, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 220), 1)
    return np.vstack([top, side])


# ---------------------------------------------------------------------------
def detect(frame):
    corners, _, frac = find_corners(frame)
    if corners is None:
        return None, None, None, None, f"A4 sheet not found (white area {100 * frac:.0f}%)"
    H = homography(corners)
    warped = cv2.warpPerspective(frame, H, (WPX, HPX))
    smap = SheetMap(warped)
    try:
        line = vis.extract_line(smap)
        msg = (f"line {line['length_mm']:.0f} mm, {2 * line['half_width_mm']:.1f} mm wide, "
               f"dot r = {line['dot'][2]:.1f} mm")
    except vis.VisionError as e:
        line, msg = None, f"sheet found, {e}"
    return corners, H, (warped, smap), line, msg


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--image", help="use a photo instead of the camera")
    ap.add_argument("--center", type=float, nargs=2, default=(150.0, 0.0),
                    help="virtual sheet centre in the robot frame [mm]")
    ap.add_argument("--portrait", action="store_true", help="long side of the sheet away from the robot")
    ap.add_argument("--once", action="store_true", help="detect once, print, no windows (test)")
    args = ap.parse_args()

    cap = None
    if args.image:
        still = cv2.imread(args.image)
        if still is None:
            raise SystemExit(f"cannot read {args.image}")
    else:
        cap = cv2.VideoCapture(args.camera, cv2.CAP_DSHOW if hasattr(cv2, "CAP_DSHOW") else cv2.CAP_ANY)
        if not cap.isOpened():
            raise SystemExit(f"cannot open camera {args.camera} (try --camera 1)")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    def grab():
        if cap is None:
            return still.copy()
        ok, f = cap.read()
        if not ok:
            raise SystemExit("camera stopped")
        return f

    center = np.array(args.center, float)
    LANDSCAPE[0] = not args.portrait
    if args.once:
        corners, H, sheet, line, msg = detect(grab())
        print(msg)
        if line is not None:
            pl = plan(line["path"], center)
            if pl["qs"] is None:
                print(f"NOT reachable with the sheet centred at {center} mm")
            else:
                print(f"reachable: stylus pitch {pl['pitch']} deg, {len(pl['qs'])} set-points, "
                      f"{pl['T']:.1f} s; q range [deg]:")
                print("   min", np.round(np.degrees(pl["qs"].min(0)), 1))
                print("   max", np.round(np.degrees(pl["qs"].max(0)), 1))
                print(f"   largest joint step per sample {np.degrees(np.abs(np.diff(pl['qs'], axis=0))).max():.2f} deg")
        if cap is not None:
            cap.release()
        return

    print(__doc__)
    locked, pl, t_lock = None, None, 0.0
    last = (None, None, None, None, "")
    try:
        while True:
            frame = grab()
            if locked is None:
                last = detect(frame)
                corners, H, sheet, line, msg = last
                marker, k = None, 0
                text = [msg, "SPACE = lock path and simulate, q = quit"]
                pl_show = plan(line["path"], center) if line is not None else None
            else:
                line = locked
                c_new, _, _ = find_corners(frame)          # keep the overlay on the moving sheet
                if c_new is not None:
                    corners, H = c_new, homography(c_new)
                    sheet = (cv2.warpPerspective(frame, H, (WPX, HPX)), sheet[1])
                n = len(pl["samples"])
                k = min(int((time.perf_counter() - t_lock) * C.CONTROL_HZ), n - 1)
                if time.perf_counter() - t_lock > n / C.CONTROL_HZ + 1.5:   # loop the playback
                    t_lock = time.perf_counter()
                marker = pl["samples"][k]
                pl_show = pl
                state = ("reachable" if pl["qs"] is not None else "NOT reachable")
                text = [f"SIMULATING: {k / C.CONTROL_HZ:4.1f} / {pl['T']:.1f} s  ({state})",
                        f"sheet centre ({center[0]:.0f}, {center[1]:.0f}) mm {'landscape' if LANDSCAPE[0] else 'portrait'}"
                        "  w/s/a/d move, o rotate, r unlock"]

            cv2.imshow("camera", draw_camera(frame, corners, H, line, marker, text))
            if sheet is not None:
                cv2.imshow("sheet (top-down, mm)", draw_sheet(sheet[0], sheet[1], line, marker))
            if line is not None:
                cv2.imshow("robot simulation", draw_robot(center, line, pl_show, k))

            key = cv2.waitKey(1 if cap is not None else 30) & 0xFF
            if key in (27, ord("q")):
                break
            if key == ord(" ") and locked is None and line is not None:
                locked, pl, t_lock = line, plan(line["path"], center), time.perf_counter()
                print(f"locked: {msg}; " + (f"reachable, pitch {pl['pitch']} deg, {pl['T']:.1f} s"
                                            if pl["qs"] is not None else "NOT reachable at this placement"))
            if key == ord("r"):
                locked, pl = None, None
            moves = {ord("w"): (10, 0), ord("s"): (-10, 0), ord("a"): (0, 10), ord("d"): (0, -10)}
            if key == ord("o"):
                LANDSCAPE[0] = not LANDSCAPE[0]
                print("sheet orientation:", "landscape" if LANDSCAPE[0] else "portrait")
            if key in moves or key == ord("o"):
                center += moves.get(key, (0, 0))
                if locked is not None:
                    pl, t_lock = plan(locked["path"], center), time.perf_counter()
    finally:
        if cap is not None:
            cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
