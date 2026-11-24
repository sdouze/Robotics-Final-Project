"""Calibrate the camera intrinsics with a printed checkerboard.

Print checkerboard_9x6_A4.pdf (9 x 6 INNER corners, 24 mm squares) at 100 %,
glue it flat, then run
    python calibrate_camera.py                 (camera = CAMERA_INDEX in config.py)
Hold the board in front of the camera and press SPACE when the corners are
drawn: 15-25 images, near and far, tilted up to ~45 deg in all directions,
and with the board in the CORNERS and along the EDGES of the image too (the
dots show where corners were captured so far). Then press 'c'.

The model is kept simple so few images cannot give nonsense: square pixels
(fx = fy) and radial distortion k1, k2 only. Every captured image is saved in
calib_images/, so the calibration can be recomputed without the robot:
    python calibrate_camera.py --from-images
The result is checked (field of view, distortion, reprojection error) and only
then saved to camera_calibration.npz, which line_follower.py loads.
"""

import argparse
import glob
import os

import cv2
import numpy as np

import config as C

IMG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calib_images")
FLAGS = cv2.CALIB_FIX_ASPECT_RATIO | cv2.CALIB_FIX_K3 | cv2.CALIB_ZERO_TANGENT_DIST


def find_corners(gray, pattern, fast):
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    if fast:
        flags += cv2.CALIB_CB_FAST_CHECK
    found, corners = cv2.findChessboardCorners(gray, pattern, None, flags)
    if found:
        crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), crit)
    return found, corners


def calibrate(obj_pts, img_pts, size):
    """Returns (ok, message, K, dist)."""
    K0 = np.array([[1.0, 0, size[0] / 2], [0, 1.0, size[1] / 2], [0, 0, 1]])   # fx/fy ratio 1
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(obj_pts, img_pts, size, K0, np.zeros(5), flags=FLAGS)
    hfov = 2 * np.degrees(np.arctan(size[0] / 2 / K[0, 0]))
    per_img = []
    for o, i, r, t in zip(obj_pts, img_pts, rvecs, tvecs):
        p, _ = cv2.projectPoints(o, r, t, K, dist)
        per_img.append(float(np.sqrt(np.mean(np.sum((p.reshape(-1, 2) - i.reshape(-1, 2)) ** 2, axis=1)))))
    msg = (f"RMS reprojection error {rms:.3f} px (worst image {max(per_img):.2f} px), HFOV {hfov:.1f} deg, "
           f"fx = fy = {K[0, 0]:.1f}, centre ({K[0, 2]:.0f}, {K[1, 2]:.0f}), k1 {dist.ravel()[0]:+.3f}, "
           f"k2 {dist.ravel()[1]:+.3f}")
    problems = []
    # enough variety? (flat boards in the middle of the image give a wrong focal length)
    tilts = [np.degrees(np.arccos(abs(cv2.Rodrigues(r)[0][2, 2]))) for r in rvecs]
    n_tilted = sum(t > 20.0 for t in tilts)
    allp = np.vstack([i.reshape(-1, 2) for i in img_pts])
    span_x = (allp[:, 0].max() - allp[:, 0].min()) / size[0]
    span_y = (allp[:, 1].max() - allp[:, 1].min()) / size[1]
    msg += f"\n  variety: {n_tilted} images tilted > 20 deg, corners cover {100 * span_x:.0f}% x {100 * span_y:.0f}% of the image"
    if n_tilted < 3:
        problems.append(f"only {n_tilted} images with the board tilted more than 20 deg (need 3+)")
    if span_x < 0.75 or span_y < 0.7:
        problems.append("the board was not taken near the edges/corners of the image")
    if not 35.0 <= hfov <= 120.0:
        problems.append(f"field of view {hfov:.0f} deg is not plausible for a webcam")
    if abs(K[0, 2] - size[0] / 2) > 0.15 * size[0] or abs(K[1, 2] - size[1] / 2) > 0.15 * size[1]:
        problems.append("image centre far from the middle of the image")
    if abs(dist.ravel()[0]) > 1.0 or abs(dist.ravel()[1]) > 3.0:
        problems.append("lens distortion values are extreme")
    if rms > 1.0:
        problems.append(f"reprojection error {rms:.2f} px is high (blurred or moving board?)")
    if problems:
        msg += "\nNOT SAVED: " + "; ".join(problems) + (
            ".\nTake more VARIED images: board tilted ~30-45 deg in different directions, near and far, "
            "and in the image corners.")
    return not problems, msg, K, dist


def save(K, dist, size):
    np.savez(C.CALIBRATION_FILE, K=K, dist=dist, size=np.array(size))
    print(f"K =\n{np.round(K, 1)}\nsaved to {C.CALIBRATION_FILE}")
    print("Now teach again (the old correction belongs to the old camera model):\n"
          "    python line_follower.py --teach --dry")


def from_images(pattern, obj):
    obj_pts, img_pts, size = [], [], None
    for f in sorted(glob.glob(os.path.join(IMG_DIR, "*.png"))):
        gray = cv2.cvtColor(cv2.imread(f), cv2.COLOR_BGR2GRAY)
        size = gray.shape[::-1]
        found, corners = find_corners(gray, pattern, fast=False)
        if found:
            obj_pts.append(obj)
            img_pts.append(corners)
    print(f"{len(obj_pts)} images with the board found in {IMG_DIR}")
    if len(obj_pts) < 8:
        return
    ok, msg, K, dist = calibrate(obj_pts, img_pts, size)
    print(msg)
    if ok:
        save(K, dist, size)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--camera", type=int, default=C.CAMERA_INDEX)
    ap.add_argument("--cols", type=int, default=9, help="inner corners per row")
    ap.add_argument("--rows", type=int, default=6, help="inner corners per column")
    ap.add_argument("--square", type=float, default=24.0, help="square size in mm")
    ap.add_argument("--from-images", action="store_true", help="recompute from calib_images/ (no camera)")
    args = ap.parse_args()
    pattern = (args.cols, args.rows)
    obj = np.zeros((args.cols * args.rows, 3), np.float32)
    obj[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square
    if args.from_images:
        from_images(pattern, obj)
        return

    os.makedirs(IMG_DIR, exist_ok=True)
    for f in glob.glob(os.path.join(IMG_DIR, "*.png")):           # a new session starts empty
        os.remove(f)
    cap = cv2.VideoCapture(args.camera, cv2.CAP_DSHOW if os.name == "nt" else cv2.CAP_ANY)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, C.FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, C.FRAME_HEIGHT)
    obj_pts, img_pts, size = [], [], None
    print("SPACE = capture (when the corners are drawn), c = calibrate, q = quit")
    while True:
        ok, frame = cap.read()
        if not ok:
            raise RuntimeError("camera gives no images")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        size = gray.shape[::-1]
        found, corners = find_corners(gray, pattern, fast=True)
        view = frame.copy()
        for pts in img_pts:                                         # coverage so far
            for p in pts.reshape(-1, 2)[::4]:
                cv2.circle(view, (int(p[0]), int(p[1])), 2, (255, 0, 255), -1)
        if found:
            cv2.drawChessboardCorners(view, pattern, corners, found)
        cv2.putText(view, f"captured {len(obj_pts)}  SPACE=capture  c=calibrate  q=quit",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
        cv2.imshow("calibration", view)
        key = cv2.waitKey(1) & 0xFF
        if key == ord(" ") and found:
            obj_pts.append(obj)
            img_pts.append(corners)
            cv2.imwrite(os.path.join(IMG_DIR, f"img_{len(obj_pts):02d}.png"), frame)
            print(f"  image {len(obj_pts)} captured ({size[0]}x{size[1]})")
        elif key == ord("c"):
            if len(obj_pts) < 8:
                print(f"  need at least 8 images (have {len(obj_pts)})")
                continue
            ok, msg, K, dist = calibrate(obj_pts, img_pts, size)
            print(msg)
            if ok:
                save(K, dist, size)
                break
            print("  -> capture more images and press 'c' again (or q to quit)")
        elif key in (27, ord("q")):
            break
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
