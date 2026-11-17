"""Optional: calibrate the camera intrinsics with a printed checkerboard.

Print a checkerboard (default 9 x 6 INNER corners, 24 mm squares), run
    python calibrate_camera.py --camera 1
hold the board in front of the camera at different angles/distances and press
SPACE to capture (15-20 images), then 'c' to calibrate and save
camera_calibration.npz (used automatically by line_follower.py).
"""

import argparse
import os

import cv2
import numpy as np

import config as C


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int, default=C.CAMERA_INDEX)
    ap.add_argument("--cols", type=int, default=9, help="inner corners per row")
    ap.add_argument("--rows", type=int, default=6, help="inner corners per column")
    ap.add_argument("--square", type=float, default=24.0, help="square size in mm")
    args = ap.parse_args()

    cap = cv2.VideoCapture(args.camera, cv2.CAP_DSHOW if os.name == "nt" else cv2.CAP_ANY)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, C.FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, C.FRAME_HEIGHT)
    pattern = (args.cols, args.rows)
    obj = np.zeros((args.cols * args.rows, 3), np.float32)
    obj[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square
    obj_pts, img_pts, size = [], [], None
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)

    while True:
        ok, frame = cap.read()
        if not ok:
            raise RuntimeError("camera gives no images")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        size = gray.shape[::-1]
        found, corners = cv2.findChessboardCorners(gray, pattern, None)
        view = frame.copy()
        if found:
            cv2.drawChessboardCorners(view, pattern, corners, found)
        cv2.putText(view, f"captured {len(obj_pts)}  SPACE=capture  c=calibrate  q=quit",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
        cv2.imshow("calibration", view)
        key = cv2.waitKey(1) & 0xFF
        if key == ord(" ") and found:
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), crit)
            obj_pts.append(obj)
            img_pts.append(corners)
        elif key == ord("c") and len(obj_pts) >= 8:
            rms, K, dist, _, _ = cv2.calibrateCamera(obj_pts, img_pts, size, None, None)
            np.savez(C.CALIBRATION_FILE, K=K, dist=dist, size=np.array(size))
            hfov = 2 * np.degrees(np.arctan(size[0] / 2 / K[0, 0]))
            print(f"RMS reprojection error {rms:.3f} px, HFOV {hfov:.1f} deg")
            print(f"K =\n{K}\nsaved to {C.CALIBRATION_FILE}")
            break
        elif key in (27, ord("q")):
            break
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
