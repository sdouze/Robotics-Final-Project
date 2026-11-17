"""Offline simulator: fake servo bus + virtual camera looking at a virtual A4.

Run  python line_follower.py --sim  to test the whole pipeline without the
robot. The scene is rendered with the same camera model the robot uses.
"""

import math

import cv2
import numpy as np

import config as C
import kinematics as kin

TEX_RES = 0.25                                   # mm per texture pixel
TEX_X = (-100.0, 350.0)
TEX_Y = (-300.0, 300.0)
TABLE_Z = 0.0                                    # TRUE table height (frame 0), independent
                                                 # of what config.PAPER_Z assumes


def _ticks_to_q(ticks):
    d, off = np.array(C.JOINT_DIRECTION, float), np.array(C.JOINT_OFFSET_DEG, float)
    return np.radians((np.asarray(ticks, float) - C.TICKS_CENTER) * C.DEG_PER_TICK / d - off)


def _q_to_ticks(q):
    d, off = np.array(C.JOINT_DIRECTION, float), np.array(C.JOINT_OFFSET_DEG, float)
    return np.round(C.TICKS_CENTER + d * (np.degrees(q) + off) / C.DEG_PER_TICK).astype(int)


class SimArm:
    """Same tick-level interface as robot.DxlArm. Servos are ideal, but the
    stylus cannot go into the table at TABLE_Z: a goal below the surface leaves
    the tip ON the surface (it slides along it), like a pressing real stylus."""

    def __init__(self):
        self.ticks = np.array([C.TICKS_CENTER] * 4)
        self.log = []

    def open(self):
        pass

    def close(self, release=False):
        pass

    def set_torque(self, on):
        pass

    def read_ticks(self):
        return self.ticks.copy()

    def is_moving(self):
        return False

    def write_goal_speed(self, goals, speeds):
        goals = np.clip(np.asarray(goals, int), 0, 1023)
        q = _ticks_to_q(goals)
        tip, _ = kin.fk_tool(q)
        if tip[2] < TABLE_Z:                           # contact: the stylus stops on the surface
            try:
                goals = _q_to_ticks(kin.ik_tool((tip[0], tip[1], TABLE_Z), kin.tool_pitch_of(q), limits=False))
            except kin.IKError:
                goals = self.ticks
        self.ticks = goals
        self.log.append(self.ticks.copy())


def make_scene(paper_center=(150.0, 10.0), paper_angle_deg=8.0, seed=1):
    """Table texture with an A4 sheet, a curved black line and a start dot."""
    rng = np.random.default_rng(seed)
    w = int((TEX_X[1] - TEX_X[0]) / TEX_RES)
    h = int((TEX_Y[1] - TEX_Y[0]) / TEX_RES)
    tex = np.zeros((h, w, 3), np.uint8)
    tex[:] = (70, 105, 140)                                   # wooden table (BGR)
    tex = cv2.add(tex, rng.integers(0, 25, (h, w, 3), dtype=np.uint8))

    def to_px(xy):
        xy = np.asarray(xy, float)
        return np.round(np.column_stack(((xy[:, 0] - TEX_X[0]) / TEX_RES,
                                         (xy[:, 1] - TEX_Y[0]) / TEX_RES))).astype(np.int32)

    a = math.radians(paper_angle_deg)
    R = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
    hw, hh = C.A4_SIZE_MM[0] / 2, C.A4_SIZE_MM[1] / 2      # short side along x
    corners = np.array([[-hw, -hh], [hw, -hh], [hw, hh], [-hw, hh]]) @ R.T + paper_center
    cv2.fillConvexPoly(tex, to_px(corners), (238, 240, 242), cv2.LINE_AA)

    # a smooth S-curve, drawn on the sheet (moves and rotates with the sheet;
    # with the default sheet pose it lies inside the reachable area)
    t = np.linspace(0, 1, 300)
    line = np.column_stack((105 + 55 * t + 10 * np.sin(2 * math.pi * t),
                            -70 + 140 * t + 25 * np.sin(3 * math.pi * t)))
    b = math.radians(paper_angle_deg - 8.0)
    Rb = np.array([[math.cos(b), -math.sin(b)], [math.sin(b), math.cos(b)]])
    k = C.A4_SIZE_MM[1] / 297.0                    # shrink the drawing for a smaller sheet
    line = k * (line - (150.0, 10.0)) @ Rb.T + paper_center
    th = max(1, int(round(3.0 / TEX_RES)))                  # 3 mm wide pen line
    cv2.polylines(tex, [to_px(line)], False, (25, 25, 25), th, cv2.LINE_AA)
    cv2.circle(tex, tuple(to_px(line[:1])[0]), int(6.0 / TEX_RES), (20, 20, 20), -1, cv2.LINE_AA)
    return tex, line, corners


class SimCamera:
    def __init__(self, robot, cam_model, scene=None):
        self.robot = robot
        self.cam = cam_model
        self.tex, self.truth, self.corners = scene or make_scene()
        u, v = np.meshgrid(np.arange(cam_model.width), np.arange(cam_model.height))
        self.uv = np.column_stack((u.ravel(), v.ravel())).astype(float)

    def read(self, fresh=False):                 # same signature as the real Camera
        q = self.robot.ticks_to_q(self.robot.arm.read_ticks())
        T0c = kin.camera_pose(q)
        P = self.cam.pixel_to_plane(T0c, self.uv, z_plane=TABLE_Z)
        mx = ((P[:, 0] - TEX_X[0]) / TEX_RES).reshape(self.cam.height, self.cam.width)
        my = ((P[:, 1] - TEX_Y[0]) / TEX_RES).reshape(self.cam.height, self.cam.width)
        img = cv2.remap(self.tex, np.nan_to_num(mx, nan=-1).astype(np.float32),
                        np.nan_to_num(my, nan=-1).astype(np.float32),
                        cv2.INTER_LINEAR, borderValue=(30, 30, 30))
        noise = np.random.default_rng().normal(0, 3, img.shape)
        return np.clip(cv2.GaussianBlur(img, (3, 3), 0) + noise, 0, 255).astype(np.uint8)

    def release(self):
        pass
