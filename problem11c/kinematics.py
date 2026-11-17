"""Forward / inverse kinematics and the camera model for Problem 11c.

Same DH model as Problems 1-3. Joint angles are in radians, lengths in mm.

Pitch: the stylus x4 axis always lies in the vertical plane of the arm. Its
angle above the horizontal is

    pitch = (q2 + pi/2) + q3 + q4          (-pi/2 = stylus pointing down)

so a horizontal stylus (Problem 3) is q2 + q3 + q4 = -pi/2.
"""

import math

import numpy as np

import config as C


class IKError(ValueError):
    """Raised when a pose is unreachable or violates the joint limits."""


JOINT_LIMITS = np.radians(np.array(C.JOINT_LIMITS_DEG))


# ---------------------------------------------------------------------------
# Forward kinematics
# ---------------------------------------------------------------------------
def dh(theta, d, a, alpha):
    """Standard DH link transform Rot_z(theta) Trans_z(d) Trans_x(a) Rot_x(alpha)."""
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([[ct, -st * ca, st * sa, a * ct],
                     [st, ct * ca, -ct * sa, a * st],
                     [0.0, sa, ca, d],
                     [0.0, 0.0, 0.0, 1.0]])


def fk_frames(q):
    """Return [T01, T02, T03, T04]."""
    q1, q2, q3, q4 = q
    links = (dh(q1, C.BASE_HEIGHT, 0.0, math.pi / 2),
             dh(q2 + math.pi / 2, 0.0, C.L2, 0.0),
             dh(q3, 0.0, C.L3, 0.0),
             dh(q4, 0.0, C.L4, 0.0))
    T = np.eye(4)
    frames = []
    for A in links:
        T = T @ A
        frames.append(T)
    return frames


def fk_tip(q):
    """T04: base -> stylus tip."""
    return fk_frames(q)[-1]


T45 = np.eye(4)
T45[:3, 3] = C.CAMERA_OFFSET_F4


def _camera_rotation_in_f5():
    x_c = np.asarray(C.CAMERA_IMAGE_RIGHT_IN_F5, float)
    y_c = np.asarray(C.CAMERA_IMAGE_DOWN_IN_F5, float)
    z_c = np.asarray(getattr(C, "CAMERA_OPTICAL_AXIS_IN_F5", (1.0, 0.0, 0.0)), float)
    x_c /= np.linalg.norm(x_c)
    y_c /= np.linalg.norm(y_c)
    z_c /= np.linalg.norm(z_c)
    if abs(x_c @ y_c) > 1e-6 or abs(x_c @ z_c) > 1e-6 or abs(y_c @ z_c) > 1e-6:
        raise ValueError("Camera axes in config.py must be perpendicular to each other")
    if not np.allclose(np.cross(x_c, y_c), z_c):
        raise ValueError("Camera axes in config.py are left-handed (mirrored image). "
                         "Flip CAMERA_IMAGE_RIGHT_IN_F5 or CAMERA_IMAGE_DOWN_IN_F5.")
    return np.column_stack((x_c, y_c, z_c))


T5C = np.eye(4)
T5C[:3, :3] = _camera_rotation_in_f5()

# Angle of the optical axis relative to the stylus, measured in the arm plane
# (positive = towards y4). 0 if the lens looks along the stylus, +-90 deg if it
# looks along -+y4, 180 deg if it looks backwards along the stylus. None if the
# lens looks sideways (out of the arm plane, along z4).
_ax = T5C[:3, 2]
CAMERA_AXIS_OFFSET = None if abs(_ax[2]) > 0.5 else math.atan2(_ax[1], _ax[0])


def fk_camera(q):
    """T05: base -> camera frame 5 (Problem 1)."""
    return fk_tip(q) @ T45


def camera_pose(q):
    """Base -> camera optical frame (x right, y down, z forward in the image)."""
    return fk_tip(q) @ T45 @ T5C


def pitch_of(q):
    """Angle of x4 (the last link) above the horizontal, in the arm plane."""
    return q[1] + math.pi / 2 + q[2] + q[3]


# ---------------------------------------------------------------------------
# The stylus (tool): a point and a direction fixed in frame 4
# ---------------------------------------------------------------------------
TOOL_TIP_F4 = np.asarray(getattr(C, "TOOL_TIP_IN_F4", (0.0, 0.0, 0.0)), float)
TOOL_DIR_F4 = np.asarray(getattr(C, "TOOL_DIR_IN_F4", (1.0, 0.0, 0.0)), float)
TOOL_DIR_F4 = TOOL_DIR_F4 / np.linalg.norm(TOOL_DIR_F4)
if abs(TOOL_DIR_F4[2]) > 0.5:
    raise ValueError("TOOL_DIR_IN_F4: the stylus must point within the plane of the arm (not along z4)")
# angle of the stylus relative to x4 in the arm plane: stylus pitch = x4 pitch + offset
TOOL_DIR_OFFSET = math.atan2(TOOL_DIR_F4[1], TOOL_DIR_F4[0])


def fk_tool(q):
    """Stylus tip position and stylus pointing direction in frame 0."""
    T = fk_tip(q)
    return T[:3, :3] @ TOOL_TIP_F4 + T[:3, 3], T[:3, :3] @ TOOL_DIR_F4


def tool_pitch_of(q):
    """Angle of the stylus above the horizontal (-pi/2 = pointing straight down)."""
    return _wrap(pitch_of(q) + TOOL_DIR_OFFSET)


# ---------------------------------------------------------------------------
# Inverse kinematics
# ---------------------------------------------------------------------------
def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def check_limits(q):
    q = np.asarray(q, float)
    bad = (q < JOINT_LIMITS[:, 0] - 1e-9) | (q > JOINT_LIMITS[:, 1] + 1e-9)
    if np.any(bad):
        j = int(np.argmax(bad))
        raise IKError(f"joint {j + 1} = {math.degrees(q[j]):.1f} deg is outside "
                      f"{C.JOINT_LIMITS_DEG[j]}")


def ik_tip(p, pitch, limits=True):
    """Frame 4 origin (end of the last link, the stylus tip of Figure 1) at
    p = (x, y, z) with x4 at the given pitch (rad). Elbow-up branch.
    For the real stylus use ik_tool()."""
    x, y, z = p
    r = math.hypot(x, y)
    if r < 1e-6:
        raise IKError("target on the base axis")
    q1 = math.atan2(y, x)

    # Wrist (frame 3 origin) in the arm plane, relative to frame 1.
    r_w = r - C.L4 * math.cos(pitch)
    z_w = z - C.BASE_HEIGHT - C.L4 * math.sin(pitch)
    c3 = (r_w ** 2 + z_w ** 2 - C.L2 ** 2 - C.L3 ** 2) / (2 * C.L2 * C.L3)
    if abs(c3) > 1.0:
        raise IKError(f"target {np.round(p, 1)} out of reach (pitch "
                      f"{math.degrees(pitch):.0f} deg)")
    q3 = -math.acos(c3)                      # elbow up, as in Problem 3
    a2 = math.atan2(z_w, r_w) - math.atan2(C.L3 * math.sin(q3),
                                           C.L2 + C.L3 * math.cos(q3))
    q2 = a2 - math.pi / 2
    q4 = pitch - a2 - q3
    q = np.array([_wrap(q1), _wrap(q2), _wrap(q3), _wrap(q4)])
    if limits:
        check_limits(q)
    return q


def ik_point(p, offset_f4, pitch, limits=True):
    """Put a point that is fixed in frame 4 (offset_f4) at p, with x4 at the
    given pitch.

    The offset (ox, oy, oz) maps to the arm plane as
    dr = ox cos(p) - oy sin(p) (radial) and dz = ox sin(p) + oy cos(p) (up),
    plus oz along z4, which is horizontal and perpendicular to the arm plane.
    """
    ox, oy, oz = offset_f4
    dr = ox * math.cos(pitch) - oy * math.sin(pitch)
    dz = ox * math.sin(pitch) + oy * math.cos(pitch)
    X, Y, Z = p
    R2 = X * X + Y * Y
    if R2 <= oz * oz:
        raise IKError("target too close to the base axis")
    rho = math.sqrt(R2 - oz * oz)            # radial distance of (frame 4 origin + dr)
    q1 = math.atan2(Y, X) + math.atan2(oz, rho)
    r_tip = rho - dr
    if r_tip <= 1.0:
        raise IKError("target too close to the base axis")
    o4 = (r_tip * math.cos(q1), r_tip * math.sin(q1), Z - dz)
    return ik_tip(o4, pitch, limits)


def ik_camera(p_cam, pitch, limits=True):
    """Put the camera lens (frame 5 origin) at p_cam, with x4 at the given pitch."""
    return ik_point(p_cam, C.CAMERA_OFFSET_F4, pitch, limits)


def ik_tool(p, tool_pitch, limits=True):
    """Put the STYLUS TIP at p with the stylus pointing tool_pitch above the
    horizontal (-pi/2 = straight down). Works for any stylus mounting given in
    TOOL_TIP_IN_F4 / TOOL_DIR_IN_F4."""
    return ik_point(p, TOOL_TIP_F4, tool_pitch - TOOL_DIR_OFFSET, limits)


def ik_camera_lowering(xy, pitch, height, min_height):
    """ik_camera at the requested height, lowered in 10 mm steps if needed."""
    h = height
    while h >= min_height - 1e-9:
        try:
            return ik_camera((xy[0], xy[1], C.PAPER_Z + h), pitch), h
        except IKError:
            h -= 10.0
    raise IKError(f"camera position {np.round(xy, 1)} unreachable")


# ---------------------------------------------------------------------------
# Pinhole camera / table plane
# ---------------------------------------------------------------------------
class CameraModel:
    def __init__(self, K, width, height):
        self.K = np.asarray(K, float)
        self.width = width
        self.height = height

    @classmethod
    def from_fov(cls, width, height, hfov_deg):
        f = (width / 2) / math.tan(math.radians(hfov_deg) / 2)
        return cls([[f, 0, width / 2], [0, f, height / 2], [0, 0, 1]], width, height)

    def project(self, T0c, P):
        """World points (N,3) -> pixels (N,2) and depth (N,)."""
        R, t = T0c[:3, :3], T0c[:3, 3]
        Pc = (np.asarray(P, float) - t) @ R          # = R^T (P - t)
        z = Pc[:, 2]
        zs = np.where(z > 1e-6, z, np.nan)
        u = self.K[0, 0] * Pc[:, 0] / zs + self.K[0, 2]
        v = self.K[1, 1] * Pc[:, 1] / zs + self.K[1, 2]
        return np.column_stack((u, v)), z

    def pixel_to_plane(self, T0c, uv, z_plane=None):
        """Pixels (N,2) -> points (N,3) on the plane z = z_plane (frame 0)."""
        z_plane = C.PAPER_Z if z_plane is None else z_plane
        uv = np.atleast_2d(np.asarray(uv, float))
        rays_c = np.column_stack(((uv[:, 0] - self.K[0, 2]) / self.K[0, 0],
                                  (uv[:, 1] - self.K[1, 2]) / self.K[1, 1],
                                  np.ones(len(uv))))
        rays = rays_c @ T0c[:3, :3].T
        t = T0c[:3, 3]
        with np.errstate(divide="ignore", invalid="ignore"):
            s = (z_plane - t[2]) / rays[:, 2]
        s = np.where(s > 0, s, np.nan)               # behind the camera => nan
        return t + s[:, None] * rays

    def footprint(self, height):
        """Approximate size (w, h) in mm seen on the table from a given height."""
        return (height * self.width / self.K[0, 0], height * self.height / self.K[1, 1])
