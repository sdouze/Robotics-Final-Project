"""Problem 11c - all tunable parameters in one place.

Units: millimetres, degrees (converted to radians in code), seconds.
Everything marked CALIBRATE must be checked on the real robot before the
arm is allowed to touch the paper (see the usage notes in line_follower.py).
"""

import os

# ---------------------------------------------------------------------------
# Dynamixel bus (Annex C of the assignment)
# ---------------------------------------------------------------------------
PORT = "COM9"                 # CALIBRATE: port shown by Dynamixel Wizard
BAUDRATE = 1_000_000
PROTOCOL_VERSION = 1.0
DXL_IDS = (1, 2, 3, 4)        # joint 1..4
AX12A_MODEL_NUMBER = 12

# Path to the SDK shipped in this repository (used if dynamixel_sdk is not
# pip-installed).
def _find_sdk():
    """Look for 'Dynamixel Library/.../python/src' in this folder and its parents,
    so the project still works after being moved."""
    here = os.path.dirname(os.path.abspath(__file__))
    rel = os.path.join("Dynamixel Library", "MyRobot", "DynamixelSDK", "python", "src")
    d = here
    while True:
        cand = os.path.join(d, rel)
        if os.path.isdir(cand):
            return cand
        parent = os.path.dirname(d)
        if parent == d:
            return os.path.join(here, rel)          # not found: pip-installed dynamixel_sdk may still work
        d = parent


SDK_SRC_DIR = _find_sdk()

# Servo settings written at start-up
COMPLIANCE_MARGIN = 1         # ticks of dead band around the goal
COMPLIANCE_SLOPE = 32         # softer = smoother, 32 is the AX-12A default
PUNCH = 32
TORQUE_LIMIT = 1023           # 0..1023

# ---------------------------------------------------------------------------
# Joint <-> servo mapping
#   ticks = TICKS_CENTER + DIRECTION * (q_deg + OFFSET_DEG) / DEG_PER_TICK
# q = 0 is the DH zero pose of Figure 1 (arm straight up).
# ---------------------------------------------------------------------------
TICKS_CENTER = 512            # 150 deg on the AX-12A scale
DEG_PER_TICK = 300.0 / 1023.0
JOINT_DIRECTION = (1, 1, 1, 1)        # CALIBRATE: python calibrate_robot.py writes these
JOINT_OFFSET_DEG = (19.94, 0, 0, 0)  # CALIBRATE: (direction + straight-up offset)

# Software joint limits (DH angles, degrees). The AX-12A itself is +-150.
JOINT_LIMITS_DEG = ((-90.0, 90.0),
                    (-125.0, 125.0),
                    (-140.0, 140.0),
                    (-10.0, 140.0))   # joint 4: mechanical stop when bending down at
                                      # Wizard 137.7 deg = q4 -12.3 deg, minus 2 deg margin

# AX-12A moving speed: 1 unit = 0.111 rpm = 0.666 deg/s. 0 means "maximum",
# so the code never sends 0.
SPEED_UNIT_DEG_S = 0.111 * 6.0

# ---------------------------------------------------------------------------
# Geometry (Figure 1, DH table from Problem 1)
#   i  theta        d    a    alpha
#   1  q1           50   0    pi/2
#   2  q2 + pi/2    0    93   0
#   3  q3           0    93   0
#   4  q4           0    50   0
# ---------------------------------------------------------------------------
BASE_HEIGHT = 50.0
L2 = 93.0
L3 = 93.0
L4 = 50.0                     # CALIBRATE: real stylus length
CAMERA_OFFSET_F4 = (0.0, 0.0, 0.0)    # camera lens (frame 5 origin) expressed in frame 4
# ^ ASSUMED (not measured yet): lens at the end of the last link. Observed on
#   the robot: putting the frame 4 origin on the dot put the lens on the dot.
#   Figure 1 value was (-15.0, 45.0, 0.0). Measure: calibrate_robot.py --tool

# The STYLUS as a tool fixed to the last link: tip position and pointing
# direction expressed in frame 4. Frame 4 origin = 50 mm (L4) from the wrist
# axis along the last link; with the arm straight up: x4 = up, y4 = backward,
# z4 = robot's right. Figure 1 = tip at the frame 4 origin, pointing along x4.
# MEASURE YOUR ROBOT: python calibrate_robot.py --tool  (writes these values
# and CAMERA_OFFSET_F4 from a few ruler measurements).
TOOL_TIP_IN_F4 = (0.0, -10.0, 0.0)   # current stylus: tip ~10 mm in front of the lens (estimate) ...
TOOL_DIR_IN_F4 = (0.0, -1.0, 0.0)    # ... stylus pointing the same way as the lens
# (Figure 1 stylus would be TOOL_TIP_IN_F4 = (0, 0, 0), TOOL_DIR_IN_F4 = (1, 0, 0).)
# No stylus mounted yet? Run line_follower.py with --dry: the line is traced
# with the (imagined) stylus tip DRY_HOVER_MM above the paper and nothing touches.
DRY_HOVER_MM = 15.0

# Camera optical frame expressed in frame 5: the direction the lens looks
# (optical axis z_c), the image "right" (x_c) and image "down" (y_c) directions.
# Frame 5 = frame 4 orientation: x = along the stylus, y = in the arm plane,
# z = sideways. CALIBRATE: python calibrate_robot.py writes these three.
CAMERA_OPTICAL_AXIS_IN_F5 = (0, -1, 0)
CAMERA_IMAGE_RIGHT_IN_F5 = (0, 0, 1)
CAMERA_IMAGE_DOWN_IN_F5 = (-1, 0, 0)

# ---------------------------------------------------------------------------
# Camera
# ---------------------------------------------------------------------------
CAMERA_INDEX = 0              # CALIBRATE: 0 is usually the laptop webcam
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
CAMERA_HFOV_DEG = 60.0        # used only if no calibration file exists
CALIBRATION_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "camera_calibration.npz")
FRAMES_TO_FLUSH = 2           # after a move, use only frames captured this many frames later
SETTLE_TIME_S = 0.35          # wait after a move before grabbing an image

# ---------------------------------------------------------------------------
# Work surface
# ---------------------------------------------------------------------------
PAPER_Z = 0.0                 # CALIBRATE: height of the paper in frame 0
PEN_DEPTH = 1.0               # press the stylus this far "into" the paper
HOVER_HEIGHT = 25.0           # approach height above the paper
#A4_SIZE_MM = (210.0, 297.0)     # full A4
A4_SIZE_MM = (105.0, 148.5)     # quarter A4 (A6)
#A4_SIZE_MM = (148.5, 210.0)      # half A4 (A5) - short side first
# Paper height. PROBE_PAPER: before following, the stylus is lowered onto the
# paper at the start dot until it touches; the measured height replaces PAPER_Z
# (accurate to ~1 mm, independent of the camera). ESTIMATE_PAPER_Z: estimate
# the height from the known sheet size in the images instead (assignment
# Hint 2) - noisier (~2-3 mm) and fooled by a wrong camera field of view.
PROBE_PAPER = True
PROBE_SPEED_MM_S = 5.0        # lowering speed while probing
PROBE_RANGE_MM = 35.0         # search this far below PAPER_Z for the paper
PROBE_TRIGGER_MM = 2.0        # contact = tip lags the command by this much more than in free air
ESTIMATE_PAPER_Z = False
PAPER_Z_MAX_CORRECTION_MM = 40.0
PAPER_Z_TOLERANCE_MM = 1.0

# ---------------------------------------------------------------------------
# Motion
# ---------------------------------------------------------------------------
CONTROL_HZ = 25.0             # streaming rate of set-points
JOINT_SPEED_DEG_S = 35.0      # peak joint speed for point-to-point moves
MIN_MOVE_TIME_S = 0.8
SPEED_HEADROOM = 1.3          # commanded servo speed / needed speed
SETTLE_TOL_DEG = 2.0          # plus the servo's own "Moving" flag must be clear
STUCK_ERROR_DEG = 10.0        # a joint this far from its goal ...
STUCK_TIME_S = 0.6            # ... that has not moved for this long is BLOCKED -> stop
SETTLE_TIMEOUT_S = 3.0
LINEAR_SPEED_MM_S = 15.0      # approach / retreat
FOLLOW_SPEED_MM_S = 12.0      # speed along the black line
FOLLOW_ACCEL_MM_S2 = 20.0
MAX_JOINT_STEP_DEG = 8.0      # larger jumps between samples => abort

# Stylus pitch (angle of x4 above the horizontal). -90 = pointing straight
# down. The first pitch for which the whole line is reachable is used.
FOLLOW_PITCHES_DEG = (-90, -85, -80)
# The stylus sticks out only ~10 mm beyond the camera, so it must stay nearly
# VERTICAL: tilted by 15 deg the camera edge is only ~4.5 mm above the paper,
# by 30 deg it would touch. Straight down the stylus reaches the paper
# 100-230 mm from the base. With a longer stylus (30-40 mm) flatter angles
# can be added again: (-90, -85, ..., -30).

# ---------------------------------------------------------------------------
# Search for the A4 sheet (camera positions in frame 0)
# ---------------------------------------------------------------------------
OBSERVE_HEIGHT = 230.0        # max camera height above the paper. Straight down the arm
                              # reaches ~196 mm; tilted ~70-80 deg it reaches 225-239 mm and
                              # one view covers a whole half A4 (~200 x 285 mm or more)
MIN_OBSERVE_HEIGHT = 60.0     # ... lowered in 10 mm steps if unreachable. With the wrist
                              # limited to -10 deg the views end up ~80 mm high, ~50 deg down
OBSERVE_PITCH_DEG = -90.0     # camera looking straight down (preferred) ...
# ... and steeper-to-flatter fallbacks, used automatically when the preferred
# pose needs a joint (usually the wrist, joint 4) beyond JOINT_LIMITS_DEG.
OBSERVE_PITCHES_DEG = (-90.0, -80.0, -70.0, -60.0, -55.0, -50.0, -45.0, -40.0)
HOME_CAMERA_XY = (160.0, 0.0)  # table point the camera looks at first
SEARCH_CAMERA_RADII = (150.0, 105.0)
SEARCH_Q1_DEG = (-60, -40, -20, 0, 20, 40, 60)
PAPER_MIN_AREA_FRACTION = 0.02   # of the image, to count as "paper seen"
LOCK_TOLERANCE_MM = 8.0
LOCK_MAX_ITERATIONS = 4
CAMERA_REACH_RADIUS = (70.0, 200.0)  # clip camera targets to this annulus

# ---------------------------------------------------------------------------
# Mapping (mosaic in frame 0 on the plane z = PAPER_Z)
# ---------------------------------------------------------------------------
GRID_RES_MM = 0.5
GRID_X_MM = (-60.0, 300.0)
GRID_Y_MM = (-260.0, 260.0)
IMAGE_BORDER_PX = 12          # ignore image borders (distortion, vignetting)
VIEW_OVERLAP = 0.70           # spacing of mapping views = overlap * footprint
MAX_MAP_VIEWS = 14
EXTRA_MAP_ROUNDS = 2          # extra views where the line leaves the map

# ---------------------------------------------------------------------------
# Image processing
# ---------------------------------------------------------------------------
PAPER_MIN_VALUE = 130         # HSV V of white paper (0..255)
PAPER_MAX_SATURATION = 70     # HSV S of white paper
BLACK_RATIO = 0.55            # dark = gray < BLACK_RATIO * paper brightness
BLACK_MAX_GRAY = 110
PAPER_EDGE_ERODE_MM = 2.0     # ignore shadows at the paper border
MIN_LINE_AREA_MM2 = 25.0
DOT_MIN_RADIUS_MM = 2.0       # start dot: radius at least this ...
DOT_LINE_RATIO = 1.4          # ... and this much thicker than the other end of the line
MIN_LINE_LENGTH_MM = 15.0     # shorter ink marks are not taken as the line
LINE_GAP_MM = 2.0             # gaps in the pen line up to this size are bridged
PATH_SMOOTH_MM = 3.0          # moving-average window along the line
PATH_STEP_MM = 1.5            # resampling distance along the line
