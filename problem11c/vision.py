"""Computer vision for Problem 11c (OpenCV + NumPy).

Pipeline
  1. paper_mask():  white, unsaturated, largest blob = the A4 sheet (holes
                    filled so the black ink still counts as paper).
  2. dark_mask():   pixels much darker than the paper, inside the paper.
  3. Mosaic:        every image is re-projected onto the table plane
                    z = PAPER_Z in robot frame 0 (camera pose from forward
                    kinematics). Votes from several views are combined into a
                    top-down map with GRID_RES_MM resolution, so the line can
                    be larger than one camera view and everything is in mm.
  4. extract_line(): start dot = thickest black blob (distance transform),
                    line = skeleton of the black component, ordered by a
                    breadth-first search starting at the dot.
"""

from collections import deque

import cv2
import numpy as np

import config as C


class VisionError(RuntimeError):
    """retry=False: looking at more of the paper will not help (e.g. the whole
    line is visible but no end of it looks like a start dot)."""

    def __init__(self, message, retry=True):
        super().__init__(message)
        self.retry = retry


# ---------------------------------------------------------------------------
# Single image
# ---------------------------------------------------------------------------
def _largest_component(mask):
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    if n <= 1:
        return np.zeros_like(mask), 0
    k = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    return np.where(labels == k, 255, 0).astype(np.uint8), int(stats[k, cv2.CC_STAT_AREA])


def _convex_fill(mask):
    pts = cv2.findNonZero(mask)
    out = np.zeros_like(mask)
    if pts is not None and len(pts) >= 3:
        cv2.fillConvexPoly(out, cv2.convexHull(pts), 255)
    return out


def paper_mask(frame):
    """Mask of the A4 sheet in the image and its area fraction.

    A thick pen line or a crease cuts the white area into pieces (e.g. a thin
    strip between the line and the image edge). So dark gaps up to ~5 % of the
    image width are bridged first, then all large white pieces are kept and
    their convex hull is used (the sheet is a rectangle, which stays convex in
    a perspective image).
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    white = cv2.inRange(hsv, (0, 0, C.PAPER_MIN_VALUE), (180, C.PAPER_MAX_SATURATION, 255))
    white = cv2.morphologyEx(white, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    k = max(15, int(0.05 * frame.shape[1]) | 1)
    white = cv2.morphologyEx(white, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k)))
    n, labels, stats, _ = cv2.connectedComponentsWithStats(white, 8)
    if n <= 1:
        return np.zeros_like(white), 0.0
    areas = stats[1:, cv2.CC_STAT_AREA]
    keep = np.zeros(n, bool)
    keep[1:] = (areas >= 0.25 * areas.max()) & (areas >= 0.005 * white.size)
    mask = _convex_fill(np.where(keep[labels], 255, 0).astype(np.uint8))
    return mask, float(np.count_nonzero(mask)) / mask.size


def dark_mask(frame, paper):
    """Ink pixels: clearly darker than the local paper brightness."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    inside = paper > 0
    if np.count_nonzero(inside) < 100:
        return np.zeros_like(gray)
    level = np.median(gray[inside & (gray > 100)]) if np.any(inside & (gray > 100)) else 200
    thr = min(C.BLACK_MAX_GRAY, C.BLACK_RATIO * level)
    dark = np.where((gray < thr) & inside, 255, 0).astype(np.uint8)
    return cv2.morphologyEx(dark, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))


# ---------------------------------------------------------------------------
# Top-down mosaic on the table plane
# ---------------------------------------------------------------------------
class Mosaic:
    def __init__(self, cam):
        self.cam = cam
        self.res = C.GRID_RES_MM
        self.xs = np.arange(C.GRID_X_MM[0], C.GRID_X_MM[1], self.res)
        self.ys = np.arange(C.GRID_Y_MM[0], C.GRID_Y_MM[1], self.res)
        X, Y = np.meshgrid(self.xs, self.ys)              # rows = y, cols = x
        self.shape = X.shape
        self.XY = np.column_stack((X.ravel(), Y.ravel()))
        self.frames = []                                  # (frame, T0c) of every view
        self.reset(C.PAPER_Z)

    def reset(self, z_plane):
        """Empty the map and set the height of the plane it is projected onto."""
        self.z_plane = float(z_plane)
        self.P = np.column_stack((self.XY, np.full(len(self.XY), self.z_plane)))
        self.seen = np.zeros(self.shape, np.uint16)
        self.paper_votes = np.zeros(self.shape, np.uint16)
        self.dark_votes = np.zeros(self.shape, np.uint16)
        self.gray_sum = np.zeros(self.shape, np.float32)
        self.views = []                                   # camera xy of each view

    def rebuild(self, z_plane):
        """Re-project all stored views onto a plane at a different height."""
        frames = self.frames
        self.frames = []
        self.reset(z_plane)
        for frame, T0c in frames:
            self.add_view(frame, T0c)

    # grid <-> world
    def to_world(self, rc):
        rc = np.asarray(rc, float)
        return np.column_stack((self.xs[0] + rc[:, 1] * self.res,
                                self.ys[0] + rc[:, 0] * self.res))

    def to_grid(self, xy):
        xy = np.atleast_2d(np.asarray(xy, float))
        return np.column_stack(((xy[:, 1] - self.ys[0]) / self.res,
                                (xy[:, 0] - self.xs[0]) / self.res))

    def add_view(self, frame, T0c):
        self.frames.append((frame, np.array(T0c, float)))
        uv, depth = self.cam.project(T0c, self.P)
        b = C.IMAGE_BORDER_PX
        inside = ((depth > 1) & (uv[:, 0] >= b) & (uv[:, 0] <= self.cam.width - 1 - b)
                  & (uv[:, 1] >= b) & (uv[:, 1] <= self.cam.height - 1 - b))
        inside = inside.reshape(self.shape)
        if not np.any(inside):
            return
        mx = np.nan_to_num(uv[:, 0], nan=-1).reshape(self.shape).astype(np.float32)
        my = np.nan_to_num(uv[:, 1], nan=-1).reshape(self.shape).astype(np.float32)

        paper, _ = paper_mask(frame)
        dark = dark_mask(frame, paper)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        warp = lambda img: cv2.remap(img.astype(np.float32), mx, my, cv2.INTER_LINEAR)  # noqa: E731
        self.seen += inside
        self.paper_votes += inside & (warp(paper) > 127)
        self.dark_votes += inside & (warp(dark) > 127)
        self.gray_sum += np.where(inside, warp(gray), 0)
        self.views.append(T0c[:2, 3].copy())

    # derived maps
    def seen_mask(self):
        return self.seen > 0

    def paper_region(self, erode=True):
        """Largest paper blob in the map (filled); eroded at the edges by default."""
        with np.errstate(invalid="ignore", divide="ignore"):
            ratio = self.paper_votes / np.maximum(self.seen, 1)
        m = np.where((ratio > 0.5) & (self.seen > 0), 255, 0).astype(np.uint8)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
        m, _ = _largest_component(m)
        m = _convex_fill(m)                                # the sheet is a rectangle
        if not erode:
            return m
        e = max(1, int(round(C.PAPER_EDGE_ERODE_MM / self.res)))
        return cv2.erode(m, np.ones((2 * e + 1, 2 * e + 1), np.uint8))

    def ink(self, paper=None):
        paper = self.paper_region() if paper is None else paper
        with np.errstate(invalid="ignore", divide="ignore"):
            ratio = self.dark_votes / np.maximum(self.seen, 1)
        m = np.where((ratio >= 0.5) & (self.seen > 0) & (paper > 0), 255, 0).astype(np.uint8)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
        n, labels, stats, _ = cv2.connectedComponentsWithStats(m, 8)
        min_px = C.MIN_LINE_AREA_MM2 / self.res ** 2
        keep = np.zeros(n, bool)
        keep[1:] = stats[1:, cv2.CC_STAT_AREA] >= min_px
        return np.where(keep[labels], 255, 0).astype(np.uint8)

    def open_edge_points(self):
        """World xy (mm) of sheet-boundary pixels that touch unmapped area,
        i.e. where the sheet continues outside what has been seen so far."""
        paper = self.paper_region(erode=False)
        if not np.any(paper):
            return np.zeros((0, 2))
        unseen = cv2.dilate((~self.seen_mask()).astype(np.uint8) * 255, np.ones((5, 5), np.uint8))
        edge = cv2.morphologyEx(paper, cv2.MORPH_GRADIENT, np.ones((3, 3), np.uint8))
        return self.to_world(np.argwhere((edge > 0) & (unseen > 0)))

    def paper_info(self):
        """Centre (mm), size (mm, sorted), angle and whether it touches unseen area."""
        paper = self.paper_region(erode=False)             # true size, not the eroded one
        pts = np.argwhere(paper > 0)
        if len(pts) < 50:
            return None
        xy = self.to_world(pts).astype(np.float32)
        (cx, cy), (w, h), ang = cv2.minAreaRect(xy)
        unseen = cv2.dilate((~self.seen_mask()).astype(np.uint8) * 255, np.ones((5, 5), np.uint8))
        edge = cv2.morphologyEx(paper, cv2.MORPH_GRADIENT, np.ones((3, 3), np.uint8))
        complete = not np.any((edge > 0) & (unseen > 0))

        # A side length is trustworthy when BOTH opposite edges of the sheet were
        # observed (boundary pixels not next to unseen area exist near both ends
        # of that axis), even if the corners lie outside the mapped area.
        a = np.radians(ang)
        axes = np.array([[np.cos(a), np.sin(a)], [-np.sin(a), np.cos(a)]])   # along w, along h
        real_edge = self.to_world(np.argwhere((edge > 0) & (unseen == 0))) - (cx, cy)
        valid, sizes = [], []
        for axis, size in zip(axes, (w, h)):
            proj = real_edge @ axis if len(real_edge) else np.zeros(0)
            lo, hi = proj[proj < -0.45 * size], proj[proj > 0.45 * size]
            ok = len(lo) >= 10 and len(hi) >= 10
            valid.append(ok)
            # distance between the two observed edges: unlike minAreaRect it is
            # not biased when a corner of the sheet lies outside the mapped area
            sizes.append(float(np.median(hi) - np.median(lo)) if ok else float(size))
        order = np.argsort(sizes)                                   # short side first
        return dict(center=np.array([cx, cy]), size=tuple(np.array(sizes)[order]),
                    valid=tuple(np.array(valid)[order]), angle=ang,
                    complete=complete, bbox=(xy.min(0), xy.max(0)))

    DISPLAY_SIZE = 600.0

    def _disp_scale(self):
        return self.DISPLAY_SIZE / max(self.shape)

    def to_display(self, xy):
        """World xy -> display pixel. Display: +x up, +y left (behind the robot)."""
        rc = self.to_grid(xy)
        s = self._disp_scale()
        return np.round(np.column_stack(((self.shape[0] - 1 - rc[:, 0]) * s,
                                         (self.shape[1] - 1 - rc[:, 1]) * s))).astype(np.int32)

    def render(self, path_xy=None, dot=None):
        with np.errstate(invalid="ignore", divide="ignore"):
            g = np.where(self.seen > 0, self.gray_sum / np.maximum(self.seen, 1), 40)
        img = cv2.cvtColor(np.clip(g, 0, 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
        img[self.ink() > 0] = (0, 0, 255)
        img = np.ascontiguousarray(cv2.transpose(img)[::-1, ::-1])
        s = self._disp_scale()
        img = cv2.resize(img, (int(round(self.shape[0] * s)), int(round(self.shape[1] * s))),
                         interpolation=cv2.INTER_AREA)
        if path_xy is not None and len(path_xy) > 1:
            cv2.polylines(img, [self.to_display(path_xy)], False, (0, 200, 0), 2)
        if dot is not None:
            cv2.circle(img, tuple(self.to_display([dot[:2]])[0]),
                       max(3, int(dot[2] / self.res * s)), (255, 120, 0), 2)
        cv2.drawMarker(img, tuple(self.to_display([(0.0, 0.0)])[0]), (0, 255, 255),
                       cv2.MARKER_TILTED_CROSS, 20, 2)
        return img

    def draw_progress(self, base_img, xy):
        img = base_img.copy()
        cv2.circle(img, tuple(self.to_display([xy])[0]), 5, (255, 0, 255), -1)
        return img


# ---------------------------------------------------------------------------
# Line and start dot
# ---------------------------------------------------------------------------
def thin(mask):
    """Zhang-Suen thinning (vectorised NumPy). mask: uint8 0/255."""
    img = (mask > 0).astype(np.uint8)
    img = np.pad(img, 1)
    while True:
        changed = False
        for step in (0, 1):
            P = img
            p2, p3, p4 = P[:-2, 1:-1], P[:-2, 2:], P[1:-1, 2:]
            p5, p6, p7 = P[2:, 2:], P[2:, 1:-1], P[2:, :-2]
            p8, p9 = P[1:-1, :-2], P[:-2, :-2]
            nb = [p2, p3, p4, p5, p6, p7, p8, p9]
            B = sum(n.astype(np.int32) for n in nb)
            seq = nb + [p2]
            A = sum(((seq[i] == 0) & (seq[i + 1] == 1)).astype(np.int32) for i in range(8))
            if step == 0:
                c1, c2 = p2 * p4 * p6, p4 * p6 * p8
            else:
                c1, c2 = p2 * p4 * p8, p2 * p6 * p8
            core = P[1:-1, 1:-1]
            delete = (core == 1) & (B >= 2) & (B <= 6) & (A == 1) & (c1 == 0) & (c2 == 0)
            if np.any(delete):
                core[delete] = 0
                changed = True
        if not changed:
            break
    return (img[1:-1, 1:-1] * 255).astype(np.uint8)


NEIGHBOURS = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


def _bfs(skel, start):
    """Geodesic BFS on skeleton pixels. Returns (parent dict, farthest pixel)."""
    h, w = skel.shape
    parent = {start: None}
    dist = {start: 0.0}
    q = deque([start])
    far = start
    while q:
        r, c = q.popleft()
        for dr, dc in NEIGHBOURS:
            rr, cc = r + dr, c + dc
            if 0 <= rr < h and 0 <= cc < w and skel[rr, cc] and (rr, cc) not in parent:
                parent[(rr, cc)] = (r, c)
                dist[(rr, cc)] = dist[(r, c)] + (1.4142 if dr and dc else 1.0)
                if dist[(rr, cc)] > dist[far]:
                    far = (rr, cc)
                q.append((rr, cc))
    return parent, far, dist[far]


def smooth_resample(xy, window_mm, step_mm):
    xy = np.asarray(xy, float)
    seg = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    s = np.concatenate(([0.0], np.cumsum(seg)))
    if s[-1] < step_mm:
        return xy[[0, -1]]
    # resample uniformly first so the moving-average window is in mm
    s_u = np.arange(0.0, s[-1], 0.5)
    u = np.column_stack([np.interp(s_u, s, xy[:, k]) for k in range(2)])
    k = max(1, int(round(window_mm / 0.5)))
    if k > 1 and len(u) > 2 * k:
        pad = np.vstack([np.repeat(u[:1], k, 0), u, np.repeat(u[-1:], k, 0)])
        ker = np.ones(2 * k + 1) / (2 * k + 1)
        u = np.column_stack([np.convolve(pad[:, j], ker, "valid") for j in range(2)])
        u[0], u[-1] = xy[0], xy[-1]                     # keep the true end points
    seg = np.linalg.norm(np.diff(u, axis=0), axis=1)
    s = np.concatenate(([0.0], np.cumsum(seg)))
    s_new = np.append(np.arange(0.0, s[-1], step_mm), s[-1])
    return np.column_stack([np.interp(s_new, s, u[:, j]) for j in range(2)])


def extract_line(mosaic):
    """Find the black line and its start dot in the map (all lengths in mm).

    line     = the ink mark with the LONGEST skeleton, so stray marks, shadows
               or smudges do not win just because they are dark or thick;
    ends     = the two ends of the longest path along that skeleton;
    start    = the end that is clearly THICKER than the other (the dot). No
               absolute dot size is needed, only DOT_LINE_RATIO;
    complete = both ends, including the whole dot, lie inside what the camera
               has seen, i.e. the line is not cut off at the edge of a view.
    """
    ink_full = mosaic.ink()
    if not np.any(ink_full):
        raise VisionError("no black ink found on the paper")
    res = mosaic.res

    # work on a crop around the ink (much faster than the whole map)
    pad = int(round(12.0 / res))
    ys, xs = np.nonzero(ink_full)
    y0, y1 = max(0, ys.min() - pad), min(ink_full.shape[0], ys.max() + pad + 1)
    x0, x1 = max(0, xs.min() - pad), min(ink_full.shape[1], xs.max() + pad + 1)
    k = max(3, int(round(C.LINE_GAP_MM / res)) | 1)            # bridge small gaps in the pen line
    ink = cv2.morphologyEx(ink_full[y0:y1, x0:x1], cv2.MORPH_CLOSE, np.ones((k, k), np.uint8))
    seen = mosaic.seen_mask()[y0:y1, x0:x1]
    # "near the edge of what was seen": 3 mm, plus the strip the paper mask trims off
    m = int(round((3.0 + C.PAPER_EDGE_ERODE_MM) / res))
    near_unseen = cv2.dilate((~seen).astype(np.uint8), np.ones((2 * m + 1, 2 * m + 1), np.uint8)) > 0

    dist = cv2.distanceTransform(ink, cv2.DIST_L2, 5) * res      # mm from each ink pixel to white
    skel = thin(ink) > 0
    n, labels = cv2.connectedComponents(ink, connectivity=8)

    # the line = the component with the longest skeleton
    lengths = np.bincount(labels[skel], minlength=n)
    lengths[0] = 0
    comp = int(np.argmax(lengths))
    if lengths[comp] * res < C.MIN_LINE_LENGTH_MM:
        raise VisionError(f"no line found (longest ink mark only {lengths[comp] * res:.0f} mm)")
    line_skel = skel & (labels == comp)
    half_width = float(np.median(dist[line_skel]))

    # its two ends = ends of the longest path along the skeleton (two BFS passes)
    first = tuple(np.argwhere(line_skel)[0])
    _, end_a, _ = _bfs(line_skel, first)
    parent, end_b, length_px = _bfs(line_skel, end_a)
    chain = []                                                  # end_b ... end_a
    node = end_b
    while node is not None:
        chain.append(node)
        node = parent[node]

    # the start dot = the clearly thicker end (ink of a separate dot nearby counts too)
    rad = int(np.ceil(max(6.0, 3.0 * half_width) / res))

    def thickest_near(rc):
        r, c = rc
        r0_, r1_ = max(0, r - rad), min(dist.shape[0], r + rad + 1)
        c0_, c1_ = max(0, c - rad), min(dist.shape[1], c + rad + 1)
        win = dist[r0_:r1_, c0_:c1_]
        i = np.unravel_index(np.argmax(win), win.shape)
        return float(win[i]), (r0_ + int(i[0]), c0_ + int(i[1]))

    t_a, c_a = thickest_near(end_a)
    t_b, c_b = thickest_near(end_b)
    if t_a >= t_b:
        dot_r, dot_rc, far_end, t_far = t_a, c_a, end_b, t_b
        chain = chain[::-1]                                     # dot (end_a) -> far end
    else:
        dot_r, dot_rc, far_end, t_far = t_b, c_b, end_a, t_a
    ratio = dot_r / max(t_far, half_width, 1e-6)

    # Is an end cut off at the edge of what has been seen? Checked on the ink
    # AROUND each end: the skeleton end itself stops ~half a line width short
    # of the end of the ink.
    def cut_near(rc, radius_mm):
        rr = int(np.ceil(radius_mm / res))
        win = near_unseen[max(0, rc[0] - rr):rc[0] + rr + 1, max(0, rc[1] - rr):rc[1] + rr + 1]
        ink_win = ink[max(0, rc[0] - rr):rc[0] + rr + 1, max(0, rc[1] - rr):rc[1] + rr + 1] > 0
        return bool(np.any(win & ink_win))

    dot_cut = cut_near(dot_rc, dot_r + 1.0)
    far_cut = cut_near(far_end, half_width + 2.0)
    to_world = lambda rc: mosaic.to_world([(rc[0] + y0, rc[1] + x0)])[0]       # noqa: E731

    if ratio < C.DOT_LINE_RATIO or dot_r < C.DOT_MIN_RADIUS_MM:
        if not (dot_cut or far_cut):                  # whole line visible, no dot: a drawing problem
            raise VisionError(
                f"cannot tell which end is the start dot: the ends are {2 * dot_r:.1f} mm and "
                f"{2 * t_far:.1f} mm thick (line {2 * half_width:.1f} mm). Make the dot a filled "
                f"disc at least {max(2 * C.DOT_MIN_RADIUS_MM, 2 * C.DOT_LINE_RATIO * half_width):.0f} mm "
                "across.", retry=False)
        # one end is cut off and no visible end looks like a dot: the dot is most
        # likely the part that is not visible yet -> report where to look next
        cut_rc, other_rc = (far_end, dot_rc) if far_cut else (dot_rc, far_end)
        cut_xy, other_xy = to_world(cut_rc), to_world(other_rc)
        return dict(path=None, dot=np.array([other_xy[0], other_xy[1], 0.0]),
                    length_mm=length_px * res, half_width_mm=half_width,
                    end_at_unseen=True, end_xy=cut_xy, complete=False,
                    why=(f"the line runs out of the camera view near ({cut_xy[0]:.0f}, {cut_xy[1]:.0f}) mm "
                         "and the start dot is not visible yet"),
                    open_xy=cut_xy, open_is_dot=False)

    dot_xy = to_world(dot_rc)
    xy = np.vstack([dot_xy, mosaic.to_world([(r + y0, c + x0) for r, c in chain])])
    path = smooth_resample(xy, C.PATH_SMOOTH_MM, C.PATH_STEP_MM)
    far_xy = to_world(far_end)
    if dot_cut:
        why = "the start dot is cut off at the edge of the camera view"
    elif far_cut:
        why = f"the line runs out of the camera view near ({far_xy[0]:.0f}, {far_xy[1]:.0f}) mm"
    else:
        why = ""
    return dict(path=path, dot=np.array([dot_xy[0], dot_xy[1], dot_r]),
                length_mm=length_px * res, half_width_mm=half_width,
                end_at_unseen=far_cut, end_xy=far_xy,
                complete=not (dot_cut or far_cut), why=why,
                open_xy=dot_xy if dot_cut else (far_xy if far_cut else None), open_is_dot=dot_cut)
