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

import math
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


def paper_threshold(hsv):
    """Minimum brightness (HSV V) of paper, adapted to the lighting.

    The paper is grey in a dim room (V ~ 130) and the table can be as bright
    as 130 in a bright one, so no fixed value works. The black ink, the text
    and deep shadows are removed first (else the split is "ink / everything
    else" and the table counts as paper). Then Otsu's threshold splits the
    rest into a darker group (table) and a brighter one (paper). If the two
    groups are not clearly different, only paper (or only table) is in view
    and PAPER_ONLY_RATIO x the paper brightness is used."""
    v = hsv[..., 2][hsv[..., 1] <= C.PAPER_MAX_SATURATION]
    if v.size < 100:
        return C.PAPER_MIN_VALUE
    bright = float(np.percentile(v, 95))
    v = v[v > 0.6 * bright]                                # drop ink, text, deep shadows
    if v.size < 100:
        return C.PAPER_MIN_VALUE
    t, _ = cv2.threshold(v.reshape(-1, 1).astype(np.uint8), 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    lo, hi = v[v <= t], v[v > t]
    two_groups = (lo.size > 0.05 * v.size and hi.size > 0.05 * v.size
                  and float(hi.mean()) - float(lo.mean()) >= C.PAPER_TABLE_CONTRAST)
    thr = t + 1 if two_groups else C.PAPER_ONLY_RATIO * bright
    return int(max(C.PAPER_MIN_VALUE_FLOOR, min(thr, C.PAPER_RELATIVE_VALUE * bright)))


def _paper_mask_edges(frame, hsv):
    """Sheet = the bright region enclosed by SHARP brightness edges.

    Shading across the sheet (one side 25 % darker than the other) can make
    paper as dark as the table, so no brightness threshold separates them.
    But the sheet border is a sudden step in brightness, while shading changes
    slowly. Canny edges (blurred image) mark the border; the low-saturation,
    not-too-dark pixels are split into regions along those edges, and the
    brightest large region - plus other large regions almost as bright (parts
    of the sheet separated by the black line) - is the sheet."""
    V, S = hsv[..., 2], hsv[..., 1]
    low = S <= C.PAPER_MAX_SATURATION
    if np.count_nonzero(low) < 100:
        return None
    bright = float(np.percentile(V[low], 95))
    cand = low & (V >= max(C.PAPER_MIN_VALUE_FLOOR, C.PAPER_LENIENT_RATIO * bright))
    edges = cv2.Canny(cv2.GaussianBlur(V, (5, 5), 0), *C.PAPER_EDGE_CANNY)
    free = (cand & (cv2.dilate(edges, np.ones((3, 3), np.uint8)) == 0)).astype(np.uint8)
    n, lab, st, _ = cv2.connectedComponentsWithStats(free, 4)
    big = [i for i in range(1, n) if st[i, cv2.CC_STAT_AREA] >= 0.005 * free.size]
    if not big:
        return None
    med = {i: float(np.median(V[lab == i])) for i in big}
    top = max(big, key=lambda i: med[i] * st[i, cv2.CC_STAT_AREA] ** 0.25)
    keep = [i for i in big if med[i] >= C.PAPER_SAME_RATIO * med[top]]
    return np.isin(lab, keep).astype(np.uint8) * 255


def paper_mask(frame):
    """Mask of the A4 sheet in the image and its area fraction.

    The sheet region comes from _paper_mask_edges() (robust to shading); if
    that finds nothing, from a brightness threshold (paper_threshold). A thick
    pen line or a crease cuts the white area into pieces, so dark gaps up to
    ~5 % of the image width are bridged, then all large white pieces are kept
    and their convex hull is used (the sheet is a rectangle, which stays
    convex in a perspective image).
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    white = _paper_mask_edges(frame, hsv)
    if white is None:
        white = cv2.inRange(hsv, (0, 0, paper_threshold(hsv)), (180, C.PAPER_MAX_SATURATION, 255))
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
        self.aligns = []                                  # alignment used for every view (or None)
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
        frames, aligns = self.frames, getattr(self, "aligns", [None] * len(self.frames))
        self.frames, self.aligns = [], []
        self.reset(z_plane)
        for (frame, T0c), align in zip(frames, aligns):
            self.add_view(frame, T0c, align)

    # grid <-> world
    def to_world(self, rc):
        rc = np.asarray(rc, float)
        return np.column_stack((self.xs[0] + rc[:, 1] * self.res,
                                self.ys[0] + rc[:, 0] * self.res))

    def to_grid(self, xy):
        xy = np.atleast_2d(np.asarray(xy, float))
        return np.column_stack(((xy[:, 1] - self.ys[0]) / self.res,
                                (xy[:, 0] - self.xs[0]) / self.res))

    def add_view(self, frame, T0c, align=None):
        """Project one camera image onto the table map.

        align = (R, t): the view is first moved by the rigid 2-D transform
        p -> R p + t (mm, frame 0), found by align_view(), so that its line
        lies on the line of the views already in the map. Small calibration
        errors otherwise place the same line a few mm apart in different
        views and the combined map shows it doubled."""
        self.frames.append((frame, np.array(T0c, float)))
        if not hasattr(self, "aligns"):
            self.aligns = []
        self.aligns.append(align)
        P = self.P
        if align is not None:
            R, t = align
            xy = (self.XY - t) @ R                        # R^T (p - t): the same point before the move
            P = np.column_stack((xy, np.full(len(xy), self.z_plane)))
        uv, depth = self.cam.project(T0c, P)
        b = C.IMAGE_BORDER_PX
        inside = ((depth > 1) & (uv[:, 0] >= b) & (uv[:, 0] <= self.cam.width - 1 - b)
                  & (uv[:, 1] >= b) & (uv[:, 1] <= self.cam.height - 1 - b))
        inside = inside.reshape(self.shape)
        if not np.any(inside):
            return
        mx = np.nan_to_num(uv[:, 0], nan=-1).reshape(self.shape).astype(np.float32)
        my = np.nan_to_num(uv[:, 1], nan=-1).reshape(self.shape).astype(np.float32)
        # the black corners/edges left by the lens undistortion are not seen
        # (cam.valid: computed once from the calibration, NOT from the image,
        # whose black ink can be pure black too)
        valid = getattr(self.cam, "valid", None)
        if valid is not None:
            inside &= cv2.remap(valid.astype(np.float32), mx, my, cv2.INTER_NEAREST) > 0.5

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
        # shrink at the REAL edges of the sheet (shadows), but not where the map
        # just ends because the camera did not see further: a line that runs out
        # of the picture must keep its ink right up to the edge of the view
        e = max(1, int(round(C.PAPER_EDGE_ERODE_MM / self.res)))
        k = np.ones((2 * e + 1, 2 * e + 1), np.uint8)
        eroded = cv2.erode(m, k)
        view_edge = cv2.dilate((~self.seen_mask()).astype(np.uint8), np.ones((4 * e + 1, 4 * e + 1), np.uint8)) > 0
        far_from_sheet_edge = cv2.erode(cv2.bitwise_or(m, np.where(view_edge, 255, 0).astype(np.uint8)), k) > 0
        return np.where(view_edge & far_from_sheet_edge & (m > 0), 255, eroded).astype(np.uint8)

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
    # "near the edge of what was seen": the strip the paper mask trims off
    # (ink of a line that continues stops there) + 1 mm for blur. A larger
    # margin wrongly calls a line "cut" whose end is just inside the picture.
    m = int(round(C.CUT_MARGIN_MM / res)) + 1             # ink this close to the edge of the view = cut
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

    # windows big enough for a line cut off at a slant (its skeleton stops well
    # before the end of the ink); the MARGIN to the view edge stays small
    dot_cut = cut_near(dot_rc, max(dot_r + 1.0, 2.0 * half_width + 2.0))
    far_cut = cut_near(far_end, 2.0 * half_width + 2.0)
    to_world = lambda rc: mosaic.to_world([(rc[0] + y0, rc[1] + x0)])[0]       # noqa: E731

    # The seen line in world mm (thick end first) and, for every end that is cut
    # off at the edge of the view, where it leaves and in which direction: used
    # to plan a camera view that shows the WHOLE line.
    chain_xy = mosaic.to_world([(r + y0, c + x0) for r, c in chain])
    k_back = min(len(chain_xy) - 1, int(round(10.0 / res)))
    cut_ends = []
    for is_cut, end_xy, back_xy in ((dot_cut, chain_xy[0], chain_xy[k_back]),
                                    (far_cut, chain_xy[-1], chain_xy[-1 - k_back])):
        if is_cut:
            d = end_xy - back_xy
            cut_ends.append((end_xy, d / max(np.linalg.norm(d), 1e-6)))
    seen = dict(points_xy=chain_xy[::4], cut_ends=cut_ends)

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
                    open_xy=cut_xy, open_is_dot=False, **seen)

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
                open_xy=dot_xy if dot_cut else (far_xy if far_cut else None), open_is_dot=dot_cut, **seen)


# ---------------------------------------------------------------------------
# Combining views: line up every view on the line itself
# ---------------------------------------------------------------------------
def _bilinear(img, rows, cols, outside):
    ok = (rows >= 0) & (cols >= 0) & (rows < img.shape[0] - 1) & (cols < img.shape[1] - 1)
    r = np.clip(rows, 0, img.shape[0] - 1.001)
    c = np.clip(cols, 0, img.shape[1] - 1.001)
    r0, c0 = r.astype(int), c.astype(int)
    a, b = r - r0, c - c0
    v = (img[r0, c0] * (1 - a) * (1 - b) + img[r0 + 1, c0] * a * (1 - b)
         + img[r0, c0 + 1] * (1 - a) * b + img[r0 + 1, c0 + 1] * a * b)
    return np.where(ok, v, outside)


def view_line_points(cam, frame, T0c, z_plane):
    """World xy (mm) of the skeleton of the ink seen in ONE view, and its ink area."""
    single = Mosaic(cam)
    single.reset(z_plane)
    single.add_view(frame, T0c)
    ink = single.ink()
    if not np.any(ink):
        return np.zeros((0, 2)), 0
    return single.to_world(np.argwhere(thin(ink) > 0)[::2]), int(np.count_nonzero(ink))


def align_view(target, pts, max_shift_mm=25.0, max_rot_deg=8.0, cutoff_mm=6.0):
    """Rigid 2-D move (R, t) that puts the line seen in one view (its skeleton
    points pts, from view_line_points) onto the line already in the map
    `target`, or None if they do not overlap enough.

    The skeleton points of the view's ink are matched against the distance
    map of the target's ink (chamfer matching, Gauss-Newton on rotation +
    shift). Only points inside the part the target has already seen count, and
    points more than cutoff_mm from any ink (no overlap, other marks) do not
    pull. Returns ((R, t), median distance before, after) or (None, ...)."""
    ink_t = target.ink()
    if len(pts) == 0 or not np.any(ink_t):
        return None, np.nan, np.nan
    seen_t = cv2.erode(target.seen_mask().astype(np.uint8), np.ones((11, 11), np.uint8)) > 0
    g = np.round(target.to_grid(pts)).astype(int)
    inb = (g[:, 0] >= 0) & (g[:, 1] >= 0) & (g[:, 0] < target.shape[0]) & (g[:, 1] < target.shape[1])
    pts, g = pts[inb], g[inb]
    A = pts[seen_t[g[:, 0], g[:, 1]]]
    if len(A) < 40:
        return None, np.nan, np.nan
    dist = cv2.distanceTransform(np.where(ink_t > 0, 0, 255).astype(np.uint8), cv2.DIST_L2, 5) * target.res
    c = A.mean(axis=0)

    def moved(p):
        th, tx, ty = p
        R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
        return (A - c) @ R.T + c + (tx, ty)

    def resid(p):
        g2 = target.to_grid(moved(p))
        return np.minimum(_bilinear(dist, g2[:, 0], g2[:, 1], cutoff_mm), cutoff_mm)

    p = np.zeros(3)
    r = resid(p)
    before = float(np.median(r))
    lam = 1e-3
    steps = np.array([math.radians(0.1), 0.1, 0.1])
    for _ in range(40):
        J = np.column_stack([(resid(p + np.eye(3)[i] * steps[i]) - r) / steps[i] for i in range(3)])
        A_ = J.T @ J
        g_ = J.T @ r
        improved = False
        for _ in range(8):
            dp = np.linalg.solve(A_ + lam * np.diag(np.diag(A_) + 1e-9), -g_)
            rn = resid(p + dp)
            if rn @ rn < r @ r:
                p, r, lam, improved = p + dp, rn, lam * 0.3, True
                break
            lam *= 10.0
        if not improved or np.max(np.abs(dp)) < 1e-4:
            break
    after = float(np.median(r))
    th, tx, ty = p
    if abs(math.degrees(th)) > max_rot_deg or math.hypot(tx, ty) > max_shift_mm or after > 0.6 * cutoff_mm:
        return None, before, after
    R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
    return (R, c - R @ c + (tx, ty)), before, after


def combine_aligned(cam, views, z_plane=None, log=print):
    """One map from several views (list of (frame, T0c)), each view lined up
    on the line of the views added before it (align_view). The view with the
    most ink is the reference; the others follow, best overlap first."""
    z_plane = C.PAPER_Z if z_plane is None else z_plane
    lines = [view_line_points(cam, frame, T0c, z_plane) for frame, T0c in views]
    order = [int(i) for i in np.argsort([area for _, area in lines])[::-1]]
    out = Mosaic(cam)
    out.reset(z_plane)
    first = order.pop(0)
    out.add_view(*views[first])
    while order:
        best = None
        for i in order:                                  # the view that overlaps the map best goes next
            al, before, after = align_view(out, lines[i][0])
            if al is not None and (best is None or after < best[3]):
                best = (i, al, before, after)
        if best is None:
            for i in order:
                log(f"  view {i + 1}: does not overlap the line of the other views - left out")
            break
        i, al, before, after = best
        R, t = al
        log(f"  view {i + 1}: moved by ({t[0]:.1f}, {t[1]:.1f}) mm, {math.degrees(math.atan2(R[1, 0], R[0, 0])):.1f} deg "
            f"to line up with the others (mismatch {before:.1f} -> {after:.1f} mm)")
        out.add_view(*views[i], align=al)
        order.remove(i)
    return out
