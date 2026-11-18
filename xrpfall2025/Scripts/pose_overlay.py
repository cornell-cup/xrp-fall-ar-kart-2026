"""
pose_overlay.py

Draws laptop_fusion's estimated robot pose on top of an overhead camera's view
of the track, to compare by eye how well the filter follows the real robot.

What's drawn:
  - see-through ellipse   where the filter thinks the robot is. Its size is the
                          position uncertainty: SIGMA standard deviations (2 = the
                          robot should be inside it ~86% of the time). It grows when
                          the filter is unsure and shrinks as tags correct it
  - solid dot and line    the estimate's centre and heading; the two thin lines
                          either side are the heading uncertainty (SIGMA std devs)
  - faded line            the estimate's recent path
  - hollow green square   the pose the latest AprilTag gives on its own, while fresh
  - NIS table (bottom left)  a check of the filter's tuning, per sensor, over the
                          last NIS_WINDOW_S seconds:
        avg      the average NIS (normalized innovation squared: y^T S^-1 y for
                 each correction). With the right noise values it averages to
                 the number of values measured (dof).
        95% band the range the average should fall in 95% of the time if the
                 filter is right. Green = inside. Red = above: the sensor
                 disagrees with the filter more than it expects, so the filter
                 is overconfident (its noise values are too small). Yellow =
                 below: it's too cautious (noise values too big).
        >95%     the share of single corrections above the 95% line; about 5%
                 if the filter is right.

laptop_fusion.py sends the estimate with its covariance to UDP 6005
(DEBUG_POSE_PORT), so run fusion (or the game, which starts it) as usual and
this script alongside it. It needs numpy and opencv (the ar-kart environment
has both).

Usage:
    python Scripts/pose_overlay.py                  # camera 0
    python Scripts/pose_overlay.py --camera 1       # another USB camera
    python Scripts/pose_overlay.py --camera rtsp://...   # or a stream / video file
    python Scripts/pose_overlay.py --recalibrate    # click the floor points again
    python Scripts/pose_overlay.py --record run1.mp4

Calibration: the first time (or with --recalibrate, or key c), click each floor
point in CALIB_POINTS in the camera image, in the order shown: the floor under
each AprilTag's centre, then the measured points towards the ends of the track.
With 6 or more, the overhead camera's lens distortion is corrected too.
The result is saved to --calib (pose_overlay_calib.json) and reused, so keep
the camera still afterwards, or recalibrate if it moves.

Overhead camera delay: the overhead camera's picture arrives a little late, so
the robot in it is where the robot was a moment ago, while the estimate is
where fusion thinks it is now; moving, the estimate would be drawn ahead of the
robot. The estimate is drawn as it was --delay seconds ago instead (see
OVERHEAD_DELAY_S). To tune it, drive along a straight: if the estimate is drawn
ahead of the robot, raise it ( ] key ); behind, lower it ( [ key ). The value is
shown on screen.

Keys: q / Esc quit, c recalibrate, t trail on/off, r start/stop recording,
      [ / ] overhead camera delay -/+ 10 ms
"""

import argparse
import json
import math
import os
import socket
import threading
import time
from collections import deque

import numpy as np
import cv2

# ============================================================
# Config
# ============================================================

# Floor points with known coordinates in laptop_fusion's map frame (metres),
# clicked in the overhead image to line the two up: the floor under the centre of
# each AprilTag. Each is the middle of that tag's gate, the average of its two
# ends (x1/y1 and x2/y2 for each tag in laptop_fusion.py).
#
# Click the floor directly below each tag's centre, not the tag itself: the tags
# hang above the floor (tags 0 and 3 about 35 cm up), and from an overhead camera
# a raised point looks shifted away from the middle of the picture.
#
# The overhead camera's wide lens bends the picture (straight walls curve near
# the edges), which a flat perspective fit can't follow: drawn positions were
# off near the ends of the track. With 6 or more points, the overhead camera is
# fitted as a real camera with lens distortion instead (see fit_floor_model), so
# spread the points out, including at the ends. With fewer, the flat fit is used.
GATE_ENDS = {
    0: ((1.4144, -0.4572), (0.5856, -0.4572)),
    1: ((0.2708, 1.8247), (1.685, 0.4105)),
    2: ((0.69758178, -1.78660678), (-0.71663178, -0.37239322)),
    3: ((-1.0, 0.41275), (1.0, 0.41275)),
}
CALIB_POINTS = [
    ("floor under tag {}".format(tag), (a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
    for tag, (a, b) in sorted(GATE_ENDS.items())
] + [
    # other measured floor points, two towards each end of the track
    ("marked point (0.443, +0.640)", 0.44325, 0.6401),
    ("marked point (0.443, +1.085)", 0.44325, 1.0846),
    ("marked point (0.443, -0.678)", 0.44325, -0.6782),
    ("marked point (0.443, -1.123)", 0.44325, -1.1227),
    ("floor under tag 4", 0.29845, 1.1811),
    ("floor under tag 5", 0.71755, -1.21285),
]
MIN_CALIB_POINTS = 4
# With at least this many points the lens distortion is fitted too
MIN_LENS_POINTS = 6

DEBUG_POSE_PORT = 6005      # DEBUG_POSE_PORT in laptop_fusion.py

SIGMA = 2.0                 # ellipse and heading lines at this many standard deviations
ALPHA = 0.35                # how opaque the filled ellipse is (0 = invisible, 1 = solid)
MIN_DOT_PX = 6              # the filled area is never smaller than this, so a very
                            # certain estimate is still visible
HEADING_LEN_M = 0.12        # length of the heading line, in metres on the floor
TRAIL_S = 5.0               # how much of the recent path is drawn
STALE_S = 1.0               # an estimate older than this is drawn grey
# How late the overhead camera's picture is, in seconds: the estimate is drawn as
# it was this long ago, so it lines up with the robot in the picture (see the top
# of the file). ~0.15 s matched one run; adjust with [ and ]
OVERHEAD_DELAY_S = 0.15
HISTORY_S = 3.0             # how much past estimate is kept for the delay
NIS_WINDOW_S = 5.0          # the NIS table covers this many seconds
NIS_ORDER = ["encoder", "IMU", "camera"]   # table rows, in this order

# Colours (BGR)
EST_COLOR = (0, 165, 255)   # orange
TAG_COLOR = (0, 220, 0)     # green
STALE_COLOR = (150, 150, 150)
TEXT_COLOR = (255, 255, 255)
NIS_OK_COLOR = (80, 220, 80)      # green: average inside the 95% band
NIS_HIGH_COLOR = (80, 80, 255)    # red: above (filter overconfident)
NIS_LOW_COLOR = (0, 220, 255)     # yellow: below (filter too cautious)


# ============================================================
# Estimate receiver
# ============================================================
class PoseReceiver(threading.Thread):
    """Keeps the latest message from laptop_fusion, and the recent path."""

    def __init__(self, port):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(0.5)
        self.lock = threading.Lock()
        self.history = deque()  # (time received, message), for drawing with a delay
        self.trail = deque()   # (time, x, y)
        self.nis = {}          # sensor -> deque of (time, NIS)
        self.nis_dof = {}      # sensor -> degrees of freedom

    def run(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(4096)
                msg = json.loads(data.decode())
                now = time.time()
            except socket.timeout:
                continue
            except (ValueError, UnicodeDecodeError):
                continue
            with self.lock:
                self.history.append((now, msg))
                while self.history and now - self.history[0][0] > HISTORY_S:
                    self.history.popleft()
                self.trail.append((now, msg["x"], msg["y"]))
                while self.trail and now - self.trail[0][0] > TRAIL_S + HISTORY_S:
                    self.trail.popleft()
                for kind, entry in msg.get("nis", {}).items():
                    self.nis_dof[kind] = int(entry["dof"])
                    q = self.nis.setdefault(kind, deque())
                    q.extend((now, float(v)) for v in entry["values"])
                for q in self.nis.values():
                    while q and now - q[0][0] > NIS_WINDOW_S:
                        q.popleft()

    def get(self, delay=0.0):
        """The estimate as it was delay seconds ago (the newest one received by
        then), when it was received, and the path up to then."""
        cutoff = time.time() - delay
        with self.lock:
            msg, received_at = None, 0.0
            for t, m in reversed(self.history):
                if t <= cutoff:
                    msg, received_at = m, t
                    break
            if msg is None and self.history:
                # not that much history yet: the oldest there is
                received_at, msg = self.history[0]
            trail = [(x, y) for t, x, y in self.trail if cutoff - TRAIL_S <= t <= cutoff]
            return msg, received_at, trail

    def get_nis(self):
        """sensor -> (dof, [NIS values in the window])"""
        now = time.time()
        with self.lock:
            return {k: (self.nis_dof[k], [v for t, v in q if now - t <= NIS_WINDOW_S])
                    for k, q in self.nis.items()}


# ============================================================
# NIS statistics
# ============================================================
def chi2_quantile(p, k):
    """Approximate chi-square quantile (Wilson-Hilferty), k degrees of freedom.
    Within ~1% of the exact value for the sizes used here."""
    z = {0.025: -1.959964, 0.05: -1.644854, 0.95: 1.644854, 0.975: 1.959964}[p]
    a = 2.0 / (9.0 * k)
    return k * max(0.0, 1 - a + z * math.sqrt(a)) ** 3


def nis_stats(dof, values):
    """(average, band low, band high, share of single values above the 95% line)
    for NIS values with dof degrees of freedom. If the filter is right, the sum
    of n values is chi-square with n*dof degrees of freedom."""
    n = len(values)
    avg = sum(values) / n
    low = chi2_quantile(0.025, n * dof) / n
    high = chi2_quantile(0.975, n * dof) / n
    # one value alone: chi-square with dof; exact 95% points for small dof
    line = {1: 3.841, 2: 5.991, 3: 7.815}.get(dof) or chi2_quantile(0.95, dof)
    above = sum(v > line for v in values) / n
    return avg, low, high, above


# ============================================================
# Map frame <-> image
# ============================================================
class FloorCamera:
    """The overhead camera as a real camera: its position and angle over the
    floor, its focal length and its lens distortion (one radial term, k1). The
    floor is the z = 0 plane of the map frame."""

    def __init__(self, K, dist, rvec, tvec):
        self.K = np.asarray(K, np.float64)
        self.dist = np.asarray(dist, np.float64).ravel()
        self.rvec = np.asarray(rvec, np.float64).reshape(3, 1)
        self.tvec = np.asarray(tvec, np.float64).reshape(3, 1)

    def project(self, world_xy):
        xy = np.asarray(world_xy, np.float64).reshape(-1, 2)
        pts = np.hstack([xy, np.zeros((len(xy), 1))])
        img, _ = cv2.projectPoints(pts, self.rvec, self.tvec, self.K, self.dist)
        return img.reshape(-1, 2)

    def to_json(self):
        return {"K": self.K.tolist(), "dist": self.dist.tolist(),
                "rvec": self.rvec.ravel().tolist(), "tvec": self.tvec.ravel().tolist()}

    @classmethod
    def from_json(cls, d):
        return cls(d["K"], d["dist"], d["rvec"], d["tvec"])


def to_px(model, world_pts):
    """Map-frame points (metres) -> image pixels, through the floor model: a
    FloorCamera, or a plain floor homography (3x3 array)."""
    if isinstance(model, FloorCamera):
        return model.project(world_pts)
    pts = np.asarray(world_pts, np.float64).reshape(-1, 1, 2)
    return cv2.perspectiveTransform(pts, model).reshape(-1, 2)


def fit_floor_model(world, pixels, image_size):
    """Fits the floor -> image mapping to clicked points. Returns (model,
    per-point error in px, description). With MIN_LENS_POINTS or more, a
    FloorCamera with lens distortion if it fits better than a flat homography."""
    H, _ = cv2.findHomography(world, pixels, 0)
    if H is None:
        return None, None, None
    h_err = np.linalg.norm(to_px(H, world) - pixels, axis=1)
    if len(world) < MIN_LENS_POINTS:
        return H, h_err, "flat fit (add points for lens distortion)"

    # Camera fit: focal length (square pixels, optical centre in the middle of
    # the picture), position, angle, and one radial distortion term. 8 numbers,
    # so 6+ points (12+ values) pin it down, with some left over as a check
    w, h = image_size
    K = np.array([[0.6 * w, 0, w / 2], [0, 0.6 * w, h / 2], [0, 0, 1]], np.float64)
    obj = [np.hstack([world, np.zeros((len(world), 1))]).astype(np.float32)]
    img = [pixels.reshape(-1, 1, 2).astype(np.float32)]
    flags = (cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_PRINCIPAL_POINT |
             cv2.CALIB_FIX_ASPECT_RATIO | cv2.CALIB_ZERO_TANGENT_DIST |
             cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3)
    try:
        _, K, dist, rvecs, tvecs = cv2.calibrateCamera(obj, img, (w, h), K, np.zeros(5), flags=flags)
    except cv2.error as e:
        print("[Calib] Lens fit failed ({}); using the flat fit".format(e))
        return H, h_err, "flat fit (lens fit failed)"
    cam = FloorCamera(K, dist, rvecs[0], tvecs[0])
    c_err = np.linalg.norm(cam.project(world) - pixels, axis=1)
    print("[Calib] Flat fit: error mean {:.1f} px, max {:.1f} px | with lens distortion "
          "(k1 = {:+.3f}): mean {:.1f} px, max {:.1f} px".format(
              h_err.mean(), h_err.max(), cam.dist[0], c_err.mean(), c_err.max()))
    if c_err.mean() > h_err.mean():
        return H, h_err, "flat fit (fit better than the lens fit)"
    return cam, c_err, "lens distortion fit"


def ellipse_world(x, y, cov_xy, nsigma, n=48):
    """Points around the nsigma ellipse of a 2x2 position covariance."""
    c = np.asarray(cov_xy, np.float64)
    c = (c + c.T) / 2
    vals, vecs = np.linalg.eigh(c)
    vals = np.clip(vals, 0.0, None)
    t = np.linspace(0, 2 * math.pi, n, endpoint=False)
    circle = np.stack([np.cos(t), np.sin(t)])
    pts = vecs @ (np.sqrt(vals)[:, None] * nsigma * circle)
    return np.stack([x + pts[0], y + pts[1]], axis=1)


def ipt(p):
    return int(round(p[0])), int(round(p[1]))


# ============================================================
# Drawing
# ============================================================
def draw_text_lines(img, lines, origin=(10, 24)):
    x, y = origin
    for line in lines:
        (w, h), _ = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
        cv2.rectangle(img, (x - 4, y - h - 5), (x + w + 4, y + 6), (0, 0, 0), -1)
        cv2.putText(img, line, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, TEXT_COLOR, 1, cv2.LINE_AA)
        y += h + 12


def draw_estimate(frame, H, msg, age, trail, show_trail):
    stale = age > STALE_S
    color = STALE_COLOR if stale else EST_COLOR
    x, y, th = msg["x"], msg["y"], msg["theta"]
    cov = np.asarray(msg["cov"], np.float64)

    # recent path
    if show_trail and len(trail) >= 2:
        pts = to_px(H, trail).round().astype(np.int32)
        cv2.polylines(frame, [pts], False, color, 1, cv2.LINE_AA)

    # uncertainty ellipse, filled see-through
    centre = to_px(H, [(x, y)])[0]
    ring = to_px(H, ellipse_world(x, y, cov[:2, :2], SIGMA)).round().astype(np.int32)
    overlay = frame.copy()
    cv2.fillPoly(overlay, [ring], color, cv2.LINE_AA)
    cv2.circle(overlay, ipt(centre), MIN_DOT_PX, color, -1, cv2.LINE_AA)
    cv2.addWeighted(overlay, ALPHA, frame, 1 - ALPHA, 0, dst=frame)
    cv2.polylines(frame, [ring], True, color, 1, cv2.LINE_AA)

    # centre and heading, with the heading's uncertainty either side
    cv2.circle(frame, ipt(centre), 3, color, -1, cv2.LINE_AA)
    sd_th = math.sqrt(max(cov[2, 2], 0.0))
    for d, width in ((0.0, 2), (SIGMA * sd_th, 1), (-SIGMA * sd_th, 1)):
        a = th + d
        tip = to_px(H, [(x + HEADING_LEN_M * math.cos(a), y + HEADING_LEN_M * math.sin(a))])[0]
        cv2.line(frame, ipt(centre), ipt(tip), color, width, cv2.LINE_AA)

    # the latest tag's own estimate
    tag = msg.get("tag")
    if tag is not None:
        tp = to_px(H, [(tag["x"], tag["y"])])[0]
        tip = to_px(H, [(tag["x"] + HEADING_LEN_M * math.cos(tag["theta"]),
                         tag["y"] + HEADING_LEN_M * math.sin(tag["theta"]))])[0]
        cx, cy = ipt(tp)
        cv2.rectangle(frame, (cx - 7, cy - 7), (cx + 7, cy + 7), TAG_COLOR, 2, cv2.LINE_AA)
        cv2.line(frame, (cx, cy), ipt(tip), TAG_COLOR, 1, cv2.LINE_AA)

    sd_x, sd_y = math.sqrt(max(cov[0, 0], 0.0)), math.sqrt(max(cov[1, 1], 0.0))
    lines = [
        "EKF  x={:+.3f} y={:+.3f} m  th={:+.1f} deg  v={:+.2f} m/s".format(
            x, y, math.degrees(th), msg.get("v", 0.0)),
        "1 sd  x={:.3f} y={:.3f} m  th={:.1f} deg   (drawn at {:g} sd)".format(
            sd_x, sd_y, math.degrees(sd_th), SIGMA),
    ]
    if tag is not None:
        lines.append("tag {}  x={:+.3f} y={:+.3f} m  th={:+.1f} deg".format(
            tag.get("id", "?"), tag["x"], tag["y"], math.degrees(tag["theta"])))
    if stale:
        lines.append("estimate STALE ({:.1f} s old)".format(age))
    return lines


def draw_nis_table(img, nis):
    """The NIS table, in the bottom left corner."""
    font, scale = cv2.FONT_HERSHEY_SIMPLEX, 0.5
    cols = [0, 130, 200, 275, 395]   # x of each column, from the table's left edge
    header = ["NIS last {:g} s".format(NIS_WINDOW_S), "avg", "dof", "95% band", ">95%"]
    rows = []
    kinds = [k for k in NIS_ORDER if k in nis] + sorted(k for k in nis if k not in NIS_ORDER)
    for kind in kinds:
        dof, values = nis[kind]
        if not values:
            rows.append(([kind + " (none)", "-", str(dof), "-", "-"], TEXT_COLOR))
            continue
        avg, low, high, above = nis_stats(dof, values)
        color = NIS_HIGH_COLOR if avg > high else NIS_LOW_COLOR if avg < low else NIS_OK_COLOR
        rows.append(([
            "{} ({})".format(kind, len(values)),
            "{:.2f}".format(avg),
            str(dof),
            "{:.2f}-{:.2f}".format(low, high),
            "{:.0f}%".format(100 * above),
        ], color))
    if not rows:
        rows.append((["no corrections yet", "", "", "", ""], TEXT_COLOR))

    line_h = 20
    width = cols[-1] + 60
    height = line_h * (len(rows) + 1) + 10
    x0 = 10
    y0 = img.shape[0] - 10 - height
    cv2.rectangle(img, (x0 - 4, y0), (x0 + width, y0 + height), (0, 0, 0), -1)
    for r, (cells, color) in enumerate([(header, TEXT_COLOR)] + rows):
        y = y0 + line_h * (r + 1)
        for c, text in enumerate(cells):
            cell_color = color if (c == 1 or r == 0 or c == 0) else TEXT_COLOR
            cv2.putText(img, text, (x0 + cols[c], y), font, scale, cell_color, 1, cv2.LINE_AA)


# ============================================================
# Calibration
# ============================================================
def calibrate(cap, path, window):
    """Has the user click CALIB_POINTS in the live image, then fits and saves
    the floor homography. Returns it, or None if cancelled."""
    clicked = [None] * len(CALIB_POINTS)
    state = {"i": 0}

    def on_mouse(event, u, v, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and state["i"] < len(CALIB_POINTS):
            clicked[state["i"]] = (float(u), float(v))
            state["i"] += 1

    cv2.setMouseCallback(window, on_mouse)
    frame = None
    try:
        while True:
            ok, new = cap.read()
            if ok:
                frame = new
            if frame is None:
                if cv2.waitKey(30) & 0xFF in (27, ord("q")):
                    return None
                continue
            show = frame.copy()
            for (name, wx, wy), p in zip(CALIB_POINTS, clicked):
                if p is not None:
                    cv2.drawMarker(show, ipt(p), (0, 255, 255), cv2.MARKER_CROSS, 16, 2)
                    cv2.putText(show, name, (ipt(p)[0] + 8, ipt(p)[1] - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)
            n_done = sum(p is not None for p in clicked)
            if state["i"] < len(CALIB_POINTS):
                name, wx, wy = CALIB_POINTS[state["i"]]
                prompt = "Click: {} ({:+.2f}, {:+.2f} m)   [{}/{}]".format(
                    name, wx, wy, state["i"] + 1, len(CALIB_POINTS))
            else:
                prompt = "All points done"
            draw_text_lines(show, [
                "CALIBRATION - " + prompt,
                "s skip   u undo   Enter finish ({} clicked, need {})   Esc cancel".format(
                    n_done, MIN_CALIB_POINTS),
            ])
            cv2.imshow(window, show)
            key = cv2.waitKey(30) & 0xFF
            # clicks arrive during waitKey, so count them again now
            n_done = sum(p is not None for p in clicked)
            if key == 27:
                return None
            if key == ord("s") and state["i"] < len(CALIB_POINTS):
                state["i"] += 1
            elif key == ord("u") and state["i"] > 0:
                state["i"] -= 1
                clicked[state["i"]] = None
            elif key in (13, 10) or state["i"] >= len(CALIB_POINTS):
                if n_done >= MIN_CALIB_POINTS:
                    break
                if state["i"] >= len(CALIB_POINTS):
                    print("[Calib] Only {} points clicked, need {}; starting over".format(
                        n_done, MIN_CALIB_POINTS))
                    clicked[:] = [None] * len(CALIB_POINTS)
                    state["i"] = 0
    finally:
        cv2.setMouseCallback(window, lambda *a: None)

    used = [(c, p) for c, p in zip(CALIB_POINTS, clicked) if p is not None]
    world = np.array([[c[1], c[2]] for c, _ in used], np.float64)
    pixels = np.array([p for _, p in used], np.float64)
    h, w = frame.shape[:2]
    model, err, kind = fit_floor_model(world, pixels, (w, h))
    if model is None:
        print("[Calib] Couldn't fit the points (are 3 of them in a line?); try again")
        return None

    print("[Calib] {} points, {}: error mean {:.1f} px, max {:.1f} px".format(
        len(used), kind, err.mean(), err.max()))
    for (c, p), e in zip(used, err):
        if e > max(5.0, 3 * err.mean()):
            print("[Calib]   {} is {:.0f} px off: re-click it if that seems wrong".format(c[0], e))
    data = {
        "image_size": [w, h],
        "points": [{"name": c[0], "world": [c[1], c[2]], "pixel": list(p)} for c, p in used],
    }
    if isinstance(model, FloorCamera):
        data["camera"] = model.to_json()
    else:
        data["H"] = model.tolist()
    with open(path, "w") as f:
        json.dump(data, f, indent=2)
    print("[Calib] Saved to " + path)
    return model


def load_calibration(path, frame_size):
    try:
        with open(path) as f:
            data = json.load(f)
    except (OSError, ValueError):
        return None
    if tuple(data.get("image_size", frame_size)) != tuple(frame_size):
        print("[Calib] {} was made at {}x{}, but the camera gives {}x{}; recalibrate (c)".format(
            path, *data["image_size"], *frame_size))
    print("[Calib] Loaded " + path)
    if "camera" in data:
        return FloorCamera.from_json(data["camera"])
    if len(data.get("points", [])) < len(CALIB_POINTS):
        print("[Calib] It has {} of the {} calibration points; press c to recalibrate with "
              "all of them (corrects for the lens)".format(len(data.get("points", [])), len(CALIB_POINTS)))
    return np.array(data["H"], np.float64)


# ============================================================
# Main
# ============================================================
def open_camera(source):
    cap = cv2.VideoCapture(int(source) if source.isdigit() else source)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def main():
    ap = argparse.ArgumentParser(description="Draw the EKF estimate over an overhead camera view")
    ap.add_argument("--camera", default="0", help="camera index, stream URL or video file (default 0)")
    ap.add_argument("--port", type=int, default=DEBUG_POSE_PORT, help="UDP port fusion sends to")
    ap.add_argument("--calib", default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                    "pose_overlay_calib.json"),
                    help="calibration file (default: next to this script)")
    ap.add_argument("--recalibrate", action="store_true", help="click the floor points again")
    ap.add_argument("--record", help="also save the overlaid video to this .mp4 file")
    ap.add_argument("--delay", type=float, default=OVERHEAD_DELAY_S,
                    help="overhead camera delay in seconds (default %(default)s; [ and ] adjust it)")
    args = ap.parse_args()

    cap = open_camera(args.camera)
    ok, frame = cap.read()
    if not ok:
        raise SystemExit("Can't read from camera '{}'".format(args.camera))
    h, w = frame.shape[:2]

    window = "EKF pose overlay"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, w, h)

    H = None if args.recalibrate else load_calibration(args.calib, (w, h))
    if H is None:
        H = calibrate(cap, args.calib, window)
        if H is None:
            raise SystemExit("Calibration cancelled")

    receiver = PoseReceiver(args.port)
    receiver.start()
    print("[Overlay] Listening for the estimate on UDP {}".format(args.port))

    writer = None
    record_path = args.record
    show_trail = True
    delay = max(0.0, args.delay)
    # When recent frames arrived. The recording's frame rate is measured from
    # these: cameras (on macOS especially) often report a wrong rate, and the
    # video would then play too fast or too slow
    frame_times = deque(maxlen=60)
    # --record: start once the frame rate has been measured
    record_pending = record_path

    def start_recording(p):
        if len(frame_times) >= 2 and frame_times[-1] > frame_times[0]:
            fps = (len(frame_times) - 1) / (frame_times[-1] - frame_times[0])
        else:
            fps = 30.0
        print("[Overlay] Recording to {} at {:.1f} fps (measured)".format(p, fps))
        return cv2.VideoWriter(p, cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))

    while True:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.01)
            if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                break
            continue
        frame_times.append(time.time())
        if record_pending and len(frame_times) >= 30:
            writer = start_recording(record_pending)
            record_pending = None

        msg, received_at, trail = receiver.get(delay)
        if msg is None:
            lines = ["Waiting for the estimate on UDP {} (is fusion running?)".format(args.port)]
        else:
            # age compared with the moment being drawn, so the delay doesn't count as stale
            lines = draw_estimate(frame, H, msg, time.time() - delay - received_at, trail, show_trail)
        lines.append("drawn {:.0f} ms late to match the overhead camera ([ / ] to adjust)".format(delay * 1000))
        if writer is not None:
            lines.append("REC")
        draw_text_lines(frame, lines)
        draw_nis_table(frame, receiver.get_nis())

        if writer is not None:
            writer.write(frame)
        cv2.imshow(window, frame)

        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q")):
            break
        if key == ord("t"):
            show_trail = not show_trail
        elif key == ord("["):
            delay = max(0.0, delay - 0.01)
        elif key == ord("]"):
            delay = min(HISTORY_S - 0.1, delay + 0.01)
        elif key == ord("c"):
            new_H = calibrate(cap, args.calib, window)
            if new_H is not None:
                H = new_H
        elif key == ord("r"):
            if writer is None:
                writer = start_recording(time.strftime("pose_overlay_%Y%m%d_%H%M%S.mp4"))
                record_pending = None
            else:
                writer.release()
                writer = None
                print("[Overlay] Recording stopped")

    if writer is not None:
        writer.release()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()