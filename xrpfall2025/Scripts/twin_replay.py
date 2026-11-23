"""
twin_replay.py

Replays a recording from twin_record.py, offline, with two things running on it:

  THE TWIN (magenta outline)   the recorded throttle and servo inputs run through
                               the dynamics model (car_model.py) on their own, no
                               sensors: where the model says the kart goes
  THE EKF  (orange, ellipse)   laptop_fusion's whole fusion (fusion_core.py, the
                               same code that runs live) re-run on the recorded
                               encoder, IMU and tag data, with your settings

both drawn over the overhead video, with:
  grey dot and path      the estimate fusion actually made live, as recorded
  green square           each tag pose, drawn at the moment its frame was taken
                         (arrival - CAMERA_LATENCY_S): if it trails or leads the
                         kart while driving, CAMERA_LATENCY_S is off
  NIS table              per sensor, for the re-run EKF (as in pose_overlay.py)

Two slider windows change the settings, and everything re-runs:
  "Model"  the dynamics model (shared by the twin and the EKF), plus the twin's
           resync interval and the overhead camera delay
  "EKF"    the EKF's noise values and the tag handling settings
Values are shown under the sliders. Sliders are coarse; for exact values edit
the params file (--params) and press r.

Resync: on its own the twin drifts further and further from the kart. With a
resync interval (slider, or --resync) it restarts from the EKF's estimate every N
seconds, so each stretch shows how far the model alone drifts in N seconds.
2-5 s is good for tuning; 0 = never.

The numbers (on screen, and with --metrics):
  twin   speed vs the encoders, heading vs the EKF (which follows the IMU),
         position vs the tags; RMS over the run, and at the end of each resync
         stretch (the drift over one stretch)
  EKF    position vs the tags (RMS), and NIS per sensor over the whole run: its
         average should be inside the 95% band (green)

Setup: put car_model.py, fusion_core.py and ackermann_simple_ekf.py next to
laptop_fusion.py (this looks for them in the folder above this one, or give
--fusion-dir), and this file next to pose_overlay.py. Needs numpy and opencv,
nothing online.

Usage:
    python Scripts/twin_replay.py runs/run_20261123_120000
    python Scripts/twin_replay.py RUN --resync 3
    python Scripts/twin_replay.py RUN --metrics                  # numbers only
    python Scripts/twin_replay.py RUN --export overlay.mp4       # render to a video
    python Scripts/twin_replay.py RUN --fit STEER_RATIO,SPEED_RESPONSE,XRP_MAX_RPM --resync 3 --save
    python Scripts/twin_replay.py RUN --from-run                 # start from the run's own settings

Keys:
    space  play / pause           a / d  one frame back / forward
    j / l  2 s back / forward     - / =  slower / faster
    [ / ]  overhead delay -/+ 10 ms      t  paths on / off   g  live estimate on / off
    s      save the settings (and print the lines for laptop_fusion.py)
    r      reload the settings file   e  export the overlay video   h  help
    q / Esc  quit
"""

import argparse
import csv
import json
import math
import os
import sys
import time

import numpy as np
import cv2

import pose_overlay as po

HERE = os.path.dirname(os.path.abspath(__file__))


def import_fusion(fusion_dir):
    """car_model and fusion_core live next to laptop_fusion.py."""
    candidates = [fusion_dir] if fusion_dir else [os.path.dirname(HERE), HERE, os.getcwd()]
    for d in candidates:
        if d and os.path.exists(os.path.join(d, "fusion_core.py")):
            sys.path.insert(0, d)
            break
    else:
        raise SystemExit("Can't find fusion_core.py (looked in {}); give --fusion-dir".format(", ".join(candidates)))
    global car_model,fusion_core
    import car_model
    import xrpfall2025.Scripts.fusion_core as fusion_core


# ============================================================
# Settings: what the sliders show, and their ranges
# ============================================================
# name -> (low, high, scale); scale: "lin", "log", "sign" (+1/-1) or "bool"
MODEL_SETTINGS = {
    "WHEELBASE":      (0.08, 0.25, "lin"),
    "STEER_RATIO":    (0.1, 1.0, "lin"),
    "STEER_SIGN":     (-1, 1, "sign"),
    "SPEED_RESPONSE": (1.0, 40.0, "log"),
    "XRP_MAX_RPM":    (50.0, 250.0, "lin"),
}
EKF_SETTINGS = {
    "MODEL_POS_VAR":       (1e-6, 1e-1, "log"),
    "MODEL_HEADING_VAR":   (1e-5, 1.0, "log"),
    "MODEL_SPEED_VAR":     (1e-3, 10.0, "log"),
    "ENCODER_SPEED_VAR":   (1e-6, 1e-1, "log"),
    "IMU_HEADING_VAR":     (1e-9, 1e-2, "log"),
    "CAMERA_POS_VAR":      (1e-5, 1e-1, "log"),
    "CAMERA_HEADING_VAR":  (1e-4, 1.0, "log"),
    "CAMERA_LATENCY_S":    (0.0, 0.6, "lin"),
    "TAG_HEADING_GAIN":    (0.0, 0.5, "lin"),
    "GYRO_SCALE":          (0.9, 1.1, "lin"),
    "MAX_TAG_DISTANCE_M":  (0.3, 3.0, "lin"),
    "TAG_NOISE_REF_M":     (0.2, 2.0, "lin"),
    "CAMERA_NIS_GATE":     (1.0, 1000.0, "log"),
    "APPLY_CAMERA_CORRECTION": (0, 1, "bool"),
}
SLIDER_STEPS = 1000

TWIN_COLOR = (255, 0, 255)        # magenta (BGR)
EKF_COLOR = po.EST_COLOR          # orange
LIVE_COLOR = (170, 170, 170)      # grey
TAG_COLOR = po.TAG_COLOR          # green
TRAIL_S = 3.0
TAG_SHOW_S = 0.2                  # a tag pose is drawn for this long after its frame was taken
TWIN_DT = 0.01                    # twin integration step, s


def fmt_value(v):
    if isinstance(v, bool):
        return str(v)
    if isinstance(v, int):
        return str(v)
    a = abs(v)
    return "{:.4g}".format(v) if (a == 0 or 1e-3 <= a < 1e4) else "{:.3e}".format(v)


# ============================================================
# A recording
# ============================================================
class Recording:
    def __init__(self, folder):
        self.folder = folder
        with open(os.path.join(folder, "meta.json")) as f:
            self.meta = json.load(f)
        self.video_path = os.path.join(folder, "video.mp4")
        with open(os.path.join(folder, "frames.csv")) as f:
            self.frame_t = np.array([float(r["t"]) for r in csv.DictReader(f)])
        cal = self.meta["calibration"]
        self.floor = po.FloorCamera.from_json(cal["camera"]) if "camera" in cal else np.array(cal["H"], np.float64)

        self.sensors = []        # (t, text, ip, tag_time)
        self.tags = []           # (published t, pose)
        live = []
        self.config = None
        with open(os.path.join(folder, "events.jsonl")) as f:
            for line in f:
                try:
                    ev = json.loads(line)
                except ValueError:
                    continue    # a cut-off last line
                k = ev.get("kind")
                if k == "sensor":
                    self.sensors.append((ev["t"], ev["text"], ev["ip"], ev.get("tag_time")))
                elif k == "tag":
                    self.tags.append((ev["t"], ev["pose"]))
                elif k == "fused":
                    live.append((ev["t"], ev["x"], ev["y"], ev["theta"], ev["v"]))
                elif k == "config" and self.config is None:
                    self.config = ev["config"]
        if not self.sensors:
            raise SystemExit("No sensor packets in {} (was laptop_fusion.py's TWIN_TAP on?)".format(folder))
        self.sensors.sort(key=lambda s: s[0])
        self.tags.sort(key=lambda s: s[0])
        self.tag_by_time = {p["time"]: p for _, p in self.tags}
        self.tag_pub_t = np.array([t for t, _ in self.tags])
        live.sort()
        self.live_t = np.array([r[0] for r in live])
        self.live = np.array([r[1:] for r in live]).reshape(-1, 4)

    def tag_for(self, t, tag_time):
        """The tag pose fusion used with a packet at time t."""
        if tag_time is not None and tag_time in self.tag_by_time:
            return self.tag_by_time[tag_time]
        if tag_time is None and len(self.tags) and self.tags[0][0] > t:
            return None
        i = int(np.searchsorted(self.tag_pub_t, t, side="right")) - 1
        return self.tags[i][1] if i >= 0 else None


# ============================================================
# Re-running the EKF
# ============================================================
class EkfRun:
    """fusion_core.FusionCore run over a recording."""

    def __init__(self, rec, cfg):
        t_start = time.time()
        core = fusion_core.FusionCore(cfg, log=None)
        T, S, C = [], [], []
        nis_t, nis_k, nis_v, nis_dof = [], [], [], []
        enc_t, enc_v = [], []
        in_t, in_thr, in_servo = [], [], []
        for t, text, ip, tag_time in rec.sensors:
            kind = core.process(text, ip, t, rec.tag_for(t, tag_time))
            if kind is None:
                continue
            T.append(t)
            S.append((core.x, core.y, core.theta, core.speed))
            C.append(core.ekf.get_uncertainty()[:2, :2].copy())
            for k, v, dof in core.take_nis():
                nis_t.append(t); nis_k.append(k); nis_v.append(v); nis_dof.append(dof)
            if kind == "ENC":
                if core.last_v_enc is not None:
                    enc_t.append(t); enc_v.append(core.last_v_enc)
                in_t.append(t); in_thr.append(core.throttle); in_servo.append(core.servo_deg)
        self.cfg = core.cfg
        self.t = np.array(T)
        self.state = np.array(S).reshape(-1, 4)
        self.cov = np.array(C).reshape(-1, 2, 2)
        self.nis_t = np.array(nis_t)
        self.nis_k = np.array(nis_k)
        self.nis_v = np.array(nis_v)
        self.nis_dof = {k: d for k, d in zip(nis_k, nis_dof)}
        self.enc_t, self.enc_v = np.array(enc_t), np.array(enc_v)
        self.in_t, self.in_thr, self.in_servo = np.array(in_t), np.array(in_thr), np.array(in_servo)
        self.seconds = time.time() - t_start

    def index(self, t):
        return max(int(np.searchsorted(self.t, t, side="right")) - 1, 0)

    def nis_window(self, t0, t1):
        a, b = np.searchsorted(self.nis_t, [t0, t1], side="right")
        out = {}
        for k in self.nis_dof:
            sel = self.nis_k[a:b] == k
            out[k] = (self.nis_dof[k], list(self.nis_v[a:b][sel]))
        return out


# ============================================================
# The twin
# ============================================================
class Twin:
    """The recorded inputs through car_model, from the EKF's state at the start
    and at each resync."""

    def __init__(self, ekf, P, resync_s):
        self.P = P
        if len(ekf.in_t) == 0:
            raise SystemExit("No ENC packets (inputs) in the recording")
        t0, t1 = ekf.in_t[0], ekf.t[-1]
        ts = t0 + TWIN_DT * np.arange(int((t1 - t0) / TWIN_DT) + 1)
        j = np.maximum(np.searchsorted(ekf.in_t, ts, side="right") - 1, 0)
        thr, servo = ekf.in_thr[j], ekf.in_servo[j]
        states = np.empty((len(ts), 4))
        resyncs = []
        st = tuple(ekf.state[ekf.index(t0)])
        next_resync = t0 + resync_s if resync_s > 0 else math.inf
        step, turn = car_model.step, car_model.turn_per_metre
        for i in range(len(ts)):
            if ts[i] >= next_resync - 1e-9:
                st = tuple(ekf.state[ekf.index(ts[i])])
                resyncs.append(ts[i])
                next_resync += resync_s
            states[i] = st
            st = step(st[0], st[1], st[2], st[3], thr[i], turn(math.radians(servo[i]), P), P, TWIN_DT)
        self.ts, self.states, self.resyncs = ts, states, resyncs
        self.thr, self.servo = thr, servo

    def idx(self, t):
        return np.clip(np.round((np.asarray(t) - self.ts[0]) / TWIN_DT).astype(int), 0, len(self.ts) - 1)

    def covers(self, t):
        return self.ts[0] <= t <= self.ts[-1]


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def tag_truth(rec, ekf):
    """Tag positions as rear-axle positions at the moment their frame was taken,
    for the tags fusion would use (not too far): (times, xy)."""
    cfg = ekf.cfg
    d = cfg["TAG_POINT_AHEAD_OF_REAR_AXLE_M"]
    T, XY = [], []
    for _, p in rec.tags:
        if math.hypot(p["rel_x"], p["rel_y"]) > cfg["MAX_TAG_DISTANCE_M"]:
            continue
        tc = p["time"] - cfg["CAMERA_LATENCY_S"]
        if len(ekf.t) == 0 or tc < ekf.t[0]:
            continue
        th = ekf.state[ekf.index(tc), 2]
        T.append(tc)
        XY.append((p["x"] - d * math.cos(th), p["y"] - d * math.sin(th)))
    return np.array(T), np.array(XY).reshape(-1, 2)


def horizon_ends(times, resyncs, t_end):
    """Index of the last sample before each resync (and the end)."""
    out = []
    for b in list(resyncs) + [t_end + 1e-9]:
        j = int(np.searchsorted(times, b, side="left")) - 1
        if j >= 0 and (not out or out[-1] != j):
            out.append(j)
    return out


def metrics(rec, ekf, twin):
    m = {}
    # twin speed vs the encoders
    sel = (ekf.enc_t >= twin.ts[0]) & (ekf.enc_t <= twin.ts[-1])
    if sel.any():
        err = twin.states[twin.idx(ekf.enc_t[sel]), 3] - ekf.enc_v[sel]
        m["speed_rms"] = float(np.sqrt(np.mean(err ** 2)))
    # twin heading vs the EKF (which follows the IMU), every ~50 ms
    sel = np.where((ekf.t >= twin.ts[0]) & (ekf.t <= twin.ts[-1]))[0][::5]
    if len(sel):
        err = np.abs(wrap(twin.states[twin.idx(ekf.t[sel]), 2] - ekf.state[sel, 2]))
        m["head_rms"] = float(np.sqrt(np.mean(err ** 2)))
        if twin.resyncs:
            m["head_end"] = float(np.mean(err[horizon_ends(ekf.t[sel], twin.resyncs, twin.ts[-1])]))
    # positions vs the tags
    tt, xy = tag_truth(rec, ekf)
    sel = (tt >= twin.ts[0]) & (tt <= twin.ts[-1])
    if sel.any():
        tt, xy = tt[sel], xy[sel]
        tw = twin.states[twin.idx(tt)]
        err = np.hypot(tw[:, 0] - xy[:, 0], tw[:, 1] - xy[:, 1])
        m["pos_n"] = int(len(tt))
        m["pos_rms"] = float(np.sqrt(np.mean(err ** 2)))
        if twin.resyncs:
            m["pos_end"] = float(np.mean(err[horizon_ends(tt, twin.resyncs, twin.ts[-1])]))
        k = np.maximum(np.searchsorted(ekf.t, tt, side="right") - 1, 0)
        e2 = np.hypot(ekf.state[k, 0] - xy[:, 0], ekf.state[k, 1] - xy[:, 1])
        m["ekf_pos_rms"] = float(np.sqrt(np.mean(e2 ** 2)))
    # NIS over the whole run
    m["nis"] = {}
    for k, dof in ekf.nis_dof.items():
        vals = list(ekf.nis_v[ekf.nis_k == k])
        if vals:
            m["nis"][k] = (dof,) + po.nis_stats(dof, vals) + (len(vals),)
    return m


def metric_lines(m):
    twin = "twin vs encoders {} | vs IMU heading {} | vs tags {}".format(
        "{:.3f} m/s".format(m["speed_rms"]) if "speed_rms" in m else "-",
        "{:.1f} deg{}".format(math.degrees(m["head_rms"]),
                              " (end {:.1f})".format(math.degrees(m["head_end"])) if "head_end" in m else "")
        if "head_rms" in m else "-",
        "{:.3f} m{}".format(m["pos_rms"], " (end {:.3f})".format(m["pos_end"]) if "pos_end" in m else "")
        if "pos_rms" in m else "-")
    nis = "  ".join("{} {:.2f} [{:.2f}-{:.2f}]".format(k, v[1], v[2], v[3]) for k, v in sorted(m["nis"].items()))
    ekf = "EKF vs tags {} | NIS avg [95% band]: {}".format(
        "{:.3f} m".format(m["ekf_pos_rms"]) if "ekf_pos_rms" in m else "-", nis or "-")
    return [twin, ekf]


def print_metrics(m):
    print("[Twin] speed vs encoders: RMS {}".format(
        "{:.3f} m/s".format(m["speed_rms"]) if "speed_rms" in m else "-"))
    if "head_rms" in m:
        print("[Twin] heading vs EKF/IMU: RMS {:.2f} deg{}".format(
            math.degrees(m["head_rms"]),
            ", {:.2f} deg at the end of each stretch".format(math.degrees(m["head_end"])) if "head_end" in m else ""))
    if "pos_rms" in m:
        print("[Twin] position vs {} tag poses: RMS {:.3f} m{}".format(
            m["pos_n"], m["pos_rms"], ", {:.3f} m at the end of each stretch".format(m["pos_end"]) if "pos_end" in m else ""))
        print("[EKF]  position vs tag poses: RMS {:.3f} m".format(m["ekf_pos_rms"]))
    for k, (dof, avg, lo, hi, above, n) in sorted(m["nis"].items()):
        verdict = "ok" if lo <= avg <= hi else ("too high: filter overconfident, raise its noise" if avg > hi
                                                 else "too low: filter too cautious, lower its noise")
        print("[EKF]  NIS {:8s} avg {:6.2f} (dof {}, 95% band {:.2f}-{:.2f}), {:.0f}% above the 95% line, n={} -> {}".format(
            k, avg, dof, lo, hi, 100 * above, n, verdict))


# ============================================================
# Fitting the model
# ============================================================
def fit(rec, ekf, cfg, names, resync_s, max_evals=500):
    """Pattern search over the named model settings (in MODEL_SETTINGS) for the
    twin that best matches: speed vs the encoders, heading vs the IMU/EKF,
    position vs the tags, each relative to a typical size (5 cm/s, 5 deg, 5 cm).
    The EKF is kept as it is (it's mostly the encoders and IMU anyway)."""
    for n in names:
        if n not in MODEL_SETTINGS or MODEL_SETTINGS[n][2] in ("sign", "bool"):
            raise SystemExit("Can fit: {}".format(", ".join(k for k, v in MODEL_SETTINGS.items() if v[2] in ("lin", "log"))))
    if resync_s <= 0:
        print("[Fit] Without --resync the whole run is one stretch, which fits poorly; try --resync 3")

    def cost(c):
        m = metrics(rec, ekf, Twin(ekf, fusion_core.model_params(c), resync_s))
        return ((m.get("speed_rms", 0) / 0.05) ** 2 + (m.get("head_rms", 0) / math.radians(5)) ** 2
                + (m.get("pos_rms", 0) / 0.05) ** 2)

    cfg = dict(cfg)
    best, evals = cost(cfg), 1
    show = lambda c: ", ".join("{}={}".format(n, fmt_value(c[n])) for n in names)
    print("[Fit] start: cost {:.3f}  ({})".format(best, show(cfg)))
    step = {n: 0.1 for n in names}   # as a fraction of the range (log range for log ones)

    def moved(c, n, frac):
        lo, hi, scale = MODEL_SETTINGS[n]
        if scale == "log":
            v = math.exp(math.log(c[n]) + frac * (math.log(hi) - math.log(lo)))
        else:
            v = c[n] + frac * (hi - lo)
        return float(np.clip(v, lo, hi))

    while evals < max_evals:
        improved = False
        for n in names:
            for sgn in (1, -1):
                trial = dict(cfg)
                trial[n] = moved(cfg, n, sgn * step[n])
                if trial[n] == cfg[n]:
                    continue
                c = cost(trial)
                evals += 1
                if c < best:
                    cfg, best, improved = trial, c, True
                    step[n] *= 1.5
                    break
        if not improved:
            for n in names:
                step[n] *= 0.5
            if max(step.values()) < 1e-4:
                break
    print("[Fit] done after {} runs: cost {:.3f}  ({})".format(evals, best, show(cfg)))
    return cfg


# ============================================================
# Drawing
# ============================================================
def car_outline(x, y, th, cfg):
    L, half = cfg["WHEELBASE"], cfg["CAR_WIDTH"] / 2 + 0.01
    body = [(-0.03, -half), (L + 0.03, -half), (L + 0.03, half), (-0.03, half)]
    c, s = math.cos(th), math.sin(th)
    return [(x + c * bx - s * by, y + s * bx + c * by) for bx, by in body]


def path_px(floor, pts):
    return po.to_px(floor, pts).round().astype(np.int32)


def draw_frame(img, rec, ekf, twin, cfg, frame_i, delay, opts, extra_lines):
    floor = rec.floor
    tf = rec.frame_t[frame_i]
    tw = tf - delay                       # the moment this picture shows
    lines = ["t = {:6.2f} s of {:.1f}   frame {}".format(tf - rec.frame_t[0], rec.frame_t[-1] - rec.frame_t[0], frame_i)]

    # the live estimate, as recorded
    if opts["live"] and len(rec.live_t):
        k = int(np.searchsorted(rec.live_t, tw, side="right")) - 1
        if k >= 0:
            a = int(np.searchsorted(rec.live_t, tw - TRAIL_S))
            if opts["trails"] and k - a >= 2:
                cv2.polylines(img, [path_px(floor, rec.live[a:k + 1:3, :2])], False, LIVE_COLOR, 1, cv2.LINE_AA)
            c = po.to_px(floor, [rec.live[k, :2]])[0]
            cv2.circle(img, po.ipt(c), 5, LIVE_COLOR, 2, cv2.LINE_AA)

    # the re-run EKF
    k = ekf.index(tw)
    x, y, th, v = ekf.state[k]
    if opts["trails"]:
        a = int(np.searchsorted(ekf.t, tw - TRAIL_S))
        if k - a >= 2:
            cv2.polylines(img, [path_px(floor, ekf.state[a:k + 1:3, :2])], False, EKF_COLOR, 1, cv2.LINE_AA)
    ring = path_px(floor, po.ellipse_world(x, y, ekf.cov[k], po.SIGMA))
    over = img.copy()
    cv2.fillPoly(over, [ring], EKF_COLOR, cv2.LINE_AA)
    cv2.addWeighted(over, 0.3, img, 0.7, 0, dst=img)
    c = po.to_px(floor, [(x, y)])[0]
    tip = po.to_px(floor, [(x + 0.12 * math.cos(th), y + 0.12 * math.sin(th))])[0]
    cv2.circle(img, po.ipt(c), 4, EKF_COLOR, -1, cv2.LINE_AA)
    cv2.line(img, po.ipt(c), po.ipt(tip), EKF_COLOR, 2, cv2.LINE_AA)

    # the tag pose whose frame was taken about now
    lat = cfg["CAMERA_LATENCY_S"]
    best = None
    for tp, p in rec.tags[max(0, int(np.searchsorted(rec.tag_pub_t, tw)) - 5):
                          int(np.searchsorted(rec.tag_pub_t, tw + lat + 0.5))]:
        tc = p["time"] - lat
        if 0 <= tw - tc <= TAG_SHOW_S and (best is None or tc > best[0]):
            best = (tc, p)
    if best is not None:
        p = best[1]
        far = math.hypot(p["rel_x"], p["rel_y"]) > cfg["MAX_TAG_DISTANCE_M"]
        col = (0, 110, 0) if far else TAG_COLOR
        cx, cy = po.ipt(po.to_px(floor, [(p["x"], p["y"])])[0])
        tip = po.to_px(floor, [(p["x"] + 0.12 * math.cos(p["theta"]), p["y"] + 0.12 * math.sin(p["theta"]))])[0]
        cv2.rectangle(img, (cx - 7, cy - 7), (cx + 7, cy + 7), col, 2, cv2.LINE_AA)
        cv2.line(img, (cx, cy), po.ipt(tip), col, 1, cv2.LINE_AA)

    # the twin
    if twin.covers(tw):
        i = int(twin.idx(tw))
        tx, ty, tth, tv = twin.states[i]
        if opts["trails"]:
            start = max([r for r in twin.resyncs if r <= tw], default=twin.ts[0])
            i0 = int(twin.idx(max(tw - TRAIL_S, start)))
            if i - i0 >= 2:
                cv2.polylines(img, [path_px(floor, twin.states[i0:i + 1:4, :2])], False, TWIN_COLOR, 1, cv2.LINE_AA)
        cv2.polylines(img, [path_px(floor, car_outline(tx, ty, tth, cfg))], True, TWIN_COLOR, 2, cv2.LINE_AA)
        k_turn = car_model.turn_per_metre(math.radians(twin.servo[i]), twin.P)
        wheel = math.atan(k_turn * cfg["WHEELBASE"])     # the front wheels' angle
        L = cfg["WHEELBASE"]
        fx, fy = tx + L * math.cos(tth), ty + L * math.sin(tth)
        a = tth + wheel
        for side in (-1, 1):
            cx_, cy_ = fx - side * cfg["CAR_WIDTH"] / 2 * math.sin(tth), fy + side * cfg["CAR_WIDTH"] / 2 * math.cos(tth)
            e = po.to_px(floor, [(cx_ - 0.03 * math.cos(a), cy_ - 0.03 * math.sin(a)),
                                 (cx_ + 0.03 * math.cos(a), cy_ + 0.03 * math.sin(a))])
            cv2.line(img, po.ipt(e[0]), po.ipt(e[1]), TWIN_COLOR, 3, cv2.LINE_AA)
        lines.append("twin x={:+.3f} y={:+.3f} th={:+6.1f} v={:+.2f} | throttle {:+.2f} servo {:.0f}".format(
            tx, ty, math.degrees(tth), tv, twin.thr[i], twin.servo[i]))
        lines.append("EKF  x={:+.3f} y={:+.3f} th={:+6.1f} v={:+.2f} | twin off by {:.3f} m, {:.1f} deg".format(
            x, y, math.degrees(th), v, math.hypot(tx - x, ty - y), math.degrees(abs(wrap(tth - th)))))
    else:
        lines.append("EKF  x={:+.3f} y={:+.3f} th={:+6.1f} v={:+.2f}".format(x, y, math.degrees(th), v))

    lines += extra_lines
    lines.append("magenta twin   orange EKF (re-run)   grey EKF (live)   green tag")
    if opts["help"]:
        lines += ["space play  a/d frame  j/l 2 s  -/= speed  [/] delay  t paths  g live",
                  "s save  r reload  e export  h help  q quit"]
    po.draw_text_lines(img, lines)
    po.draw_nis_table(img, ekf.nis_window(tw - po.NIS_WINDOW_S, tw))
    draw_timeline(img, rec, twin, tf)


TIMELINE_H = 26


def timeline_rect(img):
    h, w = img.shape[:2]
    return 10, h - TIMELINE_H + 8, w - 10, h - 6


def draw_timeline(img, rec, twin, t_now):
    x0, y0, x1, y1 = timeline_rect(img)
    ta, tb = rec.frame_t[0], rec.frame_t[-1]
    px = lambda t: int(x0 + (x1 - x0) * (t - ta) / max(tb - ta, 1e-6))
    cv2.rectangle(img, (x0, y0), (x1, y1), (40, 40, 40), -1)
    cv2.rectangle(img, (px(twin.ts[0]), y0 + 4), (px(twin.ts[-1]), y1 - 4), (90, 60, 90), -1)
    for r in twin.resyncs:
        cv2.line(img, (px(r), y0), (px(r), y1), TWIN_COLOR, 1)
    cv2.line(img, (px(t_now), y0 - 3), (px(t_now), y1 + 3), (255, 255, 255), 2)


# ============================================================
# Settings files
# ============================================================
TUNABLE = list(MODEL_SETTINGS) + list(EKF_SETTINGS)


def load_settings(rec, params_path, from_run):
    cfg = dict(fusion_core.DEFAULT_CONFIG)
    if rec.config:
        cfg.update(rec.config)
    else:
        print("[Twin] The recording has no settings from fusion; using fusion_core.DEFAULT_CONFIG")
    if not from_run and params_path and os.path.exists(params_path):
        with open(params_path) as f:
            saved = json.load(f)
        changed = {k: v for k, v in saved.items() if k in cfg and cfg[k] != v}
        cfg.update({k: v for k, v in saved.items() if k in cfg})
        if changed:
            print("[Twin] From {} (different from the recording): {}".format(
                params_path, ", ".join("{}={}".format(k, fmt_value(v)) for k, v in changed.items())))
    return cfg


def save_settings(path, cfg, rec_cfg):
    with open(path, "w") as f:
        json.dump({k: cfg[k] for k in TUNABLE}, f, indent=2)
    lines = []
    for k in TUNABLE:
        if rec_cfg is None or rec_cfg.get(k) != cfg[k]:
            v = cfg[k]
            lines.append("{} = {}".format(k, repr(v) if isinstance(v, bool) else fmt_value(v)))
    txt = os.path.splitext(path)[0] + "_for_laptop_fusion.txt"
    with open(txt, "w") as f:
        f.write("# Settings changed from the recording, for laptop_fusion.py\n")
        f.write("# (STEER_RATIO replaces the line computing it from FULL_LOCK_TURN_DIAMETER_M;\n")
        f.write("#  XRP_MAX_RPM must still match MAX_DRIVE_RPM on the XRP: if the fit moved it,\n")
        f.write("#  the kart's real top speed differs from what that setting implies)\n")
        f.write("\n".join(lines) + "\n")
    print("[Twin] Saved {} and {}".format(path, txt))
    if lines:
        print("[Twin] Changed from the recording:\n    " + "\n    ".join(lines))


# ============================================================
# Interactive player
# ============================================================
class Player:
    def __init__(self, rec, cfg, args):
        self.rec, self.cfg, self.args = rec, cfg, args
        self.resync = max(0.0, args.resync)
        self.delay = args.delay if args.delay is not None else rec.meta.get("overhead_delay_s", po.OVERHEAD_DELAY_S)
        self.cap = cv2.VideoCapture(rec.video_path)
        nv = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.n = min(len(rec.frame_t), nv) if nv > 0 else len(rec.frame_t)
        self.cap_next, self.i, self.frame, self.frame_i = 0, 0, None, -1
        self.playing, self.speed = False, 1.0
        self.opts = {"trails": True, "live": True, "help": True}
        self.rerun(ekf_too=True)

    def rerun(self, ekf_too):
        t = time.time()
        if ekf_too:
            self.ekf = EkfRun(self.rec, self.cfg)
        self.twin = Twin(self.ekf, fusion_core.model_params(self.cfg), self.resync)
        self.m = metrics(self.rec, self.ekf, self.twin)
        self.run_ms = (time.time() - t) * 1000

    # --- video ---
    def read(self, i):
        if i == self.frame_i:
            return self.frame
        if not (self.cap_next <= i <= self.cap_next + 15):
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, i)
            self.cap_next = i
        while self.cap_next < i:
            self.cap.grab()
            self.cap_next += 1
        ok, f = self.cap.read()
        self.cap_next += 1
        if ok:
            self.frame, self.frame_i = f, i
        return self.frame

    def seek(self, i):
        self.i = int(np.clip(i, 0, self.n - 1))
        if self.playing:
            self.start_clock()

    def seek_time(self, t):
        self.seek(int(np.searchsorted(self.rec.frame_t[:self.n], t)))

    def start_clock(self):
        self.wall0, self.vid0 = time.time(), self.rec.frame_t[self.i]

    # --- sliders ---
    @staticmethod
    def to_pos(spec, v):
        lo, hi, scale = spec
        if scale == "sign":
            return 1 if v > 0 else 0
        if scale == "bool":
            return 1 if v else 0
        if scale == "log":
            f = (math.log(max(v, 1e-30)) - math.log(lo)) / (math.log(hi) - math.log(lo))
        else:
            f = (v - lo) / (hi - lo)
        return int(round(np.clip(f, 0, 1) * SLIDER_STEPS))

    @staticmethod
    def from_pos(spec, pos):
        lo, hi, scale = spec
        if scale == "sign":
            return 1.0 if pos else -1.0
        if scale == "bool":
            return bool(pos)
        f = pos / SLIDER_STEPS
        if scale == "log":
            return math.exp(math.log(lo) + f * (math.log(hi) - math.log(lo)))
        return lo + f * (hi - lo)

    def make_sliders(self, x_right):
        self.groups = {"Model (twin + EKF)": MODEL_SETTINGS, "EKF settings": EKF_SETTINGS}
        self.pos = {}
        y = 0
        for win, specs in self.groups.items():
            cv2.namedWindow(win, cv2.WINDOW_NORMAL)
            rows = len(specs) + (2 if win.startswith("Model") else 0)
            height = 34 * rows + 24 * (rows + 3)
            cv2.resizeWindow(win, 520, height)
            cv2.moveWindow(win, x_right, y)
            y += height + 40
            for name, spec in specs.items():
                cv2.createTrackbar(name, win, 0, 1 if spec[2] in ("sign", "bool") else SLIDER_STEPS, lambda v: None)
        mw = "Model (twin + EKF)"
        cv2.createTrackbar("resync x0.1 s", mw, 0, 200, lambda v: None)
        cv2.createTrackbar("overhead delay ms", mw, 0, 500, lambda v: None)
        self.set_sliders()

    def set_sliders(self):
        for win, specs in self.groups.items():
            for name, spec in specs.items():
                p = self.to_pos(spec, self.cfg[name])
                cv2.setTrackbarPos(name, win, p)
                self.pos[(win, name)] = p
        mw = "Model (twin + EKF)"
        self.pos[(mw, "resync")] = int(round(self.resync * 10))
        self.pos[(mw, "delay")] = int(round(self.delay * 1000))
        cv2.setTrackbarPos("resync x0.1 s", mw, self.pos[(mw, "resync")])
        cv2.setTrackbarPos("overhead delay ms", mw, self.pos[(mw, "delay")])
        self.draw_panels()

    def poll_sliders(self):
        """Applies sliders the user moved (only those, so values from the file
        keep their full precision). Returns (model changed, EKF changed)."""
        model = ekf = False
        for win, specs in self.groups.items():
            for name, spec in specs.items():
                p = cv2.getTrackbarPos(name, win)
                if p != self.pos[(win, name)]:
                    self.pos[(win, name)] = p
                    self.cfg[name] = self.from_pos(spec, p)
                    if name in MODEL_SETTINGS:
                        model = True
                    else:
                        ekf = True
        mw = "Model (twin + EKF)"
        p = cv2.getTrackbarPos("resync x0.1 s", mw)
        if p != self.pos[(mw, "resync")]:
            self.pos[(mw, "resync")] = p
            self.resync = p / 10.0
            model = model or "twin"
        p = cv2.getTrackbarPos("overhead delay ms", mw)
        if p != self.pos[(mw, "delay")]:
            self.pos[(mw, "delay")] = p
            self.delay = p / 1000.0
        if model or ekf:
            self.draw_panels()
        return model, ekf

    def draw_panels(self):
        for win, specs in self.groups.items():
            rows = [(n, fmt_value(self.cfg[n])) for n in specs]
            if win.startswith("Model"):
                rows += [("resync", "{:.1f} s".format(self.resync) if self.resync > 0 else "never"),
                         ("overhead delay", "{:.0f} ms".format(self.delay * 1000)),
                         ("(top speed)", "{:.3f} m/s".format(fusion_core.xrp_max_speed(self.cfg)))]
            img = np.zeros((22 * len(rows) + 12, 460, 3), np.uint8)
            for r, (n, v) in enumerate(rows):
                changed = self.rec.config is not None and n in self.rec.config and self.rec.config[n] != self.cfg.get(n)
                col = (0, 220, 255) if changed else (230, 230, 230)
                cv2.putText(img, n, (8, 20 + 22 * r), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)
                cv2.putText(img, v, (300, 20 + 22 * r), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)
            cv2.imshow(win, img)

    # --- main loop ---
    def run_gui(self):
        win = "Digital twin replay"
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        first = self.read(0)
        if first is None:
            raise SystemExit("Can't read " + self.rec.video_path)
        h, w = first.shape[:2]
        cv2.resizeWindow(win, w, h)
        cv2.moveWindow(win, 0, 0)
        self.make_sliders(w + 20)

        def on_mouse(event, u, v, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN or (event == cv2.EVENT_MOUSEMOVE and flags & cv2.EVENT_FLAG_LBUTTON):
                x0, y0, x1, y1 = timeline_rect(first)
                if y0 - 10 <= v <= y1 + 10:
                    f = np.clip((u - x0) / max(x1 - x0, 1), 0, 1)
                    ta, tb = self.rec.frame_t[0], self.rec.frame_t[self.n - 1]
                    self.seek_time(ta + f * (tb - ta))
        cv2.setMouseCallback(win, on_mouse)

        pending, last_change = (False, False), None
        while True:
            model, ekf = self.poll_sliders()
            if model or ekf:
                # a model change re-runs the EKF too (it predicts with the same model)
                pending = (pending[0] or bool(model), pending[1] or ekf or model is True)
                last_change = time.time()
            if last_change is not None and time.time() - last_change > 0.3:
                self.rerun(ekf_too=pending[1])
                pending, last_change = (False, False), None
                if self.playing:
                    self.start_clock()

            if self.playing:
                target = self.vid0 + (time.time() - self.wall0) * self.speed
                i = int(np.searchsorted(self.rec.frame_t[:self.n], target, side="right")) - 1
                if i >= self.n - 1:
                    i, self.playing = self.n - 1, False
                self.i = max(i, self.i)

            frame = self.read(self.i)
            if frame is None:
                break
            show = frame.copy()
            extra = metric_lines(self.m) + ["{}  x{:g}   re-run {:.0f} ms{}".format(
                "PLAY" if self.playing else "PAUSED", self.speed, self.run_ms,
                "   (settings changed: re-running...)" if last_change else "")]
            draw_frame(show, self.rec, self.ekf, self.twin, self.cfg, self.i, self.delay, self.opts, extra)
            cv2.imshow(win, show)

            key = cv2.waitKey(1 if self.playing else 20) & 0xFF
            if key in (27, ord("q")):
                break
            elif key == ord(" "):
                self.playing = not self.playing
                if self.playing:
                    if self.i >= self.n - 1:
                        self.i = 0
                    self.start_clock()
            elif key in (ord("a"), ord("d")):
                self.playing = False
                self.seek(self.i + (1 if key == ord("d") else -1))
            elif key in (ord("j"), ord("l")):
                self.seek_time(self.rec.frame_t[self.i] + (2.0 if key == ord("l") else -2.0))
            elif key in (ord("-"), ord("=")):
                self.speed = max(0.125, self.speed / 2) if key == ord("-") else min(8.0, self.speed * 2)
                if self.playing:
                    self.start_clock()
            elif key in (ord("["), ord("]")):
                self.delay = float(np.clip(self.delay + (0.01 if key == ord("]") else -0.01), 0, 0.5))
                self.set_sliders()
            elif key == ord("t"):
                self.opts["trails"] = not self.opts["trails"]
            elif key == ord("g"):
                self.opts["live"] = not self.opts["live"]
            elif key == ord("h"):
                self.opts["help"] = not self.opts["help"]
            elif key == ord("s"):
                save_settings(self.args.params, self.cfg, self.rec.config)
                print_metrics(self.m)
            elif key == ord("r"):
                self.cfg = load_settings(self.rec, self.args.params, False)
                self.set_sliders()
                self.rerun(ekf_too=True)
                print("[Twin] Reloaded " + self.args.params)
            elif key == ord("e"):
                path = os.path.join(self.rec.folder, time.strftime("overlay_%H%M%S.mp4"))
                export_video(self.rec, self.ekf, self.twin, self.cfg, self.delay, self.m, path)
        self.cap.release()
        cv2.destroyAllWindows()


def export_video(rec, ekf, twin, cfg, delay, m, path):
    cap = cv2.VideoCapture(rec.video_path)
    nv = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    n = min(len(rec.frame_t), nv) if nv > 0 else len(rec.frame_t)
    fps = (n - 1) / max(rec.frame_t[n - 1] - rec.frame_t[0], 1e-3) if n > 1 else 30.0
    writer, done = None, 0
    opts = {"trails": True, "live": True, "help": False}
    extra = metric_lines(m)
    print("[Export] Writing {} ...".format(path))
    for i in range(n):
        ok, frame = cap.read()
        if not ok:
            break
        if writer is None:
            hh, ww = frame.shape[:2]
            writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"), fps, (ww, hh))
        draw_frame(frame, rec, ekf, twin, cfg, i, delay, opts, extra)
        writer.write(frame)
        done += 1
    if writer is not None:
        writer.release()
    cap.release()
    print("[Export] Wrote {} frames to {}".format(done, path))


# ============================================================
# Main
# ============================================================
def main():
    ap = argparse.ArgumentParser(description="Re-run the fusion and the digital twin on a recording, over its video")
    ap.add_argument("run", help="run folder from twin_record.py")
    ap.add_argument("--params", default=os.path.join(HERE, "twin_params.json"),
                    help="settings file: loaded over the recording's settings, saved with s "
                         "(default: twin_params.json next to this script)")
    ap.add_argument("--from-run", action="store_true", help="ignore the settings file; start from the recording's")
    ap.add_argument("--fusion-dir", help="folder with fusion_core.py and car_model.py (default: the one above this)")
    ap.add_argument("--resync", type=float, default=3.0, help="restart the twin from the EKF every N s (0 = never)")
    ap.add_argument("--delay", type=float, default=None, help="overhead camera delay, s (default: the run's)")
    ap.add_argument("--metrics", action="store_true", help="print the numbers and exit")
    ap.add_argument("--export", help="render the overlay to this video file and exit")
    ap.add_argument("--fit", help="comma-separated model settings to fit, e.g. STEER_RATIO,SPEED_RESPONSE,XRP_MAX_RPM")
    ap.add_argument("--save", action="store_true", help="save the settings (after --fit) to --params")
    args = ap.parse_args()

    import_fusion(args.fusion_dir)
    rec = Recording(args.run)
    cfg = load_settings(rec, args.params, args.from_run)
    print("[Twin] {}: {:.1f} s of video, {} sensor packets, {} tag poses".format(
        args.run, rec.frame_t[-1] - rec.frame_t[0], len(rec.sensors), len(rec.tags)))

    # check: with the recording's own settings, the re-run should match what fusion did live
    if rec.config and len(rec.live_t):
        live_run = EkfRun(rec, dict(rec.config))
        k = np.clip(np.searchsorted(live_run.t, rec.live_t), 0, len(live_run.t) - 1)
        same = np.abs(live_run.t[k] - rec.live_t) < 1e-9
        if same.any():
            d = np.hypot(*(live_run.state[k[same], :2] - rec.live[same, :2]).T)
            print("[Twin] Re-run with the recording's settings vs live: max {:.4f} m apart "
                  "({:.0f} ms per run){}".format(d.max(), live_run.seconds * 1000,
                                                 "" if d.max() < 1e-6 else
                                                 " (not identical: some packets missing from the recording, "
                                                 "or fusion_core.py changed since)"))

    if args.fit:
        ekf = EkfRun(rec, cfg)
        cfg = fit(rec, ekf, cfg, [n.strip() for n in args.fit.split(",") if n.strip()], args.resync)
    if args.save:
        save_settings(args.params, cfg, rec.config)

    if args.metrics or args.export or args.fit:
        ekf = EkfRun(rec, cfg)
        twin = Twin(ekf, fusion_core.model_params(cfg), args.resync)
        m = metrics(rec, ekf, twin)
        print_metrics(m)
        if args.export:
            delay = args.delay if args.delay is not None else rec.meta.get("overhead_delay_s", po.OVERHEAD_DELAY_S)
            export_video(rec, ekf, twin, cfg, delay, m, args.export)
        return

    Player(rec, cfg, args).run_gui()


if __name__ == "__main__":
    main()
