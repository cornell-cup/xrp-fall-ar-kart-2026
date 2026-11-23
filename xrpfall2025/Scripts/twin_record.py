"""
twin_record.py

Records a run for the digital twin and for tuning the EKF offline:
    - the overhead camera's video, and when each frame arrived
    - everything laptop_fusion.py sends to TWIN_TAP_PORT (6006), with arrival times:
        sensor  every XRP packet as received (ENC packets carry the throttle
                and servo angle, so these are also the inputs), and which tag
                pose fusion used with it
        tag     every tag pose the detector published
        fused   the live estimate after each packet, and its NIS values
        config  fusion's settings (sent every 2 s)
Everything is timestamped with time.time() on this laptop, so video and data line
up. Scripts/twin_replay.py then re-runs the fusion and the twin on the recording.

Run fusion (or the game, which starts it) as usual, and this alongside it. It
needs the overhead camera, so close pose_overlay.py first (it uses the same
camera). The live estimate is drawn here as a check that data is arriving.

Saved in a new folder (runs/run_YYYYmmdd_HHMMSS by default):
    video.mp4      the raw overhead video, nothing drawn on it
    frames.csv     frame index, time.time() when it was read
    events.jsonl   one JSON event per line, as sent by fusion
    meta.json      camera, image size, overhead delay, the floor calibration

Usage:
    python Scripts/twin_record.py                 # camera 0
    python Scripts/twin_record.py --camera 1 --out runs/figure8

Keys: q / Esc stop and save
"""

import argparse
import csv
import json
import os
import socket
import threading
import time
from collections import deque

import cv2

import pose_overlay as po

HERE = os.path.dirname(os.path.abspath(__file__))
TWIN_TAP_PORT = 6006       # TWIN_TAP_PORT in laptop_fusion.py


class TapLogger(threading.Thread):
    """Receives fusion's events and appends them to events.jsonl."""

    def __init__(self, port, path):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
        except OSError:
            pass
        self.sock.bind(("127.0.0.1", port))
        self.sock.settimeout(0.3)
        self.out = open(path, "w", buffering=1 << 16)
        self.lock = threading.Lock()
        self.counts = {}
        self.latest_fused = None
        self.latest_config = None
        self.trail = deque(maxlen=400)
        self.running = True

    def run(self):
        while self.running:
            try:
                data, _ = self.sock.recvfrom(65536)
            except socket.timeout:
                continue
            except OSError:
                continue
            try:
                ev = json.loads(data.decode())
            except (ValueError, UnicodeDecodeError):
                continue
            self.out.write(json.dumps(ev) + "\n")
            kind = ev.get("kind")
            with self.lock:
                self.counts[kind] = self.counts.get(kind, 0) + 1
                if kind == "fused":
                    self.latest_fused = ev
                    self.trail.append((ev["x"], ev["y"]))
                elif kind == "config":
                    self.latest_config = ev["config"]
        self.out.close()


def main():
    ap = argparse.ArgumentParser(description="Record overhead video + fusion's raw data for the digital twin")
    ap.add_argument("--camera", default="0", help="camera index, stream URL or video file (default 0)")
    ap.add_argument("--port", type=int, default=TWIN_TAP_PORT, help="fusion's TWIN_TAP_PORT")
    ap.add_argument("--calib", default=os.path.join(HERE, "pose_overlay_calib.json"),
                    help="pose_overlay's floor calibration file (copied into the run)")
    ap.add_argument("--out", help="run folder (default runs/run_<date>_<time>)")
    ap.add_argument("--delay", type=float, default=po.OVERHEAD_DELAY_S,
                    help="overhead camera delay in seconds, saved with the run (default %(default)s)")
    args = ap.parse_args()

    out_dir = args.out or os.path.join("runs", time.strftime("run_%Y%m%d_%H%M%S"))
    os.makedirs(out_dir, exist_ok=True)

    cap = po.open_camera(args.camera)
    ok, frame = cap.read()
    if not ok:
        raise SystemExit("Can't read from camera '{}'".format(args.camera))
    h, w = frame.shape[:2]

    window = "Twin recorder"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, w, h)

    # the replay needs the floor calibration, so make sure there is one now
    model = po.load_calibration(args.calib, (w, h))
    if model is None:
        model = po.calibrate(cap, args.calib, window)
        if model is None:
            raise SystemExit("Calibration cancelled")
    with open(args.calib) as f:
        calib_data = json.load(f)

    tap = TapLogger(args.port, os.path.join(out_dir, "events.jsonl"))
    tap.start()

    frames_f = open(os.path.join(out_dir, "frames.csv"), "w", newline="")
    frames_csv = csv.writer(frames_f)
    frames_csv.writerow(["frame", "t"])

    # The video file's frame rate is only for ordinary players (the replay times
    # frames from frames.csv): measured over the first 30 frames, held until then
    pending, writer, n_frames = [], None, 0
    t_start = time.time()
    rate_counts, rate_t = {}, t_start
    rates = {}
    print("[Record] Recording to {} (q to stop)".format(out_dir))

    def open_writer(frames):
        fps = (len(frames) - 1) / max(frames[-1][0] - frames[0][0], 1e-3) if len(frames) > 1 else 30.0
        print("[Record] Video at {:.1f} fps (measured)".format(fps))
        wr = cv2.VideoWriter(os.path.join(out_dir, "video.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
        for _, f in frames:
            wr.write(f)
        return wr

    from_file = os.path.isfile(args.camera)
    try:
        while True:
            ok, frame = cap.read()
            t = time.time()
            if not ok:
                if from_file:
                    break       # a video file (for testing) has ended
                time.sleep(0.005)
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    break
                continue
            frames_csv.writerow([n_frames, "{:.6f}".format(t)])
            n_frames += 1
            if writer is None:
                pending.append((t, frame))
                if len(pending) >= 30:
                    writer = open_writer(pending)
                    pending = []
            else:
                writer.write(frame)

            # live view, on a copy (the saved video stays clean)
            with tap.lock:
                counts = dict(tap.counts)
                fused = tap.latest_fused
                trail = list(tap.trail)
                have_config = tap.latest_config is not None
            if t - rate_t >= 1.0:
                rates = {k: (counts.get(k, 0) - rate_counts.get(k, 0)) / (t - rate_t) for k in counts}
                rate_counts, rate_t = counts, t
            show = frame.copy()
            lines = ["REC {:.1f} s   {} frames".format(t - t_start, n_frames),
                     "per second: {} sensor packets, {} tag poses, {} estimates".format(
                         int(rates.get("sensor", 0)), int(rates.get("tag", 0)), int(rates.get("fused", 0)))]
            if fused is not None:
                lines += po.draw_estimate(show, model, fused, t - fused["t"], trail, True)[:2]
            if not counts.get("sensor"):
                lines.append("no data from fusion on UDP {} yet: is laptop_fusion.py running "
                             "(the version with TWIN_TAP)?".format(args.port))
            elif not have_config:
                lines.append("waiting for fusion's settings (sent every 2 s)")
            po.draw_text_lines(show, lines)
            cv2.imshow(window, show)
            if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                break
    finally:
        tap.running = False
        if writer is None and pending:
            writer = open_writer(pending)
        if writer is not None:
            writer.release()
        frames_f.close()
        tap.join(timeout=1.0)
        meta = {
            "camera": args.camera,
            "image_size": [w, h],
            "overhead_delay_s": args.delay,
            "started": t_start,
            "frames": n_frames,
            "calibration": calib_data,
        }
        with open(os.path.join(out_dir, "meta.json"), "w") as f:
            json.dump(meta, f, indent=2)
        cap.release()
        cv2.destroyAllWindows()
        print("[Record] Saved {} frames and {} to {}".format(
            n_frames, ", ".join("{} {}".format(v, k) for k, v in sorted(tap.counts.items())), out_dir))


if __name__ == "__main__":
    main()
