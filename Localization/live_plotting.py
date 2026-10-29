"""
live_xy_plot.py

Live (x, y) plot of the XRP's fused EKF position, read from the CSV that
laptop_fusion_test.py writes. Runs as a separate process, so it never
touches the OpenCV windows or threads.

Usage (in a second terminal, while laptop_fusion_test.py is running):
  python live_xy_plot.py [path/to/fusion_log.csv]

Keys in the plot window:
  c  - clear the trail
  q  - quit
"""

import sys
import os
import time
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ---------------- Settings ----------------
CSV_PATH = sys.argv[1] if len(sys.argv) > 1 else "fusion_log.csv"

TRAIL_SECONDS = 3.0       # only show the last N seconds of data (None = keep all)
TRAIL_LEN = 3000          # hard cap on stored points (safety limit)
UPDATE_MS = 50            # plot refresh interval (~20 fps)

X_RANGE = (-1, 2)     # world x limits (m)
Y_RANGE = (-1.5, 1.5)     # world y limits (m)

# True: draw like the overhead camera sees it (+x up, +y left), so the
# plot and the colour-detection feed move the same way side by side.
# False: conventional math axes (+x right, +y up).
CAMERA_ORIENTATION = True

SHOW_BALL = False         # also plot the colour-detected ball position
# ------------------------------------------


def wait_for_file(path):
    while not os.path.exists(path):
        print(f"[Plot] Waiting for {path} ...")
        time.sleep(0.5)
    f = open(path, "r", newline="")
    # Wait for the header line
    while True:
        header = f.readline()
        if header.endswith("\n"):
            return f, header.strip().split(",")
        f.seek(0)
        time.sleep(0.2)


f, header = wait_for_file(CSV_PATH)
col = {name: i for i, name in enumerate(header)}
print(f"[Plot] Reading {CSV_PATH} with columns {header}")

fused_t, fused_x, fused_y = (deque(maxlen=TRAIL_LEN) for _ in range(3))
ball_t, ball_x, ball_y = (deque(maxlen=TRAIL_LEN) for _ in range(3))
state = {"t0": None, "t": None, "partial": ""}


def read_new_rows():
    """Read every complete line appended since the last call."""
    while True:
        line = f.readline()
        if not line:
            return
        line = state["partial"] + line
        if not line.endswith("\n"):          # writer is mid-line; finish next time
            state["partial"] = line
            return
        state["partial"] = ""
        parts = line.strip().split(",")
        try:
            t = float(parts[col["time"]])
            x = float(parts[col["x_fused"]])
            y = float(parts[col["y_fused"]])
        except (ValueError, IndexError, KeyError):
            continue
        if state["t0"] is None:
            state["t0"] = t
        state["t"] = t
        fused_t.append(t); fused_x.append(x); fused_y.append(y)

        if SHOW_BALL and "ball_x" in col:
            try:
                bx = float(parts[col["ball_x"]]); by = float(parts[col["ball_y"]])
                if not (bx == 0.0 and by == 0.0):   # 0,0 = not detected yet
                    ball_t.append(t); ball_x.append(bx); ball_y.append(by)
            except (ValueError, IndexError):
                pass


def to_plot(xs, ys):
    """Map world coords to plot axes depending on orientation."""
    return (list(ys), list(xs)) if CAMERA_ORIENTATION else (list(xs), list(ys))


fig, ax = plt.subplots(figsize=(7, 8))
fig.canvas.manager.set_window_title("XRP live (x, y)")

if CAMERA_ORIENTATION:
    ax.set_xlim(Y_RANGE[1], Y_RANGE[0])      # +y to the left
    ax.set_ylim(X_RANGE)                     # +x up
    ax.set_xlabel("world y (m)  ← +y")
    ax.set_ylabel("world x (m)  ↑ +x")
else:
    ax.set_xlim(X_RANGE); ax.set_ylim(Y_RANGE)
    ax.set_xlabel("world x (m)"); ax.set_ylabel("world y (m)")

ax.set_aspect("equal")
ax.grid(True, alpha=0.3)
ax.axhline(0, color="gray", lw=0.8); ax.axvline(0, color="gray", lw=0.8)

trail_line, = ax.plot([], [], "-", lw=1.5, color="tab:blue", label="EKF fused")
head_dot, = ax.plot([], [], "o", ms=10, color="tab:blue")
if SHOW_BALL:
    ball_line, = ax.plot([], [], "-", lw=1, color="orange", alpha=0.8, label="ball (camera)")
    ball_dot, = ax.plot([], [], "o", ms=8, color="orange")
ax.legend(loc="upper right")

info = ax.text(0.02, 0.98, "waiting for data...", transform=ax.transAxes,
               va="top", family="monospace", fontsize=11,
               bbox=dict(facecolor="white", alpha=0.8, edgecolor="none"))


def prune_old(ts, xs, ys, t_now):
    """Drop points older than TRAIL_SECONDS before the newest sample."""
    while ts and ts[0] < t_now - TRAIL_SECONDS:
        ts.popleft(); xs.popleft(); ys.popleft()


def update(_):
    read_new_rows()
    if TRAIL_SECONDS is not None and state["t"] is not None:
        prune_old(fused_t, fused_x, fused_y, state["t"])
        prune_old(ball_t, ball_x, ball_y, state["t"])
    if fused_x:
        px, py = to_plot(fused_x, fused_y)
        trail_line.set_data(px, py)
        head_dot.set_data([px[-1]], [py[-1]])
        elapsed = state["t"] - state["t0"]
        clock = time.strftime("%H:%M:%S", time.localtime(state["t"]))
        info.set_text(f"t = {elapsed:7.2f} s  ({clock})\n"
                      f"x = {fused_x[-1]:+.3f} m\n"
                      f"y = {fused_y[-1]:+.3f} m")
    if SHOW_BALL and ball_x:
        bx, by = to_plot(ball_x, ball_y)
        ball_line.set_data(bx, by)
        ball_dot.set_data([bx[-1]], [by[-1]])
    return ()


def on_key(event):
    if event.key == "c":
        for d in (fused_t, fused_x, fused_y, ball_t, ball_x, ball_y):
            d.clear()
        trail_line.set_data([], []); head_dot.set_data([], [])
    elif event.key == "q":
        plt.close(fig)


fig.canvas.mpl_connect("key_press_event", on_key)

# Skip anything already in the file so the trail starts "now".
# Comment this out to replay the whole log from the start.
f.seek(0, os.SEEK_END)

anim = FuncAnimation(fig, update, interval=UPDATE_MS, cache_frame_data=False)
plt.show()
f.close()