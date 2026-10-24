# IMPORTANT: env vars must be set BEFORE `import cv2`. TCP transport
# gives a clean image with retransmits.

import os, sys, tempfile, traceback, threading

def _open_log():
    if getattr(sys, "frozen", False):
        here = os.path.dirname(sys.executable)
    else:
        here = os.path.dirname(os.path.abspath(__file__))

    for d in (here, os.path.expanduser("~"), tempfile.gettempdir()):
        try:
            p = os.path.join(d, "laptop_fusion.log")
            return open(p, "w", buffering=1), p
        except OSError:
            continue
    return None, None

_log_f, _log_path = _open_log()
if _log_f:
    try:
        os.dup2(_log_f.fileno(), 1)
        os.dup2(_log_f.fileno(), 2)
    except OSError:
        pass
    sys.stdout = _log_f
    sys.stderr = _log_f
    print(f"[BOOT] python={sys.executable} cwd={os.getcwd()} "
          f"argv={sys.argv} log={_log_path}", flush=True)

    def _thread_excepthook(args):
        print(f"[THREAD CRASH] {args.thread.name}", file=_log_f)
        traceback.print_exception(args.exc_type, args.exc_value,
                                  args.exc_traceback, file=_log_f)
        _log_f.flush()
    threading.excepthook = _thread_excepthook

import os
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
    "rtsp_transport;tcp"
    "|fflags;nobuffer"
    "|flags;low_delay"
    "|reorder_queue_size;0"
    "|max_delay;0"
    "|buffer_size;65536"
)
os.environ["OPENCV_LOG_LEVEL"] = "ERROR"
os.environ["AV_LOG_FORCE_NOCOLOR"] = "1"

import sys
import time
import math
import json
import socket
import struct
import threading
from contextlib import contextmanager

import numpy as np
import cv2
from pupil_apriltags import Detector

from ackermann_simple_ekf import ackermann_simple_ekf

"""
laptop_fusion.py

Single laptop-side fusion + routing node. Combines:

    1. RTSP video grab from the Pi (background thread)
    2. AprilTag detection + camera->world transform (background thread)
    3. EKF predict + correct + camera_correction from XRP raw sensors
    4. Forwards fused state to Godot (UDP 6000, JSON {x, y, angle})
    5. Streams raw RGB video frames to Godot (TCP 6001)
    6. Forwards Godot gamepad packets to the XRP (UDP 4003 -> XRP:4002)
    7. NEW: Forwards AprilTag detections to Godot (UDP 6002, JSON ar_tags)
       for AR UI overlays — labels above each detected tag in-game.

Architecture:
    XRP   --RAW,enc_l,enc_r,heading-UDP:5005--> us
    Pi    --RTSP------------------------------> us
    us    --{x,y,angle} JSON--UDP:6000--------> Godot (kart pose)
    us    --yaw correctional data--UDP:5007---> XRP
    us    --raw RGB frames---TCP:6001---------> Godot (video backdrop)
    us    --ar_tags JSON-----UDP:6002---------> Godot (AR overlays)
    Godot --gamepad bytes----UDP:4003---------> us --UDP:4002--> XRP
"""

# ============================================================
# Config
# ============================================================

pose_w = None

# Where to look for the Pi's camera. The first address that answers on
# the RTSP port wins, so the same build works whether the Pi is running
# its own hotspot or is on CornellCup-Web. An address passed on the
# command line (Godot's PI_IP) is tried first.
PI_CANDIDATES = [a for a in sys.argv[1:2] if a] + [
    "cornellcup2.local",   # works on any network (mDNS name)
    "10.42.0.1",           # Pi hotspot
    "192.168.4.95",        # last known CornellCup-Web address
]
RTSP_PORT = 8554

SENSOR_PORT = 5005

GODOT_HOST = "127.0.0.1"
GODOT_POSE_PORT = 6000
GODOT_VIDEO_PORT = 6001
GODOT_AR_TAGS_PORT = 6002    # NEW: AR overlay events

GAMEPAD_RELAY_PORT = 4003
XRP_GAMEPAD_PORT = 4002

XRP_IP = "192.168.4.8"
YAW_CORRECTION_PORT = 5007

yaw_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


PI_IP = sys.argv[1] if len(sys.argv) > 1 else "192.168.4.95"

# PI_IP = sys.argv[1] if len(sys.argv) > 1 else "192.168.0.193"

RTSP_URL = f"rtsp://{PI_IP}:8554/cam"

# Set to None to disable CSV logging. Otherwise: file path.
CSV_LOG_PATH = "fusion_log.csv"

# Apply camera correction to the EKF? Disable to compare encoder+IMU
# only against the raw tag pose, which is useful for debugging.
APPLY_CAMERA_CORRECTION = True

TAG_FRESHNESS_S = 0.2

# How often the status lines print (seconds).
PRINT_EVERY_S = 2.0

# Raw RGB to Godot is ~0.9 MB per frame and Godot decodes it on its main
# thread. Cap it so a 60 fps camera doesn't slow the game down. The
# detector still gets every frame.
VIDEO_MAX_FPS = 30

# fx, fy, cx, cy - calibrated for the camera's 640x480 cropped mode. If tag
# distances come out ~2.4x too far, the Pi is using the wide mode instead.
Camera_params = [1.26828550e+03, 1.25414697e+03, 3.61108041e+02, 2.97975274e+02]
TAG_SIZE_TAG3 = 0.06503
TAG_SIZE_TAG0 = 0.08584

POSITIVE_Y = 1
X_THRESHOLD = 0.4572
FAR_FROM_ORIGIN = 0

x = 0
y = 0
theta = np.pi/2
enc_l = 0
enc_r = 0
heading = 0

# Per-tag display names for the AR overlay. IDs not in here still get
# tracked for localization but show up generically. Add or rename as
# you set up the physical tags.
TAG_LABELS = {
    0: "TAG 0",
    1: "TAG 1",
    2: "TAG 2",
    3: "TAG 3",
    4: "TAG 4",
    5: "TAG 5",
    6: "TAG 6",
    7: "TAG 7",
}

# Frame dimensions used by the AR side. Send the actual capture size
# so Godot can convert tag corners (in pixel coords) to backdrop coords.
FRAME_W_DEFAULT = 640
FRAME_H_DEFAULT = 480

# ============================================================
# Frame transforms (from pi_detect_aprilTag.py)
# ============================================================

_theta_mount = np.radians(0)
s_theta = np.sin(_theta_mount); c_theta = np.cos(_theta_mount)
h1, l1, d = 0.03047, 0.007625, 0.02792
trans_R_C = np.array([
    [0,        1,         0,        0                          ],
    [s_theta,  0,         c_theta,  h1*c_theta + l1*s_theta    ],
    [c_theta,  0,        -s_theta, -d - h1*s_theta + l1*c_theta],
    [0,        0,         0,        1                          ],
])

h3_tag3, l3_tag3 = 0.02483, 0.02155
trans_H_A_tag3 = np.array([
    [1, 0, 0,   0  ], [0, 1, 0, -h3_tag3 ], [0, 0, 1, -l3_tag3 ], [0, 0, 0, 1],
])

h3_tag0, l3_tag0 = -0.0047, 0
trans_H_A_tag0 = np.array([
    [1, 0, 0,   0  ], [0, 1, 0, -h3_tag3 ], [0, 0, 1, -l3_tag3 ], [0, 0, 0, 1],
])

alpha_tag3 = np.radians(0)
c_alpha_tag3, s_alpha_tag3 = np.cos(alpha_tag3), np.sin(alpha_tag3)
l4_tag3 = 0.01895
trans_g_H_tag3 = np.array([
    [1, 0,        0,         0 ],
    [0, c_alpha_tag3, -s_alpha_tag3,    0 ],
    [0, s_alpha_tag3,  c_alpha_tag3,  -l4_tag3],
    [0, 0,        0,          1],
])

alpha_tag0 = np.radians(0)
c_alpha_tag0, s_alpha_tag0 = np.cos(alpha_tag0), np.sin(alpha_tag0)
l4_tag0 = 0.01187
trans_g_H_tag0 = np.array([
    [1, 0,        0,         0 ],
    [0, c_alpha_tag0, -s_alpha_tag0,    0 ],
    [0, s_alpha_tag0,  c_alpha_tag0,  -l4_tag0],
    [0, 0,        0,          1],
])

x1_tag3, y1_tag3, x2_tag3, y2_tag3 = -1, 0.6096, 1, 0.6096
x3_tag3, y3_tag3 = (x1_tag3+x2_tag3)/2, (y1_tag3+y2_tag3)/2
h4_tag3 = 0.352425
if x1_tag3 == x2_tag3:
    beta_tag3 = np.radians(90) if y1_tag3 > y2_tag3 else np.radians(270)
else:
    beta_tag3 = np.arctan((y1_tag3-y2_tag3)/(x2_tag3-x1_tag3))
    if x1_tag3 <= x2_tag3 and y1_tag3 >= y2_tag3: beta_tag3 = beta_tag3
    elif x1_tag3 > x2_tag3:             beta_tag3 = beta_tag3 + np.radians(180)
    else:                     beta_tag3 = beta_tag3 + np.radians(270)
c_beta_tag3, s_beta_tag3 = np.cos(beta_tag3), np.sin(beta_tag3)
trans_G_g_tag3 = np.array([
    [ c_beta_tag3, 0,  s_beta_tag3, x3_tag3],
    [-s_beta_tag3, 0,  c_beta_tag3, y3_tag3],
    [ 0,     -1,  0,      h4_tag3],
    [ 0,      0,  0,       1],
])
trans_G_A_tag3= trans_G_g_tag3 @ trans_g_H_tag3 @ trans_H_A_tag3
print(f"[Transforms] beta_tag3 = {math.degrees(beta_tag3):.1f}")

x1_tag0, y1_tag0, x2_tag0, y2_tag0 = 1.4144, -0.4572, 0.5856, -0.4572
x3_tag0, y3_tag0 = (x1_tag0+x2_tag0)/2, (y1_tag0+y2_tag0)/2
h4_tag0 = 0.352425
if x1_tag0 == x2_tag0:
    beta_tag0 = np.radians(90) if y1_tag0 > y2_tag0 else np.radians(270)
else:
    beta_tag0 = np.arctan((y1_tag0-y2_tag0)/(x2_tag0-x1_tag0))
    if x1_tag0 <= x2_tag0 and y1_tag0 >= y2_tag0: beta_tag0 = beta_tag0
    elif x1_tag0 > x2_tag0:             beta_tag0 = beta_tag0 + np.radians(180)
    else:                     beta_tag0 = beta_tag0 + np.radians(270)
c_beta_tag0, s_beta_tag0 = np.cos(beta_tag0), np.sin(beta_tag0)
trans_G_g_tag0 = np.array([
    [ c_beta_tag0, 0,  s_beta_tag0, x3_tag0],
    [-s_beta_tag0, 0,  c_beta_tag0, y3_tag0],
    [ 0,     -1,  0,      h4_tag0],
    [ 0,      0,  0,       1],
])
trans_G_A_tag0 = trans_G_g_tag0 @ trans_g_H_tag0 @ trans_H_A_tag0
print(f"[Transforms] beta_tag0 = {math.degrees(beta_tag0):.1f}")

# ============================================================
# EKF
# ============================================================
# TODO measure: xrpgodot.py used radius 0.033. Wrong values make the kart's
# in-game position drift out of scale.
WHEEL_RADIUS = 0.052   # meters (wheel diameter / 2)
TRACK_WIDTH  = 0.109   # meters between the two driven wheels
r_xy = 0.4
r_th = 5e-2
q_th = math.radians(1.0) ** 2

ekf = None
ekf_lock = threading.Lock()
theta_zero = None

# ============================================================
# Detector
# ============================================================
# nthreads: use several CPU cores. quad_decimate 2.0 roughly halves
# detection time; set it back to 1.0 if far-away tags stop being found.
detector = Detector(families="tag36h11", nthreads=4, quad_decimate=2.0)


@contextmanager
def suppress_stderr():
    devnull = os.open(os.devnull, os.O_WRONLY)
    orig = os.dup(2)
    try:
        os.dup2(devnull, 2); yield
    finally:
        os.dup2(orig, 2); os.close(orig); os.close(devnull)


# ============================================================
# Shared state
# ============================================================
latest_frame = None
latest_frame_id = 0
latest_frame_lock = threading.Lock()

latest_display_frame = None
display_frame_lock = threading.Lock()

latest_robot_pose_world = None
latest_robot_pose_lock = threading.Lock()

latest_fused = None  # {"x": ..., "y": ..., "theta": ...}
latest_fused_lock = threading.Lock()

# running = True

# NEW: list of every visible tag this frame (for AR overlay).
# Refreshed by the detector thread on each processed frame.
# Each entry: {"id": int, "corners": [[x,y]*4], "center": [x,y], "name": str}
latest_visible_tags = []
latest_visible_tags_lock = threading.Lock()
latest_visible_tags_time = 0.0

xrp_ip = "192.168.4.8"
xrp_ip_lock = threading.Lock()

running = True


# ============================================================
# RTSP grabber thread
# ============================================================
def open_rtsp(url):
    cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
    cap.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 10000)
    cap.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, 10000)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def find_pi():
    """Return the IP of the first candidate whose RTSP port answers."""
    # IPv4 only: on the hotspot, cornellcup2.local can resolve to an IPv6
    # link-local address (fe80::...), which FFmpeg can't put in an RTSP URL.
    for host in PI_CANDIDATES:
        try:
            infos = socket.getaddrinfo(host, RTSP_PORT, socket.AF_INET, socket.SOCK_STREAM)
        except OSError:
            continue
        for info in infos:
            ip = info[4][0]
            try:
                with socket.create_connection((ip, RTSP_PORT), timeout=1.5):
                    pass
                print(f"[Camera] Pi found at {host} ({ip})")
                return ip
            except OSError:
                pass
    return None


# def grabber_thread():
#     global latest_frame, latest_frame_id, running
#     while running:
#         pi_ip = find_pi()
#         if pi_ip is None:
#             print(f"[Camera] Pi not reachable at any of {PI_CANDIDATES}, retrying in 3s...")
#             time.sleep(3); continue
#         rtsp_url = f"rtsp://{pi_ip}:{RTSP_PORT}/cam"
#         print(f"[Camera] Connecting to {rtsp_url}")
#         cap = open_rtsp(rtsp_url)
#         if not cap.isOpened():
#             print("[Camera] Failed to open, retrying in 3s...")
#             cap.release(); time.sleep(3); continue
#         print("[Camera] RTSP connected")
#         fail = 0
#         while running:
#             ret, frame = cap.read()
#             if not ret:
#                 fail += 1
#                 if fail > 10:
#                     print("[Camera] Lost connection, reconnecting...")
#                     break
#                 time.sleep(0.05); continue
#             fail = 0
#             with latest_frame_lock:
#                 latest_frame = frame
#                 latest_frame_id += 1
#         cap.release()
#         if running: time.sleep(2)

def grabber_thread(rtsp_url):
    global latest_frame, latest_frame_id, running
    while running:
        print(f"[Camera] Connecting to {rtsp_url}")
        cap = open_rtsp(rtsp_url)
        if not cap.isOpened():
            print("[Camera] Failed to open, retrying in 3s...")
            cap.release(); time.sleep(3); continue
        print("[Camera] RTSP connected")
        fail = 0
        while running:
            ret, frame = cap.read()
            if not ret:
                fail += 1
                if fail > 10:
                    print("[Camera] Lost connection, reconnecting...")
                    break
                time.sleep(0.05); continue
            fail = 0
            with latest_frame_lock:
                latest_frame = frame
                latest_frame_id += 1
        cap.release()
        if running: time.sleep(2)


# ============================================================
# Detection thread — runs AprilTag, publishes BOTH
#   (a) world-frame robot pose for EKF
#   (b) per-tag corners for AR overlay
# ============================================================
def detector_thread():
    global latest_robot_pose_world, latest_visible_tags, latest_visible_tags_time, latest_display_frame, running, POSITIVE_Y, X_THRESHOLD, FAR_FROM_ORIGIN
    
    trans_C_A_tag3 = np.identity(4)
    trans_C_A_tag0 = np.identity(4)
    last_processed_id = -1
    fps_n = 0
    fps_t = time.time()

    while running:
        with latest_frame_lock:
            if latest_frame is None or latest_frame_id == last_processed_id:
                frame = None
            else:
                frame = latest_frame.copy()    # copy so we can draw on it
                last_processed_id = latest_frame_id

        if frame is None:
            time.sleep(0.005); continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        with suppress_stderr():
        
            if FAR_FROM_ORIGIN:
                results = detector.detect(
                    gray, estimate_tag_pose=True,
                    camera_params=Camera_params, tag_size=TAG_SIZE_TAG0,
                )
            else:
                results = detector.detect(
                    gray, estimate_tag_pose=True,
                    camera_params=Camera_params, tag_size=TAG_SIZE_TAG3,
                )

        valid = [t for t in results if 0 <= t.tag_id <= 7]

        # ---- Build per-tag overlay info ----
        # The AR side just needs the corners so it can position labels
        # in 2D. The EKF side needs the world-frame pose from the
        # closest tag. We compute both from the same detection list.
        overlay_tags = []
        for tag in valid:
            corners = tag.corners.tolist()      # [[x,y]*4]
            center = tag.center.tolist()        # [x,y]
            overlay_tags.append({
                "id": int(tag.tag_id),
                "name": TAG_LABELS.get(int(tag.tag_id), f"TAG {int(tag.tag_id)}"),
                "corners": corners,
                "center": center,
            })

        with latest_visible_tags_lock:
            latest_visible_tags = overlay_tags
            latest_visible_tags_time = time.time()

        # ---- Localization update from closest tag ----
        if valid:
            tag = min(valid, key=lambda t: abs(t.pose_t[2][0]))
            if tag.tag_id == 3:
                trans_C_A_tag3[:3, :3] = tag.pose_R
                trans_C_A_tag3[0, 3] = tag.pose_t[0][0]
                trans_C_A_tag3[1, 3] = tag.pose_t[1][0]
                trans_C_A_tag3[2, 3] = tag.pose_t[2][0]
                try:
                    trans_A_C_tag3 = np.linalg.inv(trans_C_A_tag3)
                    trans_R_G_tag3 = trans_G_A_tag3 @ trans_A_C_tag3 @ trans_R_C
                    r_x = float(trans_R_G_tag3[0, 3])
                    r_y = float(trans_R_G_tag3[1, 3])
                    r_theta = (math.atan2(trans_R_G_tag3[1, 0], trans_R_G_tag3[0, 0]))
                    with latest_robot_pose_lock:
                        latest_robot_pose_world = {
                            "x": r_x, "y": r_y, "theta": r_theta,
                            "tag_id": tag.tag_id, "time": time.time(),
                        }

                    # Send absolute robot yaw to XRP
                    msg = "TAG_YAW,{:.6f}".format(r_theta)

                    try:
                        yaw_sock.sendto(msg.encode("utf-8"),(XRP_IP, YAW_CORRECTION_PORT))
                    except OSError as e:
                        print("[YAW] UDP send error:", e)

                except np.linalg.LinAlgError:
                    pass
            if tag.tag_id == 0:
                trans_C_A_tag0[:3, :3] = tag.pose_R
                trans_C_A_tag0[0, 3] = tag.pose_t[0][0]
                trans_C_A_tag0[1, 3] = tag.pose_t[1][0]
                trans_C_A_tag0[2, 3] = tag.pose_t[2][0]
                try:
                    trans_A_C_tag0 = np.linalg.inv(trans_C_A_tag0)
                    trans_R_G_tag0 = trans_G_A_tag0 @ trans_A_C_tag0 @ trans_R_C
                    r_x = float(trans_R_G_tag0[0, 3])
                    r_y = float(trans_R_G_tag0[1, 3])
                    r_theta = math.atan2(trans_R_G_tag0[1, 0], trans_R_G_tag0[0, 0])
                    with latest_robot_pose_lock:
                        latest_robot_pose_world = {
                            "x": r_x, "y": r_y, "theta": r_theta,
                            "tag_id": tag.tag_id, "time": time.time(),
                        }

                    # Send absolute robot yaw to XRP
                    msg = "TAG_YAW,{:.6f}".format(r_theta)

                    try:
                        yaw_sock.sendto(msg.encode("utf-8"),(XRP_IP, YAW_CORRECTION_PORT))
                    except OSError as e:
                        print("[YAW] UDP send error:", e)

                except np.linalg.LinAlgError:
                    pass

        with display_frame_lock:
            latest_display_frame = frame

        fps_n += 1
        now = time.time()
        if now - fps_t >= PRINT_EVERY_S:
            fps_n = int(fps_n / (now - fps_t))   # report per second
            tag_str = "no tag"
            if latest_robot_pose_world:
                age = now - latest_robot_pose_world["time"]
                tag_str = f"tag {latest_robot_pose_world['tag_id']} ({'fresh' if age < TAG_FRESHNESS_S else f'STALE {age:.1f}s'})"
            print(f"[Detect] {fps_n} fps | {tag_str}")
            fps_n = 0; fps_t = now


# ============================================================
# AR tag broadcaster — sends overlay info to Godot at ~30 Hz
# ============================================================
def ar_tags_thread():
    """Push the latest detected tag list to Godot for AR overlays.
    Runs at ~30 Hz so labels track the tag smoothly even between detections.
    Sends an empty 'tags' array when nothing is visible so Godot can
    hide stale labels."""
    global running
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (GODOT_HOST, GODOT_AR_TAGS_PORT)
    interval = 1.0 / 30.0
    print(f"[AR] Broadcasting tag overlays to {target}")

    while running:
        with latest_visible_tags_lock:
            tags = list(latest_visible_tags)
            ts = latest_visible_tags_time

        # If no detection in a while, send empty array so Godot hides labels
        if time.time() - ts > TAG_FRESHNESS_S:
            tags = []

        try:
            payload = json.dumps({
                "type": "ar_tags",
                "tags": tags,
                "frame_w": FRAME_W_DEFAULT,
                "frame_h": FRAME_H_DEFAULT,
                "timestamp": time.time(),
            })
            sock.sendto(payload.encode(), target)
        except Exception as e:
            print("[AR] send error:", e)

        time.sleep(interval)


# ============================================================
# Sensor loop + EKF (runs in main thread on non-mac, separate on mac)
# ============================================================
def sensor_ekf_thread(csv_writer):
    # global ekf, theta_zero, latest_fused, running, POSITIVE_Y
    global ekf, theta_zero, latest_fused, running, POSITIVE_Y, X_THRESHOLD, FAR_FROM_ORIGIN
    global x, y, theta, enc_l, enc_r, heading, xrp_ip_lock, xrp_ip, pose_w
    print("[DEBUG] Entered sensor_ekf_thread()", flush=True)

    sensor_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sensor_sock.bind(("0.0.0.0", SENSOR_PORT))
    sensor_sock.settimeout(0.5)
    print(f"[Sensor] Listening for XRP raw sensors on UDP {SENSOR_PORT}")

    pkt_count = 0
    last_print = time.time()

    while running:
        # print("Running")
        try:
            data, addr = sensor_sock.recvfrom(256)

            with xrp_ip_lock:
                if xrp_ip != addr[0]:
                    xrp_ip = addr[0]
                    print(f"[XRP] IP captured: {xrp_ip}")

            # print(f"[DEBUG] Received {data!r} from {addr}", flush=True)
        except socket.timeout:
            continue

        try:
            text = data.decode("utf-8").strip()
        except UnicodeDecodeError:
            continue

        parts = text.split(",")
        if len(parts) != 4 or parts[0] != "RAW":
            continue
        try:
            enc_l = float(parts[1]); enc_r = float(parts[2]); heading = float(parts[3])
        except ValueError:
            continue

        if ekf is None:
            with ekf_lock:
                if ekf is None:
                    ekf = ackermann_simple_ekf(
                        wheel_radius=WHEEL_RADIUS,
                        distance_between_wheels=TRACK_WIDTH,
                        initial_position_uncertainty=r_xy,
                        initial_heading_uncertainty=r_th,
                        sensor_uncertainty=q_th,
                        initial_left_encoder_value=enc_l,
                        initial_right_encoder_value=enc_r,
                        initial_x=0.0, initial_y=0.0, initial_theta=np.pi/2,
                    )
                    theta_zero = heading
                    print(f"[EKF] Initialized at heading={heading:.3f} rad from {addr[0]}")
            continue

        with ekf_lock:
            x, y, theta = ekf.predict(enc_l, enc_r)
            # z_theta = heading - theta_zero
            # x, y, theta = ekf.correct(z_theta)

            # z_theta = heading - theta_zero
            x, y, theta = ekf.correct(heading)

            # print("IMU raw: " + str(np.degrees(heading)))
            # print("EKF corrected: " + str(np.degrees(theta)))
            # print("\n")

            # if y >= 0:
            #     POSITIVE_Y = 1
            # else:
            #     POSITIVE_Y = 0

            if x >= X_THRESHOLD:
                FAR_FROM_ORIGIN = 1
            else:
                FAR_FROM_ORIGIN = 0

            with latest_robot_pose_lock:
                pose_w = latest_robot_pose_world
            tag_is_fresh = (
                pose_w is not None
                and (time.time() - pose_w["time"]) < TAG_FRESHNESS_S
            )

            if tag_is_fresh and APPLY_CAMERA_CORRECTION:
                try:
                    x, y, theta = ekf.camera_correction(
                        pose_w["x"], pose_w["y"], pose_w["theta"],
                    )

                    # if y >= 0:
                    #     POSITIVE_Y = 1
                    # else:
                    #     POSITIVE_Y = 0

                    if x >= X_THRESHOLD:
                        FAR_FROM_ORIGIN = 1
                    else:
                        FAR_FROM_ORIGIN = 0

                except Exception as e:
                    print(f"[EKF] camera_correction error: {e}")

        with latest_fused_lock:
            latest_fused = {"x": x, "y": y, "theta": theta}

        # CSV log
        # if csv_writer is not None:
        #     tag_x = pose_w["x"] if (pose_w and tag_is_fresh) else ""
        #     tag_y = pose_w["y"] if (pose_w and tag_is_fresh) else ""
        #     tag_th = pose_w["theta"] if (pose_w and tag_is_fresh) else ""
        #     csv_writer.writerow([
        #         time.time(), x, y, theta, tag_x, tag_y, tag_th, ball_x, ball_y
        #     ])

        pkt_count += 1
        now = time.time()
        if now - last_print >= 1.0:
            if pose_w is None:
                tag_info = "no tag yet"
            else:
                age = time.time() - pose_w["time"]
                if age < TAG_FRESHNESS_S:
                    tag_info = (f"tag x={pose_w['x']:+.2f} y={pose_w['y']:+.2f} "
                                f"th={math.degrees(pose_w['theta']):+.0f} ({age*1000:.0f}ms)")
                else:
                    tag_info = f"tag stale {age:.1f}s"
            print(f"[EKF] {pkt_count} pkts/s | "
                  f"FUSED x={x:+.2f} y={y:+.2f} th={math.degrees(theta):+.0f} | {tag_info}")
            pkt_count = 0; last_print = now

    sensor_sock.close()


# ============================================================
# Video forwarder to Godot (raw RGB over TCP)
# ============================================================
godot_video_conn = None


def godot_video_server_thread():
    global godot_video_conn, running
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", GODOT_VIDEO_PORT))
    server.listen(5)
    server.settimeout(0.5)
    print(f"[Video] TCP server on {GODOT_VIDEO_PORT}, waiting for Godot...")

    last_sent_id = -1
    last_sent_t = 0.0
    min_interval = 1.0 / VIDEO_MAX_FPS
    while running:
        if godot_video_conn is None:
            try:
                conn, addr = server.accept()
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                godot_video_conn = conn
                print(f"[Video] Godot connected from {addr}")
                last_sent_id = -1
            except socket.timeout:
                continue

        if time.time() - last_sent_t < min_interval:
            time.sleep(0.002)
            continue

        with latest_frame_lock:
            if latest_frame is None or latest_frame_id == last_sent_id:
                frame = None
            else:
                frame = latest_frame
                last_sent_id = latest_frame_id

        if frame is None:
            time.sleep(0.005)
            continue
        last_sent_t = time.time()

        try:
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w = rgb.shape[:2]
            data = rgb.tobytes()
            header = struct.pack("!III", w, h, len(data))
            godot_video_conn.sendall(header + data)
        except (BrokenPipeError, ConnectionResetError, OSError):
            print("[Video] Godot disconnected")
            try: godot_video_conn.close()
            except: pass
            godot_video_conn = None


# ============================================================
# Gamepad relay
# ============================================================
def gamepad_relay_thread():
    global running
    relay_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    relay_sock.bind(("127.0.0.1", GAMEPAD_RELAY_PORT))
    relay_sock.settimeout(0.5)

    forward_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"[Gamepad] Relay listening on 127.0.0.1:{GAMEPAD_RELAY_PORT}, "
          f"forwarding to XRP:{XRP_GAMEPAD_PORT}")

    pkt_count = 0
    last_print = time.time()
    last_warn_no_xrp = 0

    while running:
        try:
            data, _ = relay_sock.recvfrom(256)
        except socket.timeout:
            continue

        with xrp_ip_lock:
            target_ip = xrp_ip

        if target_ip is None:
            now = time.time()
            if now - last_warn_no_xrp > 5.0:
                print("[Gamepad] Got packet from Godot but XRP IP unknown yet")
                last_warn_no_xrp = now
            continue

        try:
            forward_sock.sendto(data, (target_ip, XRP_GAMEPAD_PORT))
            # print(f"[Gamepad] Forwarded data: " + str(data))
            pkt_count += 1
        except OSError as e:
            print(f"[Gamepad] Forward error: {e}")

        now = time.time()
        if now - last_print >= 5.0:
            print(f"[Gamepad] Forwarded {pkt_count} packets to {target_ip} in last 5s")
            pkt_count = 0
            last_print = now


# ============================================================
# Main: sensor packet loop + EKF + Godot pose output
# ============================================================
def main():
    global ekf, theta_zero, xrp_ip, running, x, y, theta, enc_l, enc_r, heading, pose_w

    # sensor_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # sensor_sock.bind(("0.0.0.0", SENSOR_PORT))
    # sensor_sock.settimeout(0.5)
    # print(f"[Sensor] Listening for XRP raw sensors on UDP {SENSOR_PORT}")

    pose_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"[Pose] Will send fused state to {GODOT_HOST}:{GODOT_POSE_PORT}")

    threading.Thread(target=grabber_thread, args=(RTSP_URL,), daemon=True).start()
    threading.Thread(target=detector_thread, daemon=True).start()
    threading.Thread(target=godot_video_server_thread, daemon=True).start()
    threading.Thread(target=gamepad_relay_thread, daemon=True).start()
    threading.Thread(target=ar_tags_thread, daemon=True).start()

    sensor_thread = threading.Thread(
        target=sensor_ekf_thread,
        args=(None,),
        daemon=True,
        name="SensorEKF"
    )

    print("[DEBUG] Starting sensor thread", flush=True)
    sensor_thread.start()
    print("[DEBUG] Sensor thread started:", sensor_thread.is_alive(), flush=True)

    pkt_count = 0
    last_print = time.time()

    try:
        while running:
            # try:
            #     data, addr = sensor_sock.recvfrom(256)
            # except socket.timeout:
            #     continue

            with display_frame_lock:
                frame = None if latest_display_frame is None else latest_display_frame.copy()
            

            # with xrp_ip_lock:
            #     if xrp_ip != addr[0]:
            #         xrp_ip = addr[0]
            #         print(f"[XRP] IP captured: {xrp_ip}")

            # try:
            #     text = data.decode("utf-8").strip()
            # except UnicodeDecodeError:
            #     continue

            # parts = text.split(",")
            # if len(parts) != 4 or parts[0] != "RAW":
            #     continue

            # try:
            #     enc_l = float(parts[1])
            #     enc_r = float(parts[2])
            #     heading = float(parts[3])
            # except ValueError:
            #     continue

            # if ekf is None:
            #     with ekf_lock:
            #         if ekf is None:
            #             ekf = ackermann_simple_ekf(
            #                 wheel_radius=WHEEL_RADIUS,
            #                 distance_between_wheels=TRACK_WIDTH,
            #                 initial_position_uncertainty=r_xy,
            #                 initial_heading_uncertainty=r_th,
            #                 sensor_uncertainty=q_th,
            #                 initial_left_encoder_value=enc_l,
            #                 initial_right_encoder_value=enc_r,
            #                 initial_x=0.0,
            #                 initial_y=0.0,
            #                 initial_theta=0.0,
            #             )
            #             theta_zero = heading
            #             print(f"[EKF] Initialized at heading={heading:.3f} rad")
            #     continue

            # with ekf_lock:
            #     x_pred, y_pred, th_pred = ekf.predict(enc_l, enc_r)
            #     z_theta = heading - theta_zero
            #     x, y, theta = ekf.correct(x_pred, y_pred, th_pred, z_theta)

            #     with latest_robot_pose_lock:
            #         pose_w = latest_robot_pose_world

            #     tag_is_fresh = (
            #         pose_w is not None
            #         and (time.time() - pose_w["time"]) < TAG_FRESHNESS_S
            #     )

            #     if tag_is_fresh:
            #         try:
            #             x, y, theta = ekf.camera_correction(
            #                 x_pred, y_pred, th_pred,
            #                 pose_w["x"], pose_w["y"], pose_w["theta"],
            #             )
            #         except Exception as e:
            #             print(f"[EKF] camera_correction error: {e}")

            if frame is not None:
                # Overlay fused EKF state in the corner
                with latest_fused_lock:
                    fused = latest_fused
                if fused is not None:
                    txt = (f"FUSED  x={fused['x']:+.2f}m  y={fused['y']:+.2f}m  "
                            f"th={math.degrees(fused['theta']):+.0f}")
                #     cv2.putText(frame, txt, (10, 25),
                #                 cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                # else:
                #     cv2.putText(frame, "Waiting for XRP sensors...", (10, 25),
                #                 cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)

                with latest_robot_pose_lock:
                    pose_w = latest_robot_pose_world
                if pose_w is not None:
                    age = time.time() - pose_w["time"]
                    color = (0, 255, 0) if age < TAG_FRESHNESS_S else (128, 128, 128)
                #     txt = (f"TAG{pose_w['tag_id']}  x={pose_w['x']:+.2f}  "
                #             f"y={pose_w['y']:+.2f}  th={math.degrees(pose_w['theta']):+.0f}  "
                #             f"({age*1000:.0f}ms)")
                #     cv2.putText(frame, txt, (10, 50),
                #                 cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

                # cv2.imshow("AprilTag Test", frame)
            

            try:
                payload = json.dumps({
                    "x": -(x * 100),
                    "y": y * 100,
                    "angle": math.degrees(theta) % 360.0,
                })
                pose_sock.sendto(payload.encode(), (GODOT_HOST, GODOT_POSE_PORT))
            except Exception as e:
                print("[Pose] send error:", e)

            pkt_count += 1
            now = time.time()
            if now - last_print >= PRINT_EVERY_S:
                if pose_w is None:
                    tag_info = "no tag yet"
                else:
                    age = time.time() - pose_w["time"]
                    if age < TAG_FRESHNESS_S:
                        tag_info = f"tag fresh ({age*1000:.0f}ms old)"
                    else:
                        tag_info = f"tag STALE ({age:.1f}s old)"
                rate = pkt_count / (now - last_print)
                print(f"[Sensor] enc_l={enc_l:+.3f} enc_r={enc_r:+.3f} rev | "
                      f"heading={math.degrees(heading):+.1f}° ({rate:.0f} pkts/s)")
                print(f"[EKF]    fused x={x:.2f} y={y:.2f} m "
                      f"th={math.degrees(theta):+.0f}° | {tag_info}")
                pkt_count = 0
                last_print = now

    except KeyboardInterrupt:
        print("\n[Main] Interrupted, shutting down...")


    except Exception:
        traceback.print_exc()
    finally:
        running = False
        time.sleep(0.3)
        # sensor_sock.close()
        # pose_sock.close()
        
if __name__ == "__main__":
    main()