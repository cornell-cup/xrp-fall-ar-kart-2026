# IMPORTANT: env vars must be set BEFORE `import cv2`

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

def _output_is_piped():
    """True when launched by the game, which reads our output through a pipe."""
    import stat
    try:
        return stat.S_ISFIFO(os.fstat(1).st_mode)
    except OSError:
        return False

if _output_is_piped():
    # Flush every printed line, so the game's fusion.log fills in as fusion runs
    sys.stdout.reconfigure(line_buffering=True)
    _log_f, _log_path = None, None
else:
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

# How the camera video comes from the Pi. "udp": a lost WiFi packet only smears
# part of one frame. "tcp": nothing is lost, but everything behind a lost packet
# waits for it to be resent, so the video freezes and lags. Use "tcp" if the
# picture gets smeary or there's no video at all (e.g. a firewall blocking UDP)
RTSP_TRANSPORT = "udp"
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
    f"rtsp_transport;{RTSP_TRANSPORT}"
    "|fflags;nobuffer"
    "|flags;low_delay"
    "|reorder_queue_size;0"
    "|max_delay;0"
    # UDP receive buffer: room for a whole keyframe, which arrives as a burst
    "|buffer_size;1048576"
)
os.environ["OPENCV_LOG_LEVEL"] = "ERROR"
os.environ["AV_LOG_FORCE_NOCOLOR"] = "1"

import time
import math
import json
import socket
import struct
import statistics
from contextlib import contextmanager

import numpy as np
import cv2
from pupil_apriltags import Detector

# The original EKF (dead reckoning from the wheel turns + IMU heading + tags);
# the newer Ackermann model is in ackermann_simple_ekf.py
from ackermann_simple_ekf_old import ackermann_simple_ekf

"""
laptop_fusion.py

Single laptop-side fusion + routing node. Combines:

    1. RTSP video grab from the Pi (background thread)
    2. AprilTag detection + camera->world transform (background thread)
    3. EKF from the XRP's sensors (sensor thread, ackermann_simple_ekf_old.py):
       each encoder packet runs predict() with the wheel turns since the last
       one, each IMU packet correct() with the heading, each tag reading
       camera_correction()
    4. Forwards fused state to Godot (UDP 6000, JSON {x, y, angle})
    5. Streams raw RGB video frames to Godot (TCP 6001)
    6. Forwards Godot gamepad packets to the XRP (UDP 4003 -> XRP:4002)
    7. Forwards AprilTag detections to Godot (UDP 6002, JSON ar_tags)
       for the labels above each detected tag in AR mode

Architecture:
    XRP   --ENC / IMU packets--UDP:5005---------> us  (see xrp_raw_sensors.py)
    Pi    --RTSP------------------------------> us
    us    --{x,y,angle} JSON--UDP:6000--------> Godot (kart pose)
    us    --raw RGB frames---TCP:6001---------> Godot (video backdrop)
    us    --ar_tags JSON-----UDP:6002---------> Godot (AR overlays)
    us    --pose + covariance-UDP:6005--------> pose_overlay.py (optional)
    Godot --gamepad bytes----UDP:4003---------> us --UDP:4002--> XRP
"""

# ============================================================
# Config
# ============================================================

# Where to look for the Pi's camera. The first address that answers on
# the RTSP port wins, so the same build works whether the Pi is running
# its own hotspot or is on CornellCup-Web. An address passed on the
# command line (Godot's PI_IP) is tried first.
PI_CANDIDATES = [a for a in sys.argv[1:2] if a] or [
    "192.168.0.138"       
]

RTSP_PORT = 8554

SENSOR_PORT = 5005

GODOT_HOST = "127.0.0.1"
GODOT_POSE_PORT = 6000
GODOT_VIDEO_PORT = 6001
GODOT_AR_TAGS_PORT = 6002    # AR overlay events
# The estimate with its uncertainty, and the latest tag-only pose, as JSON, for
# Scripts/pose_overlay.py (drawing the estimate over an overhead camera view).
# Sent along with every pose to the game; nothing needs to be listening
DEBUG_POSE_HOST = "127.0.0.1"
DEBUG_POSE_PORT = 6005

GAMEPAD_RELAY_PORT = 4003
XRP_GAMEPAD_PORT = 4002

# This laptop locks onto the first XRP it hears and ignores any other XRP on the
# network until that one has been silent this long; then it picks up whichever
# XRP is sending. Any laptop can still drive any free XRP. Gamepad packets go to
# whichever XRP we're locked onto
XRP_LEASE_S = 3.0

# DSCP "expedited forwarding": WiFi access points send these packets ahead of
# bulk traffic like the camera stream, so gamepad commands aren't held up
CONTROL_PACKET_TOS = 0xB8

# How often the fused pose is sent to the game
POSE_SEND_HZ = 100

# Apply camera correction to the EKF? Disable to compare encoder+IMU
# only against the raw tag pose, which is useful for debugging.
APPLY_CAMERA_CORRECTION = True

TAG_FRESHNESS_S = 0.3

# How often the status lines print (seconds).
PRINT_EVERY_S = 2.0

# Raw RGB to Godot is ~0.9 MB per frame and Godot decodes it on its main
# thread. Cap it so a 60 fps camera doesn't slow the game down. The
# detector still gets every frame.
VIDEO_MAX_FPS = 30

# fx, fy, cx, cy - for the camera's wide mode 
Camera_params = [558.546, 561.275, 368.271, 264.430]
# Side length of the tags' black square in metres (all the tags are printed this size)
TAG_SIZE = 0.08576

# Latest fused state and sensor values, for the status lines
x = 0
y = 0
theta = np.pi/2
enc_l = 0
enc_r = 0
heading = 0
speed = 0.0
# Raw IMU turning since fusion started, in radians (for measuring GYRO_SCALE),
# and the distance the encoders say the kart has driven, in metres (average of
# the two rear wheels; forward +). Turning at full lock, their ratio is the
# turning radius the encoders imply, to compare with the measured one
total_turned_raw = 0.0
total_driven_m = 0.0

# Per-tag display names for the AR overlay (IDs not in here show up as "TAG n").
# Only the tags in TAG_WORLD are used for localization
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

_theta_mount = np.radians(2)
s_theta = np.sin(_theta_mount); c_theta = np.cos(_theta_mount)
h1, l1, d = 0.03047, 0.007625, 0.02792
trans_R_C = np.array([
    [0,        1,         0,        0                          ],
    [s_theta,  0,         c_theta,  h1*c_theta + l1*s_theta    ],
    [c_theta,  0,        -s_theta, -d - h1*s_theta + l1*c_theta],
    [0,        0,         0,        1                          ],
])

# robot frame from the camera frame, for where a seen tag is relative to the robot
trans_C_R = np.linalg.inv(trans_R_C)

def gate_beta(x1, y1, x2, y2):
    """A tag's facing angle from its gate's two ends: (cos b, -sin b) is the
    direction from end 1 to end 2 (the tag frame's x axis on the floor), so the
    tag is seen square-on by a kart heading 90° - b. (This replaced an
    if/elif/else that came out 90° off when end 2 is both right of and above
    end 1; tags 0-3 and 5 give the same angles either way.)"""
    return np.arctan2(y1 - y2, x2 - x1)


h3_tag3, l3_tag3 = 0, 0.0099568
trans_H_A_tag3 = np.array([
    [1, 0, 0,   0  ], [0, 1, 0, -h3_tag3 ], [0, 0, 1, -l3_tag3 ], [0, 0, 0, 1],
])

h3_tag0, l3_tag0 = -0.0058039, 0.010414
trans_H_A_tag0 = np.array([
    [1, 0, 0,   0  ], [0, 1, 0, -h3_tag0 ], [0, 0, 1, -l3_tag0 ], [0, 0, 0, 1],
])

alpha_tag3 = np.radians(0)
c_alpha_tag3, s_alpha_tag3 = np.cos(alpha_tag3), np.sin(alpha_tag3)
l4_tag3 = 0.0
trans_g_H_tag3 = np.array([
    [1, 0,        0,         0 ],
    [0, c_alpha_tag3, -s_alpha_tag3,    0 ],
    [0, s_alpha_tag3,  c_alpha_tag3,  -l4_tag3],
    [0, 0,        0,          1],
])

alpha_tag0 = np.radians(0)
c_alpha_tag0, s_alpha_tag0 = np.cos(alpha_tag0), np.sin(alpha_tag0)
l4_tag0 = 0.0
trans_g_H_tag0 = np.array([
    [1, 0,        0,         0 ],
    [0, c_alpha_tag0, -s_alpha_tag0,    0 ],
    [0, s_alpha_tag0,  c_alpha_tag0,  -l4_tag0],
    [0, 0,        0,          1],
])

x1_tag3, y1_tag3, x2_tag3, y2_tag3 = -1, 0.41275, 1, 0.41275
x3_tag3, y3_tag3 = (x1_tag3+x2_tag3)/2, (y1_tag3+y2_tag3)/2
h4_tag3 = 0.352425
beta_tag3 = gate_beta(x1_tag3, y1_tag3, x2_tag3, y2_tag3)
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
beta_tag0 = gate_beta(x1_tag0, y1_tag0, x2_tag0, y2_tag0)
c_beta_tag0, s_beta_tag0 = np.cos(beta_tag0), np.sin(beta_tag0)
trans_G_g_tag0 = np.array([
    [ c_beta_tag0, 0,  s_beta_tag0, x3_tag0],
    [-s_beta_tag0, 0,  c_beta_tag0, y3_tag0],
    [ 0,     -1,  0,      h4_tag0],
    [ 0,      0,  0,       1],
])
trans_G_A_tag0 = trans_G_g_tag0 @ trans_g_H_tag0 @ trans_H_A_tag0

print(f"[Transforms] beta_tag0 = {math.degrees(beta_tag0):.1f}")


h3_tag2, l3_tag2 = 0, 0.01013
trans_H_A_tag2 = np.array([
    [1, 0, 0,   0  ], [0, 1, 0, -h3_tag2 ], [0, 0, 1, -l3_tag2 ], [0, 0, 0, 1],
])

alpha_tag2 = np.radians(0)
c_alpha_tag2, s_alpha_tag2 = np.cos(alpha_tag2), np.sin(alpha_tag2)
l4_tag2 = 0.0
trans_g_H_tag2 = np.array([
    [1, 0,        0,         0 ],
    [0, c_alpha_tag2, -s_alpha_tag2,    0 ],
    [0, s_alpha_tag2,  c_alpha_tag2,  -l4_tag2],
    [0, 0,        0,          1],
])

x1_tag2, y1_tag2, x2_tag2, y2_tag2 = 0.69758178, -1.78660678, -0.71663178, -0.37239322
x3_tag2, y3_tag2 = (x1_tag2+x2_tag2)/2, (y1_tag2+y2_tag2)/2
h4_tag2 = 0.11324
beta_tag2 = gate_beta(x1_tag2, y1_tag2, x2_tag2, y2_tag2)
c_beta_tag2, s_beta_tag2 = np.cos(beta_tag2), np.sin(beta_tag2)
trans_G_g_tag2 = np.array([
    [ c_beta_tag2, 0,  s_beta_tag2, x3_tag2],
    [-s_beta_tag2, 0,  c_beta_tag2, y3_tag2],
    [ 0,     -1,  0,      h4_tag2],
    [ 0,      0,  0,       1],
])
trans_G_A_tag2= trans_G_g_tag2 @ trans_g_H_tag2 @ trans_H_A_tag2
print(f"[Transforms] beta_tag2 = {math.degrees(beta_tag2):.1f}")



h3_tag1, l3_tag1 = 0, 0.01102
trans_H_A_tag1 = np.array([
    [1, 0, 0,   0  ], [0, 1, 0, -h3_tag1 ], [0, 0, 1, -l3_tag1 ], [0, 0, 0, 1],
])

alpha_tag1 = np.radians(0)
c_alpha_tag1, s_alpha_tag1 = np.cos(alpha_tag1), np.sin(alpha_tag1)
l4_tag1 = 0.0
trans_g_H_tag1 = np.array([
    [1, 0,        0,         0 ],
    [0, c_alpha_tag1, -s_alpha_tag1,    0 ],
    [0, s_alpha_tag1,  c_alpha_tag1,  -l4_tag1],
    [0, 0,        0,          1],
])

x1_tag1, y1_tag1, x2_tag1, y2_tag1 = 0.2708, 1.8247, 1.685, 0.4105
x3_tag1, y3_tag1 = (x1_tag1+x2_tag1)/2, (y1_tag1+y2_tag1)/2
h4_tag1 = 0.1169
beta_tag1 = gate_beta(x1_tag1, y1_tag1, x2_tag1, y2_tag1)
c_beta_tag1, s_beta_tag1 = np.cos(beta_tag1), np.sin(beta_tag1)
trans_G_g_tag1 = np.array([
    [ c_beta_tag1, 0,  s_beta_tag1, x3_tag1],
    [-s_beta_tag1, 0,  c_beta_tag1, y3_tag1],
    [ 0,     -1,  0,      h4_tag1],
    [ 0,      0,  0,       1],
])
trans_G_A_tag1= trans_G_g_tag1 @ trans_g_H_tag1 @ trans_H_A_tag1
print(f"[Transforms] beta_tag1 = {math.degrees(beta_tag1):.1f}")




h3_tag5, l3_tag5 = 0, 0
trans_H_A_tag5 = np.array([
    [1, 0, 0,   0  ], [0, 1, 0, -h3_tag5 ], [0, 0, 1, -l3_tag5 ], [0, 0, 0, 1],
])

alpha_tag5 = np.radians(0)
c_alpha_tag5, s_alpha_tag5 = np.cos(alpha_tag5), np.sin(alpha_tag5)
l4_tag5 = 0.0
trans_g_H_tag5 = np.array([
    [1, 0,        0,         0 ],
    [0, c_alpha_tag5, -s_alpha_tag5,    0 ],
    [0, s_alpha_tag5,  c_alpha_tag5,  -l4_tag5],
    [0, 0,        0,          1],
])

x1_tag5, y1_tag5, x2_tag5, y2_tag5 = 0.8763, -1.1684, 0.5588, -1.2573
x3_tag5, y3_tag5 = (x1_tag5+x2_tag5)/2, (y1_tag5+y2_tag5)/2
h4_tag5 = 0.10973
beta_tag5 = gate_beta(x1_tag5, y1_tag5, x2_tag5, y2_tag5)
c_beta_tag5, s_beta_tag5 = np.cos(beta_tag5), np.sin(beta_tag5)
trans_G_g_tag5 = np.array([
    [ c_beta_tag5, 0,  s_beta_tag5, x3_tag5],
    [-s_beta_tag5, 0,  c_beta_tag5, y3_tag5],
    [ 0,     -1,  0,      h4_tag5],
    [ 0,      0,  0,       1],
])
trans_G_A_tag5 = trans_G_g_tag5 @ trans_g_H_tag5 @ trans_H_A_tag5
print(f"[Transforms] beta_tag5 = {math.degrees(beta_tag5):.1f}")




h3_tag4, l3_tag4 = 0, 0
trans_H_A_tag4 = np.array([
    [1, 0, 0,   0  ], [0, 1, 0, -h3_tag4 ], [0, 0, 1, -l3_tag4 ], [0, 0, 0, 1],
])

alpha_tag4 = np.radians(0)
c_alpha_tag4, s_alpha_tag4 = np.cos(alpha_tag4), np.sin(alpha_tag4)
l4_tag4 = 0.0
trans_g_H_tag4 = np.array([
    [1, 0,        0,         0 ],
    [0, c_alpha_tag4, -s_alpha_tag4,    0 ],
    [0, s_alpha_tag4,  c_alpha_tag4,  -l4_tag4],
    [0, 0,        0,          1],
])

x1_tag4, y1_tag4, x2_tag4, y2_tag4 = 0.15875, 1.13665, 0.43815, 1.22555
x3_tag4, y3_tag4 = (x1_tag4+x2_tag4)/2, (y1_tag4+y2_tag4)/2
h4_tag4 = 0.10951
beta_tag4 = gate_beta(x1_tag4, y1_tag4, x2_tag4, y2_tag4)
c_beta_tag4, s_beta_tag4 = np.cos(beta_tag4), np.sin(beta_tag4)
trans_G_g_tag4 = np.array([
    [ c_beta_tag4, 0,  s_beta_tag4, x3_tag4],
    [-s_beta_tag4, 0,  c_beta_tag4, y3_tag4],
    [ 0,     -1,  0,      h4_tag4],
    [ 0,      0,  0,       1],
])
trans_G_A_tag4 = trans_G_g_tag4 @ trans_g_H_tag4 @ trans_H_A_tag4
print(f"[Transforms] beta_tag4 = {math.degrees(beta_tag4):.1f}")



# Each tag's place on the track (tag frame -> world), by tag ID
TAG_WORLD = {
    0: trans_G_A_tag0,
    1: trans_G_A_tag1,
    2: trans_G_A_tag2,
    3: trans_G_A_tag3,
    4: trans_G_A_tag4,
    5: trans_G_A_tag5,
}


# ============================================================
# EKF
# ============================================================
WHEEL_RADIUS = 0.04176   # meters (wheel diameter / 2)
CAR_WIDTH  = 0.11928   # meters between the two driven wheels (measured)
# Distance between the front and rear axles, in metres (the EKF's L, measured)
WHEELBASE = 0.13643
# Which point on the kart the filter tracks: the middle of the rear axle, the only
# point that moves straight along the heading (the motion model and encoders are
# exact for it). The tag transforms (trans_R_C) locate a different point, this far
# ahead of the rear axle, so each tag reading is moved back to the rear axle first.
# Without that, the gap swung round in every turn: the estimate ended a turn up to
# 2x this outside / towards the end of the track, then jumped at the next tag
TAG_POINT_AHEAD_OF_REAR_AXLE_M = 0.03513
# The point sent to the game and to pose_overlay.py, this far ahead of the rear
# axle. Default: the same point the tags locate (what was sent before). For the
# middle of the kart between the axles, use WHEELBASE / 2
OUTPUT_POINT_AHEAD_OF_REAR_AXLE_M = TAG_POINT_AHEAD_OF_REAR_AXLE_M
# The old EKF moves the kart by its wheel turns and turns it by the IMU; it
# doesn't model steering, throttle or speed, so there are no steering-ratio,
# top-speed or process-noise settings here. Its noise values are inside
# ackermann_simple_ekf_old.py. These were passed to it before:
EKF_INITIAL_POSITION_UNCERTAINTY = 0.4
EKF_INITIAL_HEADING_UNCERTAINTY = 5e-2
EKF_SENSOR_UNCERTAINTY = math.radians(1.0) ** 2
# The XRP's ticks_ms() wraps around at this
XRP_TICKS_PERIOD_MS = 1 << 30


def new_ekf():
    return ackermann_simple_ekf(
        wheel_radius=WHEEL_RADIUS,
        distance_between_wheels=CAR_WIDTH,
        initial_position_uncertainty=EKF_INITIAL_POSITION_UNCERTAINTY,
        initial_heading_uncertainty=EKF_INITIAL_HEADING_UNCERTAINTY,
        sensor_uncertainty=EKF_SENSOR_UNCERTAINTY,
        initial_left_encoder_value=0.0,
        initial_right_encoder_value=0.0,
        initial_x=0.0, initial_y=0.0, initial_theta=np.pi/2,
    )

ekf = None
ekf_lock = threading.Lock()

# Each new tag pose goes into the EKF's camera_correction(). The tag shows where
# the kart was when the camera frame was taken, not now, so the filter's own
# movement since then is added to it first (see CAMERA_LATENCY_S)
# How long after a frame is taken it reaches this script (Pi encode + network +
# decode). Matching tag poses to the filter's past at the wrong time makes the
# estimate lag or lead while driving
CAMERA_LATENCY_S = 0.27
# How much of the filter's recent past is kept for that comparison
POSE_HISTORY_S = 1.0
# A small or far tag's angle can flip between two answers ~12° apart. A tag
# heading further than this from the estimate's is ignored as probably flipped...
TAG_HEADING_GATE_DEG = 6.0
# ...unless the tag readings of the last TAG_HEADING_WINDOW_S agree with each
# other: at least TAG_HEADING_MIN_READINGS of them, spread by no more than
# TAG_HEADING_MAX_SPREAD_DEG (median distance from their median). Then the
# estimate's heading is the one that's off (the gyro under-counting turns built
# up 10-17° over a few laps) and their median is used as the heading correction
TAG_HEADING_WINDOW_S = 2.0
TAG_HEADING_MIN_READINGS = 8
TAG_HEADING_MAX_SPREAD_DEG = 4.0
# The IMU's heading is trusted far more than a tag's, so camera_correction()
# moves the heading only a little per tag, and the next IMU packet undoes even that. But the IMU's error is an offset that
# grows (gyro drift and under-counted turns), so each tag heading also moves the
# IMU's offset this fraction of the remaining difference (0.05 per reading: ~80%
# over a 1 s tag pass of ~30 readings)
TAG_HEADING_GAIN = 0.05
# ...but a consistent difference bigger than this isn't gyro drift (that builds
# up a few degrees per lap): it's more likely a tag whose facing in TAG_WORLD is
# wrong, which would drag the heading round with it. Those are ignored, and
# fusion prints a warning naming the tag
TAG_HEADING_MAX_CONSISTENT_DEG = 25.0
# A tag pose whose NIS would be above this is ignored: the filter is sure enough
# of where the kart is that a reading this far off is more likely a bad tag
# reading than a real correction. 16.27 = chi-square 99.9% for 3 values, so a
# correct reading is rejected 1 time in 1000. But if every tag for
# CAMERA_RESYNC_S has been rejected, the filter is probably the one that's off,
# so the tags are trusted again
#
# Off with the old EKF (infinity): its camera_correction() shrinks the
# uncertainty as if each tag were applied in full, but its low-pass filter only
# moves the estimate 2.5% of the way, so the estimate soon looks far more
# certain than it is and good tags would be rejected. Set 16.27 to turn it on
CAMERA_NIS_GATE = float("inf")
CAMERA_RESYNC_S = 1.0
# Tags farther than this from the kart are ignored: their position readings were
# several cm off at 1.2 m and ~15 cm at 1.8 m, while within 1 m they agreed with
# the estimate to ~1 cm
MAX_TAG_DISTANCE_M = 1.0
# (The old EKF's camera noise is fixed, so nearer tags aren't trusted more than
# farther ones within that distance)
# Added to the XRP's IMU heading; tags adjust it gradually to correct gyro drift
imu_heading_offset = 0.0
# The XRP's IMU heading changes are multiplied by this. If its gyro under-counts
# turns (the estimate turns less than the kart and drifts to the outside of the
# turns between tags), measure it: drive N full circles the same way, ending
# facing exactly as at the start, and read "turned" in the [Sensor] line: then
# GYRO_SCALE = N*360 / |turned|. E.g. 3 circles that read -1062° -> 1080/1062 = 1.017
GYRO_SCALE = 1.0
# The XRP's gyro heading creeps even while it stands still (~9°/s in one run),
# which turned the game kart, and the AR track outline with it, in place. Once
# the wheels haven't turned for STILL_AFTER_S, any heading change is drift, so
# it's cancelled. (The drift rate itself is corrected on the XRP, which
# recalibrates its gyro whenever it's parked. Learning it here from packet
# timings went wrong when driving back and forth quickly: it learned the jolts
# at each stop as drift, then turned the kart by itself while driving)
STILL_AFTER_S = 0.3

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
latest_frame_time = 0.0   # when latest_frame arrived, so tag poses can be matched to the filter's past
latest_frame_lock = threading.Lock()

latest_robot_pose_world = None
latest_robot_pose_lock = threading.Lock()

latest_fused = None  # {"x", "y", "theta", "v", "cov"} (cov: 3x3 of x, y, theta)
latest_fused_lock = threading.Lock()
# NIS of each correction since the last debug packet, by sensor: kind -> [dof, [values]]
# (guarded by latest_fused_lock)
nis_pending = {}


def record_nis(kind, nis=None, dof=None):
    """Keeps the NIS of the EKF correction just made (or the given one), for
    pose_overlay.py."""
    if nis is None:
        nis, dof = ekf.get_last_nis()
    if nis is None:
        return
    with latest_fused_lock:
        entry = nis_pending.setdefault(kind, [dof, []])
        entry[0] = dof
        entry[1].append(round(nis, 4))

# The newest tag pose, from the detector thread, as last read by the sensor thread
pose_w = None

# List of every visible tag this frame (for AR overlay).
# Refreshed by the detector thread on each processed frame.
# Each entry: {"id": int, "corners": [[x,y]*4], "center": [x,y], "name": str}
latest_visible_tags = []
latest_visible_tags_lock = threading.Lock()
latest_visible_tags_time = 0.0

xrp_ip = None              # the XRP we're locked onto (see XRP_LEASE_S)
xrp_ip_lock = threading.Lock()
xrp_last_seen = 0.0

running = True


# ============================================================
# RTSP grabber thread
# ============================================================
def open_rtsp(url):
    cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG, [
        cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 10000,
        cv2.CAP_PROP_READ_TIMEOUT_MSEC, 10000,
        # One decoding thread. With several, FFmpeg decodes a few frames at once
        # and holds each back until the others catch up (~100-200 ms at 30 fps)
        cv2.CAP_PROP_N_THREADS, 1,
    ])
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


def grabber_thread():
    """On each (re)connect the first PI_CANDIDATES address that answers is
    used, so the camera is found on any of the networks"""
    global latest_frame, latest_frame_id, latest_frame_time
    while running:
        pi_ip = find_pi()
        if pi_ip is None:
            print(f"[Camera] Pi not reachable at any of {PI_CANDIDATES}, retrying in 3s...")
            time.sleep(3); continue
        rtsp_url = f"rtsp://{pi_ip}:{RTSP_PORT}/cam"
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
                latest_frame_time = time.time()
        cap.release()
        if running: time.sleep(2)


# ============================================================
# Detection thread — runs AprilTag, publishes BOTH
#   (a) world-frame robot pose for EKF
#   (b) per-tag corners for AR overlay
# ============================================================
def detector_thread():
    global latest_robot_pose_world, latest_visible_tags, latest_visible_tags_time

    last_processed_id = -1
    fps_n = 0
    fps_t = time.time()

    while running:
        with latest_frame_lock:
            if latest_frame is None or latest_frame_id == last_processed_id:
                frame = None
            else:
                frame = latest_frame   # the grabber makes a new array per frame, so no copy needed
                frame_time = latest_frame_time
                last_processed_id = latest_frame_id

        if frame is None:
            time.sleep(0.005); continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        with suppress_stderr():
            results = detector.detect(
                gray, estimate_tag_pose=True,
                camera_params=Camera_params, tag_size=TAG_SIZE,
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
            trans_G_A = TAG_WORLD.get(tag.tag_id)
            if trans_G_A is not None:
                trans_C_A = np.identity(4)
                trans_C_A[:3, :3] = tag.pose_R
                trans_C_A[:3, 3] = tag.pose_t[:, 0]
                try:
                    trans_A_C = np.linalg.inv(trans_C_A)
                    trans_R_G = trans_G_A @ trans_A_C @ trans_R_C
                    r_x = float(trans_R_G[0, 3])
                    r_y = float(trans_R_G[1, 3])
                    r_theta = math.atan2(trans_R_G[1, 0], trans_R_G[0, 0])
                    # Where the tag is relative to the robot. Unlike the tag's angle (which
                    # flips between two answers when the tag is small or far), this is reliable
                    rel = trans_C_R @ trans_C_A[:, 3]
                    with latest_robot_pose_lock:
                        latest_robot_pose_world = {
                            "x": r_x, "y": r_y, "theta": r_theta,
                            "tag_id": tag.tag_id, "time": frame_time,
                            "tag_x": float(trans_G_A[0, 3]), "tag_y": float(trans_G_A[1, 3]),
                            "rel_x": float(rel[0]), "rel_y": float(rel[1]),
                        }
                except np.linalg.LinAlgError:
                    pass

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
# Sensor loop + EKF (own thread)
# ============================================================
def sensor_ekf_thread():
    global ekf, latest_fused, imu_heading_offset
    global x, y, theta, speed, enc_l, enc_r, heading, xrp_ip, xrp_last_seen, pose_w, total_turned_raw, total_driven_m
    print("[DEBUG] Entered sensor_ekf_thread()", flush=True)

    sensor_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sensor_sock.bind(("0.0.0.0", SENSOR_PORT))
    sensor_sock.settimeout(0.5)
    print(f"[Sensor] Listening for XRP encoder and IMU packets on UDP {SENSOR_PORT}")

    enc_count = imu_count = 0
    last_print = time.time()
    last_ignored_print = 0.0
    warned_old_xrp = False
    # Each stream's last packet number and when it arrived; for the encoders also
    # the last totals and the XRP's clock, for working out each packet's change
    last_enc_seq = None
    last_enc_seq_time = 0.0
    last_total_l = last_total_r = 0.0
    last_enc_ms = 0
    last_imu_seq = None
    last_imu_seq_time = 0.0
    lost_enc = lost_imu = 0
    # [time, x, y, theta] after each packet, for matching tag poses to the past
    pose_history = []
    last_tag_time = 0.0
    # Recent tag heading differences, (time, tag heading - estimate's), for
    # TAG_HEADING_WINDOW_S
    recent_heading_diffs = []
    # GYRO_SCALE: the last raw IMU heading, and the scaled heading built from it
    last_scale_src = None
    scaled_heading = 0.0
    # Tags rejected by CAMERA_NIS_GATE since when (None = the last one was used)
    cam_rejected_since = None
    # When the last "check this tag's facing" warning was printed
    last_facing_warning = 0.0
    # Per-tag summary for the status line: tag id -> running sums (see the print)
    tag_stats = {}
    # Gyro drift (see STILL_AFTER_S): when the wheels last turned, and the last IMU heading
    moved_time = time.time()
    last_raw_heading = None
    # imu_heading_offset is set from the first IMU packet after the filter starts
    heading_offset_set = False

    while running:
        try:
            data, addr = sensor_sock.recvfrom(256)
        except socket.timeout:
            continue

        try:
            text = data.decode("utf-8").strip()
        except UnicodeDecodeError:
            continue

        # ENC,seq,t_ms,total_l,total_r,throttle,servo_deg   encoder running totals + the inputs
        # IMU,seq,t_ms,heading                              IMU heading
        parts = text.split(",")
        kind = parts[0]
        try:
            if kind == "ENC" and len(parts) == 7:
                seq = int(parts[1]); t_ms = int(parts[2])
                total_l = float(parts[3]); total_r = float(parts[4])
                throttle = float(parts[5]); servo_deg = float(parts[6])
            elif kind == "IMU" and len(parts) == 4:
                seq = int(parts[1]); t_ms = int(parts[2])
                new_heading = float(parts[3])
            elif kind in ("RAW", "RAW2"):
                if not warned_old_xrp:
                    print(f"[Sensor] {addr[0]} is sending the old combined {kind} packets; "
                          "upload the new xrp_raw_sensors.py to it (ENC / IMU packets)")
                    warned_old_xrp = True
                continue
            else:
                continue
        except ValueError:
            continue

        # ---- Which XRP this laptop follows (first come, first served) ----
        now = time.time()
        with xrp_ip_lock:
            if xrp_ip is not None and addr[0] != xrp_ip and now - xrp_last_seen < XRP_LEASE_S:
                if now - last_ignored_print > 5.0:
                    print(f"[XRP] ignoring {addr[0]} (locked onto {xrp_ip})")
                    last_ignored_print = now
                continue
            if addr[0] != xrp_ip:
                if xrp_ip is not None:
                    print(f"[XRP] {xrp_ip} went quiet, switching to {addr[0]}")
                    # a different robot's encoders and heading: start the filter over
                    with ekf_lock:
                        ekf = None
                xrp_ip = addr[0]
                last_enc_seq = last_imu_seq = None
                print(f"[XRP] locked onto {xrp_ip}")
            xrp_last_seen = now

        if ekf is None:
            with ekf_lock:
                if ekf is None:
                    ekf = new_ekf()
                    heading_offset_set = False
                    last_raw_heading = None
                    pose_history = []
                    print(f"[EKF] Initialized for {addr[0]}")

        if kind == "ENC":
            # ---- Encoder packet: how far each wheel turned, and over how long ----
            if last_enc_seq is not None and seq <= last_enc_seq and now - last_enc_seq_time < 1.0:
                continue   # arrived late or twice; its totals are already counted
            if last_enc_seq is None or seq <= last_enc_seq:
                # first packet, or the count started over after a gap: the XRP
                # restarted (which takes several seconds), and its encoders did too.
                # Just take this one as the starting point
                new_enc_l = new_enc_r = 0.0
                enc_dt = 0.0
            else:
                # (after a gap without a restart, e.g. briefly out of Wi-Fi range,
                # the totals still include what was driven meanwhile)
                new_enc_l = total_l - last_total_l
                new_enc_r = total_r - last_total_r
                enc_dt = ((t_ms - last_enc_ms) % XRP_TICKS_PERIOD_MS) / 1000.0
                lost_enc += seq - last_enc_seq - 1
            last_enc_seq, last_total_l, last_total_r, last_enc_ms = seq, total_l, total_r, t_ms
            last_enc_seq_time = now
            enc_l, enc_r = new_enc_l, new_enc_r
            total_driven_m += (enc_l + enc_r) / 2 * 2 * math.pi * WHEEL_RADIUS
            if enc_l != 0.0 or enc_r != 0.0:
                moved_time = now
            enc_count += 1
        else:
            # ---- IMU packet ----
            if last_imu_seq is not None and seq <= last_imu_seq and now - last_imu_seq_time < 1.0:
                continue   # arrived late or twice
            if last_imu_seq is not None and seq > last_imu_seq:
                lost_imu += seq - last_imu_seq - 1
            last_imu_seq = seq
            last_imu_seq_time = now
            # IMU heading with its turns scaled by GYRO_SCALE; the raw turning is
            # also added up for measuring GYRO_SCALE
            if last_scale_src is None:
                scaled_heading = new_heading
            else:
                d_raw = (new_heading - last_scale_src + math.pi) % (2 * math.pi) - math.pi
                scaled_heading = (scaled_heading + GYRO_SCALE * d_raw + math.pi) % (2 * math.pi) - math.pi
                total_turned_raw += d_raw
            last_scale_src = new_heading
            heading = scaled_heading

            # ---- Gyro drift ----
            if last_raw_heading is not None and now - moved_time >= STILL_AFTER_S:
                # standing still: hold the heading
                d_heading = (heading - last_raw_heading + math.pi) % (2 * math.pi) - math.pi
                imu_heading_offset = (imu_heading_offset - d_heading + math.pi) % (2 * math.pi) - math.pi
            last_raw_heading = heading

            if not heading_offset_set:
                # start facing 90° like the position starts at 0,0, even if the
                # XRP's heading drifted before fusion started
                imu_heading_offset = ekf.wrap_pi(np.pi/2 - heading)
                heading_offset_set = True
                print(f"[EKF] Heading set from IMU heading={heading:.3f} rad")
            imu_count += 1

        with ekf_lock:
            if kind == "ENC":
                # move by the wheel turns since the last encoder packet
                x, y, theta = ekf.predict(enc_l, enc_r)
                if enc_dt > 0:   # speed for the status line and the overlay
                    speed = (enc_l + enc_r) / 2 * 2 * math.pi * WHEEL_RADIUS / enc_dt
            else:
                x, y, theta = ekf.correct(ekf.wrap_pi(heading + imu_heading_offset))
                record_nis("IMU")

            with latest_robot_pose_lock:
                pose_w = latest_robot_pose_world
            tag_is_fresh = (
                pose_w is not None
                and (time.time() - pose_w["time"]) < TAG_FRESHNESS_S
            )

            now = time.time()
            pose_history.append([now, x, y, theta])
            while pose_history and now - pose_history[0][0] > POSE_HISTORY_S:
                pose_history.pop(0)

            # A new tag pose: the EKF's camera correction
            if tag_is_fresh and APPLY_CAMERA_CORRECTION and pose_w["time"] != last_tag_time:
                last_tag_time = pose_w["time"]
                tag_dist = math.hypot(pose_w["rel_x"], pose_w["rel_y"])
                st = tag_stats.setdefault(int(pose_w["tag_id"]), {
                    "n": 0, "used_n": 0, "along": 0.0, "across": 0.0, "heading": 0.0, "dist": 0.0,
                    "nis": 0.0, "rejected": 0, "heading_ignored": 0, "heading_consistent": 0, "too_far": 0})
                st["n"] += 1
                st["dist"] += tag_dist
                if tag_dist > MAX_TAG_DISTANCE_M:
                    # too far to trust: skipped entirely (no correction, and it doesn't
                    # count towards the heading or rejection checks)
                    st["too_far"] += 1
                else:
                    taken = pose_w["time"] - CAMERA_LATENCY_S
                    then = min(pose_history, key=lambda h: abs(h[0] - taken))
                    # Position from where the tag was relative to the robot, turned by the
                    # estimate's heading, so it doesn't move when the tag's angle flips
                    c, s = math.cos(then[3]), math.sin(then[3])
                    tag_pos_x = pose_w["tag_x"] - (c * pose_w["rel_x"] - s * pose_w["rel_y"])
                    tag_pos_y = pose_w["tag_y"] - (s * pose_w["rel_x"] + c * pose_w["rel_y"])
                    # that's the tags' point on the kart; the filter tracks the rear axle
                    tag_pos_x -= TAG_POINT_AHEAD_OF_REAR_AXLE_M * c
                    tag_pos_y -= TAG_POINT_AHEAD_OF_REAR_AXLE_M * s
                    tag_theta = pose_w["theta"]
                    heading_diff = ekf.wrap_pi(tag_theta - then[3])
                    raw_heading_diff = heading_diff
                    heading_ignored = heading_consistent = False
                    recent_heading_diffs.append([now, raw_heading_diff])
                    while recent_heading_diffs and now - recent_heading_diffs[0][0] > TAG_HEADING_WINDOW_S:
                        recent_heading_diffs.pop(0)
                    gate = math.radians(TAG_HEADING_GATE_DEG)
                    if abs(heading_diff) > gate:
                        diffs = [d for _, d in recent_heading_diffs]
                        med = statistics.median(diffs)
                        spread = statistics.median([abs(d - med) for d in diffs])
                        consistent = (len(diffs) >= TAG_HEADING_MIN_READINGS and abs(med) > gate
                                      and spread <= math.radians(TAG_HEADING_MAX_SPREAD_DEG))
                        if consistent and abs(med) > math.radians(TAG_HEADING_MAX_CONSISTENT_DEG):
                            # too big to be drift: probably this tag's facing in TAG_WORLD
                            heading_diff = 0.0
                            heading_ignored = True
                            if now - last_facing_warning > 5.0:
                                last_facing_warning = now
                                print(f"[Tag]    WARNING tag {int(pose_w['tag_id'])}'s heading is a steady "
                                      f"{math.degrees(med):+.0f} deg off the estimate: check its gate ends "
                                      f"(facing) in laptop_fusion.py. Its headings are being ignored")
                        elif consistent:
                            # the recent tags agree: the estimate's heading is off
                            heading_diff = med
                            heading_consistent = True
                        else:
                            heading_diff = 0.0   # probably the flipped answer: no heading correction
                            heading_ignored = True
                    # The tag measured the pose at `then`; move it on by however far the
                    # filter has moved since, so it's compared with the estimate now
                    meas_x = tag_pos_x + (x - then[1])
                    meas_y = tag_pos_y + (y - then[2])
                    # (not wrapped to -180..180: the old EKF doesn't wrap the camera's
                    # heading difference, so it's kept next to the estimate's heading)
                    meas_theta = theta + heading_diff

                    # How far off the tag says the estimate is, along and across the
                    # kart's heading, for the per-tag summary
                    ix, iy = meas_x - x, meas_y - y
                    c_now, s_now = math.cos(theta), math.sin(theta)
                    cam_nis = ekf.camera_nis(meas_x, meas_y, meas_theta)
                    record_nis("camera", cam_nis, 3)
                    rejected = cam_nis > CAMERA_NIS_GATE
                    if rejected:
                        if cam_rejected_since is None:
                            cam_rejected_since = now
                        elif now - cam_rejected_since >= CAMERA_RESYNC_S:
                            rejected = False   # every tag for a while: trust them
                    st["used_n"] += 1
                    st["along"] += ix * c_now + iy * s_now
                    st["across"] += -ix * s_now + iy * c_now
                    st["heading"] += math.degrees(raw_heading_diff)
                    st["nis"] += cam_nis
                    st["rejected"] += rejected
                    st["heading_ignored"] += heading_ignored
                    st["heading_consistent"] += heading_consistent
                    if not rejected:
                        cam_rejected_since = None
                        try:
                            theta_before = theta
                            x, y, theta = ekf.camera_correction(meas_x, meas_y, meas_theta)
                            dth = ekf.wrap_pi(theta - theta_before)
                            # Heading follows the IMU, whose next correction would undo the
                            # camera's change, so the IMU's offset takes it, plus a part of
                            # the heading difference that's left when the tag's heading is
                            # used at all (TAG_HEADING_GAIN; this is what corrects gyro
                            # drift). The next IMU packet then turns the estimate
                            extra = 0.0 if heading_ignored else TAG_HEADING_GAIN * ekf.wrap_pi(meas_theta - theta)
                            imu_heading_offset = ekf.wrap_pi(imu_heading_offset + dth + extra)
                            # the past moves with it, so the next tag isn't compared with stale history
                            dx, dy = x - pose_history[-1][1], y - pose_history[-1][2]
                            for h in pose_history:
                                h[1] += dx; h[2] += dy; h[3] += dth
                            # and the stored heading differences are now that much smaller
                            for e in recent_heading_diffs:
                                e[1] -= dth + extra
                        except Exception as e:
                            print(f"[EKF] camera_correction error: {e}")

        # (only this thread changes the filter, so it can be read here unlocked)
        cov = ekf.get_uncertainty()[:3, :3].tolist()
        with latest_fused_lock:
            # the output point (OUTPUT_POINT_AHEAD_OF_REAR_AXLE_M); the filter tracks the rear axle
            latest_fused = {"x": x + OUTPUT_POINT_AHEAD_OF_REAR_AXLE_M * math.cos(theta),
                            "y": y + OUTPUT_POINT_AHEAD_OF_REAR_AXLE_M * math.sin(theta),
                            "theta": theta, "v": speed, "cov": cov}

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
            print(f"[EKF] {enc_count} enc + {imu_count} imu pkts/s ({lost_enc}/{lost_imu} lost) | "
                  f"FUSED x={x:+.2f} y={y:+.2f} th={math.degrees(theta):+.0f} v={speed:+.2f} | {tag_info}")
            enc_count = imu_count = 0; lost_enc = lost_imu = 0; last_print = now
            # Per tag, on average: where the tag put the kart compared with the
            # estimate (ahead / left of it, and turned). A steady "ahead" while
            # driving (and none while parked) = CAMERA_LATENCY_S is off; an offset
            # for one tag only = that tag's entry in TAG_WORLD is off; offsets that
            # grow with distance = tag size, camera calibration or lens distortion
            for tid in sorted(tag_stats):
                st = tag_stats[tid]
                n, u = st["n"], st["used_n"]
                line = f"[Tag]    tag {tid}: {n} seen, {st['dist']/n:.2f} m away"
                if st["too_far"]:
                    line += f", {st['too_far']} too far (> {MAX_TAG_DISTANCE_M:g} m)"
                if u:
                    line += (f" | puts the kart {st['along']/u*100:+.1f} cm ahead, "
                             f"{st['across']/u*100:+.1f} cm left, {st['heading']/u:+.1f} deg turned "
                             f"left of the estimate | NIS {st['nis']/u:.1f}, {st['rejected']} rejected, "
                             f"{st['heading_ignored']} headings ignored, "
                             f"{st['heading_consistent']} taken from consistent readings")
                print(line)
            tag_stats.clear()

    sensor_sock.close()


# ============================================================
# Video forwarder to Godot (raw RGB over TCP)
# ============================================================
godot_video_conn = None


def godot_video_server_thread():
    global godot_video_conn
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
    relay_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    relay_sock.bind(("127.0.0.1", GAMEPAD_RELAY_PORT))
    relay_sock.settimeout(0.5)

    forward_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        forward_sock.setsockopt(socket.IPPROTO_IP, socket.IP_TOS, CONTROL_PACKET_TOS)
        print("[Gamepad] Control packets marked high priority")
    except (OSError, AttributeError) as e:
        print(f"[Gamepad] Could not mark control packets high priority ({e}), sending normally")
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
            pkt_count += 1
        except OSError as e:
            print(f"[Gamepad] Forward error: {e}")

        now = time.time()
        if now - last_print >= 5.0:
            print(f"[Gamepad] Forwarded {pkt_count} packets to {target_ip} in last 5s")
            pkt_count = 0
            last_print = now


# ============================================================
# Main: starts the threads, then sends the fused pose to Godot
# ============================================================
def send_debug_pose(sock, fused):
    """The estimate in the map frame (metres, radians) with its covariance, the
    latest tag-only pose if it's fresh, and the NIS of the corrections since the
    last packet, to DEBUG_POSE_PORT for pose_overlay.py."""
    global nis_pending
    msg = {"x": fused["x"], "y": fused["y"], "theta": fused["theta"],
           "v": fused["v"], "cov": fused["cov"], "time": time.time()}
    # the corrections' NIS since the last packet: {kind: {"dof", "values"}}
    with latest_fused_lock:
        pending, nis_pending = nis_pending, {}
    if pending:
        msg["nis"] = {k: {"dof": d, "values": v} for k, (d, v) in pending.items()}
    with latest_robot_pose_lock:
        tag = latest_robot_pose_world
    if tag is not None and time.time() - tag["time"] < TAG_FRESHNESS_S:
        msg["tag"] = {"x": tag["x"], "y": tag["y"], "theta": tag["theta"], "id": int(tag["tag_id"])}
    try:
        sock.sendto(json.dumps(msg).encode(), (DEBUG_POSE_HOST, DEBUG_POSE_PORT))
    except OSError:
        pass   # nothing listening is fine


def main():
    global running

    pose_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"[Pose] Will send fused state to {GODOT_HOST}:{GODOT_POSE_PORT}")

    threading.Thread(target=grabber_thread, daemon=True).start()
    threading.Thread(target=detector_thread, daemon=True).start()
    threading.Thread(target=godot_video_server_thread, daemon=True).start()
    threading.Thread(target=gamepad_relay_thread, daemon=True).start()
    threading.Thread(target=ar_tags_thread, daemon=True).start()

    sensor_thread = threading.Thread(target=sensor_ekf_thread, daemon=True, name="SensorEKF")
    print("[DEBUG] Starting sensor thread", flush=True)
    sensor_thread.start()
    print("[DEBUG] Sensor thread started:", sensor_thread.is_alive(), flush=True)

    sent_count = 0
    last_print = time.time()

    try:
        while running:
            try:
                # Only once an XRP has been heard recently and the filter has a state;
                # otherwise the game would treat the default pose as a live robot
                with xrp_ip_lock:
                    xrp_heard = xrp_ip is not None and time.time() - xrp_last_seen < 1.0
                with latest_fused_lock:
                    fused = latest_fused
                if xrp_heard and fused is not None:
                    payload = json.dumps({
                        "x": fused["y"] * 100,
                        "y": -(fused["x"] * 100),
                        "angle": math.degrees(fused["theta"]),
                    })
                    pose_sock.sendto(payload.encode(), (GODOT_HOST, GODOT_POSE_PORT))
                    send_debug_pose(pose_sock, fused)
                    sent_count += 1
            except Exception as e:
                print("[Pose] send error:", e)

            now = time.time()
            if now - last_print >= PRINT_EVERY_S:
                pw = pose_w
                if pw is None:
                    tag_info = "no tag yet"
                else:
                    age = now - pw["time"]
                    if age < TAG_FRESHNESS_S:
                        tag_info = f"tag fresh ({age*1000:.0f}ms old)"
                    else:
                        tag_info = f"tag STALE ({age:.1f}s old)"
                rate = sent_count / (now - last_print)
                print(f"[Sensor] enc_l={enc_l:+.3f} enc_r={enc_r:+.3f} rev | "
                      f"heading={math.degrees(heading):+.1f}° | since start: turned "
                      f"{math.degrees(total_turned_raw):+.1f}° (IMU, raw), drove {total_driven_m:.3f} m "
                      f"(encoders) | {rate:.0f} poses/s to game")
                print(f"[EKF]    fused x={x:.2f} y={y:.2f} m "
                      f"th={math.degrees(theta):+.0f}° v={speed:+.2f} m/s | {tag_info}")
                sent_count = 0
                last_print = now

            time.sleep(1.0 / POSE_SEND_HZ)

    except KeyboardInterrupt:
        print("\n[Main] Interrupted, shutting down...")
    except Exception:
        traceback.print_exc()
    finally:
        running = False
        time.sleep(0.3)


if __name__ == "__main__":
    main()