"""
ar_stream.py - Camera + Tag detection stream to Godot
  TCP 6001: JPEG frames (4-byte length prefix, 960x540)
  UDP 6000: Tag detection JSON

Detection: AprilTag tag36h11 (same as laptop_apriltag_detect.py)

Usage:
  OPENCV_AVFOUNDATION_SKIP_AUTH=1 python3 ar_stream.py --webcam
  python3 ar_stream.py --webcam --headless
  python3 ar_stream.py --rtsp rtsp://pi-ip:8554/cam
"""

import os
import sys
import time
import socket
import struct
import json
import threading
import argparse
import math
import numpy as np
import cv2
from contextlib import contextmanager
from pupil_apriltags import Detector


# ── Shared with laptop_apriltag_detect.py ─────────────────────────

GODOT_IP = "127.0.0.1"

# Pose data -- UDP
POSE_PORT = 6000
pose_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Video frames -- TCP
VIDEO_PORT = 6001
video_conn = None

# Camera intrinsics (for 640x480)
FRAME_W = 640.0
FRAME_H = 480.0
fx = 800.0
fy = 800.0
cx = FRAME_W / 2.0
cy = FRAME_H / 2.0
camera_params = [fx, fy, cx, cy]

# AprilTag config
TAG_SIZE = 0.05

detector = Detector(
    families="tag36h11",
    nthreads=2,
    quad_decimate=2.0,
    refine_edges=True
)

# Send resolution
SEND_W, SEND_H = 960, 540
JPEG_QUALITY = 80

GATE_DEFS = {
    0: {"name": "START/FINISH", "type": "finish",  "event": "checkpoint"},
    1: {"name": "SPEED BOOST",  "type": "boost",   "event": "speed_boost"},
    2: {"name": "ITEM BOX",     "type": "item",    "event": "mystery_box"},
    3: {"name": "HAZARD",       "type": "hazard",  "event": "banana_trap"},
}


@contextmanager
def suppress_stderr():
    devnull = os.open(os.devnull, os.O_WRONLY)
    orig = os.dup(2)
    try:
        os.dup2(devnull, 2)
        yield
    finally:
        os.dup2(orig, 2)
        os.close(orig)
        os.close(devnull)


def rotation_euler_from_R(R):
    yaw = math.atan2(R[0, 2], R[2, 2])
    pitch = -math.asin(R[1, 2])
    roll = math.atan2(R[1, 0], R[1, 1])
    return roll, pitch, yaw


# ── TCP video server ──────────────────────────────────────────────

def send_video_frame(jpg_bytes):
    global video_conn
    if video_conn is None:
        return
    try:
        header = struct.pack("!I", len(jpg_bytes))
        video_conn.sendall(header + jpg_bytes)
    except (BrokenPipeError, ConnectionResetError, OSError):
        print("[Video] Godot disconnected.")
        video_conn = None


def wait_for_godot_connection(server_sock):
    global video_conn
    try:
        server_sock.settimeout(0.01)
        conn, addr = server_sock.accept()
        if video_conn is not None:
            try:
                video_conn.close()
            except:
                pass
        video_conn = conn
        print(f"[Video] Godot connected from {addr}")
    except socket.timeout:
        pass


# ── UDP tag sender (JSON for ar_manager.gd) ───────────────────────

def send_tags(detections, frame_w, frame_h):
    """Send tag data to Godot. Corners scaled from camera res to SEND_W x SEND_H."""
    scale_x = SEND_W / float(frame_w)
    scale_y = SEND_H / float(frame_h)

    tag_list = []
    for d in detections:
        tag_id = d.tag_id
        if tag_id not in GATE_DEFS:
            continue
        g = GATE_DEFS[tag_id]

        # Scale corners/center to match the frame size Godot receives
        corners = [[c[0] * scale_x, c[1] * scale_y] for c in d.corners.tolist()]
        center = [d.center[0] * scale_x, d.center[1] * scale_y]

        pose_R = []
        pose_t = []
        if d.pose_R is not None and d.pose_t is not None:
            pose_R = d.pose_R.tolist()
            pose_t = d.pose_t.flatten().tolist()

        tag_list.append({
            "id": tag_id, "name": g["name"], "type": g["type"], "event": g["event"],
            "corners": corners, "center": center,
            "pose_R": pose_R, "pose_t": pose_t,
        })
    if not tag_list:
        return
    try:
        data = json.dumps({"type": "ar_tags", "tags": tag_list, "timestamp": time.time()})
        pose_sock.sendto(data.encode(), (GODOT_IP, POSE_PORT))
    except:
        pass


# ── Camera source ─────────────────────────────────────────────────

def open_rtsp(url):
    """Open RTSP stream with robust settings."""
    cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
    cap.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 10000)
    cap.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, 10000)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def open_webcam():
    """Open local webcam."""
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 540)
    return cap


# ── Main ──────────────────────────────────────────────────────────

def main():
    global video_conn

    parser = argparse.ArgumentParser()
    parser.add_argument("--rtsp", type=str, help="RTSP URL (e.g. rtsp://10.42.0.1:8554/cam)")
    parser.add_argument("--webcam", action="store_true", help="Use local webcam")
    parser.add_argument("--headless", action="store_true", help="No preview window")
    args = parser.parse_args()

    # ---- TCP server for video to Godot ----
    video_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    video_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    video_server.bind(("0.0.0.0", VIDEO_PORT))
    video_server.listen(5)
    print(f"[Video] TCP server listening on port {VIDEO_PORT}")
    print(f"[Video] Waiting for Godot to connect...")

    # ---- Open camera source ----
    if args.rtsp:
        print(f"[Camera] Connecting to RTSP stream: {args.rtsp}")
        cap = None
        for attempt in range(30):
            cap = open_rtsp(args.rtsp)
            if cap.isOpened():
                break
            print(f"[Camera] Attempt {attempt + 1}/30 failed, retrying in 3s...")
            cap.release()
            cap = None
            time.sleep(3)
        if cap is None or not cap.isOpened():
            print("[Camera] ERROR: Cannot open RTSP stream after 30 attempts.")
            return
        print("[Camera] Connected to RTSP stream!")
    else:
        print("[Camera] Opening local webcam...")
        cap = open_webcam()
        if not cap.isOpened():
            print("[Camera] ERROR: Cannot open webcam!")
            return
        print("[Camera] Webcam ready!")

    frame_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # Update camera params for actual resolution
    cam_params = [
        800.0 * frame_w / 640.0,
        800.0 * frame_h / 480.0,
        frame_w / 2.0,
        frame_h / 2.0,
    ]

    print(f"[AR] Capture: {frame_w}x{frame_h} | Send: {SEND_W}x{SEND_H} q{JPEG_QUALITY}")
    print(f"[AR] Tags->UDP:{POSE_PORT} Frames->TCP:{VIDEO_PORT}")
    if not args.headless:
        print("[AR] Press 'q' to quit")

    frame_count = 0
    fail_count = 0

    while True:
        wait_for_godot_connection(video_server)

        ret, img = cap.read()
        if not ret:
            fail_count += 1
            if fail_count >= 5:
                if args.rtsp:
                    print("[Camera] Lost RTSP connection, reconnecting...")
                    cap.release()
                    time.sleep(2)
                    cap = open_rtsp(args.rtsp)
                fail_count = 0
            continue

        fail_count = 0
        frame_count += 1

        # -------- APRILTAG DETECTION (same as laptop_apriltag_detect.py) --------
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        with suppress_stderr():
            detections = detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=cam_params,
                tag_size=TAG_SIZE,
            )

        # -------- SEND TAGS TO GODOT (scaled to SEND_W x SEND_H) --------
        if detections:
            send_tags(detections, frame_w, frame_h)

        # -------- SEND VIDEO FRAME --------
        if frame_count % 3 == 0:
            small = cv2.resize(img, (SEND_W, SEND_H))
            ret_enc, encoded = cv2.imencode(
                ".jpg", small, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
            )
            if ret_enc:
                send_video_frame(encoded.tobytes())

        # -------- DRAW OVERLAYS (preview only) --------
        if not args.headless:
            for d in detections:
                tag_id = d.tag_id
                corners = d.corners.astype(int)
                cx_p, cy_p = d.center
                top_y = int(min(corners[:, 1]))
                bot_y = int(max(corners[:, 1]))

                for i in range(4):
                    p1 = tuple(corners[i])
                    p2 = tuple(corners[(i + 1) % 4])
                    cv2.line(img, p1, p2, (0, 255, 0), 2)

                cv2.circle(img, (int(cx_p), int(cy_p)), 6, (0, 0, 255), -1)

                gate_name = GATE_DEFS.get(tag_id, {}).get("name", f"Tag {tag_id}")
                cv2.putText(
                    img, f"MARIO KART AR",
                    (int(cx_p) - 100, top_y - 10),
                    cv2.FONT_HERSHEY_DUPLEX, 0.8,
                    (255, 255, 255), 2, cv2.LINE_AA
                )
                cv2.putText(
                    img, gate_name,
                    (int(cx_p) - 60, bot_y + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 215, 255), 2, cv2.LINE_AA
                )

                if d.pose_t is not None:
                    tz = d.pose_t[2][0]
                    cv2.putText(
                        img, f"{tz:.2f}m",
                        (int(cx_p) - 30, int(cy_p) + 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (255, 255, 0), 2, cv2.LINE_AA
                    )

            status = "connected" if video_conn else "waiting"
            cv2.rectangle(img, (0, 0), (frame_w, 30), (0, 0, 0), -1)
            cv2.putText(img, f"Tags: {len(detections)} | Godot: {status}",
                        (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.imshow("AR Kart Stream (Q to quit)", img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            time.sleep(0.001)

        if frame_count % 100 == 0:
            status = "connected" if video_conn else "waiting"
            print(f"[Status] Frame {frame_count}, Godot: {status}, Tags: {len(detections)}")

    cap.release()
    video_server.close()
    if not args.headless:
        cv2.destroyAllWindows()
    print("[AR] Done.")


if __name__ == "__main__":
    main()
