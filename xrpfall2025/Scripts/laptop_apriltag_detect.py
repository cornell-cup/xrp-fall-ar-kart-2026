# IMPORTANT: these env vars must be set BEFORE `import cv2`.
# Using TCP transport for RTSP — packets are retransmitted on loss, so
# you get a clean image at the cost of occasional latency spikes when
# retransmits happen. On a clean local network (hotspot or close-range
# router) TCP is barely slower than UDP and the visual quality is
# dramatically better.
import os
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
    "rtsp_transport;tcp"
    "|fflags;nobuffer"
    "|flags;low_delay"
    "|reorder_queue_size;0"
    "|max_delay;0"
    "|buffer_size;65536"
)
# Quiet down FFmpeg's logger so the terminal stays readable
os.environ["OPENCV_LOG_LEVEL"] = "ERROR"
os.environ["AV_LOG_FORCE_NOCOLOR"] = "1"

import sys
import time
import threading
import socket
import struct
import cv2

"""
laptop_apriltag_detect.py

Pulls RTSP stream from the Pi (via MediaMTX) and forwards RAW RGB
pixel bytes to Godot via TCP on port 6001. Detection is intentionally
stubbed out — add it back later as needed.

Architecture:
  A background thread continuously reads frames from the Pi as fast as
  the network delivers them, always overwriting `latest_frame`. The main
  loop ships only NEW frames to Godot — duplicates are skipped using a
  monotonically increasing frame counter.

Protocol (per frame to Godot):
  [4 bytes width  big-endian uint32]
  [4 bytes height big-endian uint32]
  [4 bytes length big-endian uint32]  # = width * height * 3
  [length bytes raw RGB pixel data]   # row-major, top-down
"""

# Pi RTSP stream
PI_IP = sys.argv[1] if len(sys.argv) > 1 else "192.168.1.3"
RTSP_URL = f"rtsp://{PI_IP}:8554/cam"

# Send to Godot
VIDEO_PORT = 6001
video_conn = None

# ------- Shared state for the grabber thread -------
latest_frame = None
latest_frame_id = 0          # incremented every time the grabber gets a new frame
latest_frame_lock = threading.Lock()
grabber_running = True


def send_video_frame_rgb(rgb_bytes, width, height):
    """Send a frame as a raw RGB byte buffer with a 12-byte header."""
    global video_conn
    if video_conn is None:
        return
    try:
        header = struct.pack("!III", width, height, len(rgb_bytes))
        video_conn.sendall(header + rgb_bytes)
    except (BrokenPipeError, ConnectionResetError, OSError):
        print("[Video] Godot disconnected.")
        video_conn = None


def wait_for_godot_connection(server_sock):
    global video_conn
    try:
        server_sock.settimeout(0.01)
        conn, addr = server_sock.accept()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        if video_conn is not None:
            try:
                video_conn.close()
            except:
                pass
        video_conn = conn
        print(f"[Video] Godot connected from {addr}")
    except socket.timeout:
        pass


def open_rtsp(url):
    cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
    cap.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 10000)
    cap.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, 10000)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def grabber_thread(rtsp_url):
    """Continuously read frames as fast as the network delivers them.
    Always overwrites `latest_frame` so the main loop sees the freshest
    one. Old frames die silently. Handles its own reconnect logic.
    """
    global latest_frame, latest_frame_id, grabber_running

    while grabber_running:
        print(f"[Camera] Connecting to RTSP stream: {rtsp_url}")
        cap = open_rtsp(rtsp_url)
        if not cap.isOpened():
            print("[Camera] Failed to open, retrying in 3s...")
            cap.release()
            time.sleep(3)
            continue

        print("[Camera] Connected to Pi RTSP stream!")
        consecutive_failures = 0

        while grabber_running:
            ret, frame = cap.read()
            if not ret:
                consecutive_failures += 1
                if consecutive_failures > 10:
                    print("[Camera] Lost connection, reconnecting...")
                    break
                time.sleep(0.05)
                continue

            consecutive_failures = 0
            with latest_frame_lock:
                latest_frame = frame
                latest_frame_id += 1

        cap.release()
        if grabber_running:
            time.sleep(2)


def main():
    global video_conn, grabber_running

    # ---- TCP server for video to Godot ----
    video_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    video_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    video_server.bind(("0.0.0.0", VIDEO_PORT))
    video_server.listen(5)
    print(f"[Video] TCP server listening on port {VIDEO_PORT}")
    print("[Video] Waiting for Godot to connect...")

    # ---- Start background grabber thread ----
    grabber = threading.Thread(target=grabber_thread, args=(RTSP_URL,), daemon=True)
    grabber.start()

    # Wait for first frame
    print("[Camera] Waiting for first frame...")
    waited = 0
    while True:
        with latest_frame_lock:
            have_frame = latest_frame is not None
        if have_frame:
            break
        time.sleep(0.5)
        waited += 1
        if waited > 60:
            print("[Camera] ERROR: No frames received after 30s. Giving up.")
            grabber_running = False
            return
    print("[Camera] First frame received, starting forwarding loop")

    last_processed_id = -1
    fps_counter = 0
    fps_last_print = time.time()

    try:
        while True:
            wait_for_godot_connection(video_server)

            # Only process new frames — skip duplicates so FPS reports
            # actual frame rate from the Pi, not loop iterations.
            with latest_frame_lock:
                if latest_frame is None or latest_frame_id == last_processed_id:
                    img_bgr = None
                else:
                    img_bgr = latest_frame
                    last_processed_id = latest_frame_id

            if img_bgr is None:
                time.sleep(0.001)
                continue

            # Convert BGR -> RGB (Godot's FORMAT_RGB8 expects RGB)
            img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
            h, w = img_rgb.shape[:2]

            send_video_frame_rgb(img_rgb.tobytes(), w, h)
            fps_counter += 1

            # Print FPS every second
            now = time.time()
            if now - fps_last_print >= 1.0:
                status = "connected" if video_conn else "waiting"
                print(f"[FPS] {fps_counter} fps | Godot: {status}")
                fps_counter = 0
                fps_last_print = now

    except KeyboardInterrupt:
        print("\n[Main] Interrupted, shutting down...")
    finally:
        grabber_running = False
        time.sleep(0.2)
        video_server.close()


if __name__ == "__main__":
    main()