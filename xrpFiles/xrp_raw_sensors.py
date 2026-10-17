"""
xrp_raw_sensors.py

Slim XRP-side script with persistent logging AND silent-failure
detection.

Power-related changes for brownout mitigation:
  - MAX_DRIVE_RPM lowered from 200 to 100 (~half the current draw)
  - Throttle is RAMPED instead of jumped, so a 0->full transition
    takes ~250ms instead of being instant. This eliminates the big
    inrush-current spikes that were browning out the shared 5V rail.

Gamepad protocol from laptop (trimmed):
    [0x55][0x04][1][throttle_byte][2][steer_byte]   = 6 bytes

After a crash/disconnect, plug XRP into USB, open Thonny, and run:
    print(open('/xrp_log.txt').read())
"""

import time, math, network, socket, os
from XRPLib.defaults import *

# ============================================================
# WiFi Config
# ============================================================
# Networks to join, in order of preference. The XRP joins whichever one
# it can see, so the same file works with the Pi hotspot OR CornellCup-Web.
# Open network -> password "" (giving a password to an open network makes
# the Pico W attempt WPA2 and fail).
WIFI_NETWORKS = [
    ("XRP-Robot", ""),                          # Pi hotspot (open)
    ("CornellCup-Web", "passwordhere"),
]

# Sensor packets are broadcast to the whole local network, so the laptop's
# IP never needs to be known. The broadcast address is worked out from
# whatever network we joined (10.42.0.255 on the hotspot, 192.168.4.255 on
# CornellCup-Web, ...). Set a fixed string here only to override that.
LAPTOP_BROADCAST_IP = None

SENSOR_PORT = 5005
GAMEPAD_PORT = 4002
SEND_RATE_HZ = 50

# ============================================================
# Robot config
# ============================================================
JOYSTICK_DEADBAND = 0.05

# Lowered from 200 to reduce average motor current draw.
# Bump back up once power supply is properly isolated.
MAX_DRIVE_RPM = 200

# Throttle ramping — limits how fast the motor command can change
# per main-loop tick (~5ms). 0.02 per tick means 0->full takes
# 50 ticks ~= 250ms, eliminating inrush-current spikes that brown
# out the Pi.
# Larger value = snappier response but higher current spikes.
# Smaller value = smoother current draw but more "lag" in feel.
THROTTLE_RAMP_PER_TICK = 0.02

# From xrpgodot.py (SERVO_CENTER / SERVO_RANGE), the values the kart
# currently drives straight with.
STEER_CENTER_DEG = 100.0
STEER_RANGE_DEG = 75.0

GAMEPAD_HEADER = 0x55
GAMEPAD_TIMEOUT_MS = 500

# ============================================================
# Silent-failure detection
# ============================================================
DEAF_THRESHOLD_MS = 2000
DEAF_SEND_ERRORS_MIN = 5
HEARTBEAT_EVERY_MS = 5000

# ============================================================
# Logging
# ============================================================
LOG_PATH = "/xrp_log.txt"
LOG_MAX_BYTES = 50 * 1024
LOG_TO_PRINT = True
LOG_TO_FILE = True

_log_file = None
_boot_time_ms = time.ticks_ms()


def _log_init():
    global _log_file
    if not LOG_TO_FILE:
        return
    try:
        try:
            size = os.stat(LOG_PATH)[6]
            if size > LOG_MAX_BYTES:
                try:
                    with open(LOG_PATH, "r") as f:
                        f.seek(max(0, size - LOG_MAX_BYTES // 2))
                        f.readline()
                        keep = f.read()
                    with open(LOG_PATH, "w") as f:
                        f.write("=== rotated ===\n")
                        f.write(keep)
                except Exception:
                    open(LOG_PATH, "w").close()
        except OSError:
            pass

        _log_file = open(LOG_PATH, "a")
        _log_file.write("\n=== boot at ms_since_epoch={} ===\n".format(time.ticks_ms()))
        _log_file.flush()
    except Exception as e:
        print("Could not open log file:", e)
        _log_file = None


def log(level, tag, msg):
    elapsed_s = time.ticks_diff(time.ticks_ms(), _boot_time_ms) / 1000.0
    line = "[{:8.3f}s {} {}] {}".format(elapsed_s, level, tag, msg)
    if LOG_TO_PRINT:
        print(line)
    if LOG_TO_FILE and _log_file is not None:
        try:
            _log_file.write(line + "\n")
            _log_file.flush()
        except Exception as e:
            print("log write failed:", e)


def log_info(tag, msg):  log("INFO ", tag, msg)
def log_warn(tag, msg):  log("WARN ", tag, msg)
def log_error(tag, msg): log("ERROR", tag, msg)


# ============================================================
# Heading helpers
# ============================================================
HEADING_SIGN = 1.0
HEADING_OFFSET = 0.0

def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi

def imu_heading_rad():
    roll_deg = imu.get_roll()
    return wrap_pi(HEADING_SIGN * math.radians(roll_deg) + HEADING_OFFSET)


# ============================================================
# Joystick state
# ============================================================
joy_x = 0.0
joy_y = 0.0
gamepad_active = False
gamepad_last_recv = 0
gamepad_ever_received = False

def parse_gamepad_bytes(data):
    """Parse [0x55][n*2][idx,val,idx,val,...].
    Idx 1 = throttle, idx 2 = steer."""
    global joy_x, joy_y, gamepad_active, gamepad_last_recv, gamepad_ever_received
    if len(data) < 2 or data[0] != GAMEPAD_HEADER:
        return False
    if len(data) != data[1] + 2:
        return False
    new_throttle = None
    new_steer = None
    for i in range(2, data[1] + 2, 2):
        idx = data[i]
        val = data[i + 1] / 127.5 - 1.0
        if idx == 1:
            new_throttle = val
        elif idx == 2:
            new_steer = val
    if new_throttle is not None:
        joy_y = -round(new_throttle, 2)
    if new_steer is not None:
        joy_x = -round(new_steer, 2)
    gamepad_active = True
    gamepad_last_recv = time.ticks_ms()
    gamepad_ever_received = True
    return True


# ============================================================
# WiFi
# ============================================================
wlan = network.WLAN(network.STA_IF)

def stop_motors():
    motor_three.set_effort(0)
    motor_four.set_effort(0)
    servo_three.set_angle(STEER_CENTER_DEG)


def hard_wifi_reset():
    log_warn("WIFI", "performing hard chip reset")
    try:
        wlan.disconnect()
    except Exception as e:
        log_warn("WIFI", "disconnect during reset: " + str(e))
    try:
        wlan.active(False)
    except Exception as e:
        log_warn("WIFI", "active(False) failed: " + str(e))
    time.sleep(1)
    try:
        wlan.active(True)
    except Exception as e:
        log_error("WIFI", "active(True) failed: " + str(e))
    time.sleep(1)


def pick_network(attempt):
    """Return (ssid, password) of a configured network that's in range.
    If scanning fails, just rotate through the list."""
    try:
        visible = set()
        for entry in wlan.scan():
            try:
                visible.add(entry[0].decode())
            except Exception:
                pass
        for ssid, pw in WIFI_NETWORKS:
            if ssid in visible:
                return ssid, pw
        log_warn("WIFI", "none of {} in range".format([n[0] for n in WIFI_NETWORKS]))
    except Exception as e:
        log_warn("WIFI", "scan failed: " + str(e))
    return WIFI_NETWORKS[attempt % len(WIFI_NETWORKS)]


def connect_wifi():
    wlan.active(True)
    if wlan.isconnected():
        ip = wlan.ifconfig()[0]
        log_info("WIFI", "Already connected, IP=" + ip)
        return ip

    time.sleep(2)

    for attempt in range(20):
        try:
            wlan.disconnect()
        except Exception as e:
            log_warn("WIFI", "disconnect raised: " + str(e))
        time.sleep(1)

        ssid, pw = pick_network(attempt)
        log_info("WIFI", "trying '" + ssid + "'")
        try:
            if pw:
                wlan.connect(ssid, pw)
            else:
                wlan.connect(ssid)   # open network: no key at all
        except Exception as e:
            log_error("WIFI", "connect raised: " + str(e))
            time.sleep(2)
            continue

        start = time.time()
        while not wlan.isconnected():
            if time.time() - start > 8:
                break
            time.sleep(0.5)

        if wlan.isconnected():
            ip = wlan.ifconfig()[0]
            log_info("WIFI", "Connected to '{}' attempt={} IP={}".format(
                ssid, attempt + 1, ip))
            return ip

        s = wlan.status()
        log_warn("WIFI", "attempt {} on '{}' failed status={}".format(attempt + 1, ssid, s))
        time.sleep(3)

    log_error("WIFI", "All connection attempts failed!")
    return None


def broadcast_ip():
    """Broadcast address of the network we're on: ip | ~netmask."""
    if LAPTOP_BROADCAST_IP:
        return LAPTOP_BROADCAST_IP
    try:
        ip, mask = wlan.ifconfig()[0:2]
        ip_p = [int(x) for x in ip.split(".")]
        mask_p = [int(x) for x in mask.split(".")]
        return ".".join(str(a | (255 - m)) for a, m in zip(ip_p, mask_p))
    except Exception as e:
        log_warn("WIFI", "broadcast calc failed ({}), using 255.255.255.255".format(e))
        return "255.255.255.255"


def check_wifi():
    if wlan.isconnected():
        return True
    s = wlan.status()
    try:
        ifc = wlan.ifconfig()
    except Exception:
        ifc = ("?", "?", "?", "?")
    log_warn("WIFI", "lost connection! status={} ifconfig={}".format(s, ifc))
    stop_motors()
    board.led_on()
    ip = connect_wifi()
    board.led_off()
    if ip:
        log_info("WIFI", "reconnected at " + ip)
        return "reconnected"
    log_error("WIFI", "reconnect FAILED")
    return False


def force_recover():
    log_warn("WIFI", "force_recover triggered")
    stop_motors()
    board.led_on()
    hard_wifi_reset()
    ip = connect_wifi()
    board.led_off()
    if ip:
        log_info("WIFI", "force_recover succeeded at " + ip)
        return True
    log_error("WIFI", "force_recover FAILED")
    return False


# ============================================================
# Sockets
# ============================================================
def create_sockets():
    sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock_send.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    except Exception as e:
        log_warn("SOCK", "SO_BROADCAST failed: " + str(e))
    sock_recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_recv.bind(("0.0.0.0", GAMEPAD_PORT))
    sock_recv.setblocking(False)
    return sock_send, sock_recv


# ============================================================
# Boot
# ============================================================
_log_init()
log_info("BOOT", "starting xrp_raw_sensors.py (MAX_RPM={}, ramp={}/tick)".format(
    MAX_DRIVE_RPM, THROTTLE_RAMP_PER_TICK))

log_info("IMU", "calibrating (keep XRP still)")
try:
    imu.calibrate(3, 0)
    log_info("IMU", "calibration done")
except Exception as e:
    log_error("IMU", "calibration FAILED: " + str(e))

motor_three.reset_encoder_position()
motor_four.reset_encoder_position()
log_info("MOTOR", "encoders reset")


# ============================================================
# Main loop
# ============================================================
def main():
    global joy_x, joy_y, gamepad_active, gamepad_last_recv, gamepad_ever_received

    # The actually-applied throttle, ramped toward joy_y rather than
    # jumping to it. Avoids inrush current spikes from instant
    # 0 -> full transitions.
    applied_throttle = 0.0

    ip = connect_wifi()
    if ip is None:
        log_error("MAIN", "no WiFi, aborting")
        return

    for i in range(3):
        board.led_on(); time.sleep(0.15)
        board.led_off(); time.sleep(0.15)

    sock_send, sock_recv = create_sockets()
    dest = (broadcast_ip(), SENSOR_PORT)

    send_interval_ms = 1000 // SEND_RATE_HZ
    last_send = time.ticks_ms()
    last_wifi_check = time.ticks_ms()
    last_heartbeat = time.ticks_ms()

    send_count = 0
    error_count = 0
    consecutive_send_errors = 0

    log_info("LOOP", "streaming at {} Hz to {}".format(SEND_RATE_HZ, dest))

    while True:
        now = time.ticks_ms()

        # ---- Periodic heartbeat ----
        if time.ticks_diff(now, last_heartbeat) > HEARTBEAT_EVERY_MS:
            last_heartbeat = now
            try:
                s = wlan.status()
                connected = wlan.isconnected()
                gp_age_ms = time.ticks_diff(now, gamepad_last_recv) if gamepad_ever_received else -1
                log_info("HEART",
                    "alive sent={} err={} cse={} wifi_status={} connected={} gp_age={}ms gp_active={} thr={:.2f}".format(
                        send_count, error_count, consecutive_send_errors,
                        s, connected, gp_age_ms, gamepad_active, applied_throttle))
            except Exception as e:
                log_warn("HEART", "status read failed: " + str(e))

        # ---- WiFi check every 2s ----
        if time.ticks_diff(now, last_wifi_check) > 2000:
            last_wifi_check = now

            status = check_wifi()
            if not status:
                stop_motors()
                applied_throttle = 0.0
                time.sleep(2)
                continue
            if status == "reconnected":
                try: sock_send.close()
                except: pass
                try: sock_recv.close()
                except: pass
                sock_send, sock_recv = create_sockets()
                dest = (broadcast_ip(), SENSOR_PORT)
                log_info("LOOP", "sending to {}".format(dest))
                consecutive_send_errors = 0
                continue

            # Silent failure detection — requires both gamepad-quiet
            # AND active send errors to avoid false alarms when the
            # user just lets go of the controller.
            if gamepad_ever_received and consecutive_send_errors >= DEAF_SEND_ERRORS_MIN:
                gp_age = time.ticks_diff(now, gamepad_last_recv)
                if gp_age > DEAF_THRESHOLD_MS:
                    log_error("WIFI",
                        "SILENT FAILURE: gp_age={}ms send_errs={} wlan.isconnected={} status={}".format(
                            gp_age, consecutive_send_errors, wlan.isconnected(), wlan.status()))
                    if force_recover():
                        try: sock_send.close()
                        except: pass
                        try: sock_recv.close()
                        except: pass
                        sock_send, sock_recv = create_sockets()
                        dest = (broadcast_ip(), SENSOR_PORT)
                        consecutive_send_errors = 0
                        gamepad_last_recv = time.ticks_ms()
                        gamepad_active = False
                        applied_throttle = 0.0
                    else:
                        stop_motors()
                        applied_throttle = 0.0
                        time.sleep(2)
                    continue

            # Pure send-error escalation
            if consecutive_send_errors >= 10:
                log_error("WIFI",
                    "many send errors ({}) while wlan.isconnected={}; force recover".format(
                        consecutive_send_errors, wlan.isconnected()))
                if force_recover():
                    try: sock_send.close()
                    except: pass
                    try: sock_recv.close()
                    except: pass
                    sock_send, sock_recv = create_sockets()
                    dest = (broadcast_ip(), SENSOR_PORT)
                    consecutive_send_errors = 0
                continue

        # ---- Receive gamepad ----
        try:
            while True:
                data, addr = sock_recv.recvfrom(64)
                parse_gamepad_bytes(data)
        except OSError:
            pass

        # ---- Gamepad input timeout ----
        if gamepad_active and time.ticks_diff(now, gamepad_last_recv) > GAMEPAD_TIMEOUT_MS:
            gamepad_active = False
            joy_x = 0.0
            joy_y = 0.0
            stop_motors()
            applied_throttle = 0.0
            log_warn("GAMEPAD", "input timeout, motors stopped")

        # ---- Apply commands with throttle ramping ----
        if gamepad_active:
            target = max(-1.0, min(1.0, joy_y))
            # Ramp applied_throttle toward target without exceeding
            # THROTTLE_RAMP_PER_TICK per loop iteration.
            delta = target - applied_throttle
            if delta > THROTTLE_RAMP_PER_TICK:
                applied_throttle += THROTTLE_RAMP_PER_TICK
            elif delta < -THROTTLE_RAMP_PER_TICK:
                applied_throttle -= THROTTLE_RAMP_PER_TICK
            else:
                applied_throttle = target

            if abs(applied_throttle) < JOYSTICK_DEADBAND:
                motor_three.set_speed(0.0)
                motor_four.set_speed(0.0)
            else:
                target_rpm = applied_throttle * MAX_DRIVE_RPM
                motor_three.set_speed(target_rpm)
                motor_four.set_speed(target_rpm)

            steer_norm = max(-1.0, min(1.0, joy_x))
            steer_angle = STEER_CENTER_DEG + steer_norm * STEER_RANGE_DEG
            steer_angle = max(STEER_CENTER_DEG - STEER_RANGE_DEG,
                              min(STEER_CENTER_DEG + STEER_RANGE_DEG, steer_angle))
            servo_three.set_angle(steer_angle)
        else:
            # No gamepad → also ramp toward 0 instead of jumping
            if applied_throttle > THROTTLE_RAMP_PER_TICK:
                applied_throttle -= THROTTLE_RAMP_PER_TICK
                motor_three.set_speed(applied_throttle * MAX_DRIVE_RPM)
                motor_four.set_speed(applied_throttle * MAX_DRIVE_RPM)
            elif applied_throttle < -THROTTLE_RAMP_PER_TICK:
                applied_throttle += THROTTLE_RAMP_PER_TICK
                motor_three.set_speed(applied_throttle * MAX_DRIVE_RPM)
                motor_four.set_speed(applied_throttle * MAX_DRIVE_RPM)
            else:
                applied_throttle = 0.0

        # ---- Send sensor packet ----
        if time.ticks_diff(now, last_send) >= send_interval_ms:
            last_send = now
            try:
                enc_l = motor_three.get_position()
                enc_r = motor_four.get_position()
                heading = imu_heading_rad()
                msg = "RAW,{:.6f},{:.6f},{:.6f}".format(enc_l, enc_r, heading)
                sock_send.sendto(msg.encode(), dest)
                send_count += 1
                consecutive_send_errors = 0
            except OSError as e:
                error_count += 1
                consecutive_send_errors += 1
                if consecutive_send_errors <= 3:
                    log_error("SEND", "OSError #{}: {} (errno={}) wlan.isconnected={}".format(
                        error_count, e, getattr(e, 'errno', '?'), wlan.isconnected()))
                elif consecutive_send_errors == 4:
                    log_error("SEND", "(suppressing further errors)")
            except Exception as e:
                error_count += 1
                log_error("SEND", "{}: {}".format(type(e).__name__, e))

        time.sleep(0.005)


# ============================================================
# Top-level guard
# ============================================================
try:
    main()
except KeyboardInterrupt:
    log_info("EXIT", "KeyboardInterrupt")
    stop_motors()
except Exception as e:
    import sys
    log_error("CRASH", "Uncaught: {}: {}".format(type(e).__name__, e))
    try:
        import io
        s = io.StringIO()
        sys.print_exception(e, s)
        log_error("CRASH", "tb:\n" + s.getvalue())
    except Exception:
        pass
    stop_motors()
finally:
    try:
        if _log_file is not None:
            _log_file.write("=== shutdown ===\n")
            _log_file.flush()
            _log_file.close()
    except Exception:
        pass
