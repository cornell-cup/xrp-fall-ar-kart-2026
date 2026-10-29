"""
xrp_raw_sensors.py

Slim XRP-side script with persistent logging AND silent-failure
detection.

Gamepad protocol from laptop (trimmed):
    [0x55][0x04][1][throttle_byte][2][steer_byte]   = 6 bytes

After a crash/disconnect, plug XRP into USB, open Thonny, and run:
    print(open('/xrp_log.txt').read())
"""

import time, math, network, socket, os
import machine
from machine import Timer
from XRPLib.defaults import *

# ============================================================
# WiFi Config
# ============================================================
# Networks to join, in order of preference. The XRP joins whichever one
# it can see, so the same file works with the Pi hotspot OR CornellCup-Web.
# Open network -> password "" (giving a password to an open network makes
# the Pico W attempt WPA2 and fail).
WIFI_NETWORKS = [
    ("AR-Kart", "cornellcup"),   
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
SEND_RATE_HZ = 100
YAW_CORRECTION_PORT = 5007 # XRP listens for yaw correction from Laptop

# ============================================================
# Robot config
# ============================================================
JOYSTICK_DEADBAND = 0.05

MAX_DRIVE_RPM = 7

# Throttle ramping (brownout protection) — limits how fast the motor command
# can change. 4.0 per second means 0->full takes 250 ms, which avoids the
# inrush-current spikes that browned out the power rail. Measured in time, not
# loop passes, so it doesn't change with LOOP_SLEEP_MS.
# Larger value = snappier response but higher current spikes.
THROTTLE_RAMP_PER_SEC = 3.0

# Main loop pause between passes. Gamepad packets are read once per pass,
# so this is the most a new command waits on the XRP before it's applied
LOOP_SLEEP_MS = 1

ENCODER_PERCENT_OFFSET = 1/0.95 # based on empirical data gathered (the encoders seem to report about 15% less distance than acutally traveled)

# From xrpgodot.py (SERVO_CENTER / SERVO_RANGE), the values the kart
# currently drives straight with.
STEER_CENTER_DEG = 90.0
STEER_RANGE_DEG = 45.0

GAMEPAD_HEADER = 0x55
# The first laptop to send gamepad packets gets control. Packets from any other
# laptop are ignored until the controlling one has been quiet this long, then
# whichever laptop sends next takes over. (Motors still stop after
# GAMEPAD_TIMEOUT_MS; this only decides who may drive next.)
CONTROL_LEASE_MS = 1000
# No gamepad packet for this long = connection lost, motors stop. The game sends
# at least every ~20-33 ms, so this is roughly 10 missed packets in a row
GAMEPAD_TIMEOUT_MS = 300
# How often the failsafe timer checks for a lost connection
FAILSAFE_CHECK_MS = 50

# ============================================================
# Hardware watchdog
# ============================================================
# Reboots the board (which stops the motors) if the script stops checking in,
# e.g. if the WiFi driver hangs. Once started it can't be stopped, so after
# stopping the script from XRPCode/Thonny, run it again or the board resets.
# Set False while uploading or debugging.
WATCHDOG_ENABLED = True
WATCHDOG_TIMEOUT_MS = 8000   # the RP2040 maximum is ~8.3 s

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


# Whether the last run wrote its shutdown marker. False means it ended by power
# loss, brownout, watchdog reset or a hard crash. None = no log yet
_previous_clean = None

def _check_previous_shutdown(size):
    global _previous_clean
    try:
        with open(LOG_PATH, "r") as f:
            f.seek(max(0, size - 4096))
            tail = f.read()
        last_boot = tail.rfind("=== boot")
        if last_boot >= 0:
            _previous_clean = "=== shutdown ===" in tail[last_boot:]
    except Exception:
        pass

def _log_init():
    global _log_file
    if not LOG_TO_FILE:
        return
    try:
        try:
            size = os.stat(LOG_PATH)[6]
            _check_previous_shutdown(size)
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
# Watchdog, reset cause and battery helpers
# ============================================================
_wdt = None

def start_watchdog():
    global _wdt
    if not WATCHDOG_ENABLED:
        log_info("WDT", "watchdog disabled")
        return
    try:
        _wdt = machine.WDT(timeout=WATCHDOG_TIMEOUT_MS)
        log_info("WDT", "watchdog on ({} ms)".format(WATCHDOG_TIMEOUT_MS))
    except Exception as e:
        log_warn("WDT", "could not start watchdog: " + str(e))

def feed_watchdog():
    if _wdt is not None:
        _wdt.feed()

def sleep_fed(seconds):
    """time.sleep that keeps feeding the watchdog, for the long waits."""
    end = time.ticks_add(time.ticks_ms(), int(seconds * 1000))
    while time.ticks_diff(end, time.ticks_ms()) > 0:
        feed_watchdog()
        time.sleep_ms(min(200, max(1, time.ticks_diff(end, time.ticks_ms()))))
    feed_watchdog()

def reset_cause_name():
    try:
        cause = machine.reset_cause()
    except Exception:
        return "unknown"
    names = {}
    for name in ("PWRON_RESET", "WDT_RESET", "HARD_RESET", "SOFT_RESET", "DEEPSLEEP_RESET"):
        if hasattr(machine, name):
            names[getattr(machine, name)] = name
    # A brownout on the RP2040 shows up as a power-on reset
    return names.get(cause, str(cause))

def battery_voltage():
    """Battery voltage in volts, or None if this XRPLib can't measure it."""
    try:
        return board.get_battery_voltage()
    except Exception:
        return None

def battery_str():
    v = battery_voltage()
    return "{:.2f}V".format(v) if v is not None else "n/a"

# ============================================================
# Heading helpers
# ============================================================
HEADING_SIGN = 1.0
HEADING_OFFSET = math.pi/2 # 90 degrees

def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi

def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v

def imu_heading_rad():
    global HEADING_OFFSET
    roll_deg = imu.get_roll()
    return wrap_pi( HEADING_OFFSET - (HEADING_SIGN * math.radians(roll_deg)))


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

# 0xa11140 is the Pico W's "no power saving" setting (PM_NONE in newer firmware)
WIFI_PM_NONE = getattr(network.WLAN, "PM_NONE", 0xa11140)

def disable_wifi_power_save():
    """Keep the radio awake. With power saving on it naps between access
    point beacons, which can delay incoming gamepad packets by tens of ms."""
    try:
        wlan.config(pm=WIFI_PM_NONE)
        log_info("WIFI", "power saving off")
    except Exception as e:
        log_warn("WIFI", "could not turn off power saving: " + str(e))

def stop_motors():
    # set_speed(0) cancels XRPLib's speed controller. set_effort(0) alone does
    # NOT: the controller's background timer keeps driving at the last target
    # speed, which is why the kart used to keep going after losing connection.
    motor_three.set_speed(0)
    motor_four.set_speed(0)
    servo_three.set_angle(STEER_CENTER_DEG)


# ============================================================
# Connection-loss failsafe
# ============================================================
# Runs on its own timer, so it stops the kart even while the main loop is
# stuck (e.g. blocked for seconds reconnecting to WiFi).
failsafe_tripped = False
# The laptop currently in control, set by main()
controller_ip = None

def _failsafe_check(_timer):
    global gamepad_active, joy_x, joy_y, failsafe_tripped
    if gamepad_active and time.ticks_diff(time.ticks_ms(), gamepad_last_recv) > GAMEPAD_TIMEOUT_MS:
        gamepad_active = False
        joy_x = 0.0
        joy_y = 0.0
        stop_motors()
        failsafe_tripped = True   # logged by the main loop, not here in the timer callback


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
    sleep_fed(1)
    try:
        wlan.active(True)
    except Exception as e:
        log_error("WIFI", "active(True) failed: " + str(e))
    sleep_fed(1)


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
        disable_wifi_power_save()
        return ip

    sleep_fed(2)

    for attempt in range(20):
        try:
            wlan.disconnect()
        except Exception as e:
            log_warn("WIFI", "disconnect raised: " + str(e))
        sleep_fed(1)

        ssid, pw = pick_network(attempt)
        log_info("WIFI", "trying '" + ssid + "'")
        try:
            if pw:
                wlan.connect(ssid, pw)
            else:
                wlan.connect(ssid)   # open network: no key at all
        except Exception as e:
            log_error("WIFI", "connect raised: " + str(e))
            sleep_fed(2)
            continue

        start = time.time()
        while not wlan.isconnected():
            if time.time() - start > 8:
                break
            sleep_fed(0.5)

        if wlan.isconnected():
            ip = wlan.ifconfig()[0]
            log_info("WIFI", "Connected to '{}' attempt={} IP={}".format(
                ssid, attempt + 1, ip))
            disable_wifi_power_save()
            return ip

        s = wlan.status()
        log_warn("WIFI", "attempt {} on '{}' failed status={}".format(attempt + 1, ssid, s))
        sleep_fed(3)

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
log_info("BOOT", "starting xrp_raw_sensors.py (MAX_RPM={}, ramp={}/s, sensors {} Hz)".format(
    MAX_DRIVE_RPM, THROTTLE_RAMP_PER_SEC, SEND_RATE_HZ))
log_info("BOOT", "reset cause={} battery={} previous run {}".format(
    reset_cause_name(), battery_str(),
    "unknown (no log)" if _previous_clean is None else
    "shut down cleanly" if _previous_clean else
    "did NOT shut down cleanly (power loss/brownout, watchdog or crash)"))

log_info("IMU", "calibrating (keep XRP still)")
try:
    imu.calibrate(3, 0)
    log_info("IMU", "calibration done")
except Exception as e:
    log_error("IMU", "calibration FAILED: " + str(e))

motor_three.reset_encoder_position()
motor_four.reset_encoder_position()
log_info("MOTOR", "encoders reset")

failsafe_timer = Timer(-1)
failsafe_timer.init(period=FAILSAFE_CHECK_MS, mode=Timer.PERIODIC, callback=_failsafe_check)
log_info("MOTOR", "connection-loss failsafe on ({} ms timeout)".format(GAMEPAD_TIMEOUT_MS))

# Started after the 3 s IMU calibration; fed in the main loop and in every long wait
start_watchdog()

def create_yaw_socket():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", YAW_CORRECTION_PORT))
    sock.settimeout(0.0)
    return sock

def process_absolute_yaw(sock):
    global HEADING_OFFSET

    try:
        raw, addr = sock.recvfrom(128)
    except OSError:
        return

    try:
        parts = raw.decode("utf-8").strip().split(",")

        if len(parts) != 2 or parts[0] != "TAG_YAW":
            return

        tag_yaw = float(parts[1])

        difference_in_heading = tag_yaw - imu_heading_rad()

        HEADING_OFFSET = HEADING_OFFSET + difference_in_heading

    except (ValueError, UnicodeDecodeError):
        pass

# ============================================================
# Main loop
# ============================================================
def main():
    global joy_x, joy_y, gamepad_active, gamepad_last_recv, gamepad_ever_received, failsafe_tripped
    global controller_ip

    # The throttle currently applied to the motors, ramped toward joy_y
    # rather than jumping to it (brownout protection)
    applied_throttle = 0.0
    last_loop = time.ticks_ms()

    # The laptop currently in control (first come, first served, see
    # CONTROL_LEASE_MS). While it's driving, sensor packets go straight to it
    # instead of being broadcast
    controller_ip = None
    controller_last = time.ticks_ms()
    last_ignored_log = None

    ip = connect_wifi()
    if ip is None:
        log_error("MAIN", "no WiFi, aborting")
        return

    for i in range(3):
        board.led_on(); sleep_fed(0.15)
        board.led_off(); sleep_fed(0.15)

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

    yaw_sock = create_yaw_socket()

    while True:
        now = time.ticks_ms()
        loop_dt_s = time.ticks_diff(now, last_loop) / 1000.0
        last_loop = now

        # ---- Periodic heartbeat ----
        if time.ticks_diff(now, last_heartbeat) > HEARTBEAT_EVERY_MS:
            last_heartbeat = now
            try:
                s = wlan.status()
                connected = wlan.isconnected()
                gp_age_ms = time.ticks_diff(now, gamepad_last_recv) if gamepad_ever_received else -1
                log_info("HEART",
                    "alive sent={} err={} cse={} wifi_status={} connected={} gp_age={}ms gp_active={} thr={:.2f} ctrl={} batt={}".format(
                        send_count, error_count, consecutive_send_errors,
                        s, connected, gp_age_ms, gamepad_active, applied_throttle,
                        controller_ip, battery_str()))
            except Exception as e:
                log_warn("HEART", "status read failed: " + str(e))

        # ---- WiFi check every 2s ----
        if time.ticks_diff(now, last_wifi_check) > 2000:
            last_wifi_check = now

            status = check_wifi()
            if not status:
                stop_motors()
                applied_throttle = 0.0
                sleep_fed(2)
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
                        sleep_fed(2)
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
                process_absolute_yaw(yaw_sock)


                data, addr = sock_recv.recvfrom(64)
                sender = addr[0]
                t_rx = time.ticks_ms()
                lease_held = (controller_ip is not None
                              and time.ticks_diff(t_rx, controller_last) < CONTROL_LEASE_MS)
                if lease_held and sender != controller_ip:
                    # another laptop is already driving
                    if last_ignored_log is None or time.ticks_diff(t_rx, last_ignored_log) > 5000:
                        last_ignored_log = t_rx
                        log_warn("CTRL", "ignoring {} ({} is in control)".format(sender, controller_ip))
                    continue
                if parse_gamepad_bytes(data):
                    if sender != controller_ip:
                        log_info("CTRL", "control taken by " + sender)
                        controller_ip = sender
                    controller_last = t_rx
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
        if failsafe_tripped:
            failsafe_tripped = False
            applied_throttle = 0.0
            log_warn("GAMEPAD", "connection lost, failsafe stopped motors")

        # ---- Apply commands, with throttle ramping ----
        # (only while connected: on connection loss the motors stop at once)
        if gamepad_active:
            target = max(-1.0, min(1.0, joy_y))
            max_step = THROTTLE_RAMP_PER_SEC * loop_dt_s
            applied_throttle += max(-max_step, min(max_step, target - applied_throttle))

            if abs(applied_throttle) < JOYSTICK_DEADBAND:
                motor_three.set_speed(0.0)
                motor_four.set_speed(0.0)
            else:
                target_rpm = clamp(applied_throttle, -1.0, 1.0) * MAX_DRIVE_RPM
                motor_three.set_speed(target_rpm)
                motor_four.set_speed(target_rpm)

            steer_norm = max(-1.0, min(1.0, joy_x))
            steer_angle = STEER_CENTER_DEG + steer_norm * STEER_RANGE_DEG
            steer_angle = max(STEER_CENTER_DEG - STEER_RANGE_DEG,
                              min(STEER_CENTER_DEG + STEER_RANGE_DEG, steer_angle))
            servo_three.set_angle(steer_angle)

        # ---- Send sensor packet ----
        if time.ticks_diff(now, last_send) >= send_interval_ms:
            last_send = now
            try:
                enc_l = motor_three.get_position()*ENCODER_PERCENT_OFFSET
                enc_r = motor_four.get_position()*ENCODER_PERCENT_OFFSET
                motor_three.reset_encoder_position()
                motor_four.reset_encoder_position()
                heading = imu_heading_rad()
                msg = "RAW,{:.6f},{:.6f},{:.6f}".format(enc_l, enc_r, heading)
                # Straight to the laptop while it's driving us, otherwise broadcast
                target = (controller_ip, SENSOR_PORT) if gamepad_active and controller_ip else dest
                sock_send.sendto(msg.encode(), target)
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

        feed_watchdog()
        time.sleep_ms(LOOP_SLEEP_MS)


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


