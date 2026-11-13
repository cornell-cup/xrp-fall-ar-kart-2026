"""
wheel_bridge.py

Lets the game use a Thrustmaster TMX wheel on macOS, which has no driver for it.

The TMX speaks Xbox One's GIP protocol over USB instead of standard HID, so
macOS (and Godot) can't see it as a controller. Nothing else claims it either,
so this script talks to it directly with libusb:
  1. sends the GIP "power on" message, which starts the wheel's input reports
  2. reads steering, pedals and buttons
  3. sends them to the game on UDP 127.0.0.1:6004 as
         WHEEL,<steer>,<gas>,<brake>,<buttons>
     steer -1 (full left) .. 1 (full right), gas/brake 0 (up) .. 1 (floored),
     buttons = GIP button bits (see BUTTON_* below)

Packets are sent on every change and at least every HEARTBEAT_S, so the game
knows the wheel is still there when nobody is touching it. Unplugging and
replugging the wheel is handled: it keeps looking for it.

Started by NetworkReceiver.gd with the same Python as fusion (needs
`pip install pyusb libusb-package`). To test on its own:
    python Scripts/wheel_bridge.py
"""

import socket
import sys
import time

import usb.core
import usb.util
import libusb_package

# USB IDs of the TMX (Thrustmaster vendor 0x044f)
WHEELS = [(0x044F, 0xB67E)]

GAME_ADDR = ("127.0.0.1", 6004)
HEARTBEAT_S = 0.05
RETRY_S = 1.0

EP_OUT = 0x01
EP_IN = 0x81

GIP_INPUT = 0x20
GIP_POWER_ON = bytes([0x05, 0x20, 0x00, 0x01, 0x00])

# Byte layout of the TMX's input report (measured from this wheel):
#   [4]    buttons: A 0x10, B 0x20, X 0x40, Y 0x80, menu 0x04, view 0x08
#   [5]    buttons: paddles 0x10 / 0x20, d-pad 0x01-0x08
#   [6:8]  steering, 0 (full left) .. 65535 (full right), little-endian
#   [8:10] gas pedal, 0 .. 1023
#   [10:12] brake pedal, 0 .. 1023
PEDAL_MAX = 1023.0


def log(msg):
    print("[wheel_bridge] " + msg, flush=True)


def find_wheel(backend):
    for vid, pid in WHEELS:
        dev = usb.core.find(idVendor=vid, idProduct=pid, backend=backend)
        if dev is not None:
            return dev
    return None


def open_wheel(dev):
    try:
        dev.set_configuration(1)
    except usb.core.USBError:
        pass   # already configured
    usb.util.claim_interface(dev, 0)
    dev.write(EP_OUT, GIP_POWER_ON, timeout=500)


def parse_input(data):
    """Returns (steer, gas, brake, buttons) from a GIP input report, or None."""
    if len(data) < 12 or data[0] != GIP_INPUT:
        return None
    raw_steer = data[6] | (data[7] << 8)
    steer = max(-1.0, min(1.0, (raw_steer - 32768) / 32767.0))
    gas = (data[8] | (data[9] << 8)) / PEDAL_MAX
    brake = (data[10] | (data[11] << 8)) / PEDAL_MAX
    buttons = data[4] | (data[5] << 8)
    return steer, min(gas, 1.0), min(brake, 1.0), buttons


def run(backend, sock):
    dev = find_wheel(backend)
    if dev is None:
        return False
    open_wheel(dev)
    log("wheel connected")
    # An already-started wheel only reports when something changes, so start
    # from centred with the pedals up; the first touch sends the real values
    # (every report carries all of them)
    state = (0.0, 0.0, 0.0, 0)
    last_sent = 0.0
    try:
        while True:
            changed = False
            try:
                data = bytes(dev.read(EP_IN, 64, timeout=int(HEARTBEAT_S * 1000)))
                parsed = parse_input(data)
                if parsed is not None:
                    state = parsed
                    changed = True
            except usb.core.USBTimeoutError:
                pass
            now = time.monotonic()
            if changed or now - last_sent >= HEARTBEAT_S:
                sock.sendto("WHEEL,{:.4f},{:.4f},{:.4f},{}".format(*state).encode(), GAME_ADDR)
                last_sent = now
    except usb.core.USBError as e:
        log("wheel lost ({}), waiting for it".format(e))
    finally:
        try:
            usb.util.release_interface(dev, 0)
            usb.util.dispose_resources(dev)
        except Exception:
            pass
    return True


def main():
    backend = libusb_package.get_libusb1_backend()
    if backend is None:
        log("libusb not found (pip install libusb-package)")
        sys.exit(1)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    log("looking for the wheel")
    while True:
        try:
            if not run(backend, sock):
                time.sleep(RETRY_S)
        except usb.core.USBError as e:
            log("could not open the wheel: {}".format(e))
            time.sleep(RETRY_S)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
