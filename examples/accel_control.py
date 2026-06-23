"""
accel_control.py — Hold the BTT LIS2DW12. Robot TCP mirrors your tilt in world coordinates.

Mapping (velocity mode):
  Pitch (tilt forward / back)  -> TCP moves in world X (forward / backward)
  Roll  (tilt left  / right)   -> TCP moves in world Y (right / left)
  Z and orientation stay fixed.

Usage:
  python accel_control.py

Press Ctrl+C to stop.
"""

import json
import math
import serial
import signal
import socket
import sys
import time

ROBOT_IP    = "10.0.0.49"
ROBOT_PORT  = 9760
SERIAL_PORT = "COM5"     # BTT LIS2DW12 USB serial port (CircuitPython)
BAUD        = 115200

SEND_HZ      = 10           # motion command rate (Hz)
ALPHA        = 0.2          # low-pass filter on tilt (higher = more responsive)

CART_VEL_MAX = 6.0          # mm per tick at ±45° of tilt
DEAD_ZONE    = 3.0          # degrees of tilt that maps to zero movement

# Workspace limits in world-frame mm
X_MIN, X_MAX = -400.0,  400.0
Y_MIN, Y_MAX = -400.0,  400.0

_stop = False


def _sigint(sig, frame):
    global _stop
    print("\nCtrl+C pressed...")
    _stop = True


def query_world_pos(sock):
    payload = {
        "dsID": "www.hc-system.com.RemoteMonitor",
        "packID": "wpos",
        "reqType": "query",
        "queryAddr": ["world-0","world-1","world-2","world-3","world-4","world-5"],
    }
    enc = json.dumps(payload, separators=(",",":")).encode("ascii")
    sock.sendall(enc)
    r = json.loads(sock.recv(65536))
    return [float(v) for v in r.get("queryData", [])]  # [x, y, z, rx, ry, rz]


def send_cartesian_move(sock, x, y, z, rx, ry, rz, pack_id):
    payload = {
        "dsID": "HCRemoteCommand",
        "reqType": "AddRCC",
        "emptyList": "1",
        "packID": pack_id,
        "instructions": [{
            "oneshot": "1",
            "action": "10",
            "m0": f"{x:.3f}", "m1": f"{y:.3f}", "m2": f"{z:.3f}",
            "m3": f"{rx:.3f}", "m4": f"{ry:.3f}", "m5": f"{rz:.3f}",
            "m6": "0.0", "m7": "0.0",
            "ckStatus": "0x3F",
            "speed": "30.0",
            "delay": "0.0", "tool": "0", "coord": "0", "smooth": "0",
        }],
    }
    enc = json.dumps(payload, separators=(",", ":")).encode("ascii")
    sock.sendall(enc)
    sock.recv(4096)


def angles_from_accel(x, y, z):
    norm = math.sqrt(x*x + y*y + z*z)
    if norm < 0.1:
        return 0.0, 0.0
    roll  = math.degrees(math.atan2(y, z))
    pitch = math.degrees(math.atan2(-x, math.sqrt(y*y + z*z)))
    return roll, pitch


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def deadzone(v, dz):
    if abs(v) < dz:
        return 0.0
    sign = 1.0 if v > 0 else -1.0
    return sign * (abs(v) - dz)


def main():
    global _stop
    signal.signal(signal.SIGINT, _sigint)

    print(f"Opening accelerometer on {SERIAL_PORT}...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD, timeout=1.0)
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        sys.exit(1)
    time.sleep(0.5)
    ser.reset_input_buffer()

    print(f"Connecting to robot at {ROBOT_IP}:{ROBOT_PORT}...")
    try:
        sock = socket.create_connection((ROBOT_IP, ROBOT_PORT), timeout=5.0)
        sock.settimeout(2.0)
    except OSError as e:
        print(f"Robot connection failed: {e}")
        sys.exit(1)

    print("Querying current TCP position...")
    try:
        world = query_world_pos(sock)
    except Exception as e:
        print(f"Failed to read world position: {e}")
        sys.exit(1)

    tcp_x,  tcp_y,  tcp_z  = world[0], world[1], world[2]
    tcp_rx, tcp_ry, tcp_rz = world[3], world[4], world[5]
    print(f"  X={tcp_x:.1f}  Y={tcp_y:.1f}  Z={tcp_z:.1f}"
          f"  RX={tcp_rx:.1f}  RY={tcp_ry:.1f}  RZ={tcp_rz:.1f}")
    print()
    print("Hold sensor flat. Tilt to move:")
    print("  Forward / back  ->  World X")
    print("  Left   / right  ->  World Y")
    print("Ctrl+C to stop\n")

    smooth_roll, smooth_pitch = 0.0, 0.0
    for _ in range(10):
        line = ser.readline().decode("ascii", errors="ignore").strip()
        try:
            x, y, z = map(float, line.split(","))
            r, p = angles_from_accel(x, y, z)
            smooth_roll, smooth_pitch = r, p
        except Exception:
            pass

    interval  = 1.0 / SEND_HZ
    move_n    = 0
    last_send = time.monotonic()

    while not _stop:
        line = None
        while ser.in_waiting:
            line = ser.readline().decode("ascii", errors="ignore").strip()

        if line:
            try:
                x, y, z = map(float, line.split(","))
                roll, pitch = angles_from_accel(x, y, z)
                smooth_roll  = ALPHA * roll  + (1 - ALPHA) * smooth_roll
                smooth_pitch = ALPHA * pitch + (1 - ALPHA) * smooth_pitch
            except Exception:
                pass

        now = time.monotonic()
        if now - last_send >= interval:
            dvx = clamp(deadzone(smooth_pitch, DEAD_ZONE) / 45.0 * CART_VEL_MAX,
                        -CART_VEL_MAX, CART_VEL_MAX)
            dvy = clamp(deadzone(smooth_roll,  DEAD_ZONE) / 45.0 * CART_VEL_MAX,
                        -CART_VEL_MAX, CART_VEL_MAX)

            tcp_x = clamp(tcp_x + dvx, X_MIN, X_MAX)
            tcp_y = clamp(tcp_y + dvy, Y_MIN, Y_MAX)

            move_n += 1
            try:
                send_cartesian_move(sock, tcp_x, tcp_y, tcp_z,
                                    tcp_rx, tcp_ry, tcp_rz,
                                    pack_id=f"accel-{move_n:05d}")
                print(f"[{move_n:05d}]  pitch:{smooth_pitch:+5.1f}  roll:{smooth_roll:+5.1f}"
                      f"  ->  X:{tcp_x:+7.1f}  Y:{tcp_y:+7.1f}", end="\r")
            except OSError:
                print("\nRobot connection lost.")
                break

            last_send = now

    sock.close()
    ser.close()
    print("\nDone.")


if __name__ == "__main__":
    main()
