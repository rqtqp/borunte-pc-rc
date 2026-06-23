"""
accel_control.py — Hold the BTT LIS2DW12 in your hand. The robot arm mirrors your tilt.

Mapping:
  Roll  (tilt left / right)    -> J1 base rotation    (-90 to +90 deg)
  Pitch (tilt forward / back)  -> J3 elbow            (-20 to +135 deg)
  J2, J4, J5, J6 hold fixed at straight-up position

Usage:
  python accel_control.py

Press Ctrl+C to stop — arm returns to straight-up.
"""

import json
import math
import serial
import signal
import socket
import sys
import time

ROBOT_IP   = "10.0.0.49"
ROBOT_PORT = 9760
SERIAL_PORT = "COM8"     # BTT LIS2DW12 USB serial port
BAUD       = 115200

SEND_HZ    = 20          # how often to send AddRCC (Hz)
ALPHA      = 0.15        # low-pass smoothing (lower = smoother, more lag)

# Fixed joint values for joints not controlled by accel
FIXED = {
    "J2": -20.0,
    "J4":   0.0,
    "J5":  57.0,
    "J6":   0.0,
}

# Soft limits for controlled joints
J1_MIN, J1_MAX = -90.0,  90.0
J3_MIN, J3_MAX = -20.0, 135.0

_stop = False


def _sigint(sig, frame):
    global _stop
    print("\nCtrl+C — returning to straight up...")
    _stop = True


def send_move(sock, j1, j2, j3, j4, j5, j6, pack_id):
    payload = {
        "dsID": "HCRemoteCommand",
        "reqType": "AddRCC",
        "emptyList": "1",
        "packID": pack_id,
        "instructions": [{
            "oneshot": "1",
            "action": "4",
            "m0": f"{j1:.2f}", "m1": f"{j2:.2f}", "m2": f"{j3:.2f}",
            "m3": f"{j4:.2f}", "m4": f"{j5:.2f}", "m5": f"{j6:.2f}",
            "m6": "0.0", "m7": "0.0",
            "ckStatus": "0x3F",
            "speed": "30.0",
            "delay": "0.0", "tool": "0", "coord": "0", "smooth": "0",
        }],
    }
    enc = json.dumps(payload, separators=(",", ":")).encode("ascii")
    sock.sendall(enc)
    sock.recv(4096)   # drain reply, don't block on it


def angles_from_accel(x, y, z):
    """Compute roll and pitch (degrees) from gravity vector."""
    norm = math.sqrt(x*x + y*y + z*z)
    if norm < 0.1:
        return 0.0, 0.0
    roll  = math.degrees(math.atan2(y, z))
    pitch = math.degrees(math.atan2(-x, math.sqrt(y*y + z*z)))
    return roll, pitch


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


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
        sock.settimeout(0.5)
    except OSError as e:
        print(f"Robot connection failed: {e}")
        sys.exit(1)

    print("Connected. Hold sensor flat to calibrate gravity direction.")
    print("Tilt left/right  -> J1 (base)")
    print("Tilt fwd/back    -> J3 (elbow)")
    print("Ctrl+C to stop\n")

    # Warm up: read a few samples to seed the filter
    smooth_roll, smooth_pitch = 0.0, 0.0
    for _ in range(10):
        line = ser.readline().decode("ascii", errors="ignore").strip()
        try:
            x, y, z = map(float, line.split(","))
            r, p = angles_from_accel(x, y, z)
            smooth_roll, smooth_pitch = r, p
        except Exception:
            pass

    interval = 1.0 / SEND_HZ
    move_n   = 0
    last_send = time.monotonic()

    while not _stop:
        # Drain serial — keep only the latest reading
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
            # Map angles to joint targets
            j1 = clamp(smooth_roll,  J1_MIN, J1_MAX)
            j3_center = (J3_MIN + J3_MAX) / 2          # 57.5
            j3_scale  = (J3_MAX - J3_MIN) / 180.0      # degrees of J3 per degree of pitch
            j3 = clamp(j3_center + smooth_pitch * j3_scale, J3_MIN, J3_MAX)

            move_n += 1
            try:
                send_move(sock,
                    j1=j1, j2=FIXED["J2"], j3=j3,
                    j4=FIXED["J4"], j5=FIXED["J5"], j6=FIXED["J6"],
                    pack_id=f"accel-{move_n:05d}",
                )
                print(f"[{move_n:05d}]  roll:{smooth_roll:+6.1f}  pitch:{smooth_pitch:+6.1f}"
                      f"  ->  J1:{j1:+6.1f}  J3:{j3:+6.1f}", end="\r")
            except OSError:
                print("\nRobot connection lost.")
                break

            last_send = now

    # Return to straight up
    print("\nSending to straight up...")
    try:
        send_move(sock, 0.0, -20.0, 125.0, 0.0, 57.0, 0.0, pack_id="stop")
    except OSError:
        pass

    sock.close()
    ser.close()
    print("Done.")


if __name__ == "__main__":
    main()
