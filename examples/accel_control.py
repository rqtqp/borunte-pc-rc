"""
accel_control.py — Hold the BTT LIS2DW12. Robot TCP mirrors your tilt in world coordinates.

Axes (velocity mode):
  Tilt forward / back   -> World X   (gx component)
  Tilt left   / right   -> World Y   (gy component)
  Flip face-down        -> World Z   (gz deviation, activates only past ~66° from flat)

Controls:
  Enter    — toggle tracking ON / OFF
  Ctrl+C   — exit

Usage:
  python accel_control.py
"""

import json
import math
import msvcrt
import serial
import signal
import socket
import statistics
import sys
import threading
import time
from collections import deque

ROBOT_IP    = "10.0.0.49"
ROBOT_PORT  = 9760
SERIAL_PORT = "COM5"
BAUD        = 115200

SEND_HZ       = 10          # motion command rate (Hz)
MEDIAN_WINDOW = 9           # samples in median filter (~180ms at 50Hz); kills tap spikes
ALPHA         = 0.08        # EMA smoothing after median (lower = smoother, more lag)

CART_VEL_MAX  = 4.0         # mm per tick at ±1g (full tilt)
Z_VEL_MAX     = 3.0         # mm per tick at full Z gesture (face-down)
DEAD_XY       = 0.08        # fraction of g (~5°) — X/Y dead zone
DEAD_Z        = 0.40        # fraction of g (~66° from flat) — Z dead zone
MIN_DELTA     = 2.0         # mm — minimum TCP change needed to send a new command

# Workspace limits in world-frame mm
X_MIN, X_MAX = -400.0,  400.0
Y_MIN, Y_MAX = -400.0,  400.0
Z_MIN, Z_MAX =  300.0, 1400.0

_stop    = False
_active  = False          # False = paused, True = tracking
_lock    = threading.Lock()


def _sigint(sig, frame):
    global _stop
    _stop = True


def _key_thread():
    """Watch for Enter key presses to toggle tracking."""
    global _active, _stop
    while not _stop:
        if msvcrt.kbhit():
            key = msvcrt.getch()
            if key in (b'\r', b'\n'):
                with _lock:
                    _active = not _active
                    state = "TRACKING" if _active else "PAUSED"
                print(f"\n[{state}]", flush=True)
        time.sleep(0.02)


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


def query_alarm(sock):
    payload = {
        "dsID": "www.hc-system.com.RemoteMonitor",
        "packID": "alarm",
        "reqType": "query",
        "queryAddr": ["curAlarm", "curMode", "isMoving"],
    }
    enc = json.dumps(payload, separators=(",",":")).encode("ascii")
    sock.sendall(enc)
    r = json.loads(sock.recv(65536))
    vals = r.get("queryData", [])
    return dict(zip(["curAlarm","curMode","isMoving"], vals))


def send_cartesian_move(sock, x, y, z, rx, ry, rz, pack_id, verbose=False):
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
    if verbose:
        print(f"\n[TX] {enc.decode()}", flush=True)
    sock.sendall(enc)
    raw = sock.recv(4096)
    try:
        resp = json.loads(raw)
    except Exception:
        resp = {"raw": raw.decode("ascii", errors="replace")}
    if verbose:
        print(f"[RX] {json.dumps(resp)}", flush=True)
    return resp


def deadzone(v, dz):
    if abs(v) < dz:
        return 0.0
    return math.copysign(abs(v) - dz, v)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def read_latest_accel(ser):
    """Drain serial buffer and return the last valid (ax, ay, az) tuple, or None."""
    result = None
    while ser.in_waiting:
        raw = ser.readline().decode("ascii", errors="ignore").strip()
        try:
            vals = list(map(float, raw.split(",")))
            if len(vals) == 3:
                result = tuple(vals)
        except Exception:
            pass
    return result


def main():
    global _stop, _active
    signal.signal(signal.SIGINT, _sigint)

    # ── Serial ──────────────────────────────────────────────────────────────────
    print(f"Opening accelerometer on {SERIAL_PORT}...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD, timeout=1.0)
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        sys.exit(1)
    time.sleep(0.5)
    ser.reset_input_buffer()

    # ── Robot ────────────────────────────────────────────────────────────────────
    print(f"Connecting to robot at {ROBOT_IP}:{ROBOT_PORT}...")
    try:
        sock = socket.create_connection((ROBOT_IP, ROBOT_PORT), timeout=5.0)
        sock.settimeout(2.0)
    except OSError as e:
        print(f"Robot connection failed: {e}")
        sys.exit(1)

    # ── Read initial state ───────────────────────────────────────────────────────
    print("Reading arm state...")
    try:
        payload = {
            "dsID": "www.hc-system.com.RemoteMonitor",
            "packID": "init",
            "reqType": "query",
            "queryAddr": [
                "curMode", "curAlarm", "isMoving",
                "axis-0","axis-1","axis-2","axis-3","axis-4","axis-5",
                "world-0","world-1","world-2","world-3","world-4","world-5",
            ],
        }
        enc = json.dumps(payload, separators=(",",":")).encode("ascii")
        sock.sendall(enc)
        r = json.loads(sock.recv(65536))
        vals = r.get("queryData", [])
        cur_mode, cur_alarm, is_moving = vals[0], vals[1], vals[2]
        joints = [float(v) for v in vals[3:9]]
        world  = [float(v) for v in vals[9:15]]
    except Exception as e:
        print(f"Failed to read arm state: {e}")
        sys.exit(1)

    print(f"  Mode={cur_mode}  Alarm={cur_alarm}  Moving={is_moving}")
    print(f"  J1={joints[0]:+7.2f}  J2={joints[1]:+7.2f}  J3={joints[2]:+7.2f}"
          f"  J4={joints[3]:+7.2f}  J5={joints[4]:+7.2f}  J6={joints[5]:+7.2f}")

    tcp_x,  tcp_y,  tcp_z  = world[0], world[1], world[2]
    tcp_rx, tcp_ry, tcp_rz = world[3], world[4], world[5]
    print(f"  X={tcp_x:.1f}  Y={tcp_y:.1f}  Z={tcp_z:.1f}"
          f"  RX={tcp_rx:.1f}  RY={tcp_ry:.1f}  RZ={tcp_rz:.1f}")

    if str(cur_alarm) != "0":
        print(f"  WARNING: active alarm {cur_alarm} — clear it before tracking")
    if str(cur_mode) not in ("2", "7"):
        print(f"  WARNING: curMode={cur_mode}, expected 2 or 7")

    print("Reading sensor...")
    ax_s, ay_s, az_s = 0.0, 0.0, 9.81
    t0 = time.monotonic()
    while time.monotonic() - t0 < 2.0:
        v = read_latest_accel(ser)
        if v:
            ax_s, ay_s, az_s = v
    print(f"  Accel: ax={ax_s:+.3f}  ay={ay_s:+.3f}  az={az_s:+.3f}  m/s²")

    print()
    print("Axes:  tilt forward/back=X   left/right=Y   flip face-down=Z")
    print("Press Enter to start tracking. Press Enter again to pause. Ctrl+C to exit.")
    print()

    # ── Key listener ─────────────────────────────────────────────────────────────
    kt = threading.Thread(target=_key_thread, daemon=True)
    kt.start()

    # ── Filter state ─────────────────────────────────────────────────────────────
    G = 9.81
    buf = deque(maxlen=MEDIAN_WINDOW)   # raw sample ring buffer for median filter
    gx_s = ax_s / G
    gy_s = ay_s / G
    gz_s = az_s / G

    # Seed buffer
    for _ in range(MEDIAN_WINDOW):
        buf.append((ax_s, ay_s, az_s))

    interval   = 1.0 / SEND_HZ
    move_n     = 0
    last_send  = time.monotonic()
    last_tcp   = (tcp_x, tcp_y, tcp_z)     # last actually-sent position

    while not _stop:
        v = read_latest_accel(ser)
        if v:
            buf.append(v)
            # Step 1: median per axis — kills tap/touch spikes
            ax_med = statistics.median(s[0] for s in buf)
            ay_med = statistics.median(s[1] for s in buf)
            az_med = statistics.median(s[2] for s in buf)
            # Step 2: EMA on median output — smooths residual noise
            gx_s = ALPHA * (ax_med / G) + (1 - ALPHA) * gx_s
            gy_s = ALPHA * (ay_med / G) + (1 - ALPHA) * gy_s
            gz_s = ALPHA * (az_med / G) + (1 - ALPHA) * gz_s

        now = time.monotonic()
        if now - last_send < interval:
            time.sleep(0.005)
            continue

        last_send = now

        with _lock:
            active = _active

        if not active:
            continue

        # Velocity: each gravity component → world axis velocity
        # gx: nose of sensor dips forward → negative gx → +X
        # gy: right roll → positive gy → +Y
        # gz: 1.0 when flat; drops toward 0/-1 when face-down → +Z
        dvx = deadzone(-gx_s, DEAD_XY) * CART_VEL_MAX
        dvy = deadzone( gy_s, DEAD_XY) * CART_VEL_MAX
        dvz = deadzone(gz_s - 1.0, DEAD_Z) * (-Z_VEL_MAX)  # face-down → move down

        dvx = clamp(dvx, -CART_VEL_MAX, CART_VEL_MAX)
        dvy = clamp(dvy, -CART_VEL_MAX, CART_VEL_MAX)
        dvz = clamp(dvz, -Z_VEL_MAX,    Z_VEL_MAX)

        tcp_x = clamp(tcp_x + dvx, X_MIN, X_MAX)
        tcp_y = clamp(tcp_y + dvy, Y_MIN, Y_MAX)
        tcp_z = clamp(tcp_z + dvz, Z_MIN, Z_MAX)

        # Send gate: skip command if target barely moved (prevents jitter chatter)
        moved = (abs(tcp_x - last_tcp[0]) > MIN_DELTA or
                 abs(tcp_y - last_tcp[1]) > MIN_DELTA or
                 abs(tcp_z - last_tcp[2]) > MIN_DELTA)
        if not moved:
            print(f"[hold ]  gx:{gx_s:+.2f} gy:{gy_s:+.2f} gz:{gz_s:+.2f}"
                  f"  ->  X:{tcp_x:+7.1f}  Y:{tcp_y:+7.1f}  Z:{tcp_z:+7.1f}",
                  flush=True)
            continue

        last_tcp = (tcp_x, tcp_y, tcp_z)
        move_n += 1
        verbose_this = (move_n == 1)   # always log first command in full
        try:
            resp = send_cartesian_move(sock, tcp_x, tcp_y, tcp_z,
                                       tcp_rx, tcp_ry, tcp_rz,
                                       pack_id=f"acc-{move_n:05d}",
                                       verbose=verbose_this)

            # Detect error in response
            resp_str = json.dumps(resp)
            has_error = ("error" in resp_str.lower() or "alarm" in resp_str.lower()
                         or "fail" in resp_str.lower())
            if has_error or verbose_this:
                print(f"\n[{move_n:05d}] RX: {resp_str}", flush=True)

            if has_error:
                status = query_alarm(sock)
                print(f"  curAlarm={status.get('curAlarm')}  "
                      f"curMode={status.get('curMode')}  "
                      f"isMoving={status.get('isMoving')}", flush=True)
                _active = False
                print("[PAUSED — press Enter to resume or Ctrl+C to exit]", flush=True)

            print(f"[{move_n:05d}]  gx:{gx_s:+.2f} gy:{gy_s:+.2f} gz:{gz_s:+.2f}"
                  f"  ->  X:{tcp_x:+7.1f}  Y:{tcp_y:+7.1f}  Z:{tcp_z:+7.1f}",
                  flush=True)
        except OSError as e:
            print(f"\nRobot connection lost: {e}", flush=True)
            try:
                status = query_alarm(sock)
                print(f"  curAlarm={status.get('curAlarm')}  curMode={status.get('curMode')}")
            except Exception:
                pass
            break

    sock.close()
    ser.close()
    print("\nExited.")


if __name__ == "__main__":
    main()
