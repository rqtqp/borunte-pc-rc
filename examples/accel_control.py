"""
accel_control.py — Hold the BTT LIS2DW12. Arm mirrors tilt in joint space.

Mapping (velocity mode — stop tilting = arm stops):
  Roll  (left / right)   -> J1  (base rotation)
  Pitch (fwd  / back)    -> J2  (shoulder)
  J3-J6 stay fixed.

Safe zone: ±10° dead band per axis.
Speed bands: every 15° adds one tier (5 / 10 / 15 / 20 °/s).

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

SEND_HZ       = 10
MEDIAN_WINDOW = 9           # ~180ms spike filter
ALPHA         = 0.08        # EMA smoothing

# Tilt → joint velocity bands
DEAD_ANGLE  = 10.0          # degrees — no movement inside
BAND_DEG    = 15.0          # band width
BAND_SPEEDS = [0.5, 1.0, 1.5, 2.0]   # °/tick → ×10 Hz = 5/10/15/20 °/s

MIN_DELTA   = 0.3           # degrees — skip command if joint target barely changed

# Joint soft limits (degrees)
J1_MIN, J1_MAX = -150.0, 150.0
J2_MIN, J2_MAX = -100.0,  70.0

# Display range for the band bars
_DISPLAY_MAX = DEAD_ANGLE + len(BAND_SPEEDS) * BAND_DEG   # 70°
_BAND_CHARS  = ["1", "2", "3", "4"]

_stop   = False
_active = False
_lock   = threading.Lock()


def _sigint(sig, frame):
    global _stop
    _stop = True


def _key_thread():
    global _active, _stop
    while not _stop:
        if msvcrt.kbhit():
            key = msvcrt.getch()
            if key in (b'\r', b'\n'):
                with _lock:
                    _active = not _active
        time.sleep(0.02)


def read_latest_accel(ser):
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


def query_arm(sock):
    payload = {
        "dsID": "www.hc-system.com.RemoteMonitor",
        "packID": "init",
        "reqType": "query",
        "queryAddr": [
            "curMode", "curAlarm", "isMoving",
            "axis-0","axis-1","axis-2","axis-3","axis-4","axis-5",
        ],
    }
    enc = json.dumps(payload, separators=(",",":")).encode("ascii")
    sock.sendall(enc)
    r = json.loads(sock.recv(65536))
    vals = r.get("queryData", [])
    return vals[0], vals[1], vals[2], [float(v) for v in vals[3:9]]


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


def send_joint_move(sock, joints, pack_id, verbose=False):
    payload = {
        "dsID": "HCRemoteCommand",
        "reqType": "AddRCC",
        "emptyList": "1",
        "packID": pack_id,
        "instructions": [{
            "oneshot": "1",
            "action": "4",
            "m0": f"{joints[0]:.3f}", "m1": f"{joints[1]:.3f}",
            "m2": f"{joints[2]:.3f}", "m3": f"{joints[3]:.3f}",
            "m4": f"{joints[4]:.3f}", "m5": f"{joints[5]:.3f}",
            "m6": "0.0", "m7": "0.0",
            "ckStatus": "0x3F",
            "speed": "30.0",
            "delay": "0.0", "tool": "0", "coord": "0", "smooth": "0",
        }],
    }
    enc = json.dumps(payload, separators=(",",":")).encode("ascii")
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


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def tilt_to_vel(angle_deg):
    a = abs(angle_deg)
    if a < DEAD_ANGLE:
        return 0.0
    band = min(int((a - DEAD_ANGLE) / BAND_DEG), len(BAND_SPEEDS) - 1)
    return math.copysign(BAND_SPEEDS[band], angle_deg)


def control_bar(angle, width=42):
    center = width // 2
    bar = []
    for i in range(width):
        adeg = abs(i - center) / center * _DISPLAY_MAX
        if adeg < DEAD_ANGLE:
            bar.append("·")
        else:
            b = min(int((adeg - DEAD_ANGLE) / BAND_DEG), len(BAND_SPEEDS) - 1)
            bar.append(_BAND_CHARS[b])
    cursor_px = int(angle / _DISPLAY_MAX * center)
    cursor_px = max(-center, min(center - 1, cursor_px))
    idx = center + cursor_px
    vel = tilt_to_vel(angle)
    bar[idx] = "│" if vel == 0.0 else ("►" if vel > 0 else "◄")
    return "".join(bar)


def vel_label(angle):
    v = tilt_to_vel(angle)
    if v == 0.0:
        return "HOLD"
    band = min(int((abs(angle) - DEAD_ANGLE) / BAND_DEG), len(BAND_SPEEDS) - 1)
    deg_s = abs(v) * SEND_HZ
    return f"{'→' if v > 0 else '←'}  band {band+1}  {deg_s:.0f} °/s"


def draw(pitch, roll, j1, j2, joints, active, move_n, alarm_str):
    state = "TRACKING" if active else "PAUSED  "
    print("\033[2J\033[H", end="")
    print("═" * 56)
    print(f"  Arm Control  [{state}]   move #{move_n}")
    print("═" * 56)
    print()
    print(f"  ROLL  → J1  [{control_bar(roll)}]  {roll:+5.1f}°")
    print(f"               {vel_label(roll)}")
    print()
    print(f"  PITCH → J2  [{control_bar(pitch)}]  {pitch:+5.1f}°")
    print(f"               {vel_label(pitch)}")
    print()
    print(f"  · dead ±{DEAD_ANGLE:.0f}°   " +
          "  ".join(f"{DEAD_ANGLE+i*BAND_DEG:.0f}-{DEAD_ANGLE+(i+1)*BAND_DEG:.0f}°={BAND_SPEEDS[i]*SEND_HZ:.0f}°/s"
                    for i in range(len(BAND_SPEEDS))))
    print()
    print(f"  Target   J1={j1:+7.2f}°   J2={j2:+7.2f}°")
    print(f"  Current  J1={joints[0]:+7.2f}°   J2={joints[1]:+7.2f}°  "
          f"J3={joints[2]:+7.2f}°")
    print(f"           J4={joints[3]:+7.2f}°   J5={joints[4]:+7.2f}°  "
          f"J6={joints[5]:+7.2f}°")
    if alarm_str:
        print(f"\n  !! {alarm_str}")
    print()
    print("  Enter = toggle tracking   Ctrl+C = exit")


def main():
    global _stop, _active
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

    print("Reading arm state...")
    try:
        cur_mode, cur_alarm, is_moving, joints = query_arm(sock)
    except Exception as e:
        print(f"Failed: {e}")
        sys.exit(1)

    print(f"  Mode={cur_mode}  Alarm={cur_alarm}  Moving={is_moving}")
    print(f"  J1={joints[0]:+7.2f}  J2={joints[1]:+7.2f}  J3={joints[2]:+7.2f}"
          f"  J4={joints[3]:+7.2f}  J5={joints[4]:+7.2f}  J6={joints[5]:+7.2f}")
    if str(cur_alarm) != "0":
        print(f"  WARNING: active alarm {cur_alarm}")
    if str(cur_mode) not in ("2", "7"):
        print(f"  WARNING: curMode={cur_mode}, expected 2 or 7")

    print("Reading sensor...")
    ax_s, ay_s, az_s = 0.0, 0.0, 9.81
    t0 = time.monotonic()
    while time.monotonic() - t0 < 1.5:
        v = read_latest_accel(ser)
        if v:
            ax_s, ay_s, az_s = v
    print(f"  ax={ax_s:+.3f}  ay={ay_s:+.3f}  az={az_s:+.3f}  m/s²")
    print()
    print("Press Enter to start tracking. Ctrl+C to exit.")
    input()   # wait for first Enter before starting key thread + display

    kt = threading.Thread(target=_key_thread, daemon=True)
    kt.start()
    _active = True   # first Enter already consumed above

    # Filter state
    G = 9.81
    buf = deque(maxlen=MEDIAN_WINDOW)
    for _ in range(MEDIAN_WINDOW):
        buf.append((ax_s, ay_s, az_s))
    ax_med = statistics.median(s[0] for s in buf)
    ay_med = statistics.median(s[1] for s in buf)
    az_med = statistics.median(s[2] for s in buf)
    pitch_s = math.degrees(math.atan2(-ax_med, math.sqrt(ay_med**2 + az_med**2)))
    roll_s  = math.degrees(math.atan2(ay_med, az_med))

    # Joint targets start at current position
    j1 = joints[0]
    j2 = joints[1]
    last_j1, last_j2 = j1, j2

    interval  = 1.0 / SEND_HZ
    move_n    = 0
    last_send = time.monotonic()
    alarm_str = ""
    last_draw = 0.0

    while not _stop:
        v = read_latest_accel(ser)
        if v:
            buf.append(v)
            ax_med = statistics.median(s[0] for s in buf)
            ay_med = statistics.median(s[1] for s in buf)
            az_med = statistics.median(s[2] for s in buf)
            pitch_raw = math.degrees(math.atan2(-ax_med, math.sqrt(ay_med**2 + az_med**2)))
            roll_raw  = math.degrees(math.atan2(ay_med, az_med))
            pitch_s = ALPHA * pitch_raw + (1 - ALPHA) * pitch_s
            roll_s  = ALPHA * roll_raw  + (1 - ALPHA) * roll_s

        now = time.monotonic()

        # Refresh display at ~8 Hz independent of send rate
        if now - last_draw >= 0.12:
            with _lock:
                active = _active
            draw(pitch_s, roll_s, j1, j2, joints, active, move_n, alarm_str)
            last_draw = now

        if now - last_send < interval:
            time.sleep(0.005)
            continue
        last_send = now

        with _lock:
            active = _active
        if not active:
            continue

        dj1 = tilt_to_vel(roll_s)
        dj2 = tilt_to_vel(pitch_s)

        j1 = clamp(j1 + dj1, J1_MIN, J1_MAX)
        j2 = clamp(j2 + dj2, J2_MIN, J2_MAX)

        moved = (abs(j1 - last_j1) > MIN_DELTA or abs(j2 - last_j2) > MIN_DELTA)
        if not moved:
            continue

        last_j1, last_j2 = j1, j2
        move_n += 1
        target = [j1, j2, joints[2], joints[3], joints[4], joints[5]]

        try:
            resp = send_joint_move(sock, target, pack_id=f"acc-{move_n:05d}",
                                   verbose=(move_n == 1))
            resp_str = json.dumps(resp)
            has_error = any(w in resp_str.lower() for w in ("error","alarm","fail"))
            if has_error:
                status = query_alarm(sock)
                _active = False
                # Print clearly outside the live screen
                print("\033[2J\033[H", end="")
                print("═" * 56)
                print("  ERROR on move #" + str(move_n))
                print("═" * 56)
                print(f"  TX target:  J1={target[0]:.2f}  J2={target[1]:.2f}")
                print(f"  RX raw:     {resp_str}")
                print(f"  curAlarm={status.get('curAlarm')}  "
                      f"curMode={status.get('curMode')}  "
                      f"isMoving={status.get('isMoving')}")
                print()
                print("  Press Enter to resume or Ctrl+C to exit.")
                alarm_str = f"RX={resp_str}"
            else:
                alarm_str = ""
        except OSError as e:
            alarm_str = f"connection lost: {e}"
            print(f"\n!! {alarm_str}")
            break

    sock.close()
    ser.close()
    print("\033[2J\033[H", end="")
    print("Exited.")


if __name__ == "__main__":
    main()
