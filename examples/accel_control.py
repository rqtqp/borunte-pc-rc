"""
accel_control.py — Hold the BTT LIS2DW12. Arm mirrors tilt in joint space.

Mapping (velocity mode — stop tilting = arm stops):
  Roll  (left / right)   -> J6  (wrist yaw, ±360°)
  J1-J5 stay fixed.

Safe zone: ±10° dead band per axis.
Speed bands: every 15° adds one tier (25 / 50 / 75 / 100%).

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

POLL_INTERVAL = 0.02        # 20ms — isMoving check rate while arm is cruising
MEDIAN_WINDOW = 9           # ~180ms spike filter
ALPHA         = 0.08        # EMA smoothing

# Tilt → speed bands
DEAD_ANGLE  = 10.0          # degrees — no movement inside
BAND_DEG    = 15.0          # band width
BAND_SPEEDS = [100]  # speed % per band — set directly, no derivation

# Batch motion: multiple via-points per AddRCC with smooth=9 so arm never stops between them
STEP_DEG  = 45.0   # degrees per via-point
NUM_STEPS =  6     # via-points per batch (covers 270° total); arm only stops at the last one

# Joint soft limits (degrees)
J6_MIN, J6_MAX = -350.0, 350.0

# Display range for the band bars
_DISPLAY_MAX = DEAD_ANGLE + len(BAND_SPEEDS) * BAND_DEG   # 70°
_BAND_CHARS  = ["1"]

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


def query_status(sock):
    payload = {
        "dsID": "www.hc-system.com.RemoteMonitor",
        "packID": "st",
        "reqType": "query",
        "queryAddr": ["curAlarm", "curMode", "isMoving",
                      "axis-0","axis-1","axis-2","axis-3","axis-4","axis-5"],
    }
    enc = json.dumps(payload, separators=(",",":")).encode("ascii")
    sock.sendall(enc)
    r = json.loads(sock.recv(65536))
    vals = r.get("queryData", [])
    d = dict(zip(["curAlarm","curMode","isMoving"], vals[:3]))
    if len(vals) >= 9:
        d["joints"] = [float(v) for v in vals[3:9]]
    return d


def send_action_stop(sock, pack_id):
    payload = {
        "dsID": "www.hc-system.com.RemoteMonitor",
        "reqType": "actionStop",
        "packID": pack_id,
    }
    enc = json.dumps(payload, separators=(",",":")).encode("ascii")
    sock.sendall(enc)
    try:
        return json.loads(sock.recv(4096))
    except Exception:
        return {}


def send_joint_move(sock, joints, pack_id, speed=20, smooth="0", verbose=False):
    """Single-instruction move — kept for home/round-trip scripts."""
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
            "speed": str(float(speed)),
            "delay": "0.0", "tool": "0", "coord": "0", "smooth": smooth,
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


def send_batch_move(sock, j1_j5, j6_start, direction, pack_id, speed,
                    empty_list="1", verbose=False):
    """Send NUM_STEPS waypoints in one AddRCC. Returns (response, final_j6).
    empty_list='0' appends to the controller queue without interrupting current motion."""
    instructions = []
    j6 = j6_start
    for i in range(NUM_STEPS):
        j6 = clamp(j6 + direction * STEP_DEG, J6_MIN, J6_MAX)
        instructions.append({
            "oneshot": "1",
            "action": "4",
            "m0": f"{j1_j5[0]:.3f}", "m1": f"{j1_j5[1]:.3f}",
            "m2": f"{j1_j5[2]:.3f}", "m3": f"{j1_j5[3]:.3f}",
            "m4": f"{j1_j5[4]:.3f}", "m5": f"{j6:.3f}",
            "m6": "0.0", "m7": "0.0",
            "ckStatus": "0x3F",
            "speed": str(float(speed)),
            "delay": "0.0", "tool": "0", "coord": "0", "smooth": "0",
        })
    payload = {
        "dsID": "HCRemoteCommand",
        "reqType": "AddRCC",
        "emptyList": empty_list,
        "packID": pack_id,
        "instructions": instructions,
    }
    enc = json.dumps(payload, separators=(",",":")).encode("ascii")
    if verbose:
        print(f"\n[TX emptyList={empty_list} {NUM_STEPS}×{STEP_DEG}°] J6 {j6_start:.1f}→{j6:.1f}",
              flush=True)
    sock.sendall(enc)
    raw = sock.recv(4096)
    try:
        resp = json.loads(raw)
    except Exception:
        resp = {"raw": raw.decode("ascii", errors="replace")}
    if verbose:
        print(f"[RX] {json.dumps(resp)}", flush=True)
    return resp, j6


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def tilt_to_speed(angle_deg):
    """Return speed % (0 = dead zone, else BAND_SPEEDS value) and direction sign."""
    a = abs(angle_deg)
    if a < DEAD_ANGLE:
        return 0, 0
    band = min(int((a - DEAD_ANGLE) / BAND_DEG), len(BAND_SPEEDS) - 1)
    return BAND_SPEEDS[band], int(math.copysign(1, angle_deg))


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
    spd, _ = tilt_to_speed(angle)
    bar[idx] = "│" if spd == 0 else ("►" if angle > 0 else "◄")
    return "".join(bar)


def vel_label(angle):
    spd, _ = tilt_to_speed(angle)
    if spd == 0:
        return "HOLD"
    band = min(int((abs(angle) - DEAD_ANGLE) / BAND_DEG), len(BAND_SPEEDS) - 1)
    return f"{'→' if angle > 0 else '←'}  band {band+1}  {spd}%"


def draw(pitch, roll, j1, j2, joints, active, move_n, alarm_str):
    state = "TRACKING" if active else "PAUSED  "
    print("\033[2J\033[H", end="")
    print("═" * 56)
    print(f"  Arm Control  [{state}]   move #{move_n}")
    print("═" * 56)
    print()
    print(f"  ROLL  → J6  [{control_bar(roll)}]  {roll:+5.1f}°")
    print(f"               {vel_label(roll)}")
    print()
    print(f"  · dead ±{DEAD_ANGLE:.0f}°   " +
          "  ".join(f"{DEAD_ANGLE+i*BAND_DEG:.0f}-{DEAD_ANGLE+(i+1)*BAND_DEG:.0f}°={BAND_SPEEDS[i]}%"
                    for i in range(len(BAND_SPEEDS))))
    print()
    print(f"  Target   J6={j1:+7.2f}°")
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

    # J1-J5 all frozen at startup; only J6 is controlled
    j1_j5_base = joints[0:5]
    j6 = joints[5]

    move_n    = 0
    alarm_str = ""
    last_draw = 0.0
    was_active = False
    last_band = -1
    last_dir  =  0

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
            draw(pitch_s, roll_s, j6, 0, joints, active, move_n, alarm_str)
            last_draw = now

        with _lock:
            active = _active
        if not active:
            was_active = False
            time.sleep(POLL_INTERVAL)
            continue

        spd, direction = tilt_to_speed(roll_s)
        cur_band = min(int((abs(roll_s) - DEAD_ANGLE) / BAND_DEG), len(BAND_SPEEDS) - 1) if spd > 0 else -1

        if spd == 0:
            # Dead zone — arm finishes its current step then gets no new commands
            if not was_active:
                time.sleep(POLL_INTERVAL)
                continue
            was_active = False
            last_band, last_dir = -1, 0
            continue

        # Gate on isMoving=0 — controller rejects mid-motion AddRCC (error 200)
        try:
            st = query_status(sock)
            if st.get("isMoving") not in (0, "0"):
                time.sleep(POLL_INTERVAL)  # arm still cruising — poll again soon
                continue
            if "joints" in st:
                joints = st["joints"]
        except OSError:
            time.sleep(POLL_INTERVAL)
            continue

        # Skip if already at the limit in the commanded direction
        at_limit = (direction > 0 and joints[5] >= J6_MAX - 1.0) or \
                   (direction < 0 and joints[5] <= J6_MIN + 1.0)
        if at_limit:
            alarm_str = f"J6 limit reached ({joints[5]:+.0f}°) — tilt opposite to reverse"
            time.sleep(POLL_INTERVAL)
            continue

        alarm_str = ""
        was_active = True
        last_band, last_dir = cur_band, direction
        speed = spd

        move_n += 1

        try:
            # Batch 1: clear queue and start motion
            resp, j6_queued = send_batch_move(sock, j1_j5_base, joints[5], direction,
                                              pack_id=f"acc-{move_n:05d}",
                                              speed=speed, verbose=(move_n == 1))
            j6 = j6_queued  # update display target
            resp_str = json.dumps(resp)
            has_error = any(w in resp_str.lower() for w in ("error","alarm","fail"))
            if has_error:
                status = query_status(sock)
                _active = False
                print("\033[2J\033[H", end="")
                print("═" * 56)
                print("  ERROR on move #" + str(move_n))
                print("═" * 56)
                print(f"  RX raw:     {resp_str}")
                print(f"  curAlarm={status.get('curAlarm')}  "
                      f"curMode={status.get('curMode')}  "
                      f"isMoving={status.get('isMoving')}")
                print()
                print("  Press Enter to resume or Ctrl+C to exit.")
                alarm_str = f"RX={resp_str}"
                continue

            alarm_str = ""

            # Batch 2+: immediately append more steps while arm is still executing batch 1.
            # emptyList=0 adds to the queue without interrupting current motion.
            # If the controller rejects mid-motion appends we fall back gracefully.
            for _ in range(2):
                move_n += 1
                r2, j6_queued = send_batch_move(sock, j1_j5_base, j6_queued, direction,
                                                pack_id=f"acc-{move_n:05d}",
                                                speed=speed, empty_list="0")
                r2_str = json.dumps(r2)
                if any(w in r2_str.lower() for w in ("error", "fail")):
                    break  # controller rejected queue append — single-batch fallback

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
