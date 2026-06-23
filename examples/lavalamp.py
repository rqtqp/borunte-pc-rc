"""
lavalamp.py — Continuous smooth random motion demo for Borunte BRTIRUS0707A.

Generates random joint waypoints and moves through them continuously,
creating an organic flowing motion. Press Ctrl+C to stop — the robot
returns to home (0°) automatically.

Prerequisites:
  - Robot powered on, in Auto mode
  - 'pc_rc' pendant program running (curMode = 7)
  - No active alarms, origin set

Usage:
    python lavalamp.py
"""

import json
import random
import signal
import socket
import sys
import time

ROBOT_IP      = "10.0.0.49"  # <-- change to your robot's IP
PORT          = 9760
TIMEOUT       = 5.0
SPEED         = 15.0   # % of max speed
MAX_STEP      = 10.0   # max degrees any joint moves per waypoint
VEL_CHANGE    = 2.0    # max velocity nudge per step — controls how quickly direction drifts
PAUSE         = 3.0    # seconds to rest between moves — lets servos cool
ACTIVE_JOINTS = 3      # joints moving per step — others hold and rest

# Soft limits — stays comfortably inside the hardware stops
LIMITS = [
    (-130, 130),   # J1  base rotation    (hard: ±174°)
    ( -40, -20),   # J2  shoulder — already narrow, unchanged
    ( -20, 135),   # J3  elbow            (hard: -60° / +175°)
    (-135, 135),   # J4  wrist pitch      (hard: ±180°)
    ( -75,  75),   # J5  wrist roll       (hard: ±120°)
    (-315, 315),   # J6  wrist yaw        (hard: ±360°)
]

_stop_requested = False
_velocities = [random.uniform(-MAX_STEP, MAX_STEP) for _ in range(6)]


def _sigint(sig, frame):
    global _stop_requested
    print("\nCtrl+C — finishing current move then going home...")
    _stop_requested = True


def send_json(payload):
    enc = json.dumps(payload, separators=(",", ":")).encode("ascii")
    with socket.create_connection((ROBOT_IP, PORT), timeout=TIMEOUT) as s:
        s.settimeout(TIMEOUT)
        s.sendall(enc)
        return json.loads(s.recv(65536))


def query_state():
    r = send_json({
        "dsID": "www.hc-system.com.RemoteMonitor",
        "packID": "poll",
        "reqType": "query",
        "queryAddr": ["isMoving", "axis-0", "axis-1", "axis-2",
                      "axis-3", "axis-4", "axis-5"],
    })
    return dict(zip(r.get("queryAddr", []), r.get("queryData", [])))


def move_to(targets, speed=SPEED, pack_id="move"):
    reply = send_json({
        "dsID": "HCRemoteCommand",
        "reqType": "AddRCC",
        "emptyList": "1",
        "packID": pack_id,
        "instructions": [{
            "oneshot": "1",
            "action": "4",
            "m0": f"{targets[0]:.2f}", "m1": f"{targets[1]:.2f}",
            "m2": f"{targets[2]:.2f}", "m3": f"{targets[3]:.2f}",
            "m4": f"{targets[4]:.2f}", "m5": f"{targets[5]:.2f}",
            "m6": "0.0", "m7": "0.0",
            "ckStatus": "0x3F",
            "speed": str(speed),
            "delay": "0.0", "tool": "0", "coord": "0", "smooth": "0",
        }],
    })
    return reply.get("cmdReply", ["?"])


def wait_done():
    deadline = time.monotonic() + 120
    while time.monotonic() < deadline:
        st = query_state()
        if st.get("isMoving") != "1":
            return [float(st.get(f"axis-{i}", 0)) for i in range(6)]
        time.sleep(0.2)
    raise TimeoutError("Move timed out after 120 s")


def next_waypoint(current):
    # Only ACTIVE_JOINTS randomly chosen joints move — the rest hold position and rest.
    # Momentum-based drift prevents sharp direction reversals.
    active = set(random.sample(range(6), ACTIVE_JOINTS))
    target = []
    for i, (lo, hi) in enumerate(LIMITS):
        if i not in active:
            target.append(current[i])  # hold — servo rests this step
            continue

        _velocities[i] += random.uniform(-VEL_CHANGE, VEL_CHANGE)
        _velocities[i] = max(-MAX_STEP, min(MAX_STEP, _velocities[i]))

        new_pos = current[i] + _velocities[i]
        if new_pos <= lo:
            new_pos = lo
            _velocities[i] = abs(_velocities[i])
        elif new_pos >= hi:
            new_pos = hi
            _velocities[i] = -abs(_velocities[i])

        target.append(new_pos)
    return target


def main():
    signal.signal(signal.SIGINT, _sigint)

    print("Reading current position...")
    try:
        st = query_state()
    except OSError as e:
        print(f"Connection failed: {e}")
        sys.exit(1)

    current = [float(st.get(f"axis-{i}", 0)) for i in range(6)]
    print(f"Start: {[f'{v:+.1f}' for v in current]}")
    print(f"Speed: {SPEED}%   Max step: {MAX_STEP}°   Pause: {PAUSE}s   Active joints: {ACTIVE_JOINTS}/6")
    print("Running — Ctrl+C to stop\n")

    move_n = 0
    while not _stop_requested:
        target = next_waypoint(current)
        move_n += 1
        result = move_to(target, pack_id=f"dance-{move_n:04d}")
        joints = "  ".join(f"J{i+1}:{target[i]:+5.1f}" for i in range(6))
        print(f"[{move_n:04d}] {result}  {joints}")
        current = wait_done()
        time.sleep(PAUSE)

    # Return to home
    print("\nReturning to home (0°)...")
    move_to([0.0] * 6, speed=50.0, pack_id="home")
    wait_done()
    print("Home. Stopped.")


if __name__ == "__main__":
    main()
