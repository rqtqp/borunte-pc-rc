"""
check_state.py — Query the Borunte HC1 controller and print full status.

Usage:
    python check_state.py

No dependencies beyond Python 3 stdlib.
"""

import json
import socket
import sys

ROBOT_IP = "10.0.0.49"  # <-- change to your robot's IP
PORT     = 9760
TIMEOUT  = 5.0

MODE_NAMES = {
    "0": "init",
    "1": "manual",
    "2": "auto (idle)",
    "3": "stop",
    "7": "auto-running  <-- ready for motion commands",
}

JOINT_NAMES = [
    "J1  base rotation",
    "J2  shoulder",
    "J3  elbow",
    "J4  wrist pitch",
    "J5  wrist roll",
    "J6  wrist yaw",
]


def query_state():
    payload = {
        "dsID": "www.hc-system.com.RemoteMonitor",
        "packID": "check-state",
        "reqType": "query",
        "queryAddr": [
            "version", "curMode", "axisNum", "curAlarm",
            "isMoving", "origin",
            "axis-0", "axis-1", "axis-2", "axis-3", "axis-4", "axis-5",
        ],
    }
    enc = json.dumps(payload, separators=(",", ":")).encode("ascii")
    with socket.create_connection((ROBOT_IP, PORT), timeout=TIMEOUT) as s:
        s.settimeout(TIMEOUT)
        s.sendall(enc)
        return json.loads(s.recv(65536))


def main():
    print(f"Connecting to {ROBOT_IP}:{PORT} ...")
    try:
        response = query_state()
    except OSError as e:
        print(f"Connection failed: {e}")
        print("Check that the robot is powered on and on the same network.")
        sys.exit(1)

    keys   = response.get("queryAddr", [])
    values = response.get("queryData", [])
    state  = dict(zip(keys, values))

    mode = state.get("curMode", "?")

    print()
    print("=" * 48)
    print("  BORUNTE HC1  —  CONTROLLER STATUS")
    print("=" * 48)
    print(f"  Firmware :  {state.get('version', '?')}")
    print(f"  Mode     :  {mode}  ({MODE_NAMES.get(mode, 'unknown')})")
    print(f"  Alarm    :  {state.get('curAlarm', '?')}")
    print(f"  Moving   :  {'YES' if state.get('isMoving') == '1' else 'no'}")
    print(f"  Origin   :  {'set' if state.get('origin') == '1' else 'NOT SET — run homing on pendant'}")
    print(f"  Axes     :  {state.get('axisNum', '?')}")
    print()
    print("  Joint positions (degrees):")
    for i, name in enumerate(JOINT_NAMES):
        val = state.get(f"axis-{i}", "?")
        print(f"    {name:22s}  {float(val):+9.3f}°")
    print("=" * 48)

    # Readiness summary
    ok_mode  = mode in ("2", "7")
    ok_alarm = state.get("curAlarm") == "0"
    ok_move  = state.get("isMoving") == "0"
    ok_orig  = state.get("origin") == "1"
    ready    = ok_mode and ok_alarm and ok_move and ok_orig

    print()
    if ready:
        if mode == "7":
            print("  READY — pendant program running, motion commands accepted.")
        else:
            print("  Auto mode idle. Start the 'pc_rc' pendant program to enable motion.")
    else:
        print("  NOT READY:")
        if not ok_mode:
            print("    - switch pendant to Auto mode")
        if not ok_alarm:
            print(f"    - clear alarm {state.get('curAlarm')} on pendant")
        if not ok_move:
            print("    - wait for current motion to finish")
        if not ok_orig:
            print("    - run origin/homing procedure on pendant")
    print()


if __name__ == "__main__":
    main()
