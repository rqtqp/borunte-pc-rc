"""
send_move.py — Move all joints to home position (0 degrees).

This is the minimal working example. Read this to understand
the exact JSON structure the HC1 controller requires.

Prerequisites:
  - Robot powered on, in Auto mode
  - 'pc_rc' pendant program running (curMode = 7)
  - No active alarms

Usage:
    python send_move.py
"""

import json
import socket
import time

ROBOT_IP = "10.0.0.49"  # <-- change to your robot's IP
PORT     = 9760
TIMEOUT  = 5.0


def send_json(payload):
    enc = json.dumps(payload, separators=(",", ":")).encode("ascii")
    with socket.create_connection((ROBOT_IP, PORT), timeout=TIMEOUT) as s:
        s.settimeout(TIMEOUT)
        s.sendall(enc)
        return json.loads(s.recv(65536))


# --- Move all joints to 0 degrees ---

move_payload = {
    "dsID": "HCRemoteCommand",  # motion command service
    "reqType": "AddRCC",
    "emptyList": "1",           # "1" = clear queue first (always use this)
    "packID": "my-first-move",  # any unique string
    "instructions": [
        {
            "oneshot": "1",     # execute once (not looping)
            "action": "4",      # 4 = joint-space / free path
            "m0": "0.0",        # J1 target in degrees
            "m1": "0.0",        # J2 target in degrees
            "m2": "0.0",        # J3 target in degrees
            "m3": "0.0",        # J4 target in degrees
            "m4": "0.0",        # J5 target in degrees
            "m5": "0.0",        # J6 target in degrees
            "m6": "0.0",        # unused — must be present
            "m7": "0.0",        # unused — must be present
            "ckStatus": "0x3F", # axis mask: 0x3F = all 6 axes
            "speed": "20.0",    # 0–100% of max speed
            "delay": "0.0",
            "tool": "0",
            "coord": "0",
            "smooth": "0",
        }
    ],
}

print("Sending move to home (0°) at 20% speed...")
reply = send_json(move_payload)
print(f"Reply: {reply['cmdReply']}")
# Expected: ['AddRCC', 'ok']

# Wait for motion to complete
print("Waiting for motion to complete...")
while True:
    status = send_json({
        "dsID": "www.hc-system.com.RemoteMonitor",
        "packID": "poll",
        "reqType": "query",
        "queryAddr": ["isMoving", "axis-0", "axis-1", "axis-2", "axis-3", "axis-4", "axis-5"],
    })
    keys   = status.get("queryAddr", [])
    values = status.get("queryData", [])
    st     = dict(zip(keys, values))

    if st.get("isMoving") != "1":
        joints = [f"J{i+1}={float(st.get(f'axis-{i}', 0)):+.2f}" for i in range(6)]
        print("Done:", "  ".join(joints))
        break
    time.sleep(0.2)
