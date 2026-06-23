# Pendant Program — pc_rc

This folder contains `pc_rc.zip` — the pendant program that must be running on the HC1 controller for PC motion commands to work.

## What it does

The program contains a single "wait for remote command" instruction in a loop. When the controller is running this program in Auto mode, it sits at the wait step and accepts `AddRCC` motion commands from the PC. Each command the PC sends is executed immediately; the program then loops back to wait for the next one.

## How to load it

### Method A — USB stick (easiest)

1. Copy `pc_rc.zip` to the root of a USB stick.
2. Insert the USB stick into the pendant's USB port.
3. On the pendant: **Program** → **Import** → select `pc_rc.zip` → confirm.
4. The program `pc_rc` will appear in the program list.

### Method B — HCEdit PC software

1. Open HCEdit and connect to the controller.
2. Go to **Program** → **Import from file**.
3. Select `pc_rc.zip`.

## How to run it

1. On the pendant, switch to **Auto mode**.
2. Open the program `pc_rc`.
3. Enable **Cycle** mode (so the program loops automatically).
4. Press **Start**.
5. The status bar should show the program running and paused at the "remote command" step.

The controller is now in `curMode = 7` (Auto-running) and ready to accept PC motion commands.

![Pendant in Auto mode with pc_rc running](../images/pendant-auto-running.jpg)

## What you should see on the PC

Run `python examples/check_state.py`. You should see:

```
  Mode     :  7  (auto-running  <-- ready for motion commands)
  ...
  READY — pendant program running, motion commands accepted.
```

## Notes

- The program does not move the robot by itself — it only waits.
- You can interrupt the program from the pendant at any time using the Stop or E-Stop button.
- If the controller loses TCP connection mid-move, the current motion will complete normally and the program will return to the wait step.
