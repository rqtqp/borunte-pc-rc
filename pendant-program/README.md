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

## How to create it manually on the pendant

If you prefer to create the program by hand instead of importing the zip:

1. On the pendant, switch to **Manual mode**.
2. Create a new program.
3. Press **New M CMD** to add an instruction.
4. In the instruction type list, select **Long Distance Command** (远程指令).
5. Set the **Data Source** to `www.hc-system.com.HCRemoteCommand::[HID:100]`.
6. Confirm and save.

![Adding Long Distance Command instruction on pendant](../images/pendant-add-instruction.jpg)

The instruction appears as: `Wait Long Distance Command Data Source: www.hc-system.com.HCRemoteComm...`

Enable **Cycle** mode on the program so it loops automatically after each command executes.

## How to run it

1. On the pendant, switch to **Auto mode**.
2. Open the program.
3. Press **Start**.
4. The status bar shows `Auto/Running` and the program pauses at the "Wait Long Distance Command" step.

The controller is now in `curMode = 7` (Auto-running) and ready to accept PC motion commands.

![Pendant in Auto/Running mode with program waiting for remote command](../images/pendant-pc-rc-running.jpg)

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
