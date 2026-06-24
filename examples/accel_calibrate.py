"""
accel_calibrate.py — Live LIS2DW12 orientation display.

Shows raw m/s², tilt angles, dominant gravity axis, and a live preview
of the arm control mapping (pitch->X, roll->Y) with dead zone markers.

Usage:
  python accel_calibrate.py

Hold the sensor in different orientations to identify axes.
Press Ctrl+C to exit.
"""

import math
import serial
import signal
import sys
import time

SERIAL_PORT = "COM5"
BAUD        = 115200

# Must match accel_control.py
DEAD_ANGLE   = 10.0    # degrees — safe zone, no arm movement
MAX_ANGLE    = 45.0    # degrees — full speed
CART_VEL_MAX = 5.0     # mm per tick at MAX_ANGLE

_stop = False


def _sigint(sig, frame):
    global _stop
    _stop = True


def read_latest(ser):
    result = None
    while ser.in_waiting:
        raw = ser.readline().decode("ascii", errors="ignore").strip()
        try:
            v = list(map(float, raw.split(",")))
            if len(v) == 3:
                result = tuple(v)
        except Exception:
            pass
    return result


def dominant_axis(gx, gy, gz):
    axes = [
        (abs(gx), "X+" if gx > 0 else "X-"),
        (abs(gy), "Y+" if gy > 0 else "Y-"),
        (abs(gz), "Z+" if gz > 0 else "Z-"),
    ]
    mag, label = max(axes, key=lambda t: t[0])
    if mag < 0.7:
        return "TILTED (no dominant axis)"
    descriptions = {
        "Z+": "Z axis UP   (sensor face-up flat)",
        "Z-": "Z axis DOWN (sensor face-down flat)",
        "X+": "X axis UP   (sensor nose-up / tilted back)",
        "X-": "X axis DOWN (sensor nose-down / tilted forward)",
        "Y+": "Y axis UP   (sensor rolled right-side up)",
        "Y-": "Y axis DOWN (sensor rolled left-side up)",
    }
    return descriptions.get(label, label)


def tilt_to_vel(angle_deg):
    a = abs(angle_deg)
    if a < DEAD_ANGLE:
        return 0.0
    eff = min(a - DEAD_ANGLE, MAX_ANGLE - DEAD_ANGLE)
    v = eff / (MAX_ANGLE - DEAD_ANGLE) * CART_VEL_MAX
    return math.copysign(v, angle_deg)


def control_bar(angle, width=40):
    """
    Angle bar with dead zone markers.
    Width covers ±MAX_ANGLE degrees.
    Dead zone region marked with dots, active region with #.
    """
    center = width // 2
    dead_px = int(DEAD_ANGLE / MAX_ANGLE * center)   # pixels for dead zone on each side

    # Build the bar character by character
    bar = []
    for i in range(width):
        pos = i - center                              # pixel offset from center
        adeg = abs(pos) / center * MAX_ANGLE         # angle this pixel represents

        if adeg < DEAD_ANGLE:
            bar.append("·")                           # dead zone
        else:
            bar.append(" ")                           # active zone (empty by default)

    # Place cursor at current angle
    cursor_px = int(angle / MAX_ANGLE * center)
    cursor_px = max(-center, min(center - 1, cursor_px))
    idx = center + cursor_px

    vel = tilt_to_vel(angle)
    if abs(angle) < DEAD_ANGLE:
        bar[idx] = "│"                               # inside dead zone
    elif vel > 0:
        bar[idx] = "►"
    else:
        bar[idx] = "◄"

    return "".join(bar)


def vel_label(angle):
    v = tilt_to_vel(angle)
    if v == 0.0:
        return "HOLD  (dead zone)"
    mm_s = abs(v) * 10                              # mm/s at SEND_HZ=10
    direction = "→" if v > 0 else "←"
    return f"{direction}  {mm_s:.0f} mm/s"


def main():
    global _stop
    signal.signal(signal.SIGINT, _sigint)

    print(f"Opening {SERIAL_PORT}...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD, timeout=1.0)
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        sys.exit(1)
    time.sleep(0.5)
    ser.reset_input_buffer()

    G     = 9.81
    ALPHA = 0.1
    ax_s = ay_s = 0.0
    az_s = G

    t0 = time.monotonic()
    while time.monotonic() - t0 < 1.0:
        v = read_latest(ser)
        if v:
            ax_s, ay_s, az_s = v

    last_print = 0.0

    while not _stop:
        v = read_latest(ser)
        if v:
            ax, ay, az = v
            ax_s = ALPHA * ax + (1 - ALPHA) * ax_s
            ay_s = ALPHA * ay + (1 - ALPHA) * ay_s
            az_s = ALPHA * az + (1 - ALPHA) * az_s

        now = time.monotonic()
        if now - last_print < 0.12:
            time.sleep(0.02)
            continue
        last_print = now

        norm = math.sqrt(ax_s**2 + ay_s**2 + az_s**2)
        if norm < 0.1:
            continue
        gx = ax_s / norm
        gy = ay_s / norm
        gz = az_s / norm

        pitch = math.degrees(math.atan2(-ax_s, math.sqrt(ay_s**2 + az_s**2)))
        roll  = math.degrees(math.atan2(ay_s, az_s))

        dom = dominant_axis(gx, gy, gz)

        print("\033[2J\033[H", end="")
        print("═" * 58)
        print("  LIS2DW12 Calibration / Axis Orientation")
        print("═" * 58)
        print()
        print(f"  Raw (m/s²)  ax={ax_s:+7.3f}  ay={ay_s:+7.3f}  az={az_s:+7.3f}")
        print(f"  Normalized  gx={gx:+6.3f}   gy={gy:+6.3f}   gz={gz:+6.3f}")
        print(f"  |g| = {norm:.3f} m/s²")
        print()
        print(f"  Gravity direction  →  {dom}")
        print()
        print("  ── Raw tilt angles ──────────────────────────────────")
        print(f"  Pitch (fwd/back) : {pitch:+7.2f}°")
        print(f"  Roll  (left/right): {roll:+7.2f}°")
        print()
        print(f"  ── Arm control preview  (dead={DEAD_ANGLE}°  full={MAX_ANGLE}°) ──")
        print(f"  ·  = dead zone   space = active   ►◄ = current position")
        print()
        print(f"  PITCH → X  [{control_bar(pitch)}]  {pitch:+5.1f}°")
        print(f"              {vel_label(pitch)}")
        print()
        print(f"  ROLL  → Y  [{control_bar(roll)}]  {roll:+5.1f}°")
        print(f"              {vel_label(roll)}")
        print()
        print("  Ctrl+C to exit")

    ser.close()
    print("\nDone.")


if __name__ == "__main__":
    main()
