"""
accel_calibrate.py — Live LIS2DW12 orientation display.

Prints raw m/s², tilt angles, and the dominant gravity axis so you know
which physical direction maps to which sensor axis.

Usage:
  python accel_calibrate.py

Hold the sensor in each orientation for 1-2 seconds; the tool auto-labels
which axis is pointing up/down/left/right/forward/back.

Press Ctrl+C to exit.
"""

import math
import serial
import signal
import sys
import time

SERIAL_PORT = "COM5"
BAUD        = 115200

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
    """Return a human label for whichever axis gravity points strongest along."""
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


def bar(v, scale=1.0, width=20):
    """ASCII bar: negative = left of center, positive = right."""
    center = width // 2
    filled = int(abs(v) / scale * center)
    filled = min(filled, center)
    if v >= 0:
        return " " * center + "#" * filled + " " * (center - filled)
    else:
        return " " * (center - filled) + "#" * filled + " " * center


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

    print("Waiting for sensor data...\n")

    G = 9.81
    ALPHA = 0.1          # heavy smoothing for stable display
    ax_s = ay_s = 0.0
    az_s = G

    # Seed smoother
    t0 = time.monotonic()
    while time.monotonic() - t0 < 1.0:
        v = read_latest(ser)
        if v:
            ax_s, ay_s, az_s = v

    print("Hold sensor still in different orientations to identify axes.")
    print("Ctrl+C to exit.\n")

    last_print = 0.0

    while not _stop:
        v = read_latest(ser)
        if v:
            ax, ay, az = v
            ax_s = ALPHA * ax + (1 - ALPHA) * ax_s
            ay_s = ALPHA * ay + (1 - ALPHA) * ay_s
            az_s = ALPHA * az + (1 - ALPHA) * az_s

        now = time.monotonic()
        if now - last_print < 0.15:     # ~7 Hz display
            time.sleep(0.02)
            continue
        last_print = now

        norm = math.sqrt(ax_s**2 + ay_s**2 + az_s**2)
        if norm < 0.1:
            continue
        gx = ax_s / norm
        gy = ay_s / norm
        gz = az_s / norm

        roll  = math.degrees(math.atan2(ay_s, az_s))
        pitch = math.degrees(math.atan2(-ax_s, math.sqrt(ay_s**2 + az_s**2)))

        dom = dominant_axis(gx, gy, gz)

        print("\033[2J\033[H", end="")   # clear screen
        print("═" * 58)
        print("  LIS2DW12 Calibration / Axis Orientation")
        print("═" * 58)
        print()
        print(f"  Raw (m/s²)   ax={ax_s:+7.3f}   ay={ay_s:+7.3f}   az={az_s:+7.3f}")
        print(f"  Normalized   gx={gx:+6.3f}    gy={gy:+6.3f}    gz={gz:+6.3f}")
        print(f"  |g| = {norm:.3f} m/s²  (ideal = {G:.2f})")
        print()
        print(f"  Roll  (left/right tilt)   : {roll:+7.2f}°")
        print(f"  Pitch (fwd/back tilt)     : {pitch:+7.2f}°")
        print()
        print(f"  Gravity direction  →  {dom}")
        print()
        print("  ── Axis bars  (each bar = ±1g scale) ────────────────")
        print(f"  ax [{bar(gx)}]  {gx:+.2f}g")
        print(f"  ay [{bar(gy)}]  {gy:+.2f}g")
        print(f"  az [{bar(gz)}]  {gz:+.2f}g")
        print()
        print("  ── Tilt bars  (each bar = ±90° scale) ───────────────")
        print(f"  pitch [{bar(pitch, scale=90)}]  {pitch:+.1f}°")
        print(f"  roll  [{bar(roll,  scale=90)}]  {roll:+.1f}°")
        print()
        print("  Ctrl+C to exit")

    ser.close()
    print("\nDone.")


if __name__ == "__main__":
    main()
