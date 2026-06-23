"""
BTT LIS2DW12 v1.0.1 — CircuitPython firmware
Streams X,Y,Z acceleration (m/s²) over USB serial at 50 Hz.
Format: "X,Y,Z\n"

SPI pins: GP10=SCK  GP11=MOSI  GP8=MISO  GP9=CS  (BTT RP2040 layout)
"""

import board
import busio
import digitalio
import struct
import time

# ── LIS2DW12 registers ────────────────────────────────────────────────────────
_WHO_AM_I = 0x0F   # should return 0x44
_CTRL1    = 0x20   # ODR + mode
_CTRL2    = 0x21   # BDU, auto-increment, etc.
_OUT_X_L  = 0x28   # first of 6 consecutive output bytes

# CTRL1: ODR=100Hz (0101), Mode=High-Performance (01), LP_MODE=00
# ODR bits [7:4]=0101, MODE bits [3:2]=01 → 0x54
_CTRL1_VAL = 0x54

# Data is 14-bit LEFT-ALIGNED in 16-bit register (bits 15:2 valid, bits 1:0 = 0)
# Sensitivity = 0.244 mg per 14-bit LSB → divide by 4 to apply to raw 16-bit value
_SCALE = 0.000244 * 9.80665 / 4   # ≈ 0.000598 m/s² per raw 16-bit count

# ── SPI setup ─────────────────────────────────────────────────────────────────
spi = busio.SPI(clock=board.GP10, MOSI=board.GP11, MISO=board.GP8)
cs  = digitalio.DigitalInOut(board.GP9)
cs.switch_to_output(value=True)


def _xfer(tx: bytes) -> bytearray:
    rx = bytearray(len(tx))
    while not spi.try_lock():
        pass
    try:
        cs.value = False
        spi.write_readinto(tx, rx)
        cs.value = True
    finally:
        spi.unlock()
    return rx


def read_reg(reg: int, count: int = 1):
    # bit7=1 → read; IF_ADD_INC default=1 so multi-byte auto-increments
    buf = _xfer(bytes([reg | 0x80]) + bytes(count))
    return buf[1] if count == 1 else buf[1:]


def write_reg(reg: int, val: int):
    _xfer(bytes([reg & 0x7F, val]))


# ── Verify sensor ──────────────────────────────────────────────────────────────
who = read_reg(_WHO_AM_I)
if who != 0x44:
    print(f"ERROR: LIS2DW12 not found (WHO_AM_I={hex(who)}, expected 0x44)")
    print("Check SPI wiring / pin assignment")
    while True:
        time.sleep(1)

# ── Configure sensor ───────────────────────────────────────────────────────────
write_reg(_CTRL2, 0x0C)   # BDU=1, IF_ADD_INC=1 (bit2 must be set for multi-byte read)
write_reg(_CTRL1, _CTRL1_VAL)
time.sleep(0.02)

print("LIS2DW12 OK — streaming X,Y,Z at 50 Hz")

# ── Stream loop ────────────────────────────────────────────────────────────────
while True:
    raw = read_reg(_OUT_X_L, 6)
    x = struct.unpack_from("<h", raw, 0)[0] * _SCALE
    y = struct.unpack_from("<h", raw, 2)[0] * _SCALE
    z = struct.unpack_from("<h", raw, 4)[0] * _SCALE
    print(f"{x:.4f},{y:.4f},{z:.4f}")
    time.sleep(0.02)   # 50 Hz
