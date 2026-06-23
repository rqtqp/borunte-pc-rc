"""
code.py — runs on BTT LIS2DW12 v1.0.1 (CircuitPython on RP2040)
Streams raw X,Y,Z acceleration over USB serial at 50 Hz.
"""

import board
import busio
import time
import adafruit_lis2dw12

# BTT LIS2DW12 uses SPI — try default SPI pins
import busio
import digitalio

spi = busio.SPI(clock=board.GP10, MOSI=board.GP11, MISO=board.GP8)
cs  = digitalio.DigitalInOut(board.GP9)

sensor = adafruit_lis2dw12.LIS2DW12_SPI(spi, cs)
sensor.data_rate    = adafruit_lis2dw12.Rate.RATE_100_HZ
sensor.range        = adafruit_lis2dw12.Range.RANGE_2G
sensor.low_noise    = True

while True:
    x, y, z = sensor.acceleration   # m/s²
    print(f"{x:.4f},{y:.4f},{z:.4f}")
    time.sleep(0.02)   # 50 Hz
