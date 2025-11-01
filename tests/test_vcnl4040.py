import time
import board
import busio
import adafruit_vcnl4040

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_vcnl4040.VCNL4040(i2c)

print("VCNL4040 detected. Move your hand closer/farther…")
print("Press Ctrl+C to stop.\n")

while True:
    prox = sensor.proximity   # arbitrary units (higher = closer)
    als  = sensor.lux         # ambient light in lux
    print(f"Proximity: {prox:5d}  |  Ambient: {als:.1f} lux")
    time.sleep(0.2)
