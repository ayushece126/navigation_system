import serial
import time

# Use the primary UART: /dev/serial0
ser = serial.Serial('/dev/serial0', 115200, timeout=1)
print("TF-Luna: reading frames… Press Ctrl+C to stop.")

def read_frame():
    # TF-Luna/TFmini-type frame: 0x59 0x59 distL distH strengthL strengthH tempL tempH checksum
    # Not all variants are identical, but this works for standard UART frames.
    b = ser.read(9)
    if len(b) != 9:
        return None
    if b[0] == 0x59 and b[1] == 0x59:
        dist = b[2] + (b[3] << 8)       # in cm
        strength = b[4] + (b[5] << 8)
        temp = (b[6] + (b[7] << 8)) / 8.0 - 256
        return dist, strength, temp
    return None

try:
    while True:
        f = read_frame()
        if f:
            dist_cm, strength, temp = f
            print(f"Distance: {dist_cm:4d} cm  |  Strength: {strength:5d}  |  Temp: {temp:.1f} C")
        else:
            # If you see many Nones, check wiring and baud.
            pass
        time.sleep(0.05)
except KeyboardInterrupt:
    ser.close()
