from gpiozero import DigitalInputDevice
from signal import pause

sensor = DigitalInputDevice(17, pull_up=None)  # external pull handled by module

def report():
    # Many modules drive LOW on black (no reflection) and HIGH on white (reflection)
    print("REFLECTIVE" if sensor.value == 1 else "NON-REFLECTIVE / FAR")

sensor.when_activated   = report
sensor.when_deactivated = report

print("TCRT5000: move white/black surface under the sensor, or hand near/far.")
report()
pause()
