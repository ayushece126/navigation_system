import time
from gpiozero import Button, PWMOutputDevice

touch = Button(22, pull_up=False)         # TTP223 drives HIGH on touch
buzzer = PWMOutputDevice(18, frequency=2000, initial_value=0.0)

print("Touch the pad to beep. Hold to sustain. Ctrl+C to exit.")
try:
    while True:
        if touch.is_pressed:
            buzzer.value = 0.6   # duty cycle
        else:
            buzzer.value = 0.0
        time.sleep(0.01)
except KeyboardInterrupt:
    buzzer.off()
