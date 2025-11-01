# What this v1 does

# Provides clean sensor drivers with timeouts + smoothing.

# Implements a simple but robust finite state machine (FSM):
# IDLE → SCAN → CLEAR_PATH or AVOID → EDGE_ALERT → (back)

# Uses hysteresis and debounce so it doesn’t flicker between states.

# Gives audible cues: short, medium, and long beeps for different events.

#!/usr/bin/env python3
import time, math, sys
from enum import Enum, auto
from dataclasses import dataclass, field
from typing import Optional

# --- GPIO / HW libs ---
import board, busio
import serial
from gpiozero import DigitalInputDevice, Button, PWMOutputDevice
import adafruit_vcnl4040

# --------------- CONFIG ---------------

# PINS (BCM numbering for gpiozero)
PIN_TCRT5000_OUT = 17        # via level shifter (5V->3V3)
PIN_TOUCH_OUT    = 22
PIN_BUZZER_PWM   = 18        # PWM-capable

# UART for TF-Luna
SERIAL_DEV       = "/dev/serial0"
SERIAL_BAUD      = 115200
SERIAL_TIMEOUT_S = 0.1

# LiDAR thresholds (in cm)
DIST_CLEAR       = 120       # > this => path is clear
DIST_CAUTION     = 80        # between CAUTION and BLOCKED => careful
DIST_BLOCKED     = 50        # < this => obstacle very near

# VCNL4040 proximity thresholds (arbitrary units)
PROX_NEAR        = 2000      # hand/object close
PROX_VERY_NEAR   = 5000      # immediate caution

# Edge sensor (TCRT) logic: many boards HIGH = reflective (white), LOW = non-reflective (edge/black)
TCRT_EDGE_IS_LOW = True

# Debounce / smoothing
EMA_ALPHA        = 0.35      # exponential moving average for distance
STATE_HOLD_S     = 0.25      # minimum time to hold a state before switching
SENSOR_STALE_S   = 0.5       # if no new LiDAR frame, mark stale

# Buzzer tones
BEEP_FREQ_HZ     = 2000
BEEP_VOL         = 0.6

# Loop
CTRL_HZ          = 20.0

# ------------- HELPERS ----------------

def now():
    return time.monotonic()

class NavState(Enum):
    IDLE        = auto()
    SCAN        = auto()
    CLEAR_PATH  = auto()
    CAUTION     = auto()
    AVOID       = auto()
    EDGE_ALERT  = auto()
    SENSOR_FAULT= auto()

@dataclass
class LiDARReading:
    dist_cm: Optional[float] = None
    strength: Optional[int]  = None
    temp_c: Optional[float]  = None
    t: float = field(default_factory=now)

@dataclass
class ProxReading:
    prox: int
    lux: float
    t: float = field(default_factory=now)

# ------------- BUZZER ------------------

class Buzzer:
    def __init__(self, pin=PIN_BUZZER_PWM, freq=BEEP_FREQ_HZ):
        self.pwm = PWMOutputDevice(pin, frequency=freq, initial_value=0.0)

    def beep(self, duration=0.06, volume=BEEP_VOL):
        self.pwm.value = volume
        time.sleep(duration)
        self.pwm.value = 0.0

    def pattern_ok(self):
        self.beep(0.05); time.sleep(0.05); self.beep(0.05)

    def pattern_caution(self):
        self.beep(0.15); time.sleep(0.1); self.beep(0.15)

    def pattern_blocked(self):
        self.beep(0.35)

    def off(self):
        self.pwm.value = 0.0

# ------------- SENSORS -----------------

class TFLuna:
    """Simple frame reader for TF-Luna/TFmini UART frames 0x59 0x59 ..."""
    def __init__(self, dev=SERIAL_DEV, baud=SERIAL_BAUD, timeout=SERIAL_TIMEOUT_S):
        self.ser = serial.Serial(dev, baud, timeout=timeout)
        self.last = LiDARReading()
        self.ema = None

    def read_once(self) -> Optional[LiDARReading]:
        b = self.ser.read(9)
        if len(b) != 9:
            return None
        if b[0] == 0x59 and b[1] == 0x59:
            dist = b[2] + (b[3] << 8)         # cm
            strength = b[4] + (b[5] << 8)
            temp = (b[6] + (b[7] << 8)) / 8.0 - 256
            r = LiDARReading(dist, strength, temp, now())
            # smooth
            if self.ema is None:
                self.ema = dist
            else:
                self.ema = EMA_ALPHA*dist + (1-EMA_ALPHA)*self.ema
            r.dist_cm = self.ema
            self.last = r
            return r
        return None

    def is_stale(self) -> bool:
        return (now() - self.last.t) > SENSOR_STALE_S

class VCNL4040Sensor:
    def __init__(self):
        i2c = busio.I2C(board.SCL, board.SDA)
        self.dev = adafruit_vcnl4040.VCNL4040(i2c)

    def read(self) -> ProxReading:
        return ProxReading(self.dev.proximity, self.dev.lux)

# ------------- EDGE / TOUCH ------------

edge_sensor  = DigitalInputDevice(PIN_TCRT5000_OUT, pull_up=None)
touch_button = Button(PIN_TOUCH_OUT, pull_up=False)
buzzer       = Buzzer()

# ------------- FSM CONTROLLER ----------

class Navigator:
    def __init__(self):
        self.lidar = TFLuna()
        self.vcnl  = VCNL4040Sensor()
        self.state = NavState.IDLE
        self._last_change = now()

    def _set_state(self, s: NavState):
        if s != self.state and (now()-self._last_change) > STATE_HOLD_S:
            self.state = s
            self._last_change = now()
            print(f"[STATE] -> {self.state.name}")
            # feedback
            if s == NavState.CLEAR_PATH: buzzer.pattern_ok()
            elif s == NavState.CAUTION: buzzer.pattern_caution()
            elif s in (NavState.AVOID, NavState.EDGE_ALERT): buzzer.pattern_blocked()
            elif s == NavState.SENSOR_FAULT: buzzer.pattern_blocked()

    def step(self):
        # --- read sensors ---
        lr = self.lidar.read_once()  # may be None if no new frame
        pr = self.vcnl.read()

        # Edge detection (LOW => edge if TCRT_EDGE_IS_LOW)
        edge = (edge_sensor.value == 0) if TCRT_EDGE_IS_LOW else (edge_sensor.value == 1)

        # Check sensor health
        if self.lidar.is_stale():
            self._set_state(NavState.SENSOR_FAULT)
            return

        # If touching -> toggle IDLE/SCAN (handled outside; here we just respect state)
        if self.state == NavState.IDLE:
            return

        # --- decision logic with hysteresis ---
        dist = self.lidar.last.dist_cm if self.lidar.last.dist_cm is not None else math.inf
        prox = pr.prox

        # Highest priority: edge alert
        if edge:
            self._set_state(NavState.EDGE_ALERT)
            return

        # Next: very close/blocked
        if dist < DIST_BLOCKED or prox > PROX_VERY_NEAR:
            self._set_state(NavState.AVOID)
            return

        # Caution zone
        if dist < DIST_CAUTION or prox > PROX_NEAR:
            self._set_state(NavState.CAUTION)
            return

        # Clear
        if dist > DIST_CLEAR and prox < PROX_NEAR:
            self._set_state(NavState.CLEAR_PATH)
            return

        # Default: scanning / transitioning
        self._set_state(NavState.SCAN)

    def run(self):
        print("Navigator ready. Touch pad to start/stop.")
        running = False
        last_touch_state = False

        while True:
            # Handle touch to toggle IDLE/SCAN
            if touch_button.is_pressed and not last_touch_state:
                running = not running
                self._set_state(NavState.SCAN if running else NavState.IDLE)
            last_touch_state = touch_button.is_pressed

            if running:
                self.step()

            time.sleep(1.0/CTRL_HZ)

# ------------- MAIN --------------------

if __name__ == "__main__":
    try:
        Navigator().run()
    except KeyboardInterrupt:
        print("\nExiting…")
        buzzer.off()
        sys.exit(0)
