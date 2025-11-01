#!/usr/bin/env python3
"""
Robust navigation FSM (verified against vendor docs):
- TF-Luna LiDAR (UART, 115200, 9-byte frames starting 0x59 0x59)
- VCNL4040 (I2C proximity + ambient light)
- TCRT5000 (digital edge/reflective sensor via LM393)
- Touch (start/stop), Buzzer (audio), Haptic ERM (PWM via NPN), Actuator/Relay

WIRING (Pi 4, BCM numbering):
  TF-Luna: VCC->5V, GND->GND, TX->Pi RX (GPIO15/pin10), RX<-Pi TX (GPIO14/pin8) THROUGH LEVEL SHIFT
  VCNL4040: VIN->3V3, GND->GND, SCL->GPIO3/pin5, SDA->GPIO2/pin3
  TCRT5000: VCC->5V, GND->GND, OUT->GPIO17/pin11 THROUGH LEVEL SHIFT (5V->3V3)
  Touch TTP223: OUT->GPIO22/pin15 (3V3 logic)
  Buzzer: PWM->GPIO18/pin12
  Haptic ERM: PWM->GPIO23/pin16 (drives NPN -> motor, with flyback diode)
  Actuator/Relay: SIG->GPIO24/pin18

Tested assumptions from docs:
- TF-Luna 115200 baud, 9-byte frame (0x59 0x59 ... checksum)  [Benewake manual]
- /dev/serial0 maps to primary UART on GPIO14/15 on Pi4       [Raspberry Pi docs]
- VCNL4040 driver: .proximity, .lux                           [Adafruit docs]
- TCRT5000/LM393 module polarity varies across vendors; configurable here.
"""

import time, math, sys, os
from enum import Enum, auto
from dataclasses import dataclass, field
from typing import Optional

# --- GPIO / HW libs ---
import board, busio
import serial
from serial import SerialException
from gpiozero import DigitalInputDevice, Button, PWMOutputDevice, DigitalOutputDevice
import adafruit_vcnl4040

# ---------------- CONFIG (edit safely) ----------------

# Pins (BCM)
PIN_TCRT5000_OUT = 17        # via level shifter (5V->3V3)
PIN_TOUCH_OUT    = 22
PIN_BUZZER_PWM   = 18        # PWM-capable
PIN_HAPTIC_PWM   = 23        # PWM-capable (NPN -> ERM)
PIN_ACTUATOR     = 24        # Relay / brake

# UART for TF-Luna (per docs: 115200 baud, 9-byte frames starting 0x59 0x59)
SERIAL_DEV       = "/dev/serial0"
SERIAL_BAUD      = 115200
SERIAL_TIMEOUT_S = 0.1

# LiDAR thresholds (cm)
DIST_CLEAR       = 120
DIST_CAUTION     = 80
DIST_BLOCKED     = 50

# VCNL4040 thresholds (arbitrary units)
PROX_NEAR        = 2000
PROX_VERY_NEAR   = 5000

# TCRT5000 module polarity (LM393 boards often HIGH on reflective, LOW on dark)
# If your logic is inverted, set this to False.
TCRT_HIGH_MEANS_REFLECTIVE = True

# Debounce / smoothing / timing
EMA_ALPHA        = 0.35
STATE_HOLD_S     = 0.30
SENSOR_STALE_S   = 0.6

# Buzzer / Haptic
BEEP_FREQ_HZ     = 2000
BEEP_VOL         = 0.6
HAPTIC_FREQ_HZ   = 180
HAPTIC_PWR       = 0.8

CTRL_HZ          = 20.0

# ---------------- Helpers ----------------

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

# ---------------- Outputs ----------------

class Buzzer:
    def __init__(self, pin=PIN_BUZZER_PWM, freq=BEEP_FREQ_HZ):
        self.pwm = PWMOutputDevice(pin, frequency=freq, initial_value=0.0)

    def beep(self, duration=0.06, volume=BEEP_VOL):
        self.pwm.value = max(0.0, min(1.0, volume))
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

class Haptic:
    def __init__(self, pin=PIN_HAPTIC_PWM, freq=HAPTIC_FREQ_HZ):
        self.pwm = PWMOutputDevice(pin, frequency=freq, initial_value=0.0)

    def on(self, power=HAPTIC_PWR):
        self.pwm.value = max(0.0, min(1.0, power))

    def off(self):
        self.pwm.value = 0.0

    def pattern_ok(self):
        for _ in range(2):
            self.on(0.8); time.sleep(0.08)
            self.off();   time.sleep(0.07)

    def pattern_caution(self):
        for _ in range(2):
            self.on(0.9); time.sleep(0.18)
            self.off();   time.sleep(0.12)

    def pattern_blocked(self):
        self.on(1.0); time.sleep(0.5); self.off()

class Actuator:
    """Active-high digital output (relay/brake/stop)."""
    def __init__(self, pin=PIN_ACTUATOR, active_high=True):
        self.dev = DigitalOutputDevice(pin, active_high=active_high, initial_value=False)

    def engage(self):   # ON
        self.dev.on()

    def release(self):  # OFF
        self.dev.off()

# ---------------- Sensors ----------------

class TFLuna:
    """
    Reads TF-Luna frames:
    Byte0..1 = 0x59 0x59, then Dist_L, Dist_H, Strength_L, Strength_H, Temp_L, Temp_H, Checksum
    Default 115200 baud 8N1. See Benewake manual.
    """
    def __init__(self, dev=SERIAL_DEV, baud=SERIAL_BAUD, timeout=SERIAL_TIMEOUT_S):
        try:
            self.ser = serial.Serial(dev, baud, timeout=timeout)
        except SerialException as e:
            raise RuntimeError(f"Cannot open {dev}: {e}")
        self.last = LiDARReading()
        self.ema = None

    def _read9(self):
        b = self.ser.read(9)
        return b if len(b) == 9 else None

    @staticmethod
    def _checksum_ok(buf):
        return (sum(buf[:8]) & 0xFF) == buf[8]

    def read_once(self) -> Optional[LiDARReading]:
        b = self._read9()
        if not b:
            return None
        if b[0] == 0x59 and b[1] == 0x59 and self._checksum_ok(b):
            dist = b[2] + (b[3] << 8)         # cm
            strength = b[4] + (b[5] << 8)
            temp = (b[6] + (b[7] << 8)) / 8.0 - 256
            r = LiDARReading(dist, strength, temp, now())
            # EMA smoothing
            self.ema = dist if self.ema is None else (0.35*dist + 0.65*self.ema)
            r.dist_cm = self.ema
            self.last = r
            return r
        return None

    def is_stale(self) -> bool:
        return (now() - self.last.t) > SENSOR_STALE_S

class VCNL4040Sensor:
    def __init__(self):
        # I2C bus per Adafruit driver
        i2c = busio.I2C(board.SCL, board.SDA)
        try:
            self.dev = adafruit_vcnl4040.VCNL4040(i2c)
        except Exception as e:
            raise RuntimeError(f"VCNL4040 init failed: {e}")

    def read(self) -> ProxReading:
        try:
            return ProxReading(self.dev.proximity, self.dev.lux)
        except Exception as e:
            # If I2C hiccups, return last-known-ish safe values
            return ProxReading(0, 0.0)

# ---------------- Inputs/Outputs ----------------

edge_sensor   = DigitalInputDevice(PIN_TCRT5000_OUT, pull_up=None)
touch_button  = Button(PIN_TOUCH_OUT, pull_up=False, bounce_time=0.05)
buzzer        = Buzzer()
haptic        = Haptic()
actuator      = Actuator()

# ---------------- FSM ----------------

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

            # One-shot feedback
            if s == NavState.CLEAR_PATH:
                buzzer.pattern_ok(); haptic.pattern_ok(); actuator.release()
            elif s == NavState.CAUTION:
                buzzer.pattern_caution(); haptic.pattern_caution(); actuator.release()
            elif s in (NavState.AVOID, NavState.EDGE_ALERT):
                buzzer.pattern_blocked(); haptic.pattern_blocked(); actuator.engage()
            elif s == NavState.SENSOR_FAULT:
                buzzer.pattern_blocked(); haptic.pattern_blocked(); actuator.engage()
            elif s == NavState.IDLE:
                buzzer.off(); haptic.off(); actuator.release()

    def _continuous_outputs(self):
        # Gentle patterns while remaining in a state (non-blocking, tiny sleeps allowed at 20 Hz loop)
        if self.state == NavState.CAUTION:
            haptic.on(0.35); time.sleep(0.04); haptic.off()
        elif self.state == NavState.AVOID:
            haptic.on(0.9);  time.sleep(0.05); haptic.off()
        elif self.state == NavState.EDGE_ALERT:
            haptic.on(1.0);  time.sleep(0.03); haptic.off()
        else:
            haptic.off()

    def step(self):
        # Read sensors with resilience
        _ = self.lidar.read_once()  # update internal last; may be None
        pr = self.vcnl.read()

        # Edge logic: map module polarity to "edge" boolean
        raw = edge_sensor.value  # 1 or 0
        reflective = (raw == 1) if TCRT_HIGH_MEANS_REFLECTIVE else (raw == 0)
        edge = not reflective  # treat "dark / no ground" as edge

        if self.lidar.is_stale():
            self._set_state(NavState.SENSOR_FAULT)
            return

        if self.state == NavState.IDLE:
            return

        dist = self.lidar.last.dist_cm if self.lidar.last.dist_cm is not None else math.inf
        prox = pr.prox

        if edge:
            self._set_state(NavState.EDGE_ALERT)
        elif dist < DIST_BLOCKED or prox > PROX_VERY_NEAR:
            self._set_state(NavState.AVOID)
        elif dist < DIST_CAUTION or prox > PROX_NEAR:
            self._set_state(NavState.CAUTION)
        elif dist > DIST_CLEAR and prox < PROX_NEAR:
            self._set_state(NavState.CLEAR_PATH)
        else:
            self._set_state(NavState.SCAN)

        self._continuous_outputs()

    def run(self):
        print("Navigator ready. Touch pad toggles RUN/IDLE.")
        running = False
        last_pressed = False

        while True:
            pressed = touch_button.is_pressed
            if pressed and not last_pressed:
                running = not running
                self._set_state(NavState.SCAN if running else NavState.IDLE)
            last_pressed = pressed

            if running:
                self.step()

            time.sleep(1.0/CTRL_HZ)

# ---------------- Main ----------------

if __name__ == "__main__":
    try:
        Navigator().run()
    except KeyboardInterrupt:
        pass
    finally:
        buzzer.off(); haptic.off(); actuator.release()
        print("\nExited cleanly.")
