#!/usr/bin/env python3
"""
Level 4: Simultaneity (Concurrency) with Grayscale Line Following

Architecture:
  Sensor thread       -> adc_bus    (writes [L,M,R])
  Interpreter thread  -> offset_bus (writes offset in [-1,1] or None if lost)
  Controller thread   -> commands steering + drive (+ recovery)

Notes:
- Uses raw ADC values (no binary conversion).
- Interpreter supports sensitivity + polarity, edge detection, baseline.
- Bus is thread-safe using readerwriterlock.
"""

from time import sleep, time
from dataclasses import dataclass
from readerwriterlock import rwlock
from picarx_improved import Picarx
import threading


# ===================== USER PARAMS =====================
FWD_POWER = 20
REV_POWER = 30

SENSOR_DT = 0.02       # sensor polling period
INTERP_DT = 0.02       # interpreter period
CONTROL_DT = 0.02      # controller period

MAX_ANGLE = 30.0       # steering clamp (deg)

# "Line strength" threshold (used for strength mapping + recovery line_seen)
THRESH = 850

# Recovery behavior
RECOVER_MAX_TIME = 2.5
RECOVER_DT = 0.01
RECOVER_STEER = 0

# PD “auto-gain” knobs (no manual Kp/Kd needed)
RESPONSE = 1.0         # 0.6 (gentle) ... 1.6 (aggressive)
DAMPING = 0.8          # 0.6..1.2  (higher = more damping, less oscillation)

# Interpreter knobs (required)
SENSITIVITY = 120      # edge threshold
POLARITY = "dark"      # "dark" line on light floor, or "light"

# Extra stability knobs (optional but helpful)
DEADBAND = 0.06        # ignore tiny offsets near 0
ALPHA = 0.30           # smoothing on offset (0.2..0.5)
# =======================================================


# ------------------ Thread-safe Bus (Level 4) ------------------
class Bus:
    """
    Thread-safe 'latest message' bus.
    - write(msg): replaces the current message
    - read(): returns the latest message (or None if nothing yet)
    """
    def __init__(self, initial_message=None):
        self.message = initial_message
        self._rw = rwlock.RWLockWriteD()   # writer-priority lock

    def write(self, msg):
        with self._rw.gen_wlock():
            self.message = msg

    def read(self):
        with self._rw.gen_rlock():
            return self.message


# Message structures (optional but clean)
@dataclass
class ADCMessage:
    t: float
    vals: list  # [L,M,R] raw ADC


@dataclass
class OffsetMessage:
    t: float
    offset: float | None  # [-1,1] or None if lost


# ------------------ 3.1 Sensing ------------------
class GrayscaleSensor:
    """
    Requirements:
    1) __init__ sets up ADC structures as attributes using self. syntax
    2) read() polls all three and returns [L, M, R]

    Implementation note:
    - We read via px.get_grayscale_data() (works on robot + sim via picarx_improved).
    - We still keep the required 'self.adc_left/mid/right' attributes as callable accessors.
    """
    def __init__(self):
        self.px = None
        self.adc_left = None
        self.adc_mid = None
        self.adc_right = None

    def attach_px(self, px: Picarx):
        self.px = px

        # callable "ADC structures" (still attributes on self)
        self.adc_left = lambda: self.px.get_grayscale_data()[0]
        self.adc_mid = lambda: self.px.get_grayscale_data()[1]
        self.adc_right = lambda: self.px.get_grayscale_data()[2]

    def read(self):
        # Poll all three ADC readings
        return [self.adc_left(), self.adc_mid(), self.adc_right()]


# ------------------ 3.2 Interpretation ------------------
class GrayscaleInterpreter:
    """
    Requirements:
    1) __init__(sensitivity=..., polarity=...) with defaults
    2) process([L,M,R]) detects sharp adjacent change (edge) and uses edge sign/loc
       to determine left/right and magnitude. Robust to lighting; polarity option.
    3) output offset in [-1,1], positive means line is LEFT of robot.

    Approach:
    - Use optional baseline (EMA) to adapt to global lighting drift.
    - Use adjacent deltas to check if an edge exists (contrast).
    - Compute line-strength per sensor using THRESH and polarity.
    - Compute centroid across positions [-1,0,+1] -> offset in [-1,1]
      with sign convention: +offset means line is LEFT.
    """
    def __init__(self, sensitivity=120, polarity="dark", auto_baseline=True, alpha=0.02, thresh=850):
        self.sensitivity = float(sensitivity)
        self.polarity = polarity.lower()
        if self.polarity not in ("dark", "light"):
            raise ValueError("polarity must be 'dark' or 'light'")
        self.auto_baseline = bool(auto_baseline)
        self.alpha = float(alpha)
        self.baseline = None  # [L,M,R]
        self.thresh = float(thresh)

    def _update_baseline(self, v):
        if self.baseline is None:
            self.baseline = [float(x) for x in v]
            return
        for i in range(3):
            self.baseline[i] = (1 - self.alpha) * self.baseline[i] + self.alpha * float(v[i])

    def process(self, vals):
        """
        vals: [L,M,R] raw ADC ints
        returns offset in [-1,1] (positive = line LEFT), or None if lost
        """
        L, M, R = [float(x) for x in vals]

        # baseline (lighting robustness)
        if self.auto_baseline:
            self._update_baseline([L, M, R])

        # edge detection between adjacent sensors
        dLM = abs(L - M)
        dMR = abs(M - R)
        edge_ok = max(dLM, dMR) >= self.sensitivity

        # polarity-aware "line strength"
        # dark line: lower ADC => stronger = THRESH - value (clamped)
        # light line: higher ADC => stronger = value - THRESH (clamped)
        if self.polarity == "dark":
            sL = max(0.0, self.thresh - L)
            sM = max(0.0, self.thresh - M)
            sR = max(0.0, self.thresh - R)
        else:
            sL = max(0.0, L - self.thresh)
            sM = max(0.0, M - self.thresh)
            sR = max(0.0, R - self.thresh)

        total = sL + sM + sR

        # Lost: no strength AND no edge
        if total < 1e-6 and (not edge_ok):
            return None

        # If no edge and strengths tiny, treat as centered (prevents jitter)
        if (not edge_ok) and max(sL, sM, sR) < self.sensitivity:
            return 0.0

        # centroid across [-1,0,+1]
        # centroid negative -> more on left sensor; centroid positive -> more on right sensor
        centroid = (-1.0 * sL + 0.0 * sM + 1.0 * sR) / (total + 1e-9)

        # We want +offset when line is LEFT.
        # If centroid is negative (left), offset should be positive -> flip sign:
        offset = float(centroid)

        # clamp
        if offset > 1.0: offset = 1.0
        if offset < -1.0: offset = -1.0
        return offset


# ------------------ 3.3 Controller ------------------
class PDController:
    """
    Controller class requirements:
    1) __init__ includes scaling factor between offset and steering angle
       (here via auto Kp derived from MAX_ANGLE and RESPONSE)
    2) control() commands steering servo and returns commanded angle
    """
    def __init__(self, px: Picarx, max_angle=30.0, dt=0.02, response=1.0, damping=0.8):
        self.px = px
        self.max_angle = float(max_angle)
        self.dt = float(dt)

        self.Kp = self.max_angle * float(response)
        self.Kd = self.Kp * self.dt * float(damping)

        self.prev_error = 0.0
        self.has_prev = False

    def reset(self):
        self.prev_error = 0.0
        self.has_prev = False

    def control(self, offset):
        """
        offset: [-1,1] or None
        returns angle (deg) or None
        """
        if offset is None:
            return None

        e = float(offset)
        if not self.has_prev:
            de = 0.0
            self.prev_error = e
            self.has_prev = True
        else:
            de = (e - self.prev_error) / self.dt
            self.prev_error = e

        angle = self.Kp * e + self.Kd * de

        # clamp
        if angle > self.max_angle: angle = self.max_angle
        if angle < -self.max_angle: angle = -self.max_angle

        self.px.set_dir_servo_angle(angle)
        return float(angle)


# ------------------ Recovery helpers ------------------
def line_seen_thresh(vals):
    """Used only for recovery: consider line seen if any sensor meets polarity condition vs THRESH."""
    L, M, R = vals
    if POLARITY.lower() == "dark":
        return (L < THRESH) or (M < THRESH) or (R < THRESH)
    else:
        return (L > THRESH) or (M > THRESH) or (R > THRESH)


def recover_reverse_until_line(px: Picarx, stop_event: threading.Event):
    px.stop()
    sleep(0.15)

    px.set_dir_servo_angle(RECOVER_STEER)
    px.backward(REV_POWER)

    t0 = time()
    while not stop_event.is_set():
        vals = px.get_grayscale_data()
        seen = line_seen_thresh(vals)
        print(f"RECOVER ADC={vals} seen={seen}")

        if seen:
            break
        if (time() - t0) > RECOVER_MAX_TIME:
            break

        sleep(RECOVER_DT)

    px.stop()
    sleep(0.1)


# ------------------ Level 4 Threads ------------------
def sensor_thread_fn(sensor: GrayscaleSensor, adc_bus: Bus, stop_event: threading.Event):
    """Producer: reads sensors and writes ADCMessage -> adc_bus."""
    while not stop_event.is_set():
        vals = sensor.read()
        adc_bus.write(ADCMessage(t=time(), vals=vals))
        sleep(SENSOR_DT)


def interpreter_thread_fn(interp: GrayscaleInterpreter, adc_bus: Bus, offset_bus: Bus, stop_event: threading.Event):
    """Consumer-producer: reads adc_bus, interprets, writes OffsetMessage -> offset_bus."""
    last_t = None
    while not stop_event.is_set():
        msg = adc_bus.read()
        if msg is None or (last_t is not None and msg.t == last_t):
            sleep(INTERP_DT)
            continue

        last_t = msg.t
        offset = interp.process(msg.vals)
        offset_bus.write(OffsetMessage(t=time(), offset=offset))
        sleep(INTERP_DT)


def controller_thread_fn(px: Picarx, controller: PDController, offset_bus: Bus, stop_event: threading.Event):
    """Consumer: reads offset_bus, commands steering + drive, handles recovery."""
    prev_offset = 0.0

    while not stop_event.is_set():
        msg = offset_bus.read()
        if msg is None:
            px.stop()
            px.set_dir_servo_angle(0)
            sleep(CONTROL_DT)
            continue

        offset = msg.offset

        # If lost, recover
        if offset is None:
            px.stop()
            recover_reverse_until_line(px, stop_event)
            controller.reset()
            prev_offset = 0.0
            sleep(CONTROL_DT)
            continue

        # deadband + smoothing
        if abs(offset) < DEADBAND:
            offset = 0.0
        offset = ALPHA * float(offset) + (1 - ALPHA) * float(prev_offset)
        prev_offset = offset

        angle = controller.control(offset)
        if angle is None:
            px.stop()
            sleep(CONTROL_DT)
            continue

        px.forward(FWD_POWER)
        # Optional debug:
        # print(f"offset={offset:+.2f} angle={angle:+.1f}")

        sleep(CONTROL_DT)


# ------------------ 3.4 Integration launcher (Level 4) ------------------
def run_level4_grayscale():
    px = Picarx()
    px.stop()
    px.set_dir_servo_angle(0)
    sleep(0.5)

    sensor = GrayscaleSensor()
    sensor.attach_px(px)

    interp = GrayscaleInterpreter(
        sensitivity=SENSITIVITY,
        polarity=POLARITY,
        auto_baseline=True,
        alpha=0.02,
        thresh=THRESH
    )

    controller = PDController(
        px=px,
        max_angle=MAX_ANGLE,
        dt=CONTROL_DT,
        response=RESPONSE,
        damping=DAMPING
    )

    print(f"Level4 running. PD gains: Kp={controller.Kp:.2f}, Kd={controller.Kd:.3f}")
    print("Ctrl+C to stop.")

    # Busses
    adc_bus = Bus(initial_message=None)
    offset_bus = Bus(initial_message=None)

    stop_event = threading.Event()

    # Threads
    t_sensor = threading.Thread(target=sensor_thread_fn, args=(sensor, adc_bus, stop_event), daemon=True)
    t_interp = threading.Thread(target=interpreter_thread_fn, args=(interp, adc_bus, offset_bus, stop_event), daemon=True)
    t_ctrl = threading.Thread(target=controller_thread_fn, args=(px, controller, offset_bus, stop_event), daemon=True)

    try:
        t_sensor.start()
        t_interp.start()
        t_ctrl.start()

        # main waits
        while t_ctrl.is_alive():
            sleep(0.1)

    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        px.stop()
        px.set_dir_servo_angle(0)
        sleep(0.2)
        print("Stopped.")


if __name__ == "__main__":
    run_level4_grayscale()
