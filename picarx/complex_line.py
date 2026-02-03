#!/usr/bin/env python3
from time import sleep
from picarx_improved import Picarx
from robot_hat.adc import ADC   # adjust import if your package path differs

last_offset = 0.0

class GrayscaleSensor:
    """
    3.1 Sensing
    - __init__ sets up ADC structures as self.adc_left/mid/right
    - read() polls the three ADCs and returns [L, M, R]
    """
    def __init__(self, left="A0", mid="A1", right="A2"):
        self.adc_left = ADC(left)
        self.adc_mid = ADC(mid)
        self.adc_right = ADC(right)

    def read(self):
        return [self.adc_left.read(), self.adc_mid.read(), self.adc_right.read()]


class GrayscaleInterpreter:
    """
    3.2 Interpretation
    Produces offset in [-1, 1]
      + => line is to the LEFT of robot
      - => line is to the RIGHT of robot

    sensitivity: how strong an edge/contrast needs to be
    polarity:
      "dark"  => line is darker than floor (lower ADC values)
      "light" => line is lighter than floor (higher ADC values)

    Robustness trick:
    - uses a slowly-updated baseline to adapt to global lighting shifts
    - uses relative contrast (baseline - value or value - baseline)
    """
    def __init__(self, sensitivity=120, polarity="dark", auto_baseline=True, alpha=0.02):
        self.sensitivity = float(sensitivity)
        self.polarity = polarity
        self.auto_baseline = auto_baseline
        self.alpha = float(alpha)
        self.baseline = None  # [L,M,R]

    def _update_baseline(self, v):
        if self.baseline is None:
            self.baseline = [float(x) for x in v]
            return
        for i in range(3):
            self.baseline[i] = (1 - self.alpha) * self.baseline[i] + self.alpha * float(v[i])

    def process(self, v):
        """
        v: [L,M,R] ints
        return offset in [-1,1]
        """
        v = [float(x) for x in v]

        if self.auto_baseline:
            self._update_baseline(v)

        b = self.baseline if self.baseline is not None else v

        # Convert to "line-likeness" score (bigger means more likely line)
        if self.polarity.lower() == "dark":
            s = [b[i] - v[i] for i in range(3)]
        elif self.polarity.lower() == "light":
            s = [v[i] - b[i] for i in range(3)]
        else:
            raise ValueError("polarity must be 'dark' or 'light'")

        # Clamp negatives (not line-like)
        s = [max(0.0, x) for x in s]

        # Edge/contrast check: look for sharp change between adjacent sensors
        # (assignment asks for "sharp change between adjacent sensor values")
        d01 = abs(v[0] - v[1])
        d12 = abs(v[1] - v[2])
        contrast_ok = max(d01, d12) >= self.sensitivity

        # If no meaningful contrast, treat as centered/unknown
        if (max(s) < self.sensitivity) and (not contrast_ok):
            return 0.0

        # Weighted centroid across positions [-1,0,+1] for [L,M,R]
        pos = [-1.0, 0.0, 1.0]
        total = sum(s) + 1e-9
        centroid = sum(pos[i] * s[i] for i in range(3)) / total

        # centroid negative => line more on LEFT sensor side
        # assignment wants positive => line to LEFT, so flip sign
        offset = -centroid

        # map to [-1,1]
        if offset > 1.0: offset = 1.0
        if offset < -1.0: offset = -1.0
        return offset


class SteeringController:
    """
    3.3 Controller
    - steer_scale maps offset [-1,1] to angle degrees
    - control() calls px.set_dir_servo_angle(angle) and returns angle
    """
    def __init__(self, px: Picarx, steer_scale=25.0, max_angle=30.0):
        self.px = px
        self.steer_scale = float(steer_scale)
        self.max_angle = float(max_angle)

    def control(self, offset):
        angle = self.steer_scale * float(offset)
        if angle > self.max_angle: angle = self.max_angle
        if angle < -self.max_angle: angle = -self.max_angle
        self.px.set_dir_servo_angle(angle)
        return angle


def run_line_follow(px_power=15, dt=0.03, sensitivity=120, polarity="dark",
                    steer_scale=25.0, max_angle=30.0, forward=True):
    """
    3.4 Sensor-control integration
    Loop: read sensors -> interpret -> steer -> (optional) forward drive
    """
    px = Picarx()

    sensor = GrayscaleSensor("A0", "A1", "A2")
    interpreter = GrayscaleInterpreter(sensitivity=sensitivity, polarity=polarity, auto_baseline=True)
    controller = SteeringController(px, steer_scale=steer_scale, max_angle=max_angle)
    last_offset = 0.0
    angle = 0.0

    try:
        while True:
            vals = sensor.read()                 # [L,M,R]
            offset = interpreter.process(vals)
            # detect "line lost"
       	    if abs(offset) < 0.05:
                # recover: turn toward last known direction
                recover_angle = 30 if last_offset > 0 else -30
                px.set_dir_servo_angle(recover_angle)
                px.backward(px_power)
            else:
                angle = controller.control(offset)
                px.forward(px_power)
                last_offset = offset

            print(f"vals={vals}  offset={offset:+.2f}  angle={angle:+.1f}")
            sleep(dt)

    except KeyboardInterrupt:
        print("\nStopping...")



    finally:
        px.stop()
        px.set_dir_servo_angle(0)
        sleep(0.1)


if __name__ == "__main__":
    # Start here, then tune:
    run_line_follow(px_power=29, dt=0.03,
                    sensitivity=120, polarity="dark",
                    steer_scale=25, max_angle=30,
                    forward=True)

