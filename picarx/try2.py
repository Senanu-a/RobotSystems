#!/usr/bin/env python3
from time import sleep, time
from picarx_improved import Picarx

px = Picarx()

# ===================== USER PARAMS =====================
FWD_POWER = 30
REV_POWER = 30

DT = 0.02                 # control loop period
MAX_ANGLE = 30.0          # steering clamp (deg)

THRESH = 850              # line if value < THRESH (dark line). used only for "line seen?"
RECOVER_MAX_TIME = 2.5
RECOVER_DT = 0.01
RECOVER_STEER = 0

# PD “auto-gain” knobs (no manual Kp/Kd needed)
# Higher RESPONSE -> more aggressive steering
RESPONSE = 1.0            # 0.6 (gentle) ... 1.6 (aggressive)
DAMPING = 0.8             # 0.6..1.2  (higher = more damping, less oscillation)
# =======================================================


def line_seen(vals):
    """True if any sensor likely sees the line (dark) using threshold."""
    L, M, R = vals
    return (L < THRESH) or (M < THRESH) or (R < THRESH)


def compute_error(vals):
    """
    Compute a continuous lateral error in [-1, +1] from RAW ADC values.

    Assumption: dark line -> lower ADC.
    We convert each sensor reading into a "line strength":
        strength = max(0, THRESH - value)
    Then we compute a centroid across positions [-1, 0, +1] for [L, M, R].

    We return error where:
      + error => line is to the LEFT  (need steer left: +angle)
      - error => line is to the RIGHT (need steer right: -angle)
    """
    L, M, R = vals

    sL = max(0.0, THRESH - float(L))
    sM = max(0.0, THRESH - float(M))
    sR = max(0.0, THRESH - float(R))

    total = sL + sM + sR
    if total < 1e-6:
        # no clear line signal
        return None

    # centroid in [-1, +1] where -1 means more on left sensor, +1 means more on right sensor
    centroid = (-1.0 * sL + 0.0 * sM + 1.0 * sR) / total

    # We want +error when line is LEFT, so flip sign:
    error = centroid

    # clamp
    if error > 1.0: error = 1.0
    if error < -1.0: error = -1.0
    return error


class PDController:
    """
    PD steering controller with auto-gain calculation.

    We treat error in [-1,1]. We want steering to reach MAX_ANGLE at |error|=1.
      angle = Kp*e + Kd*de/dt

    Auto-gain idea:
      Kp = (MAX_ANGLE) * RESPONSE
      Kd = (Kp) * (DT) * DAMPING
    These are simple, stable heuristics for this task.
    """
    def __init__(self, max_angle=30.0, dt=0.02, response=1.0, damping=0.8):
        self.max_angle = float(max_angle)
        self.dt = float(dt)

        self.Kp = self.max_angle * float(response)
        self.Kd = self.Kp * self.dt * float(damping)

        self.prev_error = 0.0
        self.has_prev = False

    def reset(self):
        self.prev_error = 0.0
        self.has_prev = False

    def step(self, error):
        if error is None:
            return None

        if not self.has_prev:
            de = 0.0
            self.prev_error = float(error)
            self.has_prev = True
        else:
            de = (float(error) - self.prev_error) / self.dt
            self.prev_error = float(error)

        angle = self.Kp * float(error) + self.Kd * de

        # clamp to servo limits
        if angle > self.max_angle: angle = self.max_angle
        if angle < -self.max_angle: angle = -self.max_angle
        return angle


def recover_reverse_until_line():
    px.stop()
    sleep(0.2)

    px.set_dir_servo_angle(RECOVER_STEER)
    px.backward(REV_POWER)

    t0 = time()
    while True:
        vals = px.get_grayscale_data()
        seen = line_seen(vals)
        print(f"RECOVER ADC={vals} seen={seen}")

        if seen:
            break
        if (time() - t0) > RECOVER_MAX_TIME:
            break

        sleep(RECOVER_DT)

    px.stop()
    sleep(0.1)


def main():
    pd = PDController(max_angle=MAX_ANGLE, dt=DT, response=RESPONSE, damping=DAMPING)
    print(f"PD gains: Kp={pd.Kp:.2f}, Kd={pd.Kd:.3f} (DT={DT})")

    px.stop()
    sleep(0.5)

    try:
        while True:
            vals = px.get_grayscale_data()
            err = compute_error(vals)

            # If no line signal, recover
            if err is None:
                recover_reverse_until_line()
                pd.reset()
                sleep(DT)
                continue

            angle = pd.step(err)
            if angle is None:
                recover_reverse_until_line()
                pd.reset()
                sleep(DT)
                continue

            px.set_dir_servo_angle(angle)
            px.forward(FWD_POWER)

            print(f"ADC={vals}  err={err:+.2f}  angle={angle:+.1f}")
            sleep(DT)

    except KeyboardInterrupt:
        pass
    finally:
        px.stop()
        px.set_dir_servo_angle(0)
        sleep(0.1)


if __name__ == "__main__":
    main()
