#!/usr/bin/env python3
from time import sleep, time
from picarx_improved import Picarx

px = Picarx()

# --- Driving params ---
FWD_POWER = 30
REV_POWER = 30
OFFSET = 25

# --- Anti-flip params ---
STABLE_K = 2
HOLD_TIME = 0.09
LOOP_DT = 0.02

# --- Threshold params (NEW) ---
THRESH = 900   # <-- tune this: pick between floor and line ADC readings
# Assumption: black line => LOWER ADC value (common)

# --- Recovery params ---
RECOVER_STEER = 0
RECOVER_MAX_TIME = 2.5
RECOVER_DT = 0.01

last_turn = "left"

prev_raw_state = None
stable_count = 0
committed_state = "stop"
hold_until = 0.0


def adc_to_status(vals):
    """
    Convert raw ADC [L,M,R] -> pseudo-binary [L,M,R]
    Output matches your old convention:
      0 = line (dark)
      1 = background (light)
    """
    s = []
    for v in vals:
        s.append(0 if v < THRESH else 1)
    return s


def get_status(vals):
    """
    SAME LOGIC as your working version,
    but s is computed from threshold instead of px.get_line_status().
    """
    s = adc_to_status(vals)  # [L, M, R] 0=line 1=bg

    if s == [1, 1, 1]:
        return "stop"

    if s[1] == 0:
        px.set_dir_servo_angle(0)
        px.forward(FWD_POWER)

    if s[0] == 0 and s[2] == 0:
        return last_turn

    if s[0] == 0:
        return "left"
    if s[2] == 0:
        return "right"

    return "stop"


def apply_action(state):
    if state == "forward":
        px.set_dir_servo_angle(0)
        #px.forward(FWD_POWER)

    elif state == "left":
        px.set_dir_servo_angle(+OFFSET)
        px.forward(FWD_POWER)

    elif state == "right":
        px.set_dir_servo_angle(-OFFSET)
        px.forward(FWD_POWER)

    else:
        px.stop()


def line_seen_now():
    """True if any sensor detects line (threshold-based)."""
    vals = px.get_grayscale_data()
    s = adc_to_status(vals)
    return (0 in s), vals, s


def recover_reverse_until_line():
    px.stop()
    sleep(1.0)

    px.set_dir_servo_angle(RECOVER_STEER)
    px.backward(REV_POWER)

    t0 = time()
    while True:
        seen, vals, s = line_seen_now()
        print(f"RECOVER raw={vals} status={s} seen={seen}")

        if seen:
            break

        if (time() - t0) > RECOVER_MAX_TIME:
            break

        sleep(RECOVER_DT)

    px.stop()
    sleep(0.1)


def main():
    global last_turn, prev_raw_state, stable_count, committed_state, hold_until

    px.stop()
    sleep(1.0)

    try:
        while True:
            vals = px.get_grayscale_data()
            raw_state = get_status(vals)

            if raw_state == "stop":
                recover_reverse_until_line()
                prev_raw_state = None
                stable_count = 0
                hold_until = 0.0
                committed_state = "stop"
                sleep(LOOP_DT)
                continue

            if raw_state in ("left", "right"):
                last_turn = raw_state

            now = time()

            if now < hold_until:
                apply_action(committed_state)
                sleep(LOOP_DT)
                continue

            if raw_state == prev_raw_state:
                stable_count += 1
            else:
                stable_count = 1
                prev_raw_state = raw_state

            if stable_count >= STABLE_K:
                if raw_state != committed_state:
                    committed_state = raw_state
                    if committed_state in ("left", "right"):
                        hold_until = now + HOLD_TIME

                # debug print shows BOTH adc + thresholded bits
                print(f"rawADC={vals} bits={adc_to_status(vals)} "
                      f"raw={raw_state} committed={committed_state} THRESH={THRESH}")

                apply_action(committed_state)

            sleep(LOOP_DT)

    except KeyboardInterrupt:
        pass
    finally:
        px.stop()
        px.set_dir_servo_angle(0)
        sleep(0.1)


if __name__ == "__main__":
    main()
