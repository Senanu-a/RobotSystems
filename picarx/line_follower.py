#!/usr/bin/env python3
from time import sleep
from picarx_improved import Picarx

px = Picarx()

px_power = 20
offset = 20          # steering angle for left/right
last_state = "stop"


def get_status(vals):
    """
    Convert grayscale readings -> driving state
    px.get_line_status returns [L, M, R]
      0 = line
      1 = background
    """
    s = px.get_line_status(vals)

    if s == [0, 0, 0]:
        return "stop"
    elif s[1] == 0:
        return "forward"
    elif s[0] == 0:
        return "left"
    elif s[2] == 0:
        return "right"
    else:
        return "stop"


def recover():
    """
    If the line is lost:
    - reverse slightly
    - steer in last known direction
    - stop once line is seen again
    """
    if last_state == "left":
        px.set_dir_servo_angle(+30)
    elif last_state == "right":
        px.set_dir_servo_angle(-30)
    else:
        px.set_dir_servo_angle(0)

    px.backward(px_power)

    while True:
        vals = px.get_grayscale_data()
        state = get_status(vals)
        print(f"RECOVER vals={vals} state={state}")

        if state != "stop":
            break

        sleep(0.01)

    px.stop()
    sleep(0.02)


def main():
    global last_state

    try:
        while True:
            vals = px.get_grayscale_data()
            state = get_status(vals)
            print(f"vals={vals} state={state}")

            if state in ("left", "right"):
                last_state = state

            if state == "forward":
                px.set_dir_servo_angle(0)
                px.forward(px_power)

            elif state == "left":
                px.set_dir_servo_angle(+offset)
                px.forward(px_power)

            elif state == "right":
                px.set_dir_servo_angle(-offset)
                px.forward(px_power)

            else:
                recover()

            sleep(0.02)

    except KeyboardInterrupt:
        print("\nStopping...")

    finally:
        px.stop()
        px.set_dir_servo_angle(0)
        sleep(0.1)
        print("Stopped")


if __name__ == "__main__":
    main()
