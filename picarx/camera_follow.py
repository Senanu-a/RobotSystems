#!/usr/bin/env python3
"""
camera_line_follow_all_in_one.py

All-in-one camera-based line following using:
- Picamera2 for frames
- OpenCV for line detection
- Picarx for steering + motion

Controls:
- Press ESC to exit (if a display is available, e.g., VNC/local screen)

Notes:
- If you're SSH'd in without a GUI, cv2.imshow() will fail.
  In that case set SHOW_WINDOW = False below.
"""

from time import sleep
import time

import cv2
import numpy as np

from picamera2 import Picamera2
from picarx import Picarx


# ======== USER TUNABLES ========
SHOW_WINDOW = True       # Set False if you don't have a display (SSH without VNC/X11)
POLARITY = "dark"        # "dark" (dark line on light floor) or "light" (light line on dark floor)
POWER = 10               # forward speed
DT = 0.03                # loop delay seconds
STEER_SCALE = 25.0       # degrees per unit offset
MAX_ANGLE = 30.0         # clamp steering degrees
FRAME_W = 320
FRAME_H = 240
ROI_Y_START = 0.55       # use bottom part of image (0..1). 0.55 means bottom 45%
MIN_CONTOUR_AREA = 200   # ignore tiny blobs/noise
# ===============================


class CameraSensor:
    """Camera sensor using Picamera2. Returns RGB frames as numpy arrays."""
    def __init__(self, width=320, height=240):
        self.cam = Picamera2()
        cfg = self.cam.create_preview_configuration(
            main={"size": (width, height), "format": "RGB888"}
        )
        self.cam.configure(cfg)
        self.cam.start()
        time.sleep(0.5)

    def read(self):
        return self.cam.capture_array()

    def close(self):
        try:
            self.cam.stop()
        except Exception:
            pass


class CameraInterpreter:
    """
    Finds the line position and returns:
      offset in [-1, 1] where + means line is to LEFT of robot.
    Also returns a visualisation frame for debugging.
    """
    def __init__(self, polarity="dark", roi_y_start=0.55, min_area=200):
        self.polarity = polarity.lower()
        self.roi_y_start = float(roi_y_start)
        self.min_area = float(min_area)

    def process(self, frame_rgb):
        h, w, _ = frame_rgb.shape

        # Region of interest (bottom portion)
        y0 = int(h * self.roi_y_start)
        roi = frame_rgb[y0:h, 0:w]

        # Convert to grayscale and blur
        gray = cv2.cvtColor(roi, cv2.COLOR_RGB2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        # Threshold with Otsu, polarity-aware
        if self.polarity == "dark":
            # dark line => invert so line becomes white in binary
            _, binary = cv2.threshold(
                blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
            )
        elif self.polarity == "light":
            _, binary = cv2.threshold(
                blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
            )
        else:
            raise ValueError("POLARITY must be 'dark' or 'light'")

        # Find contours
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            # no line found
            vis = frame_rgb.copy()
            cv2.rectangle(vis, (0, y0), (w - 1, h - 1), (0, 255, 0), 2)
            return 0.0, vis

        # Choose largest contour above area threshold
        contours = [c for c in contours if cv2.contourArea(c) >= self.min_area]
        if not contours:
            vis = frame_rgb.copy()
            cv2.rectangle(vis, (0, y0), (w - 1, h - 1), (0, 255, 0), 2)
            return 0.0, vis

        largest = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest)
        if M["m00"] == 0:
            vis = frame_rgb.copy()
            cv2.rectangle(vis, (0, y0), (w - 1, h - 1), (0, 255, 0), 2)
            return 0.0, vis

        cx_roi = int(M["m10"] / M["m00"])
        cy_roi = int(M["m01"] / M["m00"])

        # Convert ROI centroid to full-frame coordinates
        cx = cx_roi
        cy = cy_roi + y0

        # offset in [-1,1]:
        # if centroid is left of centre => offset positive (line to left)
        offset = (w / 2 - cx) / (w / 2)
        offset = max(-1.0, min(1.0, float(offset)))

        # Visualisation
        vis = frame_rgb.copy()
        cv2.rectangle(vis, (0, y0), (w - 1, h - 1), (0, 255, 0), 2)
        cv2.line(vis, (w // 2, 0), (w // 2, h), (0, 255, 0), 2)
        cv2.circle(vis, (cx, cy), 6, (255, 0, 0), -1)

        # draw contour (shift it down by y0 for vis)
        shifted = largest.copy()
        shifted[:, :, 1] += y0
        cv2.drawContours(vis, [shifted], -1, (255, 255, 255), 2)

        return offset, vis


class SteeringController:
    """Maps offset [-1,1] to steering angle and commands Picarx steering servo."""
    def __init__(self, px: Picarx, scale=25.0, max_angle=30.0):
        self.px = px
        self.scale = float(scale)
        self.max_angle = float(max_angle)

    def control(self, offset):
        angle = self.scale * float(offset)
        angle = max(-self.max_angle, min(self.max_angle, angle))
        self.px.set_dir_servo_angle(angle)
        return angle


def run_camera_line_follow():
    px = Picarx()
    cam = CameraSensor(width=FRAME_W, height=FRAME_H)
    interp = CameraInterpreter(polarity=POLARITY, roi_y_start=ROI_Y_START, min_area=MIN_CONTOUR_AREA)
    ctrl = SteeringController(px, scale=STEER_SCALE, max_angle=MAX_ANGLE)

    try:
        while True:
            frame = cam.read()  # RGB
            offset, vis = interp.process(frame)
            angle = ctrl.control(offset)

            px.forward(POWER)

            if SHOW_WINDOW:
                # Convert RGB->BGR for OpenCV display (imshow expects BGR)
                vis_bgr = cv2.cvtColor(vis, cv2.COLOR_RGB2BGR)
                cv2.imshow("Camera Line Follow (ESC to quit)", vis_bgr)
                if (cv2.waitKey(1) & 0xFF) == 27:
                    break

            print(f"offset={offset:+.2f}  angle={angle:+.1f}")
            sleep(DT)

    except KeyboardInterrupt:
        pass
    finally:
        px.stop()
        px.set_dir_servo_angle(0)
        cam.close()
        if SHOW_WINDOW:
            cv2.destroyAllWindows()
        sleep(0.1)


if __name__ == "__main__":
    run_camera_line_follow()
