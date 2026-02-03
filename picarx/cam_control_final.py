#!/usr/bin/env python3
from flask import Flask, Response
from picamera2 import Picamera2
import cv2
import time
import numpy as np
from picarx_improved import Picarx
from collections import deque


app = Flask(__name__)

# ====== Tunables ======
FRAME_W, FRAME_H = 320,240
ROI_Y_START = 0.70
POLARITY = "dark"          # "dark" or "light"
MIN_CONTOUR_AREA = 300

STEER_SCALE = 30.0
MAX_ANGLE = 30.0

FWD_POWER = 20             # forward speed (0-100-ish depending on your lib)
DELAY_FRAMES = 3      # try 2–6 (higher = more delay)
offset_buf = deque(maxlen=DELAY_FRAMES)

# ======================

# ----- Camera -----
cam = Picamera2()
cfg = cam.create_video_configuration(
    main={"size": (FRAME_W, FRAME_H), "format": "RGB888"}
)
cam.configure(cfg)
cam.start()
time.sleep(0.5)

# ----- Car -----   
px = Picarx()
px.set_dir_servo_angle(0)
px.stop()
px.set_cam_tilt_angle(-50) #keep camera down

def find_line_and_annotate(frame_rgb):
    h, w, _ = frame_rgb.shape
    y0 = int(h * 0.85)      # start very close to the bottom
    y1 = int(h * 0.98)      # stop before extreme edge
    roi = frame_rgb[y0:y1, 0:w]


    gray = cv2.cvtColor(roi, cv2.COLOR_RGB2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    if POLARITY == "dark":
        _, binary = cv2.threshold(
            blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
        )
    else:
        _, binary = cv2.threshold(
            blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
        )

    contours, _ = cv2.findContours(
        binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    contours = [c for c in contours if cv2.contourArea(c) >= MIN_CONTOUR_AREA]

    vis = frame_rgb.copy()
    cv2.rectangle(vis, (0, y0), (w - 1, y1 - 1), (0, 255, 0), 2)
    cv2.line(vis, (w // 2, 0), (w // 2, h), (0, 255, 0), 2)

    offset = None
    
    if contours:
        largest = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest)
        if M["m00"] != 0:
            cx_roi = int(M["m10"] / M["m00"])
            cy_roi = int(M["m01"] / M["m00"])

            cx = cx_roi
            cy = cy_roi + y0

            # +offset => line is to LEFT
            offset = (w / 2 - cx) / (w / 2)
            offset = max(-1.0, min(1.0, float(offset)))

            cv2.circle(vis, (cx, cy), 7, (255, 0, 0), -1)

            shifted = largest.copy()
            shifted[:, :, 1] += y0
            cv2.drawContours(vis, [shifted], -1, (255, 255, 255), 2)

    txt = f"offset: {offset:+.2f}" if offset is not None else "offset: None"
    cv2.putText(
        vis, txt, (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2
    )

    return vis, offset


def gen_frames():
    while True:
        frame = cam.capture_array()   # RGB frame
        vis, offset = find_line_and_annotate(frame)
                # --- Delay buffer ---
        offset_buf.append(offset)

        # Use delayed offset once buffer is full
        offset_used = offset_buf[0] if len(offset_buf) == DELAY_FRAMES else offset

        DEADBAND = 0.08  # try 0.05–0.12
        # ===== Steering + Motion =====
        if offset_used is not None:
            angle = -STEER_SCALE * offset_used
            angle = max(-MAX_ANGLE, min(MAX_ANGLE, angle))
            px.set_dir_servo_angle(angle)
            px.forward(FWD_POWER)
        

        elif offset_used is not None and abs(offset_used) < DEADBAND:
            offset_used = 0.0
        else:
            # If line is lost, stop for safety
            px.stop()
            px.set_dir_servo_angle(0)
        # ============================= """

        vis_bgr = cv2.cvtColor(vis, cv2.COLOR_RGB2BGR)
        ok, buf = cv2.imencode(".jpg", vis_bgr)
        if not ok:
            continue

        jpg = buf.tobytes()
        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n"
        )


@app.route("/")
def index():
    return "<h3>Annotated Line Stream (Steering + Motion)</h3><img src='/video'>"


@app.route("/video")
def video():
    return Response(
        gen_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


if __name__ == "__main__":
    try:
        app.run(host="0.0.0.0", port=5000, threaded=True)
    finally:
        px.set_cam_tilt_angle(0) #reset camera tilt to default
        px.stop()
        px.set_dir_servo_angle(0)
