# Calibrate Cap Function
# Primary developer: Jason Huang
# 10/31/2025
# Description: This captures jpg images with the camera to calibrate with

import cv2 as cv
import os
import time

# --- Settings ---
SAVE_DIR = "calib_images"     # folder to store images
CAM_INDEX = 0                 # 0 for default webcam; change if needed
IMG_PREFIX = "calib_"         # filename prefix
START_INDEX = 0            # start numbering from this image index
NEW_IMAGES = 200            # how many *new* images to capture
CAPTURE_INTERVAL = 1.0        # seconds between captures

# --- Setup directory ---
os.makedirs(SAVE_DIR, exist_ok=True)

# --- Open camera ---
cap = cv.VideoCapture(CAM_INDEX)
if not cap.isOpened():
    raise IOError("Cannot open camera")

print(f"Starting auto capture — saving {NEW_IMAGES} new images to '{SAVE_DIR}' starting at index {START_INDEX}")
print("Press ESC to stop early.\n")

count = 0
last_capture = time.time()

while count < NEW_IMAGES:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    display = frame.copy()
    current_index = START_INDEX + count
    cv.putText(display, f"Capturing {current_index}/{START_INDEX + NEW_IMAGES - 1}",
               (20, 40), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv.imshow("Calibration Capture", display)

    now = time.time()
    if now - last_capture >= CAPTURE_INTERVAL:
        filename = os.path.join(SAVE_DIR, f"{IMG_PREFIX}{current_index:03d}.jpg")
        cv.imwrite(filename, frame)
        print(f"[{current_index:03d}] Saved {filename}")
        count += 1
        last_capture = now

    if cv.waitKey(1) & 0xFF == 27:
        print("Early stop requested.")
        break

print(f"\nCaptured {count} images total (ending at {START_INDEX + count - 1}).")
cap.release()
cv.destroyAllWindows()
