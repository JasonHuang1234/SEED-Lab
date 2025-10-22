import cv2 as cv
import os
import time

# --- Settings ---
SAVE_DIR = "calib_images"     # folder to store images
CAM_INDEX = 0                 # 0 for default webcam; change if needed
IMG_PREFIX = "calib_"         # filename prefix
MAX_IMAGES = 200            # how many to capture
CAPTURE_INTERVAL = 1.0        # seconds between captures

# Make directory if needed
os.makedirs(SAVE_DIR, exist_ok=True)

# Open camera
cap = cv.VideoCapture(CAM_INDEX)
if not cap.isOpened():
    raise IOError("Cannot open camera")

# Optional: set resolution
# cap.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
# cap.set(cv.CAP_PROP_FRAME_HEIGHT, 720)

print(f"Starting auto capture — saving {MAX_IMAGES} images to '{SAVE_DIR}'")
print("Press ESC to stop early.\n")

count = 0
last_capture = time.time()

while count < MAX_IMAGES:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    # Show live feed
    display = frame.copy()
    cv.putText(display, f"Capturing {count+1}/{MAX_IMAGES}", (20, 40),
               cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv.imshow("Calibration Capture", display)

    # Capture every interval
    now = time.time()
    if now - last_capture >= CAPTURE_INTERVAL:
        filename = os.path.join(SAVE_DIR, f"{IMG_PREFIX}{count:03d}.jpg")
        cv.imwrite(filename, frame)
        print(f"[{count+1:03d}/{MAX_IMAGES}] Saved {filename}")
        count += 1
        last_capture = now

    # Allow ESC to quit early
    if cv.waitKey(1) & 0xFF == 27:
        print("Early stop requested.")
        break

print(f"\nCaptured {count} images total.")
cap.release()
cv.destroyAllWindows()
