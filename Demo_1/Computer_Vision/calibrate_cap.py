import cv2 as cv
import os

# --- settings ---
SAVE_DIR = "calib_images"   # folder to store images
CAM_INDEX = 0               # 0 for default webcam; change if you have multiple cameras
IMG_PREFIX = "calib_"       # filename prefix
MAX_IMAGES = 30             # how many to capture total
DELAY_MS = 1000             # time (ms) between captures if using auto mode

# make directory if needed
os.makedirs(SAVE_DIR, exist_ok=True)

# open camera
cap = cv.VideoCapture(CAM_INDEX)
if not cap.isOpened():
    raise IOError("Cannot open camera")

print("Press SPACE to save an image, or ESC to quit.")

count = 0
while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    # show the live feed
    cv.imshow("Calibration Capture", frame)

    key = cv.waitKey(1) & 0xFF

    if key == 27:  # ESC -> exit
        break
    elif key == 32:  # SPACE -> save image
        filename = os.path.join(SAVE_DIR, f"{IMG_PREFIX}{count:02d}.jpg")
        cv.imwrite(filename, frame)
        print(f"Saved {filename}")
        count += 1
        if count >= MAX_IMAGES:
            print("Reached image limit.")
            break

cap.release()
cv.destroyAllWindows()
print(f"Saved {count} images to '{SAVE_DIR}'")
