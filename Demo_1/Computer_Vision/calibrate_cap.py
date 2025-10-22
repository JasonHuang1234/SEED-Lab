import cv2 as cv
import os

# --- Settings ---
SAVE_DIR = "calib_images"     # folder to store images
CAM_INDEX = 0                 # 0 for default webcam; change if needed
IMG_PREFIX = "calib_"         # filename prefix
MAX_IMAGES = 50           # how many to capture

# Make directory if needed
os.makedirs(SAVE_DIR, exist_ok=True)

# Open camera
cap = cv.VideoCapture(CAM_INDEX)
if not cap.isOpened():
    raise IOError("Cannot open camera")

# Optional: set resolution
# cap.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
# cap.set(cv.CAP_PROP_FRAME_HEIGHT, 720)

print(f"Starting manual capture — press SPACE to save an image.")
print("Press ESC to exit.\n")

count = 0

while count < MAX_IMAGES:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    # Display live feed
    display = frame.copy()
    cv.putText(display, f"Captured {count}/{MAX_IMAGES}", (20, 40),
               cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv.imshow("Calibration Capture", display)

    key = cv.waitKey(1) & 0xFF

    if key == 27:  # ESC to quit
        print("Early stop requested.")
        break
    elif key == 32:  # SPACE to capture (ASCII 32)
        filename = os.path.join(SAVE_DIR, f"{IMG_PREFIX}{count:03d}.jpg")
        cv.imwrite(filename, frame)
        count += 1
        print(f"[{count:03d}/{MAX_IMAGES}] Saved {filename}")

print(f"\nCaptured {count} images total.")
cap.release()
cv.destroyAllWindows()
