import cv2 as cv
import os

# --- Settings ---
SAVE_DIR = "calib_images"     # folder to store images
CAM_INDEX = 0                 # 0 for default webcam; change if needed
IMG_PREFIX = "calib_"         # filename prefix
MAX_IMAGES = 50               # how many to capture

# Chessboard pattern (number of *inner corners*, not squares)
CHECKERBOARD = (9, 6)         # e.g., 9x6 inner corners
DELAY_BETWEEN_CAPTURES = 1.0  # seconds between auto-saves to avoid duplicates

# Make directory if needed
os.makedirs(SAVE_DIR, exist_ok=True)

# Open camera
cap = cv.VideoCapture(CAM_INDEX)
if not cap.isOpened():
    raise IOError("Cannot open camera")

print(f"Starting automatic capture for chessboard pattern {CHECKERBOARD}")
print("Press ESC to stop.\n")

criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

count = 0
last_save_time = cv.getTickCount() / cv.getTickFrequency()

while count < MAX_IMAGES:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    ret_corners, corners = cv.findChessboardCorners(gray, CHECKERBOARD, None)

    display = frame.copy()

    if ret_corners:
        # Refine corner accuracy
        corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        cv.drawChessboardCorners(display, CHECKERBOARD, corners2, ret_corners)

        # Limit how fast images are captured
        now = cv.getTickCount() / cv.getTickFrequency()
        if now - last_save_time >= DELAY_BETWEEN_CAPTURES:
            filename = os.path.join(SAVE_DIR, f"{IMG_PREFIX}{count:03d}.jpg")
            cv.imwrite(filename, frame)
            print(f"[{count+1:03d}/{MAX_IMAGES}] Saved {filename}")
            count += 1
            last_save_time = now
    else:
        cv.putText(display, "No chessboard detected", (20, 40),
                   cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    cv.putText(display, f"Captured {count}/{MAX_IMAGES}", (20, 80),
               cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv.imshow("Calibration Capture", display)

    if cv.waitKey(1) & 0xFF == 27:  # ESC to quit
        print("Early stop requested.")
        break

print(f"\nCaptured {count} images total.")
cap.release()
cv.destroyAllWindows()
