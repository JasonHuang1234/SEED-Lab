import cv2
import numpy as np
import time
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

# LCD configuration
lcd_columns = 16
lcd_rows = 2
i2c = board.I2C()  # Uses board.SCL and board.SDA
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

# Set initial LCD color and clear
lcd.color = [0, 100, 0]  # Green
lcd.clear()
lcd.message = "Starting ArUco\nDetection..."
time.sleep(2)
lcd.clear()

# Initialize camera
cap = cv2.VideoCapture(0)

# Load ArUco dictionary and detector
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

while True:
    # Check if the camera frame was successful
    # If unsuccessful throws error and retries
    ret, frame = cap.read()
    if not ret:
        lcd.clear()
        lcd.message = "Camera error"
        print("Failed to capture frame")
        time.sleep(1)
        continue

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect markers
    corners, ids, _ = detector.detectMarkers(gray)

    # Update LCD display
    lcd.clear()
    if ids is not None:
        marker_id = ids[0][0]
        msg = f"Marker ID:\n{marker_id}"
        lcd.message = msg
        print(msg)
    else:
        lcd.message = "No markers\nfound"
        print("No markers found")

    # Show frame with markers
    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
    cv2.imshow("Aruco Detection", frame)

    # Break loop on 'q' key press
    k = cv2.waitKey(1) & 0xFF
    if k == ord('q'):
        break

# Cleanup
cap.release()
cv2.destroyAllWindows()
lcd.clear()
lcd.color = [0, 0, 0]  # Turn off backlight
lcd.message = "Detection ended"
time.sleep(2)
lcd.clear()