# Main function
# Primary developer: Kiera Crawford
# 9/26/2025
# Description: This function detects 6x6 Aruco markers, 
# determines the quadrent of the camera they are in, 
# and passes an integer 0-3 to threading function to LCD and arduino

import cv2
import numpy as np
import time

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
        print("Failed to capture frame")
        time.sleep(1)
        continue

    #Calculate frame center
    framecenter = frame.shape/2 #Check this line

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect markers
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is not None:
        # Calculate center of marker
        # CONTINUE FROM HERE

        marker_id = ids[0][0]
        msg = f"Marker ID:\n{marker_id}"
        print(msg)
    else:
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
time.sleep(2)