# Main function
# Primary developer: Kiera Crawford
# 9/26/2025
# Description: This function detects 6x6 Aruco markers, 
# determines the quadrent of the camera they are in, 
# and passes an integer 0-3 to threading function to LCD and arduino

import cv2
import numpy as np
import time
import lcd_stuff

# Initialize camera
cap = cv2.VideoCapture(0)

# Load ArUco dictionary and detector
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

# Assumes SE as default position, updates whan marker is detected
north = 0 
west = 0
change = 1

while True:
    # Check if the camera frame was successful
    # If unsuccessful throws error and retries
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture frame")
        time.sleep(1)
        continue

    #Calculate frame center
    height, width = frame.shape[:2]
    framex_center = width/2
    framey_center = height/2

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect markers
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is not None:
        # Pick the lowest marker found
        marker_index = np.argmin(ids)
        marker_corners = corners[marker_index][0]

        # Calculate center of marker
        # CONTINUE FROM HERE
        xcenter = np.mean(marker_corners[:, 0])
        ycenter = np.mean(marker_corners[:, 1])

        prev_north = north
        prev_west = west
        west = xcenter <= framex_center
        north = ycenter <= framey_center
        change = (prev_north != north) | (prev_west != west)
        
        if(change):
            #Terminal check
            if(north):
                if(west):
                    print("Marker pos is NW")
                    lcd_stuff.LCD(1)
                else:
                    print("Marker pos is NE")
                    lcd_stuff.LCD(0)
            elif(west):
                print("Marker pos is SW")
                lcd_stuff.LCD(2)
            else:
                print("Marker pos is SE")
                lcd_stuff.LCD(3)            
        change = 1
        # marker_id = ids[0][0]
        # msg = f"Marker ID:\n{marker_id}"
        # print(msg)
    else:
        if change:
            print("No markers found")
            change = 0

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