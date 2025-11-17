# Main function
# Primary developer: Kiera Crawford
# 9/26/2025
# Description: This function detects an Aruco Marker based on its dictionary position
# Than the function uses a calibrated camera matrix to calculate the angle of the aruco marker relative to the camera.

import cv2
import numpy as np
import time
import threading
import lcd_stuff
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import board
with np.load('calibration_full.npz') as data:
    mtx = data['camera_matrix']
    dist = data['dist_coeff']
    mapx = data['mapx']
    mapy = data['mapy']
    newK = data['newK']
    roi = data['roi']

cx = newK[0,2]
fx = newK[0,0]
print(f"cs is {cx}")


marker_length = 0.049

#Initialize LCD
i2c_lcd = board.I2C()
lcd = character_lcd.Character_LCD_RGB_I2C(i2c_lcd, 16,2)
lcd.clear()
lcd.color = (50, 0, 50)
time.sleep(1)

# Initialize camera
cap = cv2.VideoCapture(0)

# Load ArUco dictionary and detector
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

# Assumes SE as default position, updates whan marker is detected
north = 0 
west = 0
prev_angle = 0
change = 1

avg = 0
sum = 0
while True:
    # Check if the camera frame was successful
    # If unsuccessful throws error and retries
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture frame")
        time.sleep(1)
        continue
    
    # Apply undistortion using precomputed maps
    frame = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)

    # Optional: crop ROI for perfectly rectified image
    x, y, w, h = roi
    frame = frame[y:y+h, x:x+w]

    # Show original vs undistorted

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    #Calculate frame center
    height, width = frame.shape[:2]
    framex_center = width//2
    framey_center = height//2
    cv2.circle(frame, (framex_center, framey_center), 5, (0, 0, 255), -1)
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect markers
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is not None:
        # Pick the lowest marker found
        marker_index = np.argmin(ids)
        marker_corners = corners[marker_index]


        # Calculate center of marker
        xcenter = np.mean(marker_corners[0][:, 0])
        ycenter = np.mean(marker_corners[0][:, 1])


        #The below code is more accurate but much slowe
        x = (xcenter - cx) / fx
        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(marker_corners,marker_length,mtx,dist )
        cv2.drawFrameAxes(frame,mtx,dist,rvec,tvec,0.03)
        z = tvec[0][0][2]
       # 3D geometric angle
        angle = np.degrees(np.arctan(x))
        sum += angle
        avg += 1
        if avg = 6:
            angle = sum/avg
            angle = np.round(angle, 2)
            print(angle)
            if angle == prev_angle:
                change = 0
            else:
                change = 1
            prev_angle = angle

            if (change):
                print(angle)
                myThread = threading.Thread(target=lcd_stuff.LCD, args=(angle, lcd))
                myThread.start()
            sum = 0
            avg = 0
    
    else:
        if change:
            print("No markers found")
            change = 0
            prev_north = -1
            prev_west = -1

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
lcd.clear()
lcd.color = (0,0,0)