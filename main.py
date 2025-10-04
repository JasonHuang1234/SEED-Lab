# Main function
# Primary developer: Kiera Crawford
# 9/26/2025
# Description: This function detects 6x6 Aruco markers, 
# determines the quadrent of the camera they are in, 
# and passes an integer 0-3 to threading function to LCD and arduino

import cv2
import numpy as np
import time
import threading
import lcd_stuff
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import board
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
prev_north = -1
prev_west = -1
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
        xcenter = np.mean(marker_corners[:, 0])
        ycenter = np.mean(marker_corners[:, 1])
        
        west = xcenter <= framex_center
        north = ycenter <= framey_center
        change = (prev_north != north) | (prev_west != west)
        
        if(change):
            #This needs to be threaded
            myThread = threading.Thread(target=lcd_stuff.LCD, args=(north, west, lcd))
            myThread.start()
            if(north):
                if(west):
                    print("Marker pos is NW")
                else:
                    print("Marker pos is NE")
            elif(west):
                print("Marker pos is SW")
            else:
                print("Marker pos is SE")          
        prev_north = north
        prev_west = west
        change = 1
        # marker_id = ids[0][0]
        # msg = f"Marker ID:\n{marker_id}"
        # print(msg)
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