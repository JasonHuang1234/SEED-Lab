# Main function
# Primary developer: Kiera and Jason
# 9/26/2025
# Description: This function detects an Aruco Marker based on its dictionary position
# Than the function uses a calibrated camera matrix to calculate the angle of the aruco marker relative to the camera.

# Current Command idea, begin with sending a turn command until the code receives a stop byte 

import cv2
import numpy as np
import time
import threading
import lcd_stuff
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import board
from direction import detect_arrow_color
from remoteStart import send_command

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


#Calibration was normalized for inches
marker_length = 2

#Initialize LCD
#i2c_lcd = board.I2C()
#lcd = character_lcd.Character_LCD_RGB_I2C(i2c_lcd, 16,2)
#lcd.clear()
#lcd.color = (50, 0, 50)
#time.sleep(1)

# Initialize camera
cap = cv2.VideoCapture(0)

# Load ArUco dictionary and detector
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)


prev_angle = 0
change = 1


#Averaging values for camera inputs
avg = 0
avg2 = 0
sum = 0
sum2 = 0
distsum = 0
avgtot = 1
angle = 10000
dist = 1000
firstfind = 1
done = 0

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
        # tvec provides distance info
        # rvec provides rotation info, less important/useful unless we want to course correct
        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(marker_corners,marker_length,newK,None)
        cv2.drawFrameAxes(frame,newK,dist,rvec,tvec,0.03)
        z = tvec[0][0][2]
        x_est = tvec[0][0][0]
       # 3D geometric angle
        toa = x_est/z
        angle = np.degrees(np.arctan(x))
        angle2 = np.degrees(np.arctan(toa))
        # Figure out which is better
        sum += angle
        sum2 += angle2
        avg += 1
        x_power = x_est*x_est
        z_power = z*z
        power_vals = x_power + z_power
        aprox = np.sqrt(power_vals)
        distsum += aprox
        if avg == avgtot:
            angle2 = sum2/avg
            angle = sum/avg
            angle = np.round(angle, 2)
            angle2 = np.round(angle2, 2)
            distance_val = distsum/avg
            distance_val = distance_val - 9
            if np.round(angle,1) == np.round(prev_angle,1):
                change = 0
            else:
                change = 1
            prev_angle = angle
            if firstfind == 0 and abs(angle < 4):
                send_command(0, 0, "stop")
                firstfind = 1
                time.sleep(0.5)

        
            if (change):
                if not (abs(angle) < 0.02 and abs(distance_val) < 4) and abs(angle) < 4 and done == 0:
                    print(f"angle 1 is {angle} \n")
                    print(f"angle 2 is {angle2} \n")
                    print(f"distance in inches from marker is {distance_val} \n")
                    #Update ARGs
                    send_command(distance_val,angle, "control")
            sum = 0
            sum2 = 0
            avg = 0
            distsum = 0
    
    # In this no markers section Im thinking I will send a cmd to arduino telling it to turn, so 0x00 cmd
    else:
        if change and firstfind == 0:
            print("No markers found")
            send_command(0,0, "turn")
    if abs(angle) < 0.5 and abs(distance_val) < 4 and done = 0: #and direction is less than a given error
        send_command(0, 0, "stop")
        time.sleep(0.5)
        direction = detect_arrow_color(frame, marker_corners)
        if direction is not None:
            if direction == "green":
                send_command(0, 0, "left")
            elif direction == "red":
                send_command(0, 0, "right")
            else:
                send_command(0, 0, "stop")
                time.sleep(1)
                done = 0
                break
            print(f"direction is {direction}")
            direction = None
            time.sleep(5)
            send_command(0,0, "stop")
            done = 1



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
