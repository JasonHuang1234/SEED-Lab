# Main function
# Primary developer: Kiera and Jason
# 11/14/2025
# Description: This function implements trial2 of demo2

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
direction = None
marker_corners = None


#Calibration was normalized for inches
marker_length = 2


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
dist = 10000
firstfind = 0
while True:
    # Check if the camera frame was successful
    # If unsuccessful throws error and retries
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture frame")
        time.sleep(1)
        continue
    marker_corners = None
    
    # Apply undistortion using precomputed maps
    frame = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)

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
        distances = []
        # Pick the lowest marker found
        for i, marker_id in enumerate(ids):
            marker_corners = corners[i]
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(marker_corners,marker_length,newK,None)
            z = tvec[0][0][2]
            distances.append(z)
        closest_index = np.argmin(distances)
        # Extract the corresponding ID and corners
        closest_id = ids[closest_index][0]
        marker_corners = corners[closest_index]
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
            distance_val = distance_val - 14
            if np.round(angle,1) == np.round(prev_angle,1):
                change = 0
            else:
                change = 1
            prev_angle = angle
            if firstfind == 0 and abs(angle < 30):
                print("fistfind set") 
                firstfind = 1
                time.sleep(0.1)
            if (change):
                if not (abs(angle) < 0.5 and abs(distance_val) < 4) and abs(angle) < 10 and abs(distance_val) < 60:
                    print(f"angle 1 is {angle} \n")
                    print(f"angle 2 is {angle2} \n")
                    print(f"distance in inches from marker is {distance_val} \n")
                    time.sleep(0.1)
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
    if abs(angle) < 0.5 and abs(distance_val) < 4 and firstfind == 1: #and direction is less than a given error
        time.sleep(0.1)
        send_command(0, 0, "stop")
        ret, frame = cap.read()
        firstfind = 0
        if not ret:
            continue
        frame = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        if ids is not None:
            marker_corners = corners[0]     # use the main marker again
        else: 
            print("Lost marker before direction detection!")
            continue
        if marker_corners is not None:
            direction = detect_arrow_color(frame, marker_corners)
            marker_corners = None
            if direction is not None:
                if direction == "green":
                    time.sleep(0.1)
                    send_command(0, 0, "left")
                    if not ret:
                        continue
                    frame = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    corners, ids, _ = detector.detectMarkers(gray)
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                    print("testing")
                    time.sleep(0.1)
                    send_command(0, 0, "stop")
                elif direction == "red":
                    time.sleep(0.1)
                    send_command(0, 0, "right")
                    if not ret:
                        continue
                    frame = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    corners, ids, _ = detector.detectMarkers(gray)
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                    print("testing")
                    time.sleep(0.1)
                    send_command(0, 0, "stop")
                else:
                    time.sleep(0.1)
                    send_command(0, 0, "stop")
                    done = 0
                    break
            print(f"direction is {direction}")
            direction = None
            firstfind = 0
            angle = 10000
            dist = 10000
            ret, frame = cap.read()
            firstfind = 0
            if not ret:
                continue
            frame = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        print("leaving loops")
        print(f"direction is {direction}")

    

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