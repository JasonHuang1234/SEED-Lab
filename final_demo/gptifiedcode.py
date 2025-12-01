import cv2
import numpy as np
import time
from direction import detect_arrow_color
from remoteStart import send_command, poll_turn_done

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
firstfind = 0
angle = 10000
distance_val = 10000


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


turning_after_arrow = False  # NEW

while True:
    # debug: see if loop is still running
    # (helps prove we're actually getting new frames)
    # print("loop start")

    ret, frame = cap.read()
    if not ret:
        print("Failed to capture frame")
        time.sleep(1)
        continue

    frame = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)

    height, width = frame.shape[:2]
    framex_center = width // 2
    framey_center = height // 2
    cv2.circle(frame, (framex_center, framey_center), 5, (0, 0, 255), -1)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # ---------- CASE 1: we're in a "turn after arrow" state ----------
    if turning_after_arrow:
        # still show camera feed
        cv2.imshow("Aruco Detection", frame)

        # check if Arduino says we're done turning (180)
        if poll_turn_done():
            send_command(0, 0, "stop")
            turning_after_arrow = False
            firstfind = 0
            angle = 10000
            distance_val = 10000
            print("Turn complete, resuming normal tracking")

        # keep processing GUI events
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # skip the rest of the logic this loop
        continue

    # ---------- CASE 2: normal tracking / approach ----------
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is not None:
        # your existing angle/distance/control logic unchanged,
        # except I removed the "turn" send in firstfind block

        distances = []
        for i, marker_id in enumerate(ids):
            marker_corners = corners[i]
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                marker_corners, marker_length, newK, None
            )
            z = tvec[0][0][2]
            distances.append(z)

        closest_index = np.argmin(distances)
        closest_id = ids[closest_index][0]
        marker_corners = corners[closest_index]

        xcenter = np.mean(marker_corners[0][:, 0])
        x = (xcenter - cx) / fx

        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
            marker_corners, marker_length, newK, None
        )
        cv2.drawFrameAxes(frame, newK, dist, rvec, tvec, 0.03)
        z = tvec[0][0][2]
        x_est = tvec[0][0][0]

        toa = x_est / z
        a1 = np.degrees(np.arctan(x))
        a2 = np.degrees(np.arctan(toa))

        sum += a1
        sum2 += a2
        avg += 1
        aprox = np.sqrt(x_est * x_est + z * z)
        distsum += aprox

        if avg == avgtot:
            a1 = sum / avg
            a2 = sum2 / avg
            angle = np.round(a1, 2)
            angle2 = np.round(a2, 2)
            distance_val = distsum / avg - 14

            if np.round(angle, 1) == np.round(prev_angle, 1):
                change = 0
            else:
                change = 1
            prev_angle = angle

            if firstfind == 0 and abs(angle) < 30:
                print("firstfind set")
                firstfind = 1
                # NOTE: probably stop blind turning here, not start it
                # send_command(0, 0, "stop")
                time.sleep(0.1)

            if change:
                if not (abs(angle) < 0.5 and abs(distance_val) < 4) and \
                   abs(angle) < 10 and abs(distance_val) < 50:
                    print(f"angle 1 is {angle}")
                    print(f"angle 2 is {angle2}")
                    print(f"distance in inches from marker is {distance_val}")
                    time.sleep(0.1)
                    send_command(distance_val, angle, "control")

            sum = 0
            sum2 = 0
            avg = 0
            distsum = 0

        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

    else:
        corners = None
        ids = None
        if change and firstfind == 0:
            print("No markers found")
            send_command(0, 0, "turn")  # search

    # ---------- close enough: arrow detection ----------
    if abs(angle) < 0.5 and abs(distance_val) < 4 and firstfind == 1:
        time.sleep(0.1)
        send_command(0, 0, "stop")

        if ids is not None:
            marker_corners = corners[0]
            direction = detect_arrow_color(frame, marker_corners)

            if direction is not None:
                print(f"direction is {direction}")

                if direction == "green":
                    send_command(0, 0, "left")
                elif direction == "red":
                    send_command(0, 0, "right")
                else:
                    send_command(0, 0, "stop")

                # now we enter the 'turning_after_arrow' state
                turning_after_arrow = True
                firstfind = 0
                angle = 10000
                distance_val = 10000

            else:
                print("Arrow not recognized, stopping")
                send_command(0, 0, "stop")
                # optionally: break here if you want to abort

    # ---------- show frame ----------
    cv2.imshow("Aruco Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
time.sleep(2)
