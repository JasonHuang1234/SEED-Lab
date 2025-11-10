# Camera_Calibrate
# Primary developer: Jason Huang ( More Like some random dude on github )
# 10/06/2025
# Description: This function parses through a series of captured images to create camera matrixes for camera calibration

import cv2
import numpy as np
import glob
import os

def detect_charuco_corners(image_name,image, board, dictionary, params):
    marker_ids, marker_corners = detect_aruco_markers(image_name, image, board, dictionary, params)

    if marker_ids is not None and len(marker_ids) > 0:

        charuco_retval, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(marker_corners, marker_ids, image, board)
            
        if charuco_retval:                
            num_charuco_corners = len(charuco_corners)
        else:
            num_charuco_corners = 0
        print(f"Detected {num_charuco_corners} charuco corners in {image_name}")
        return charuco_ids, charuco_corners
    else:
        print(f"No charuco corners detected in {image_name}")
        return None, None




def detect_aruco_markers(image_name, image, board, dictionary, params):
    
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    marker_corners, marker_ids, rejected_corners = cv2.aruco.detectMarkers(image,aruco_dict,parameters=params)

    if marker_ids is not None and len(marker_ids) > 0:
        refined_corners, refined_ids, _, _ = cv2.aruco.refineDetectedMarkers(
            image, board, marker_corners, marker_ids, rejected_corners,
            parameters=params
        )
        marker_corners = refined_corners
        marker_ids = refined_ids
        print(f"{len(marker_ids)} ArUco markers detected in {image_name}")
        return marker_ids, marker_corners
        
        
    else:
        print(f"No ArUco markers detected in {image_name}")
        return None, None


aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
params = cv2.aruco.DetectorParameters()
charuco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
board = cv2.aruco.CharucoBoard(
    (7, 9),               # tuple of (squaresX, squaresY)
    0.9375,               # squareLength
    0.65625,                # markerLength
    charuco_dict
)
       
img_glob     = "./calib_images/*.jpg"  # adjust path/pattern

images = sorted(glob.glob(img_glob))
if not images:
    raise FileNotFoundError(f"No images found for pattern: {img_glob}")

found_count = 0


all_charuco_corners = []
all_charuco_ids = []


for img in images:
        image = cv2.imread(img)

        charuco_ids, charuco_corners = detect_charuco_corners(img, image, board, aruco_dict, params)
        if charuco_ids is not None and len(charuco_ids) > 4:
            all_charuco_corners.append(charuco_corners)
            all_charuco_ids.append(charuco_ids)
        else:
            print(f"Could not interpolate corners for {img}")
            
if not all_charuco_corners:
    raise RuntimeError("No charuco corners detected in any images.")


imageSize = (image.shape[1], image.shape[0])  # (width, height)  
K = np.zeros((3, 3))
D = np.zeros((5, 1))
retval, K, D, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(all_charuco_corners, all_charuco_ids, board, imageSize, cameraMatrix=None, distCoeffs=None)          

h, w = image.shape[:2]
print("\nRMS reprojection error:", retval)
newK, roi = cv2.getOptimalNewCameraMatrix(K, D,(w,h) , alpha=0)
mapx, mapy = cv2.initUndistortRectifyMap(K, D, None, newK, (w, h), cv2.CV_32FC1)
np.savez("calibration_full.npz",
         camera_matrix=K,
         dist_coeff=D,
         mapx=mapx,
         mapy=mapy,
         newK=newK,
         roi=np.array(roi))
print("\nSaved calibration to calibration_full.npz")
