# Camera_Calibrate
# Primary developer: Jason Huang ( More Like some random dude on github )
# 10/06/2025
# Description: This function parses through a series of captured images to create camera matrixes for camera calibration

import cv2 as cv
import numpy as np
import glob
import os

# --- settings ---
pattern_size = (7, 5)          # inner corners (cols, rows)
square_size  = 0.03             
img_glob     = "./calib_images/*.jpg"  # adjust path/pattern

# termination criteria for corner refinement
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 50, 1e-6)

# prepare one set of 3D object points for the board (z=0 plane)
objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
objp *= square_size

objpoints = []   # 3D points in world coordinates
imgpoints = []   # 2D points in image plane

images = sorted(glob.glob(img_glob))
if not images:
    raise FileNotFoundError(f"No images found for pattern: {img_glob}")

found_count = 0
for fname in images:
    img = cv.imread(fname)
    if img is None:
        print(f"Warning: could not read {fname}")
        continue

    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # --- First attempt ---
    flags = cv.CALIB_CB_ADAPTIVE_THRESH | cv.CALIB_CB_NORMALIZE_IMAGE | cv.CALIB_CB_FAST_CHECK
    ok, corners = cv.findChessboardCorners(gray, pattern_size, flags=flags)

    # --- Second attempt (fallback, if first fails) ---
    if not ok:
        flags_retry = cv.CALIB_CB_ADAPTIVE_THRESH | cv.CALIB_CB_NORMALIZE_IMAGE
        ok, corners = cv.findChessboardCorners(gray, pattern_size, flags=flags_retry)
        if ok:
            print(f"✅ Found on second try: {os.path.basename(fname)}")

    if ok:
        # Refine corner positions
        cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        objpoints.append(objp)
        imgpoints.append(corners)
        found_count += 1
        print(f"✔ Chessboard found in {os.path.basename(fname)}")
    else:
        print(f"❌ Chessboard NOT found in {os.path.basename(fname)}")

if found_count < 8:
    raise RuntimeError(f"Only {found_count} good views found; take more (aim 15–30).")

h, w = gray.shape[:2]
rms, K, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, (w, h), None, None)
print("\nRMS reprojection error:", rms)
print("K:\n", K)
print("dist:", dist.ravel())

# --- Average per-image reprojection error (sanity check) ---
mean_err = 0.0
for i in range(len(objpoints)):
    proj, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, dist)
    err = cv.norm(imgpoints[i], proj, cv.NORM_L2) / len(proj)
    mean_err += err
mean_err /= len(objpoints)
print("Mean reprojection error per image:", mean_err)

# --- Optional: precompute undistort maps for fast video use ---
newK, roi = cv.getOptimalNewCameraMatrix(K, dist, (w, h), alpha=0)
mapx, mapy = cv.initUndistortRectifyMap(K, dist, None, newK, (w, h), cv.CV_32FC1)
np.savez("calibration_full.npz",
         camera_matrix=K,
         dist_coeff=dist,
         mapx=mapx,
         mapy=mapy,
         newK=newK,
         roi=np.array(roi))
print("\nSaved calibration to calibration_full.npz")
