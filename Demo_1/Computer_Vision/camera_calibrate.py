import cv2 as cv
import numpy as np
import glob
import os

# --- settings ---
pattern_size = (7, 5)         # inner corners (cols, rows)
square_size  = 30.0            # set to your square edge length (e.g., 24.0 for mm)
img_glob     = "./calib/*.jpg"  # adjust path/pattern

# termination criteria for corner refinement
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 50, 1e-6)

# prepare one set of 3D object points for the board (z=0 plane)
objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
objp *= square_size  # gives real-world scale if square_size != 1

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

    # Find chessboard corners
    flags = cv.CALIB_CB_ADAPTIVE_THRESH | cv.CALIB_CB_NORMALIZE_IMAGE | cv.CALIB_CB_FAST_CHECK
    ok, corners = cv.findChessboardCorners(gray, pattern_size, flags=flags)

    if ok:
        # refine
        corners = cv.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners)
        objpoints.append(objp.copy())  # copy is safer

        found_count += 1
    else:
        print(f"Chessboard NOT found in {os.path.basename(fname)}")

if found_count < 8:
    raise RuntimeError(f"Only {found_count} good views found; take more (aim 15–30).")

h, w = gray.shape[:2]
rms, K, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, (w, h), None, None)
print("RMS reprojection error:", rms)
print("K:\n", K)
print("dist:", dist.ravel())

# Average per-image reprojection error (sanity check)
mean_err = 0.0
for i in range(len(objpoints)):
    proj, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, dist)
    err = cv.norm(imgpoints[i], proj, cv.NORM_L2) / len(proj)
    mean_err += err
mean_err /= len(objpoints)
print("Mean reprojection error per image:", mean_err)

# Optional: precompute undistort maps for fast video use (save if you like)
newK, roi = cv.getOptimalNewCameraMatrix(K, dist, (w, h), alpha=0)
mapx, mapy = cv.initUndistortRectifyMap(K, dist, None, newK, (w, h), cv.CV_32FC1)
np.savez("undistort_maps.npz", mapx=mapx, mapy=mapy, newK=newK, roi=np.array(roi), dist=dist)
