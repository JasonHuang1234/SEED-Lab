#!/usr/bin/env python
import os, sys, json, cv2
import numpy as np
from glob import glob

def splitfn(fname):
    path, fname = os.path.split(fname)
    name, ext = os.path.splitext(fname)
    return path, name, ext

def main(image_dir, fisheye, pattern_size, square_size, threads, json_file=None, debug_dir=None):
    """Calibrate camera from chessboard images in a directory."""
    # Get all image files from directory
    image_files = sorted(glob(os.path.join(image_dir, '*.jpg')) + glob(os.path.join(image_dir, '*.png')))
    if not image_files:
        print(f"No images found in directory: {image_dir}")
        sys.exit(-1)

    # JSON data
    j = {}

    # Real-world 3D corner "positions"
    pattern_points = np.zeros((np.prod(pattern_size), 3), np.float32)
    pattern_points[:, :2] = np.indices(pattern_size).T.reshape(-1, 2)
    pattern_points = np.expand_dims(np.asarray(pattern_points), -2)
    pattern_points *= square_size

    j['chessboard_points'] = pattern_points.tolist()
    j['chessboard_inner_corners'] = pattern_size
    j['chessboard_spacing_m'] = square_size

    # Read first image to get resolution
    img = cv2.imread(image_files[0], cv2.IMREAD_GRAYSCALE)
    if img is None:
        print(f"Failed to read {image_files[0]} to get resolution!")
        sys.exit(-1)
    h, w = img.shape[:2]
    print(f"Image resolution {w}x{h}")
    j['image_resolution'] = (w, h)

    # Process all images
    def process_image(fname):
        img = cv2.imread(fname, 0)
        if img is None:
            return (fname, 'Failed to load')

        if w != img.shape[1] or h != img.shape[0]:
            return (fname, f"Size {img.shape[1]}x{img.shape[0]} doesn't match")

        found, corners = cv2.findChessboardCorners(img, pattern_size)
        if found:
            term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1)
            cv2.cornerSubPix(img, corners, (5, 5), (-1, -1), term)

        if debug_dir:
            vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            cv2.drawChessboardCorners(vis, pattern_size, corners, found)
            _, name, _ = splitfn(fname)
            outfile = os.path.join(debug_dir, name + '_chessboard.png')
            cv2.imwrite(outfile, vis)

        if not found:
            return (fname, 'Chessboard not found')

        return (fname, corners)

    if threads <= 1:
        print(f"Processing {len(image_files)} images")
        results = [process_image(fname) for fname in image_files]
    else:
        print(f"Processing {len(image_files)} images using {threads} threads")
        from multiprocessing.dummy import Pool as ThreadPool
        with ThreadPool(threads) as pool:
            results = pool.map(process_image, image_files)

    obj_points, img_points, cb_to_image_index = [], [], {}
    results.sort(key=lambda e: e[0])
    cb_index = 0
    for img_index, (fname, corners) in enumerate(results):
        if isinstance(corners, str):
            print(f"[{fname}] Ignoring image: {corners}")
            continue
        img_points.append(corners)
        obj_points.append(pattern_points)
        cb_to_image_index[cb_index] = img_index
        cb_index += 1

    if cb_index == 0:
        print("No valid chessboards found!")
        sys.exit(-1)

    print(f"Found chessboards in {cb_index} out of {len(image_files)} images")

    # Calibrate
    calibrate_func = cv2.fisheye.calibrate if fisheye else cv2.calibrateCamera
    print(f"Calibrating using {len(img_points)} images...")
    rms, camera_matrix, dist_coefs, rvecs, tvecs = calibrate_func(
        obj_points, img_points, (w, h), None, None
    )

    print("RMS:", rms)
    print("Camera matrix:\n", camera_matrix)
    print("Distortion coefficients:\n", dist_coefs.ravel())

    # Compute reprojection error
    project_func = cv2.fisheye.projectPoints if fisheye else cv2.projectPoints
    errors = []
    for cb_index in range(len(img_points)):
        img_points2, _ = project_func(
            obj_points[cb_index], rvecs[cb_index], tvecs[cb_index],
            camera_matrix, dist_coefs
        )
        error = cv2.norm(img_points[cb_index], img_points2, cv2.NORM_L2) / len(img_points2)
        errors.append(error)

    reprojection_error_avg = np.mean(errors)
    reprojection_error_stddev = np.std(errors)
    print(f"Average reprojection error: {reprojection_error_avg:.6f} +/- {reprojection_error_stddev:.6f}")

    j.update({
        'camera_matrix': camera_matrix.tolist(),
        'distortion_coefficients': dist_coefs.ravel().tolist(),
        'rms': rms,
        'reprojection_error': {
            'average': reprojection_error_avg,
            'stddev': reprojection_error_stddev
        }
    })

    if json_file:
        with open(json_file, 'w') as f:
            json.dump(j, f, indent=2)
        print(f"Calibration saved to {json_file}")

    print("Calibration complete.")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    # ---- Hardcoded parameters ----
    IMAGE_DIR = "./calib_images"       # path to your images
    DEBUG_DIR = "./calib_debug"        # optional directory for output
    JSON_FILE = "calibration_output.json"

    if not os.path.exists(DEBUG_DIR):
        os.makedirs(DEBUG_DIR)

    main(
        image_dir=IMAGE_DIR,
        fisheye=False,                  # Set to True for fisheye lens
        pattern_size=(7, 5),            # Inner corners
        square_size=0.03,              # Chessboard square size (meters)
        threads=4,
        json_file=JSON_FILE,
        debug_dir=DEBUG_DIR
    )
