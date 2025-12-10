#!/usr/bin/env python3
"""
Corrected Camera Calibration Code for Physical AI
This file contains the corrected camera calibration code from the sensor systems document
"""

import cv2
import numpy as np
import glob

def calibrate_camera(image_path_pattern, checkerboard_size=(9, 6), show_results=True):
    """
    Calibrate camera using checkerboard images.

    Args:
        image_path_pattern: Glob pattern for calibration images
        checkerboard_size: (width, height) in internal corners
        show_results: Whether to display calibration results

    Returns:
        camera_matrix: 3x3 intrinsic matrix
        dist_coeffs: Distortion coefficients
        rvecs, tvecs: Rotation and translation vectors for each image
    """
    # Prepare object points (0,0,0), (1,0,0), (2,0,0), ...
    objp = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2)

    # Arrays to store object points and image points
    objpoints = []  # 3D points in real world space
    imgpoints = []  # 2D points in image plane

    images = glob.glob(image_path_pattern)
    image_size = None

    if len(images) == 0:
        raise ValueError(f"No images found matching pattern: {image_path_pattern}")

    valid_image_count = 0

    for fname in images:
        img = cv2.imread(fname)
        if img is None:
            print(f"Warning: Could not load image: {fname}")
            continue
            
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        if image_size is None:
            image_size = gray.shape[::-1]

        # Find checkerboard corners
        ret, corners = cv2.findChessboardCorners(
            gray, checkerboard_size, 
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
        )

        if ret:
            objpoints.append(objp)

            # Refine corner locations
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners_refined = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), criteria
            )
            imgpoints.append(corners_refined.reshape(-1, 2))
            valid_image_count += 1
            
            if show_results:
                # Draw and display the corners
                img_with_corners = cv2.drawChessboardCorners(
                    img.copy(), checkerboard_size, corners_refined, ret
                )
                cv2.imshow('Calibration', img_with_corners)
                cv2.waitKey(500)  # Show each image for 500ms

    if valid_image_count < 10:
        print(f"Warning: Only {valid_image_count} valid calibration images found. "
              f"Recommend at least 10-20 images for good calibration.")

    if len(objpoints) < 3:
        raise ValueError("Not enough valid images for calibration. Need at least 3.")

    # Perform calibration
    try:
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            objpoints, imgpoints, image_size, None, None
        )
    except cv2.error as e:
        raise RuntimeError(f"Camera calibration failed: {e}")

    # Calculate reprojection error
    total_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(
            objpoints[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs
        )
        imgpoints2 = imgpoints2.reshape(-1, 2)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        total_error += error

    mean_error = total_error / len(objpoints) if len(objpoints) > 0 else float('inf')
    
    if show_results:
        print(f"Camera calibration complete. Mean reprojection error: {mean_error:.3f} pixels")

    return camera_matrix, dist_coeffs, rvecs, tvecs


def save_calibration(camera_matrix, dist_coeffs, filename='camera_calibration.npz'):
    """
    Save camera calibration parameters to file.
    
    Args:
        camera_matrix: 3x3 intrinsic matrix
        dist_coeffs: Distortion coefficients
        filename: Output filename
    """
    np.savez(filename,
             camera_matrix=camera_matrix,
             dist_coeffs=dist_coeffs)
    print(f"Calibration parameters saved to {filename}")


def load_calibration(filename='camera_calibration.npz'):
    """
    Load camera calibration parameters from file.
    
    Args:
        filename: Input filename
        
    Returns:
        camera_matrix: 3x3 intrinsic matrix
        dist_coeffs: Distortion coefficients
    """
    try:
        data = np.load(filename)
        camera_matrix = data['camera_matrix']
        dist_coeffs = data['dist_coeffs']
        print(f"Calibration parameters loaded from {filename}")
        return camera_matrix, dist_coeffs
    except FileNotFoundError:
        print(f"Calibration file {filename} not found")
        return None, None


def undistort_image(img, camera_matrix, dist_coeffs):
    """
    Undistort an image using calibration parameters.
    
    Args:
        img: Input image
        camera_matrix: 3x3 intrinsic matrix
        dist_coeffs: Distortion coefficients
        
    Returns:
        Undistorted image
    """
    h, w = img.shape[:2]
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
        camera_matrix, dist_coeffs, (w, h), 1, (w, h)
    )
    
    # Undistort
    dst = cv2.undistort(img, camera_matrix, dist_coeffs, None, new_camera_matrix)

    # Crop the image
    x, y, w, h = roi
    if roi:
        dst = dst[y:y+h, x:x+w]
    
    return dst


# Example usage
if __name__ == '__main__':
    try:
        # Calibrate using checkerboard images
        # Note: You need to have calibration images in the specified path
        camera_matrix, dist_coeffs, _, _ = calibrate_camera(
            'calibration_images/*.jpg',  # Change to your actual path
            checkerboard_size=(9, 6),
            show_results=True
        )

        print("\nCamera Matrix:")
        print(camera_matrix)
        print("\nDistortion Coefficients:")
        print(dist_coeffs)

        # Save calibration
        save_calibration(camera_matrix, dist_coeffs, 'camera_calibration.npz')
        
        # Example of how to use the calibration to undistort an image
        # test_img = cv2.imread('test_image.jpg')
        # if test_img is not None:
        #     undistorted_img = undistort_image(test_img, camera_matrix, dist_coeffs)
        #     cv2.imshow('Original', test_img)
        #     cv2.imshow('Undistorted', undistorted_img)
        #     cv2.waitKey(0)
        #     cv2.destroyAllWindows()

    except ValueError as e:
        print(f"Calibration error: {e}")
    except Exception as e:
        print(f"Unexpected error during calibration: {e}")