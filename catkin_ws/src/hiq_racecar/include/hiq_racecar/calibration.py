#!/usr/bin/env python

import matplotlib.pyplot as Plt

import cv2
import pickle

from chessboard import ChessBoard
from helpers import show_images, save_image


# Let's initialize 20 chessboards
# note that at instatiation, it finds all chessboard corners and object points

chessboards = []

for n in range(20):
  this_path = 'camera_cal/calibration' + str(n + 1) + '.png'
  chessboard = ChessBoard(i = n, path = this_path, nx = 9, ny = 6)
  chessboards.append(chessboard)

  # We use these corners and object points (and image dimension)
# from all the chessboards to calculate the calibration parameters

points, corners, shape = [], [], chessboards[0].dimensions

for chessboard in chessboards:
  if chessboard.has_corners:
    points.append(chessboard.object_points)
    corners.append(chessboard.corners)

r, matrix, distortion_coef, rv, tv = cv2.calibrateCamera(points, corners, shape, None, None)

# Let's store these camera calibration parameters somewhere else so we can use it later

calibration_data = {
    "camera_matrix": matrix,
    "distortion_coefficient": distortion_coef
}

pickle.dump(calibration_data, open( "calibration_data.p", "wb" ))

# Let's load the camera calibration parameters to each chessboard as additional detail
# If we don't do this, we won't be able to get an undistorted image from that instance

for chessboard in chessboards:
  chessboard.load_undistort_params(camera_matrix = matrix, distortion = distortion_coef)

# Save each image to respective files

for chessboard in chessboards:

    if chessboard.has_corners:
        save_image(chessboard.image_with_corners(), "corners", chessboard.i)

    if chessboard.can_undistort:
        save_image(chessboard.undistorted_image(), "undistortedboard", chessboard.i)

# Visualization

raw_images, images_with_corners, undistorted_images = [], [], []

for chessboard in chessboards:

    raw_images.append(chessboard.image())

    if chessboard.has_corners:
        images_with_corners.append(chessboard.image_with_corners())

    if chessboard.can_undistort:
        undistorted_images.append(chessboard.undistorted_image())

show_images(raw_images, per_row=5, per_col=4, W=10, H=5)
show_images(images_with_corners, per_row=6, per_col=3, W=12, H=4)
show_images(undistorted_images, per_row=5, per_col=4, W=10, H=5)


# Uncomment lines below for larger visualization
# show_images(raw_images, per_row = 3, per_col = 7, W = 15, H = 20)
# show_images(images_with_corners, per_row = 3, per_col = 6, W = 15, H = 18)
# show_images(undistorted_images, per_row = 3, per_col = 7, W = 13, H = 18)
