#!/usr/bin/env python
import cv2
import numpy as np
import pickle
import rospy
import sys

from birdseye import BirdsEye
from lanefilter import LaneFilter
from curves import Curves
from helpers import show_images, save_image, roi
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from std_msgs.msg import String


calibration_data = pickle.load(open("../include/hiq_racecar/calibration_data.p", "rb" ))

matrix = calibration_data['camera_matrix']
distortion_coef = calibration_data['distortion_coefficient']

source_points = [(360, 450), (50, 700), (1200, 700), (950, 450)]
destination_points = [(320, 0), (320, 720), (960, 720), (960, 0)]

p = { 'sat_thresh': 120, 'light_thresh': 40, 'light_thresh_agr': 205,
      'grad_thresh': (0.7, 1.4), 'mag_thresh': 40, 'x_thresh': 20 }

birdsEye = BirdsEye(source_points, destination_points, matrix, distortion_coef)
laneFilter = LaneFilter(p)
curves = Curves(number_of_windows = 1, margin = 100, minimum_pixels = 50,
                ym_per_pix = 30.0 / 720 , xm_per_pix = 3.7 / 700)

# ROS Publisher
pub_image = rospy.Publisher('/lane_image', Image, queue_size = 10)
pub_values = rospy.Publisher('/lane_values', String, queue_size = 10)


def pipeline(img):
    ground_img = birdsEye.undistort(img)
    binary = laneFilter.apply(ground_img)
    wb = np.logical_and(birdsEye.sky_view(binary), roi(binary)).astype(np.uint8)
    result = curves.fit(wb)
    ground_img_with_projection = birdsEye.project(ground_img, binary,
                                                  result['pixel_left_best_fit_curve'],
                                                  result['pixel_right_best_fit_curve'])

    text_pos = "vehicle position: " + result['vehicle_position_words']
    text_l = "left radius: " + str(np.round(result['left_radius'], 2))
    text_r = " right radius: " + str(np.round(result['right_radius'], 2))
    cv2.putText(ground_img_with_projection, text_l, (20, 40), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0), 2)
    cv2.putText(ground_img_with_projection, text_r, (400, 40), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0), 2)
    cv2.putText(ground_img_with_projection, text_pos, (20, 80), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0), 2)

    return ground_img_with_projection, result['vehicle_position'], result['left_radius'], result['right_radius']


def run_line_detection(image):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image, "bgr8")

    return_image, position, radius_left, radius_right = pipeline(cv_image)

    pub_image.publish(bridge.cv2_to_imgmsg(return_image, "bgr8"))
    pub_values.publish(",".join([str(position), str(radius_left), str(radius_right)]))


def run():
    rospy.init_node('lane_detection', anonymous=True)
    rospy.Subscriber("/raw_camera", Image, run_line_detection)
    rospy.spin()


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
