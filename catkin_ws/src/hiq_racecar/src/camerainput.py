#!/usr/bin/env python

import rospy
import sys
import cv2
import numpy as np
from std_msgs.msg import Float32MultiArray as Array
from std_msgs.msg import String
topic = "raw_camera" 


def camera_publish():
	publisher = rospy.Publisher(topic, String, queue_size= 10)
        print cv2.__version__
	rospy.init_node('camera', anonymous=True)
        rate = rospy.Rate(10)
	print('Hello from print')
	rospy.loginfo('Hello')
	cap = cv2.VideoCapture("nvcamerasrc ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720,format=(string)I420, framerate=(fraction)30/1 ! nvvidconv flip-method=0 ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink")	
	#cap = cv2.VideoCapture("/dev/video0")
	rospy.loginfo(cap)
	print(cap)
	if not cap.isOpened():
		rospy.loginfo("Failed to open cap")
		print("Failed to open cap")

	while not rospy.is_shutdown():
		ret_val, camera_value = cap.read()
		rospy.loginfo('New Message')
		rospy.loginfo(camera_value)
		
		if ret_val:
			publisher.publish(np.squeeze(np.asarray(camera_value)))
		rate.sleep()

if __name__ == '__main__':
	try:
		camera_publish()
	except rospy.ROSInterruptException:
		pass

