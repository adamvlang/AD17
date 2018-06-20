#!/usr/bin/env python

import rospy
import roslib
import sys
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from sensor_msgs.msg import Image

topic = "raw_camera" 
frameRate = 4

def camera_publish():
	publisher = rospy.Publisher(topic, Image, queue_size= 1)
        print cv2.__version__
	bridge = CvBridge()
	rospy.init_node('camera', anonymous=True)
        rate = rospy.Rate(frameRate)
	print('Hello from print')
	rospy.loginfo('Hello')
	cap = cv2.VideoCapture("nvcamerasrc ! video/x-raw(memory:NVMM), width=(int)640, height=(int)360,format=(string)I420, framerate=(fraction)" + str(frameRate) + "/1 ! nvvidconv flip-method=0 ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink")	
	#cap = cv2.VideoCapture("/dev/video0")
	rospy.loginfo(cap)
	rate.sleep()
	print(cap)
	if not cap.isOpened():
		rospy.loginfo("Failed to open cap")
	print("Failed to open cap")

	while not rospy.is_shutdown():
		ret_val, camera_value = cap.read()
		#rospy.loginfo('New Message')
		#rospy.loginfo(camera_value)
		#print(np.squeeze(np.asarray(camera_value)))	

		if ret_val:
			publisher.publish(bridge.cv2_to_imgmsg(camera_value, "bgr8"))
		rate.sleep()

if __name__ == '__main__':
	try:
		camera_publish()
	except rospy.ROSInterruptException:
		pass

