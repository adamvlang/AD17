#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu, MagneticField
import sys, getopt
sys.path.append('~/ad17/catkin_ws/src/hiq_racecar/include/hiq_racecar/')
import RTIMU
import os.path
import time
import math

#SETTINGS_FILE = "RTIMULib"

SETTINGS_FILE = "../include/hiq_racecar/RTIMULib"

#Add supportfunctions here to read the values from the hardware

def imu_publish():
    imu_raw = Imu()
    mag_raw = MagneticField()
    pub_imu = rospy.Publisher('imu_data', Imu, queue_size = 1)
    pub_mag = rospy.Publisher('mag_data', MagneticField, queue_size = 1)
    rospy.init_node('imu_pub')
    rate = rospy.Rate(10) # 1 Hz
    print("Using settings file " + SETTINGS_FILE + ".ini")
    
    if not os.path.exists(SETTINGS_FILE + ".ini"):
        print("Settings file does not exist, will be created")

    s = RTIMU.Settings(SETTINGS_FILE)
    imu = RTIMU.RTIMU(s)

    print("IMU Name: " + imu.IMUName())

    if (not imu.IMUInit()):
        print("IMU Init Failed")
        sys.exit(1)
    else:
        print("IMU Init Succeeded")

    # this is a good time to set any fusion parameters

    imu.setSlerpPower(0.02)
    imu.setGyroEnable(True)
    imu.setAccelEnable(True)
    imu.setCompassEnable(True)

    poll_interval = imu.IMUGetPollInterval()
    print("Recommended Poll Interval: %dmS\n" % poll_interval)   
    
    acc_x, acc_y, acc_z = imu.getAccel()
    gyro_x, gyro_y, gyro_z = imu.getGyro()
    mag_x, mag_y, mag_z = imu.getCompass()

    while not rospy.is_shutdown():        
        # Get imu data
	if imu.IMURead():        
	    acc_x, acc_y, acc_z = imu.getAccel()
            gyro_x, gyro_y, gyro_z = imu.getGyro()
            mag_x, mag_y, mag_z = imu.getCompass()
        	
        imu_raw.linear_acceleration.x = acc_x
        imu_raw.linear_acceleration.y = acc_y
	imu_raw.linear_acceleration.z = acc_z

        imu_raw.angular_velocity.x = gyro_x
        imu_raw.angular_velocity.y = gyro_y
        imu_raw.angular_velocity.z = gyro_z

        mag_raw.magnetic_field.x = mag_x
        mag_raw.magnetic_field.y = mag_y
        mag_raw.magnetic_field.z = mag_z

        #Use loginfo for loggin data in order to find errors
        rospy.loginfo('Read imu data')
        #Use pub that was created above to publish the data list, string etc.
        pub_imu.publish(imu_raw)
        pub_mag.publish(mag_raw)
        #Sleep till next time add check to sleep only if que is empty?
        rate.sleep()

#Run funktion
if __name__ == '__main__':
    try:
        imu_publish()
    except rospy.ROSInterruptException:
        pass

