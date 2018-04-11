#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import sys, getopt
sys.path.append('~/ad17/catkin_ws/src/hiq_racecar/include/hiq_racecar/')
import RTIMU
import os.path
import time
import math

SETTINGS_FILE = "RTIMULib"

import sys, getopt
import RTIMU
import os.path
import time
import math

SETTINGS_FILE = "../include/hiq_racecar/RTIMULib"

#Add supportfunctions here to read the values from the hardware

def init_imu():
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
    
    return imu

def imu_publish():
    pub = rospy.Publisher('imu_data', String, queue_size = 10)
    rospy.init_node('imu_pub')
    rate = rospy.Rate(1) # 1Hz
    imu = init_imu()

    while not rospy.is_shutdown():

        #
        #Make magic happen here, read gyros, run functons etc.
        #
        
        # Get imu data
        acc_x, acc_y, acc_z = imu.getAccel()
        gyro_x, gyro_y, gyro_z = imu.getGyro()
        mag_x, mag_y, mag_z = imu.getCompass()
        
        #Use loginfo for loggin data in order to find errors
        rospy.loginfo('Read imu data')

        #Use pub that was created above to publish the data list, string etc.
        pub.publish('Acc: ' + acc_x + acc_y + acc_z + '\n' + 
                    'Gyro: ' + gyro_x + gyro_y + gyro_z + '\n' +
                    'Mag: ' + mag_x + mag_y + mag_z + '\n')

        #Sleep till next time add check to sleep only if que is empty?
        rate.sleep()

#Run funktion
if __name__ == '__main__':
    try:
        imu_publish()
    except rospy.ROSInterruptException:
        pass

