#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import sys, getopt
sys.path.append('.')
import RTIMU
import os.path
import time
import math

SETTINGS_FILE = "RTIMULib"

#Add supportfunctions here to read the values from the hardware

def imu_publish():
    pub = rospy.Publisher('imu_data', String, queue_size = 10)
    rospy.init_node('imu_pub')
    rate = rospy.Rate(1) # 1Hz
    s = RTIMU.Settings(SETTINGS_FILE)
    imu = RTIMU.RTIMU(s)
    
    imu.setSlerpPower(0.02)
    imu.setGyroEnable(True)
    imu.setAccelEnable(True)
    imu.setCompassEnable(True)
    
    poll_interval = imu.IMUGetPollInterval()
    
    # Get imu data
    acc_x, acc_y, acc_z = imu.getAccel()
    gyro_x, gyro_y, gyro_z = imu.getGyro()
    mag_x, mag_y, mag_z = imu.getCompass()

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
    except rospy.ROSInteruptException:
        pass

