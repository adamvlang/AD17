#!/usr/bin/env python
import rospy
from std_msgs.msg import String

#Add supportfunctions here to read the values from the hardware

def imu_publish():
    pub = rospy.Publisher('imu_data', String, queue_size = 10)
    rospy.init_node('imu_pub')
    rate = rospy.Rate(1) # 1Hz

    while not rospy.is_shutdown():

        #
        #Make magic happen here, read gyros, run functons etc.
        #

        #Use loginfo for loggin data in order to find errors
        rospy.loginfo(hello_str)

        #Use pub that was created above to publish the data list, string etc.
        pub.publish(hello_str)

        #Sleep till next time add check to sleep only if que is empty?
        rate.sleep()

#Run funktion
if __name__ == '__main__':
    try:
        imu_publish()
    except rospy.ROSInteruptException:
        pass

