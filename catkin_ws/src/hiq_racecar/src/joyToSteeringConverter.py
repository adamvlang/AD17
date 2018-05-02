#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy

pub = rospy.Publisher('/steering_values', String, queue_size = 10)

def extract_steering_values(joyData):
    left_right = -100*joyData.axes[0]
    up_down = 50*(1 - joyData.axes[4])/2 # 50 should be 100 for full speed
    steering_values_string = ",".join([str(left_right), str(up_down)])
    
    # Debugging 
    #rospy.loginfo(steering_values_string)
    pub.publish(steering_values_string)

def joy_to_steering_converter():
    rospy.init_node('joyToSteeringConverter', anonymous=True)
    rospy.Subscriber('/joy', Joy, extract_steering_values)
    rospy.spin()


if __name__ == '__main__':
    try:
        joy_to_steering_converter()
    except rospy.ROSInterruptException:
        pass
