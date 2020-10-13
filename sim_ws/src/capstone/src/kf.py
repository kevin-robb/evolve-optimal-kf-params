#!/usr/bin/env python

import rospy
import time

initialized = False

def timer_callback(event):
    global initialized
    initialized = True

def main():
    # initalize the node in ROS
    rospy.init_node('kf_node')
    
    # create timer with a period of 0.1 (10 Hz)
    rospy.Timer(rospy.Duration(0.1), timer_callback)

    # pump callbacks
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
