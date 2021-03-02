#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from swc_msgs.msg import Gps
from time import time, sleep

# We will close the program if we make it a certain amount 
# of time without receiving a message from the simulator.
last_time_seen = None
TIME_LIMIT = 1
initialized = False

def receive_gps(gps_msg):
    global last_time_seen
    # advance last_time_seen to remove the gap
    last_time_seen = time()
    sleep(0.5)

def timer_callback(event):
    # check whether we need to close the program
    if initialized and time() - last_time_seen > TIME_LIMIT:
        rospy.signal_shutdown("Made it " + str(TIME_LIMIT) + " seconds without receiving GPS. Shutting down ROS.")

def start_checking(msg):
    global initialized
    initialized = True

def main():
    global last_time_seen
    # initalize the node in ROS.
    rospy.init_node('shutdown_node', disable_signals=True)

    # start the timer.
    last_time_seen = time()

    # wait until the localization_node has initialized to start
    rospy.Subscriber("/swc/init", Bool, start_checking)
    # We know the simulator has closed when we no longer receive GPS signals.
    rospy.Subscriber("/truth/gps", Gps, receive_gps, queue_size=1)

    # create timer with a period of 0.5 seconds.
    rospy.Timer(rospy.Duration(0.5), timer_callback)

    # pump callbacks
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
