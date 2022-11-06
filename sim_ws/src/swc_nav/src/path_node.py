#!/usr/bin/env python3

import rospy
from math import atan, pi
from std_msgs.msg import Float32
from swc_msgs.msg import Gps

turn_pub = None
# the robot's current global heading
robot_heading = 0
# angle is with respect to the vertical center line, where 0 = straight up and positive is CW
# should be between -pi and pi (in radians)
desired_heading = 0

def get_current_heading(heading):
    global robot_heading
    robot_heading = heading.data

def get_desired_location(rel_goal_gps):
    global desired_heading
    lat_diff = rel_goal_gps.latitude # lat ~ y (vertical position)
    lon_diff = rel_goal_gps.longitude # lon ~ x (horizontal position)
    # calculate angle, being careful about the sign
    alpha = abs(atan(lon_diff/lat_diff))
    if lat_diff >= 0 and lon_diff >= 0:
        desired_heading = alpha
    elif lat_diff >= 0 and lon_diff < 0:
        desired_heading = -alpha
    elif lat_diff < 0 and lon_diff >= 0:
        desired_heading = pi - alpha
    elif lat_diff < 0 and lon_diff < 0:
        desired_heading = -(pi - alpha)
    # not sure why, but need to reverse sign
    desired_heading *= -1

def timer_callback(event): 
    P = 0.3
    turn_angle = Float32()
    turn_angle.data = (desired_heading - robot_heading) * P
    # normalize to (-pi, pi)
    #turn_angle.data = (turn_angle.data + pi) % (2*pi) - pi
    turn_pub.publish(turn_angle)
    # for now let the controller worry about speed
    #print("desired heading: ", degrees(desired_heading))
    #print("current heading: ", degrees(robot_heading))
    

def main():
    global turn_pub

    # Initalize our node in ROS
    rospy.init_node('localization_node')

    # Create publisher for turn angle
    turn_pub = rospy.Publisher("/swc/turn_cmd", Float32, queue_size=1)

    # Create subscribers to get the desired location and current heading
    rospy.Subscriber("/swc/goal", Gps, get_desired_location, queue_size=1)
    rospy.Subscriber("/swc/current_heading", Float32, get_current_heading, queue_size=1)

    # Create a timer that calls timer_callback() with a period of 0.1 (10 Hz)
    rospy.Timer(rospy.Duration(0.1), timer_callback)

    # pump callbacks
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
