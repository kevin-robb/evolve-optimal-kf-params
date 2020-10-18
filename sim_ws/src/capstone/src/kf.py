#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import time

# Simulator information: https://github.com/SoonerRobotics/SCR-SWC-20/wiki/Docs
## Robot Characteristics
robot_base_length = 0.4 #meter
robot_base_width = 0.2 #meter
robot_axle_length = 0.26 #meter
robot_axle_separation = 0.3 #meter
# center of robot is the midpoint of the two axles
# the robot is Ackermann steered

## Available Sensors
# - GPS '/sim/gps'
# - IMU '/sim/imu'
cur_hdg = None # radians (0=north, CW)
# - LIDAR '/sim/scan'
# - Camera '/sim/image/compressed'
# - Velocity '/sim/velocity'
# - Bumper '/sim/bumper'


## Kalman Filter variables
# init flag
initialized = False

# collection of state variables' current estimation
# format: 
#   State[0] = current x-position in meters relative to start (forward = +x-axis)
#   State[1] = current y-position in meters relative to start (TODO left = +y-axis??)
#   State[2] = current right wheel angular velocity in rad/s (TODO how to get?)
#   State[3] = current left wheel angular velocity in rad/s (TODO how to get?)
#   TODO or maybe instead of r/l ang vel, just have robot fwd vel
#   State[4] = current heading in meters relative to start
State = []
# collection of measurement values
Measurements = []
# collection of predictions for state variables at next timestep
Predictions = []

## Uncertainties
meas_uncertainty = None
est_uncertainty = None

kalman_gain = None

## Kalman Filter functions
def initialize():
    global initialized, est_uncertainty, State
    # set estimate uncertainty initial guess
    est_uncertainty = 0 # TODO put something better here
    # set system state initial guess
    State = [0,0,0,0,0]
    # set initialized flag
    initialized = True

def predict():
    global State
    # calculate predicted state for next iteration using dynamic model
    # extrapolate the estimate uncertainty

def measure():
    global meas_uncertainty, Measurements
    # input measurement uncertainty
    # input measured values

def update():
    global kalman_gain, State, est_uncertainty
    # calculate kalman gain
    # estimate the current state using the state update equation
    # update the current estimate uncertainty

## Run the KF
def timer_callback(event):
    if not initialized:
        initialize()
    else:
        predict()
        measure()
        update()

## Functions to receive sensor readings. 
## Stay in buffer until measure() is run each clock cycle.
def get_cur_hdg(hdg_msg):
    global cur_hdg
    cur_hdg = hdg_msg.data

def main():
    # initalize the node in ROS
    rospy.init_node('kf_node')

    ## Subscribe to Sensor Values
    # robot's current heading is already published by localization_node from IMU
    rospy.Subscriber("/swc/current_heading", Float32, get_cur_hdg, queue_size=1)
    
    # create timer with a period of 0.1 (10 Hz)
    rospy.Timer(rospy.Duration(0.1), timer_callback)

    # pump callbacks
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
