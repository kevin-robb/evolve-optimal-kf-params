#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32, Float32MultiArray
from swc_msgs.msg import Gps
from sensor_msgs.msg import Imu
from math import sin, cos, degrees
import time

# the period the KF runs at (1/frequency)
timer_period = 0.1 #seconds

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
cur_gps = None
got_first_gps = False
start_gps = None
lat_to_m = 110949.14
lon_to_m = 90765.78
# - IMU '/sim/imu'
cur_hdg = None # radians (0=north, increasing CW)
yaw_rate = None
# - LIDAR '/sim/scan'
# - Camera '/sim/image/compressed'
# - Velocity '/sim/velocity'
cur_vel = None
# - Bumper '/sim/bumper'


## Kalman Filter variables
# init flag
initialized = False

# collection of state variables' current estimation
# format: 
#   State[0] = current x-position in meters relative to start (forward = +x-axis)
#   State[1] = current y-position in meters relative to start (TODO left = +y-axis??)
#   State[2] = current x-velocity in m/s
#   State[3] = current y-velocity in m/s
#   State[4] = current heading in radians (0=north, increasing CW)
#   State[5] = current yaw rate (angular velocity about z-axis) of robot in rad/s
State = None #[None, None, None, None, None, None]
# collection of measurement values/calculations of state variables
Measurements = None #[None, None, None, None, None, None]
# collection of predictions for state variables at next timestep
Predictions = None #[None, None, None, None, None, None]

## Uncertainties. each is a column vector with the same number of elements
meas_uncertainty = None #[None, None, None, None, None, None]
est_uncertainty = None #[None, None, None, None, None, None]
kalman_gain = None #[None, None, None, None, None, None]

## Publishers
state_pub = None

## Kalman Filter functions
def initialize():
    global initialized, est_uncertainty, meas_uncertainty, State
    ## set estimate uncertainty initial guess
    est_uncertainty = [5.0,5.0,5.0,5.0,5.0,5.0]

    ## set system state initial guess
    State = [0,0,0,0,0,0]

    # set measurement uncertainties that don't change as it runs
    meas_uncertainty = [5.0, 5.0, 3.0, 3.0, 1.0, 1.0]

    if cur_gps is not None and start_gps is not None and cur_hdg is not None and cur_vel is not None and yaw_rate is not None:
        ## set initialized flag
        initialized = True
        print("initialized KF")


def predict():
    global Predictions
    if Predictions is None:
        Predictions = [0,0,0,0,0,0]
    if State is None:
        return
    ## calculate predicted state for next iteration using dynamic model
    # x <- x + x' * delT
    Predictions[0] = State[0] + State[2] * timer_period
    # y <- y + y' * delT
    Predictions[1] = State[1] + State[3] * timer_period
    # assume constant velocity for simplicity (for now)
    Predictions[2] = State[2]
    Predictions[3] = State[3]
    # theta <- theta + theta' * delT
    Predictions[4] = State[4] + State[5] * timer_period
    # assume constant yaw_rate for simplicity (for now)
    Predictions[5] = State[5]

    ## extrapolate the estimate uncertainty
    # assume constant for now TODO

    print_state()

def measure():
    global meas_uncertainty, Measurements
    ## input measurement uncertainty
    # TODO for now assume meas_uncertainty is constant
    # TODO increase uncertainty as velocity changes, obstacles are 
    #   near, or yaw rate is high.

    if Measurements is None:
        Measurements = [0,0,0,0,0,0]
    ## input measured values
    # store current x,y position from GPS
    if cur_gps is not None and start_gps is not None:
        Measurements[0] = (cur_gps.longitude - start_gps.longitude) * lon_to_m
        Measurements[1] = (cur_gps.latitude - start_gps.latitude) * lat_to_m
    # store x,y velocity. need to use heading to do so
    if cur_vel is not None and cur_hdg is not None:
        Measurements[2] = cur_vel * cos(cur_hdg)
        Measurements[3] = cur_vel * sin(cur_hdg)
    # store heading from IMU calculated in localization_node
    if cur_hdg is not None:
        Measurements[4] = cur_hdg
    # store yaw rate from IMU
    if yaw_rate is not None:
        Measurements[5] = yaw_rate

    #print("Measurements:", Measurements)

def update():
    global kalman_gain, State, est_uncertainty

    ## calculate kalman gain
    if kalman_gain is None:
        kalman_gain = [0,0,0,0,0,0]
    for i in range(len(kalman_gain)):
        kalman_gain[i] = est_uncertainty[i] / (est_uncertainty[i] + meas_uncertainty[i])

    ## estimate the current state using the state update equation
    # make sure everything has been set
    if Measurements is None or Predictions is None:
        return
    if State is None:
        State = [0,0,0,0,0,0]
    for i in range(len(State)):
        State[i] = kalman_gain[i] * Measurements[i] + (1-kalman_gain[i]) * Predictions[i]
    
    ## update the current estimate uncertainty
    for i in range(len(est_uncertainty)):
        est_uncertainty[i] *= (1-kalman_gain[i])
    
    ## publish the state for the robot to use
    state_msg = Float32MultiArray()
    state_msg.data = State
    state_pub.publish(state_msg)

## Run the KF
def timer_callback(event):
    if not initialized:
        initialize()
    else:
        predict()
        measure()
        update()

## print State to the console in a readable format
def print_state():
    line = "State: x=" + "{:.2f}".format(State[0]) + ", y=" + "{:.2f}".format(State[1]) \
        + ", x-vel=" + "{:.2f}".format(State[2]) + ", y-vel=" + "{:.2f}".format(State[3]) \
        + ", hdg=" + "{:.2f}".format(degrees(State[4])) + ", yaw-rate=" + "{:.2f}".format(degrees(State[5]))
    print(line)

## Functions to receive sensor readings. 
## Stay in buffer until measure() is run each clock cycle.
def get_cur_hdg(hdg_msg):
    global cur_hdg
    cur_hdg = hdg_msg.data
def get_cur_gps(gps_msg):
    global cur_gps, start_gps, got_first_gps
    cur_gps = gps_msg
    if not got_first_gps:
        start_gps = gps_msg
        got_first_gps = True
def get_cur_vel(vel_msg):
    global cur_vel
    cur_vel = vel_msg.data
def get_yaw_rate(imu_msg):
    global yaw_rate
    yaw_rate = imu_msg.angular_velocity.z

def main():
    global state_pub
    # initalize the node in ROS
    rospy.init_node('kf_node')

    ## Subscribe to Sensor Values
    # robot's current heading is already published by localization_node from IMU
    rospy.Subscriber("/swc/current_heading", Float32, get_cur_hdg, queue_size=1)
    # subscribe to robot's current GPS position
    rospy.Subscriber("/sim/gps", Gps, get_cur_gps, queue_size=1)
    # subscribe to the robot's current velocity
    rospy.Subscriber("/sim/velocity", Float32, get_cur_vel, queue_size=1)
    # subscrive to the IMU to get the angular_velocity
    rospy.Subscriber("/sim/imu", Imu, get_yaw_rate, queue_size=1)

    # publish the KF state for the localization to use
    state_pub = rospy.Publisher("/kf/state", Float32MultiArray, queue_size=1)
    
    # create timer with a period of 0.1 (10 Hz)
    rospy.Timer(rospy.Duration(timer_period), timer_callback)

    # pump callbacks
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
