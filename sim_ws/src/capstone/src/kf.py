#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32, Float32MultiArray, String
from swc_msgs.msg import Gps
from sensor_msgs.msg import Imu
from math import sin, cos, degrees
import time
import numpy as np
from getpass import getuser
from datetime import datetime

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
lat_to_m = 110944.33
lon_to_m = 91058.93
# - IMU '/sim/imu'
cur_hdg = None # radians (0=north, increasing CW)
yaw_rate = None
# - LIDAR '/sim/scan'
# - Camera '/sim/image/compressed'
# - Velocity '/sim/velocity'
cur_vel = [0,0,0] # [xdot, ydot, v]
# - Bumper '/sim/bumper'


## Kalman Filter variables
# State is just x, y, xdot, ydot (initial) (4D column vector)
X = np.transpose(np.array([0,0,0,0]))
# State transition matrix (static) (4x4)
F = np.array([1,0,timer_period,0],[0,1,0,timer_period],[0,0,1,0],[0,0,0,1])
F_trans = np.transpose(F)
# Covariance Matrix (initial) (4x4)
P = np.array([25,0,0,0],[0,25,0,0],[0,0,1/4,0],[0,0,0,1/2])
# Measurements (initial) (4D column vector)
Z = np.transpose(np.array([0,0,0,0]))
# Observation Matrix (static) (4x4)
H = np.array([1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1])
# Process noise, Q #TODO (4x4)
Q = np.array([1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1])
# Measurement Uncertainty, R #TODO (4x4?)
R = np.array([1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1])

## KF equations in matrix form
# state extrapolation: X(n+1) = F*X(n) + w (process noise?)
# covariance extrapolation: P(n+1) = F*P(n)*F^T + Q
# state update: X(n+1) = X(n) + K(n)*(Z(n)-H*X(n))
# covariance update: P = (I-K*H)*P*(I-K*H)^T + K*R*K^T
# kalman gain: K = P*H^T*(H*P*H^T+R)^{-1}
## or use the optimal kalman gain:
# innovation S = H*P*H^T + R
# optimal kalman gain K = P*H^T*S
# est cov P = (I-K*H)*P
## other equations
# measurement: Z = H*X + v(n) (random noise vector)
# meas uncertainty: R = E(v*v^T)
# process noise uncertainty: Q = E(w*w^T)
# est uncertainty: P = E((x-X)*(x-X)^T)

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
State = None
# collection of measurement values/calculations of state variables
Measurements = None
# collection of predictions for state variables at next timestep
Predictions = None
# store the ground truth for position and velocity [x,y,xdot,ydot,v]
Truth = [0,0,0,0,0]

## Uncertainties. each is a column vector with the same number of elements
meas_uncertainty = None
est_uncertainty = None
kalman_gain = None
process_noise = None

## Publishers
state_pub = None
# store data to be written to file
data_for_file = []
filepath = "/home/"+getuser()+"/capstone-kf-ml/data/"
filename = None
filename_pub = None

## Kalman Filter functions
def initialize():
    global initialized, est_uncertainty, meas_uncertainty, State, process_noise
    ## set estimate uncertainty initial guess
    est_uncertainty = [5.0,5.0,5.0,5.0,5.0,5.0]

    ## set system state initial guess
    State = [0,0,0,0,0,0]

    # set cur_vel starting values
    cur_vel = [0,0,0]

    # set measurement uncertainties and process noise that don't change as it runs
    meas_uncertainty = [5.0, 5.0, 3.0, 3.0, 1.0, 1.0]
    process_noise = [10.0, 10.0, 15.0, 15.0, 10.0, 15.0]

    if cur_gps is not None and start_gps is not None and cur_hdg is not None and cur_vel is not None and yaw_rate is not None:
        ## set initialized flag
        initialized = True
        print("initialized KF")


def predict():
    global Predictions, est_uncertainty
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
    # assume constant for velocities and follow dynamic model for positions
    for i in range(len(est_uncertainty)):
        if i == 0 or i == 1 or i == 4:
            # x, y, or yaw.
            # update using dynamic model and velocity est uncertainties
            est_uncertainty[i] += timer_period**2 * est_uncertainty[i+1]
        else: #i == 2 or i == 3 or i == 5:
            # x-vel, y-vel, or yaw rate. 
            # constant velocity model, so do nothing
            pass

    #print_state()

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
        Measurements[0] = (cur_gps.latitude - start_gps.latitude) * lat_to_m
        Measurements[1] = (cur_gps.longitude - start_gps.longitude) * lon_to_m
    # store x,y velocity. need to use heading to do so
    if cur_vel is not None and cur_hdg is not None:
        Measurements[2] = cur_vel[2] * cos(cur_hdg) #cur_hdg[0]
        Measurements[3] = -cur_vel[2] * sin(cur_hdg) #cur_vel[1]
    # store heading from IMU calculated in localization_node
    if cur_hdg is not None:
        Measurements[4] = cur_hdg
    # store yaw rate from IMU
    if yaw_rate is not None:
        Measurements[5] = yaw_rate

    #print("Measurements:", Measurements)
    #print_innovation()

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
        save_to_file()

## print State to the console in a readable format
def print_state():
    line = "State: x=" + "{:.2f}".format(State[0]) + ", y=" + "{:.2f}".format(State[1]) \
        + ", x-vel=" + "{:.2f}".format(State[2]) + ", y-vel=" + "{:.2f}".format(State[3]) \
        + ", hdg=" + "{:.2f}".format(degrees(State[4])) + ", yaw-rate=" + "{:.2f}".format(degrees(State[5]))
    print(line)
## print innovation to the console in a readable format
def print_innovation():
    line = "Innovation: x=" + "{:.2f}".format(State[0]-Measurements[0]) + ", y=" + "{:.2f}".format(State[1]-Measurements[1]) \
        + ", x-vel=" + "{:.2f}".format(State[2]-Measurements[2]) + ", y-vel=" + "{:.2f}".format(State[3]-Measurements[3]) \
        + ", hdg=" + "{:.2f}".format(degrees(State[4]-Measurements[4])) + ", yaw-rate=" + "{:.2f}".format(degrees(State[5]-Measurements[5]))
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
def get_component_vels(gps_msg):
    # uses GPS message for convenience. is in m/s
    global cur_vel
    if cur_vel is None:
        return
    cur_vel[0] = gps_msg.latitude
    cur_vel[1] = gps_msg.longitude
def get_cur_vel(vel_msg):
    global cur_vel
    if cur_vel is None:
        return
    cur_vel[2] = vel_msg.data
def get_yaw_rate(imu_msg):
    global yaw_rate
    yaw_rate = imu_msg.angular_velocity.z
# Get truth values for plot comparison and NN training
def get_true_gps(gps_msg):
    global Truth
    if start_gps is None:
        return
    Truth[0] = (gps_msg.latitude - start_gps.latitude) * lat_to_m
    Truth[1] = (gps_msg.longitude - start_gps.longitude) * lon_to_m
def get_true_component_vels(gps_msg):
    # uses the GPS message type for convenience. is in m/s
    global Truth
    Truth[2] = gps_msg.latitude
    Truth[3] = gps_msg.longitude
def get_true_vel(vel_msg):
    global Truth
    Truth[4] = vel_msg.data

## Save data to a file for evaluation
def save_to_file():
    global data_for_file
    if filename is None:
        return
    data_for_file.append(Measurements + Predictions + State + Truth)
    np.savetxt(filepath + filename + ".csv", data_for_file, delimiter=",")

def set_filename():
    global filename
    # use datetime to ensure unique filenames
    dt = datetime.now()
    # filename will specify: (MANUALLY CHANGE THESE WHEN CHANGING SIM SETTINGS)
    #  obstacles (0,1=normal,2=hard)
    obstacles = 0
    #  noise (0,1=reduced,2=realistic)
    noise = 2
    filename = "kf_o" + str(obstacles) + "_n" + str(noise) + "_" + dt.strftime("%Y-%m-%d-%H-%M-%S")
    print("filename is " + filename)
    # publish the filename so our waypoints can be written to a file and referenced if needed
    # fname_msg = String()
    # fname_msg.data = filename
    # cur_time = time.time()
    # while(time.time() - cur_time < 3):
    #     filename_pub.publish(fname_msg)
    # print("sent the filename!")

def main():
    global state_pub, data_for_file, filename_pub
    # initalize the node in ROS
    rospy.init_node('kf_node')
    data_for_file = []
    # create the filename
    #filename_pub = rospy.Publisher("/kf/filename", String, queue_size=1)
    set_filename()

    ## Subscribe to Sensor Values
    # robot's current heading is already published by localization_node from IMU
    rospy.Subscriber("/swc/current_heading", Float32, get_cur_hdg, queue_size=1)
    # subscribe to robot's current GPS position
    rospy.Subscriber("/sim/gps", Gps, get_cur_gps, queue_size=1)
    # subscribe to the robot's current velocity (overall and components)
    rospy.Subscriber("/sim/linear_velocity", Gps, get_component_vels, queue_size=1)
    rospy.Subscriber("/sim/velocity", Float32, get_cur_vel, queue_size=1)
    # subscribe to the IMU to get the angular_velocity
    rospy.Subscriber("/sim/imu", Imu, get_yaw_rate, queue_size=1)
    # subscribe to the Ground Truth (for KF comparison and training)
    rospy.Subscriber("/truth/gps", Gps, get_true_gps, queue_size=1)
    rospy.Subscriber("/truth/linear_velocity", Gps, get_true_component_vels, queue_size=1)
    rospy.Subscriber("/truth/velocity", Float32, get_true_vel, queue_size=1)

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
