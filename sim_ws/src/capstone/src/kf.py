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
X = np.transpose(np.matrix([0,0,0,0])) # current
X_next = np.transpose(np.matrix([0,0,0,0])) # prediction for next
# State transition matrix (static) (4x4)
F = np.matrix([[1,0,timer_period,0],[0,1,0,timer_period],[0,0,1,0],[0,0,0,1]])
F_trans = np.transpose(F)
# Covariance Matrix (initial) (4x4)
P = np.matrix([[25,0,0,0],[0,25,0,0],[0,0,1/4,0],[0,0,0,1/2]]) # current
P_next = np.matrix([[25,0,0,0],[0,25,0,0],[0,0,1/4,0],[0,0,0,1/2]]) # prediction for next
# Measurements (initial) (4D column vector)
Z = np.transpose(np.matrix([0,0,0,0]))
# Observation Matrix (static) (4x4)
H = np.matrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
H_trans = np.transpose(H)
# Process noise, Q #TODO (4x4)
Q = np.matrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
# Measurement Uncertainty, R #TODO (4x4?)
R = np.matrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
# Identity matrix in 4D
I = np.matrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

# init flag
initialized = False

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
    global initialized
    ## set estimate uncertainty initial guess
    # already done above

    ## set system state initial guess
    # already done above

    if filename is not None and cur_gps is not None and start_gps is not None and cur_hdg is not None:
        ## set initialized flag
        initialized = True
        print("initialized KF")

def predict():
    global X_next, P_next
    ## calculate predicted state for next iteration using dynamic model
    # state extrapolation: X(n+1) = F*X(n) + w (process noise?)
    X_next = np.matmul(F,X)

    ## extrapolate the estimate uncertainty
    # covariance extrapolation: P(n+1) = F*P(n)*F^T + Q
    P_next = np.matmul(np.matmul(F,P),F_trans) #+ Q

def measure():
    global Z
    ## input measurement uncertainty
    # assuming constant for now, but #TODO update R
    if cur_gps is None or start_gps is None or cur_hdg is None:
        return
    
    ## input measured values
    Z = np.transpose(np.matrix([
        (cur_gps.latitude - start_gps.latitude) * lat_to_m,
        (cur_gps.longitude - start_gps.longitude) * lon_to_m,
        cur_vel[2] * cos(cur_hdg),
        -cur_vel[2] * sin(cur_hdg)]))

def update():
    global K, X, P
    ## calculate kalman gain
    # innovation S = H*P*H^T + R
    # optimal kalman gain K = P*H^T*S
    S = np.matmul(np.matmul(H,P),H_trans) #+ R
    K = np.matmul(np.matmul(P,H_trans),S)

    ## estimate the current state using the state update equation
    # state update: X(n+1) = X(n) + K*(Z-H*X(n))
    X = X_next + np.matmul(K,(Z-np.matmul(H,X_next)))
    
    ## update the current estimate uncertainty
    # covariance update: P = (I-K*H)*P*(I-K*H)^T + K*R*K^T
    # or P = (I-K*H)*P (simple version)
    P = np.matmul((I-np.matmul(K,H)),P_next)

## Run the KF
def timer_callback(event):
    if not initialized:
        initialize()
    else:
        predict()
        print_state()
        measure()
        #print_innovation()
        update()
        ## publish the state for the robot to use
        state_msg = Float32MultiArray()
        state_msg.data = X.tolist()
        state_pub.publish(state_msg)
        # write data for analysis
        save_to_file()

## print State to the console in a readable format
def print_state():
    state = mat_to_ls(X)
    print(state)
    line = "State: x=" + "{:.2f}".format(state[0]) + ", y=" + "{:.2f}".format(state[1]) \
        + ", x-vel=" + "{:.2f}".format(state[2]) + ", y-vel=" + "{:.2f}".format(state[3])
    print(line)
## print innovation to the console in a readable format
def print_innovation():
    Inn = mat_to_ls((Z-np.matmul(H,X_next)))
    line = "Innovation: x=" + "{:.2f}".format(Inn[0]) \
        + ", y=" + "{:.2f}".format(Inn[1]) \
        + ", xdot=" + "{:.2f}".format(Inn[2]) \
        + ", ydot=" + "{:.2f}".format(Inn[3])
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

# func to make recording column vector values easier
def mat_to_ls(mat):
    # mat is a numpy matrix with 1 column
    return np.transpose(mat).tolist()[0]

## Save data to a file for evaluation
def save_to_file():
    global data_for_file
    if filename is None:
        return
    data_for_file.append(mat_to_ls(Z) + mat_to_ls(X_next) + mat_to_ls(X) + Truth)
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
