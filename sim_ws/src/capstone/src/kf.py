#!/usr/bin/env python

import rospy, sys
from std_msgs.msg import Float32, Float32MultiArray, Bool
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
P = None #np.matrix([[0.25,0,0,0],[0,0.25,0,0],[0,0,1/4,0],[0,0,0,1/2]]) # current
P_next = None #np.matrix([[0.25,0,0,0],[0,0.25,0,0],[0,0,1/4,0],[0,0,0,1/2]]) # prediction for next
# Measurements (initial) (4D column vector)
Z = np.transpose(np.matrix([0,0,0,0]))
# Observation Matrix (static) (4x4)
H = np.matrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
H_trans = np.transpose(H)
# Process noise, Q (4x4)
Q = None #np.matrix([[0.01,0,0,0],[0,0.01,0,0],[0,0,0.01,0],[0,0,0,0.01]])
# Measurement Uncertainty, R (4x4)
R = None #np.matrix([[0.01,0,0,0],[0,0.01,0,0],[0,0,0.01,0],[0,0,0,0.01]])
# Identity matrix in 4D
I = np.matrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

# init flag
initialized = False

# store the ground truth for position and velocity [x,y,xdot,ydot,v]
Truth = [0,0,0,0,0]

## Publishers
ready_pub = None
state_pub = None
# store data to be written to file
data_for_file = []
filepath = None
filename = None

## Kalman Filter functions
def initialize():
    global initialized
    ## set estimate uncertainty initial guess
    # already done above

    ## set system state initial guess
    # already done above

    # read in the genome we are using
    read_genome()

    if filename is not None and cur_gps is not None and start_gps is not None and cur_hdg is not None:
        ## set initialized flag
        initialized = True
        ready_msg = Bool()
        ready_msg.data = True
        ready_pub.publish(ready_msg)
        print("initialized KF")

# read the genome from the file to set KF params.
def read_genome():
    global P, P_next, Q, R
    filepath = "/home/"+getuser()+"/capstone-kf-ml/config/"
    #filepath = "/home/"+getuser()+"/capstone-kf-ml/sim_ws/src/capstone/src/"
    file1 = open(filepath + "genome.csv", "r+")
    line = file1.readlines()[0]
    g = [float(g) for g in line.split(",")]

    # set everything that depends on the genome.
    P = np.matrix([[g[0],0,0,0],[0,g[1],0,0],[0,0,g[2],0],[0,0,0,g[3]]])
    P_next = P
    Q = np.matrix([[g[4],0,0,0],[0,g[5],0,0],[0,0,g[6],0],[0,0,0,g[7]]])
    R = np.matrix([[g[8],0,0,0],[0,g[9],0,0],[0,0,g[10],0],[0,0,0,g[11]]])

def predict():
    global X_next, P_next
    ## calculate predicted state for next iteration using dynamic model
    # state extrapolation: X(n+1) = F*X(n) + w (process noise?)
    X_next = np.matmul(F,X)

    ## extrapolate the estimate uncertainty
    # covariance extrapolation: P(n+1) = F*P(n)*F^T + Q
    P_next = np.matmul(np.matmul(F,P),F_trans) + Q

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
    #S = np.matmul(np.matmul(H,P),H_trans) #+ R
    S = P + R# since H is the identity and we neglect R for now
    #K = np.matmul(np.matmul(P,H_trans),S)
    K = np.matmul(P,S) # since H is the identity
    # (P is diagonal, so it is not invertible)

    ## estimate the current state using the state update equation
    # state update: X(n+1) = X(n) + K*(Z-H*X(n))
    X = X_next + np.matmul(K,(Z-np.matmul(H,X_next)))
    
    ## update the current estimate uncertainty
    # covariance update: P = (I-K*H)*P*(I-K*H)^T + K*R*K^T
    # or P = (I-K*H)*P (simple version)
    #P = np.matmul((I-np.matmul(K,H)),P_next)
    P = np.matmul(I-K,P_next) # since H is the identity

## Run the KF
def timer_callback(event):
    if not initialized:
        initialize()
    else:
        predict()
        #print_state("Prediction", X_next)
        measure()
        #print_state("Measurement", Z)
        #print_innovation()
        update()
        #print(K)
        #print_state("State", X)
        #print(P)
        ## publish the state for the robot to use
        state_msg = Float32MultiArray()
        #state_msg.data = X.tolist()
        state_msg.data = mat_to_ls(X)
        state_pub.publish(state_msg)
        # write data for analysis
        save_to_file()

## print state vector to the console in a readable format
def print_state(name, vector):
    state = mat_to_ls(vector)
    #print(state)
    line = name + ": x=" + "{:.2f}".format(state[0]) + ", y=" + "{:.2f}".format(state[1]) \
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
    data_for_file.append(mat_to_ls(Z) + mat_to_ls(X_next) + mat_to_ls(X) + Truth + [cur_hdg])
    np.savetxt(filepath + filename + ".csv", data_for_file, delimiter=",")

# read destination directory & filename from config.
def set_filename():
    global filepath, filename
    path = "/home/"+getuser()+"/capstone-kf-ml/config/"
    file1 = open(path + "kf_data_destination.csv", "r+")
    line = file1.readlines()[0].split(",")
    filepath = "/home/"+getuser()+"/capstone-kf-ml/" + line[0]
    if line[1] == "default":
        dt = datetime.now()
        filename = "kf_" + dt.strftime("%Y-%m-%d-%H-%M-%S")
    else:
        filename = line[1]
    print("filepath is " + filepath + filename)


def main():
    global state_pub, data_for_file, ready_pub
    # initalize the node in ROS
    rospy.init_node('kf_node')
    data_for_file = []

    # use config data to set the filepath for data output
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
    # publish a go-ahead signal for the control node to have it wait til the KF is ready
    ready_pub = rospy.Publisher("/kf/ready", Bool, queue_size=1)
    
    # create timer with a period of 0.1 (10 Hz)
    rospy.Timer(rospy.Duration(timer_period), timer_callback)

    # pump callbacks
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
