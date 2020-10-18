#!/usr/bin/env python

import rospy
import time

# init flag
initialized = False

# collection of state variables' current estimation
State = []
# collection of measurement values
Measurements = []
# collection of predictions for state variables at next timestep
Predictions = []

## Uncertainties
meas_uncertainty = None
est_uncertainty = None

kalman_gain = None


def initialize():
    global initialized, est_uncertainty, State
    # set estimate uncertainty initial guess
    # set system state initial guess
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

def timer_callback(event):
    if not initialized:
        initialize()
    else:
        predict()
        measure()
        update()

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
