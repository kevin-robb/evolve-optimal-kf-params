#!/usr/bin/env python

import rospy
import numpy as np
from getpass import getuser


## Publishers

# store data to be written to file
data_for_file = []
filepath = "/home/"+getuser()+"/capstone-kf-ml/nn/"
# seed will be used to denote which training set
seed = None

# When the KF initializes, it will set the NN the data to put in the file.
# When the run is over, the seed & score need to be grabbed from results.txt in the sim folder.

# The NN needs to be able to tell the KF what to modify in order to train.

def save_to_file():
    global data_for_file
    # TODO figure out what exactly to output to the file
    np.savetxt(filepath + "nn_" + str(seed) + ".csv", data_for_file, delimiter=",")

def main():
    global data_for_file
    # initalize the node in ROS
    rospy.init_node('nn_node')
    data_for_file = []

    # TODO publishers and subscribers

    # pump callbacks
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
