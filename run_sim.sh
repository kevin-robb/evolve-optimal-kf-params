#!/bin/bash
# This script will launch the simulator, initiate my ROS code, and plot the result.

# Launch the simulator.
# This assumes you are using the SWC sim version 6.0,
# and that its file has been unzipped and placed in the '~/Simulators' directory.
# You may also need to ensure the launch file is made executable.
./../Simulators/SCR_SWC_20_SIM_6.0_LINUX/SCRSWC20.x86_64 &

# Start my ROS code
source sim_ws/devel/setup.bash
roslaunch capstone kf.launch 

# TODO
# do something with results.txt, which is created in this directory.
# -> maybe make a python file to interpret it.
# -> maybe also read values from the .cfg files if they aren't in results.
# call the R plotting script, and make it able to receive the seed and score as params.
# implement NN and training.
# -> could have the NN able to modify values in the .cfg itself.