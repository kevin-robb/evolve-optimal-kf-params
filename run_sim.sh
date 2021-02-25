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

# When the ROS code finishes and closes, process the results
python3 functions/read_results.py

# TODO launch the NN to train another round with the latest results

# TODO
# call the R plotting script
# -> make it able to receive the seed and score as params.
# -> make it able to receive a cmd line param to put in the results folder rather than plots.
# implement NN and training.
# -> could have the NN able to modify values in the .cfg itself.