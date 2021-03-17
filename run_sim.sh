#!/bin/bash
# This script will launch the simulator and initiate my ROS code for the current agent's genome.

# Launch the simulator.
# This assumes you are using the SWC sim version 6.0,
# and that its file has been unzipped and placed in the '~/Simulators' directory.
# You may also need to ensure the launch file is made executable.
# Lastly, when the sim opens, you need to set the "repo root".
# For me, this is "/home/kevinrobb/capstone-kf-ml".
./../Simulators/SCR_SWC_20_SIM_6.0_LINUX/SCRSWC20.x86_64 &
# Start my ROS code
source sim_ws/devel/setup.bash
roslaunch capstone kf.launch $1 $2 &
# Wait for the simulator to finish, then kill ROS.
wait %1
#kill -9 $ROS_PID
killall -9 roscore
killall -9 rosmaster

# the script should now kill itself so we can move on
kill -9 $$