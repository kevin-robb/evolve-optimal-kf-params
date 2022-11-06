#!/usr/bin/env python3

import rospy
import time
from math import degrees
from std_msgs.msg import Float32, Bool
from swc_msgs.msg import Control
from sensor_msgs.msg import LaserScan
from getpass import getuser

control_pub = None
initialized = False
kf_init = False
# ignore obstacles farther away than clearance
clearance = 1.5
# desired turn angle to target
angle_to_target = 0
dist_to_target = 100
# bumper status (true if currently bumped)
bumped = False
almost_bumped = False
turn_dir = 0 # -1 = left, 1 = right, 0 = undecided

# # values set by the EC agent's genome
# shift = None #0.5
# scale = None #5.0
# P = None #0.3

def get_bump_status(bump_status):
    global bumped, turn_dir
    bumped = bump_status
    print("bumped", bumped)
    turn_dir = 0

def get_laserscan(laserscan):
    # max LIDAR range is 0.12 to 10.0 meters. gives 0 if too close or sees nothing.
    # 360 total samples, first one is straight ahead and continuing CCW.
    meas_min = laserscan.range_min # min (non-zero) measured value
    global almost_bumped, turn_dir
    # check front region for obstacles
    for i in range(-20, 20):
        if laserscan.ranges[i] >= meas_min and laserscan.ranges[i] < clearance:
            almost_bumped = True
    # figure out which direction is more obstructed, and turn the other way
    if almost_bumped:
        min_l = 100
        min_r = 100
        for i in range(0, 30):
            if laserscan.ranges[i] >= meas_min and laserscan.ranges[i] < min_l:
                min_l = laserscan.ranges[i]
            if laserscan.ranges[-i] >= meas_min and laserscan.ranges[-i] < min_r:
                min_r = laserscan.ranges[-i]
        # turn away from the closer obstacle (-1 = left, +1 = right)
        turn_dir = -1 if min_l > min_r else 1

def get_turn_angle(turn):
    global angle_to_target, initialized
    angle_to_target = int(degrees(turn.data)) # turn.data is in radians
    initialized = kf_init

def get_dist_to_target(dist):
    global dist_to_target
    dist_to_target = dist.data

def timer_callback(event):
    if not initialized:
        return
    global bumped, turn_dir, almost_bumped
    control_msg = Control()

    # check bumpers first
    if bumped:
        print("action: bumped")
        # decide which way to turn
        if turn_dir == 0:
            if angle_to_target > 0:
                turn_dir = -1
            else:
                turn_dir = 1
        # turn over control to the almost_bumped section
        almost_bumped = True
    # set up priority of actions
    if almost_bumped:
        if bumped:
            bumped = False
        else: #don't print both messages
            print("that was close")
        # backup a bit
        backup_time = 0.4
        last_time = time.time()
        while time.time() - last_time < backup_time:
            control_msg.speed = -2
            control_msg.turn_angle = 0
            control_pub.publish(control_msg)
        # turn according to turn_dir
        turn_time = 0.6
        last_time = time.time()
        while time.time() - last_time < turn_time:
            control_msg.speed = 2
            control_msg.turn_angle = turn_dir * 25
            control_pub.publish(control_msg)
        # reset vars
        almost_bumped = False
        turn_dir = 0
    else:
        #print("no obstructions")
        # no obstacles in the way. pursue angle to next waypoint
        # modulate speed based on angle
        control_msg.speed = 5 * (1 - abs(angle_to_target)/30)**5 + 0.5
        #print("Control Node: Setting speed: " + str(control_msg.speed))
        # reduce oscillations with a P-controller
        P = 0.3
        # if we are very close to a waypoint, don't clamp the angle as much (prevent missing)
        if dist_to_target < 6:
            P = 0.6
        control_msg.turn_angle = angle_to_target * P
        # correct for really big turns (unlikely)
        if control_msg.turn_angle < -90:
            control_msg.turn_angle += 180
            control_msg.speed *= -1
        elif control_msg.turn_angle > 90:
            control_msg.turn_angle -= 180
            control_msg.speed *= -1
        control_pub.publish(control_msg)

def init_kf(ready_msg):
    global kf_init
    #print("Control Node: Received KF_INIT.")
    #read_genome()
    kf_init = True

# # read the genome from the file to set motion shift & scale params.
# def read_genome():
#     global shift, scale, P
#     filepath = "/home/"+getuser()+"/capstone-kf-ml/config/"
#     #filepath = "/home/"+getuser()+"/capstone-kf-ml/sim_ws/src/capstone/src/"
#     file1 = open(filepath + "genome.csv", "r+")
#     line = file1.readlines()[0]
#     file1.close()
#     g = [float(g) for g in line.split(",")]

#     # the first 12 entries are for the KF, so ignore them here.
#     shift, scale, P = g[12], g[13], g[14]
#     #print("Control Node: got genome values " + str(shift) + "," + str(scale) + "," + str(P))

def main():
    global control_pub

    # Initalize our node in ROS
    rospy.init_node('control_node')

    # publish the command messages
    control_pub = rospy.Publisher("/sim/control", Control, queue_size=1)

    # subscribe to nav's commanded turn angle
    rospy.Subscriber("/swc/turn_cmd", Float32, get_turn_angle, queue_size=1)
    # subscribe to the front bump sensor
    rospy.Subscriber("/sim/bumper", Bool, get_bump_status, queue_size=1)
    # subscribe to the LIDAR (updates at 10 Hz)
    rospy.Subscriber("/scan", LaserScan, get_laserscan, queue_size=1)
    # subscribe to the distance to the current target waypoint
    rospy.Subscriber("/swc/dist", Float32, get_dist_to_target, queue_size=1)
    # receive a ready message from the KF so we can start
    rospy.Subscriber("kf/ready", Bool, init_kf, queue_size=1)
    
    # Create a timer that calls timer_callback() with a period of 0.1 (10 Hz)
    rospy.Timer(rospy.Duration(0.1), timer_callback)

    # pump callbacks
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
