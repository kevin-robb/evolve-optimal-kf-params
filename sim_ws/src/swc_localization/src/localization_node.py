#!/usr/bin/env python3

import rospy
from math import sqrt
from getpass import getuser
from tf import transformations
from std_msgs.msg import Float32, Float32MultiArray, Bool #, String
from sensor_msgs.msg import Imu
from swc_msgs.msg import Gps
from swc_msgs.srv import Waypoints

# publishers
loc_pub = None
hdg_pub = None
dist_pub = None
# robot's current GPS location
#robot_gps = Gps()
# boundaries. if the robot goes out, kill ROS
lat_bounds = [-15.0, 100.0]
lon_bounds = [-28.0, 28.0]
# waypoints
start_gps = Gps()
bonus_gps = []
goal_gps = Gps()
wp_interpreted = False
filename = None
# keep track of which waypoints have been visited
visited = [False, False, False]
lat_to_m = 110949.14
lon_to_m = 90765.78
error_margin_lat = 1/lat_to_m
error_margin_lon = 1/lon_to_m
## kalman filter integration
# KF state
State = []
# relative position of robot to start (in m), output by the KF
kf_pos = Gps()

def interpret_waypoints(waypoints):
    global start_gps, bonus_gps, goal_gps, visited, wp_interpreted
    # first waypoint is start location
    start_gps = waypoints.waypoints[0]
    # next three waypoints are bonuses. save them in meters relative to start
    bonus_gps.append(make_rel_gps(waypoints.waypoints[1]))
    bonus_gps.append(make_rel_gps(waypoints.waypoints[2]))
    bonus_gps.append(make_rel_gps(waypoints.waypoints[3]))
    # last waypoint is goal location
    goal_gps = make_rel_gps(waypoints.waypoints[4])
    # initialize, no points have been visited yet
    visited = [False, False, False]
    wp_interpreted = True
    print("waypoints interpreted")
    # save waypoints to file
    write_to_file()

# write the waypoints to a file for use in plotting later.
def write_to_file():
    print("Writing waypoints to file")
    filepath = "/home/"+getuser()+"/capstone-kf-ml/config/"
    #filepath = "/home/"+getuser()+"/capstone-kf-ml/sim_ws/src/swc_localization/src/"
    # want to replace the previous waypoints (w=write).
    file1 = open(filepath + "waypoints.csv", "w+")
    # grab the waypoints already turned to meters.
    lines = "x,y\n0.0,0.0\n"
    for bp in bonus_gps:
        lines += str(bp.latitude) + "," + str(bp.longitude) + "\n"
    lines += str(goal_gps.latitude) + "," + str(goal_gps.longitude) + "\n"
    #print("Waypoints: ", lines)
    file1.write(lines)
    file1.close()

def make_rel_gps(global_gps):
    # transform a GPS waypoint from global GPS to meters relative to start
    rel_m = Gps()
    rel_m.longitude = (global_gps.longitude - start_gps.longitude) * lon_to_m
    rel_m.latitude = (global_gps.latitude - start_gps.latitude) * lat_to_m
    return rel_m

def arrived_at_point(point_gps):
    if point_gps.latitude - kf_pos.latitude < error_margin_lat and point_gps.longitude - kf_pos.longitude < error_margin_lon:
        return True
    else:
        return False

def check_in_bounds(robot_pos):
    if float(robot_pos.latitude) < lat_bounds[0]:
        print("SHUTDOWN: OUT OF BOUNDS (lat low)")
    elif float(robot_pos.latitude) > lat_bounds[1]:
        print("SHUTDOWN: OUT OF BOUNDS (lat high)")
    elif float(robot_pos.longitude) < lon_bounds[0]:
        print("SHUTDOWN: OUT OF BOUNDS (lon low)")
    elif float(robot_pos.longitude) > lon_bounds[1]:
        print("SHUTDOWN: OUT OF BOUNDS (lon high)")
    else:
        return True
    # we hit a fail case
    rospy.signal_shutdown("Robot went out of bounds")

def get_kf_state(state_msg):
    global State, kf_pos, visited
    State = state_msg.data
    kf_pos.latitude = State[0]
    kf_pos.longitude = State[1]
    #print("KF Position:[" + str(kf_pos.latitude) + "," + str(kf_pos.longitude) + "]")
    
    # make sure the robot is still in bounds
    check_in_bounds(kf_pos)
    # check all the bonus waypoints to see if visited.
    # make sure the waypoints have been interpreted first.
    if wp_interpreted:
        if not visited[0]:
            if arrived_at_point(bonus_gps[0]):
                visited[0] = True
                print("arrived at point 1")
        if not visited[1]:
            if arrived_at_point(bonus_gps[1]):
                visited[1] = True
                print("arrived at point 2")
        if not visited[2]:
            if arrived_at_point(bonus_gps[2]):
                visited[2] = True
                print("arrived at point 3")
        # make the list of not-yet-visited points' relative positions
        pub_next_pt()

def pub_next_pt():
    # Find and publish relative location of next non-visited point.
    rel = Gps()
    # bonus waypoint 1
    if not visited[0]:
        #print("going for point 1")
        rel.latitude = bonus_gps[0].latitude - kf_pos.latitude
        rel.longitude = bonus_gps[0].longitude - kf_pos.longitude
        loc_pub.publish(rel)
        pub_dist_to_next_pt(bonus_gps[0])
    # bonus waypoint 2
    elif not visited[1]:
        #print("going for point 2")
        rel.latitude = bonus_gps[1].latitude - kf_pos.latitude
        rel.longitude = bonus_gps[1].longitude - kf_pos.longitude
        loc_pub.publish(rel)
        pub_dist_to_next_pt(bonus_gps[1])
    # bonus waypoint 3
    elif not visited[2]:
        #print("going for point 3")
        rel.latitude = bonus_gps[2].latitude - kf_pos.latitude
        rel.longitude = bonus_gps[2].longitude - kf_pos.longitude
        loc_pub.publish(rel)
        pub_dist_to_next_pt(bonus_gps[2])
    # final goal waypoint 
    else:
        #print("going for final goal")
        rel.latitude = goal_gps.latitude - kf_pos.latitude
        rel.longitude = goal_gps.longitude - kf_pos.longitude
        loc_pub.publish(rel)
        pub_dist_to_next_pt(goal_gps)

def pub_dist_to_next_pt(point_gps):
    dist = Float32()
    # convert dist from GPS to meters
    lat_diff = (point_gps.latitude - kf_pos.latitude)
    lon_diff = (point_gps.longitude - kf_pos.longitude)
    dist.data = sqrt(lat_diff**2 + lon_diff**2)
    #print("dist to target:", dist.data)
    dist_pub.publish(dist)
    # used in control_node to allow sharp turns when near the goal to prevent missing it

# similar function to beebotics' reactive_node.py/get_current_heading()
def update_heading(imu_data):
    # returns robot's current heading in radians (0 north, CW)
    orientation = imu_data.orientation
    quaternion = (
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w)
    yaw_rads = transformations.euler_from_quaternion(quaternion)[2]

    current_heading = Float32()
    current_heading.data = -yaw_rads
    hdg_pub.publish(current_heading)

def main():
    global loc_pub, hdg_pub, dist_pub

    # Initalize our node in ROS
    rospy.init_node('localization_node', disable_signals=True)

    # publish target point location relative to robot's current position
    loc_pub = rospy.Publisher("/swc/goal", Gps, queue_size=1)
    # publish robot's current heading
    hdg_pub = rospy.Publisher("/swc/current_heading", Float32, queue_size=1)
    # publish distance to current target waypoint
    dist_pub = rospy.Publisher("/swc/dist", Float32, queue_size=1)

    # subscribe to robot's current GPS position and IMU data
    #rospy.Subscriber("/sim/gps", Gps, update_robot_gps, queue_size=1)
    rospy.Subscriber("/sim/imu", Imu, update_heading, queue_size=1)
    # subscribe to KF State
    rospy.Subscriber("/kf/state", Float32MultiArray, get_kf_state, queue_size=1)

    # Wait for Waypoints service and then request waypoints
    rospy.wait_for_service('/sim/waypoints')
    waypoints = rospy.ServiceProxy('/sim/waypoints', Waypoints)()
    # Create GPS variables of the waypoints
    interpret_waypoints(waypoints)

    # pump callbacks
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass