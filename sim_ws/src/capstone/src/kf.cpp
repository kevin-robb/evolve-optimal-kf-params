#include "capstone/ekf.h"

ros::Publisher ekf_pub;

EKF::EKF()
{
    // set everything up and assign values to variables
    
}


void EKF::init(Eigen::VectorXd x0)
{
    // set the initial state and initialize the timer

}


int main(int argc, char **argv)
{
    // initalize the node in ROS
    ros::init(argc, argv, "cpp_robot_control_node");
    ros::NodeHandle node;

    // publish messages to the TODO topic
    ekf_pub = node.advertise<swc_msgs::Control>(node.resolveName("TODO"), 1);

    // create a timer that calls timer_callback() with a period of 0.1 (10 Hz)
    //ros::Timer control_timer = node.createTimer(ros::Duration(0.1), &controlTimerCallback, false);

    // pump callbacks
    ros::spin();

    return 0;
}