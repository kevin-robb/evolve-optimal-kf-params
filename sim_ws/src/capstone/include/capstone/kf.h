#ifndef SIM_KF_H
#define SIM_KF_H

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>

// unit conversions
#define degreesToRadians(angleDegrees) ((angleDegrees) * M_PI / 180.0)
#define radiansToDegrees(angleRadians) ((angleRadians) * 180.0 / M_PI)

// define any global constants here too

// define the EKF class structure
class EKF
{
    public:
        // constructors and functions to run the whole thing
        EKF();
        void init(Eigen::VectorXd x0);

    private:
        // define variables for the KF process

        // identity matrix
        Eigen::MatrixXd I;

};

#endif