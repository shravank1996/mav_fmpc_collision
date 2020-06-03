#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>


namespace mav_fmpc_multi_robot
{
class PIDLowLevelControl
{
public: PIDLowLevelControl(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
void initializeParameters();

void setRollPitchYawReference(double p, double q, double r, double thrust)
{
angular_reference_<<p,q,r ;
thrust_= thrust;
}
void computeAngularVelocity(Eigen::Matrix<double,4,1>* thrust_angular_velocity);
void setOdometry(mav_msgs::EigenOdometry odometry)
{
odometry_ = odometry;
}

EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:

ros::NodeHandle nh_ ,private_nh_ ;

double omega1_gain_ , omega2_gain_ , omega3_gain_ ;
Eigen::Vector3d angular_reference_ ;
double thrust_ ;

mav_msgs::EigenOdometry odometry_ ;

};

}
