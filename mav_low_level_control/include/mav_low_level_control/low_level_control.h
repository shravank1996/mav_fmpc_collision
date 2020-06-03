#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <unsupported/Eigen/MatrixFunctions>

namespace mav_fmpc_multi_robot
{
class LowLevelControl
{public:
LowLevelControl(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
void initializeParameters();
void setAngularVelocityReference(double p, double q, double r, double thrust)
{
angular_velocity_reference_<<p,q,r ;
thrust_angular_acceleration_(3)= thrust;
}
void computeRotorSpeed(Eigen::VectorXd* rotor_speeds, Eigen::Vector3d angular_velocity, Eigen::Matrix3d angular_velocity_covariance);
void computeAngularAcceleration(Eigen::Vector3d angular_velocity);

bool kalmanPredict(Eigen::Vector3d angular_velocity, Eigen::Matrix3d angular_velocity_covariance);
bool kalmanUpdate(Eigen::Vector3d angular_velocity, Eigen::Matrix3d angular_velocity_covariance, Eigen::Matrix<double, 6, 1> predicted_state);

EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
ros::NodeHandle nh_, private_nh_;

int number_of_rotors_ ;
double sampling_time_;

Eigen::Vector4d thrust_angular_acceleration_ ;
Eigen::Vector3d angular_velocity_reference_ ;
Eigen::Vector3d moment_of_inertia_ ; //(Diagonal matrix)
Eigen::Matrix<double, 3, 1> angular_velocity_ ;
Eigen::Matrix<double, 6, 6> system_noise_ ;
Eigen::Matrix<double, 6, 6> process_noise_ ;
Eigen::Matrix<double, 3, 6> measurement_matrix_ ;

double arm_length_ , motor_thrust_constant_ , drag_constant_ ;
double omega1_gain_ , omega2_gain_ , omega3_gain_ ;

Eigen::Vector3d rotaional_drag_coefficients_ ; 

Eigen::MatrixX4d control_allocation_matrix_ ;


};
}
