#include <mav_low_level_control/PID_low_level_control.h>

namespace mav_fmpc_multi_robot
{
	PIDLowLevelControl::PIDLowLevelControl(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh):nh_(nh),private_nh_(private_nh)
	{
		initializeParameters();
	}
	
	void PIDLowLevelControl::initializeParameters()
	{
		if (!private_nh_.getParam("omega1_gain", omega1_gain_)) {
    ROS_ERROR(
        "omega1_gain in PID attitude controller is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("omega2_gain", omega2_gain_)) {
    ROS_ERROR(
        "omega2_gain in PID attitude controller is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("omega3_gain_", omega3_gain_)) {
    ROS_ERROR("omega3_gain in PID attitude controller is not loaded from ros parameter server");
    abort();
  }

  }
  
  void PIDLowLevelControl::computeAngularVelocity(Eigen::Matrix<double,4,1>* thrust_angular_velocity)
  {
  
  Eigen::Vector3d current_rpy;
  odometry_.getEulerAngles(&current_rpy);
  
  double error_roll = current_rpy(0) - angular_reference_(0) ;
  double error_pitch = current_rpy(1) -angular_reference_(1) ;
  double error_r = angular_reference_(2) - odometry_.angular_velocity_B.z() ;
  
  double error_p = 0 - odometry_.angular_velocity_B(0);
  double error_q = 0 - odometry_.angular_velocity_B(1);
	
	*thrust_angular_velocity<<-omega1_gain_ * error_roll + error_p, -omega2_gain_ * error_pitch + error_q, error_r,thrust_; 
  }
}
