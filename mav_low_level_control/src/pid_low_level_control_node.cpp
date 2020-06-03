#include <mav_low_level_control/PID_low_level_control_node.h>
#include <mav_msgs/default_topics.h>


namespace mav_fmpc_multi_robot
{
	PIDLowLevelControlNode::PIDLowLevelControlNode(const ros::NodeHandle& nh,const ros::NodeHandle& private_nh):nh_(nh),private_nh_(private_nh),low_level_control_(nh,private_nh),got_first_attitude_command_(false)
	{
		command_roll_pitch_yawrate_thrust_sub_ = nh_.subscribe(mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST, 1,&PIDLowLevelControlNode::CommandRollPitchYawRateThrustCallback, this);

  odometry_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,&PIDLowLevelControlNode::odomCallBack, this,ros::TransportHints().tcpNoDelay());
  
  angular_rate_thrust_sub_ = nh_.advertise<asl_msgs::AngularRateThrust>("command/angular_rate_thrust", 1);
	}

void PIDLowLevelControlNode::CommandRollPitchYawRateThrustCallback(const mav_msgs::RollPitchYawrateThrustConstPtr& roll_pitch_yawrate_thrust_reference)
{
	low_level_control_.setRollPitchYawReference(roll_pitch_yawrate_thrust_reference->roll,
                                              roll_pitch_yawrate_thrust_reference->pitch,
                                              roll_pitch_yawrate_thrust_reference->yaw_rate,
                                              roll_pitch_yawrate_thrust_reference->thrust.z);
  got_first_attitude_command_ = true;
}

void PIDLowLevelControlNode::odomCallBack(const nav_msgs::OdometryConstPtr& odometry_msg)
{
	 ROS_INFO_ONCE("PIDAttitudeController got first odometry message.");
	 
	 if (!got_first_attitude_command_)
    return;
    
     mav_msgs::EigenOdometry odometry;
  eigenOdometryFromMsg(*odometry_msg, &odometry);
    
    low_level_control_.setOdometry(odometry);
    
    Eigen::Matrix<double,4,1> thrust_angular_velocity;
    
    low_level_control_.computeAngularVelocity(&thrust_angular_velocity);
    
    asl_msgs::AngularRateThrust rpythrust_;
    rpythrust_.header.stamp = odometry_msg->header.stamp;
    rpythrust_.omega_1=thrust_angular_velocity(0);
    rpythrust_.omega_2=thrust_angular_velocity(1);
    rpythrust_.omega_3=thrust_angular_velocity(2);
    rpythrust_.thrust=thrust_angular_velocity(3);
    
    angular_rate_thrust_sub_.publish(rpythrust_);
}

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "PIDAttitudeControllerNode");

  ros::NodeHandle nh, private_nh("~");

  mav_fmpc_multi_robot::PIDLowLevelControlNode PID_attitude_controller(nh, private_nh);

  ros::spin();

  return 0;
}

