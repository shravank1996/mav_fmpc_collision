#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <asl_msgs/AngularRateThrust.h>
#include <mav_msgs/Actuators.h>
#include <Eigen/Eigen>
#include <mav_msgs/default_topics.h>
	

#include <mav_low_level_control/low_level_control.h>

namespace mav_fmpc_multi_robot
{
class LowLevelControlNode
{
public:
LowLevelControlNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh):nh_(nh),
      private_nh_(private_nh),
      odometry_received_(false),
      low_level_control_(nh, private_nh)
{
	Odom_ = nh_.subscribe("command/angular_rate_thrust", 1 , &LowLevelControlNode::OdomCallBack, this);
	accel_ = nh_.subscribe("ground_truth/imu", 1 , &LowLevelControlNode::ImuCallBack, this);
	mav_actuators = nh_.advertise<mav_msgs::Actuators>(mav_msgs::default_topics::COMMAND_ACTUATORS, 1) ;
}      

void ImuCallBack(const sensor_msgs::Imu::ConstPtr& Accel)
{
	if (!odometry_received_)
    return;
    
	Eigen::Vector3d angular_velocity ;
	//Eigen::Matrix3d angular_velocity_covariance ;
	
	
	angular_velocity<<Accel->angular_velocity.x, Accel->angular_velocity.y, Accel->angular_velocity.z ;
	
	
	std::vector<double> temp_covariance_matrix;
	temp_covariance_matrix.clear();
	for(int i=0; i<9; i++)
	temp_covariance_matrix.push_back(Accel->angular_velocity_covariance[i]);
	
	Eigen::Map<Eigen::Matrix3d> angular_velocity_covariance(temp_covariance_matrix.data(), 3, 3);
	
	Eigen::VectorXd rotor_speeds;
	//std::cout<<std::endl<<"rotor speeds debug"<< angular_velocity_covariance<<std::endl ;
	//std::cout<<"angular speeds debug"<< angular_velocity<<std::endl ;
	low_level_control_.computeRotorSpeed(&rotor_speeds, angular_velocity, angular_velocity_covariance);
	
	mav_msgs::Actuators rotor_speeds_msg;
  rotor_speeds_msg.angular_velocities.clear();
  for (int i = 0; i < rotor_speeds.size(); i++)
    rotor_speeds_msg.angular_velocities.push_back(rotor_speeds[i]);
  rotor_speeds_msg.header.stamp = Accel->header.stamp;

  mav_actuators.publish(rotor_speeds_msg); 
}

void OdomCallBack(const asl_msgs::AngularRateThrust::ConstPtr& Odom)
{
	low_level_control_.setAngularVelocityReference(Odom->omega_1,Odom->omega_2, Odom->omega_3, Odom->thrust);
	odometry_received_ = true;
}

private:
LowLevelControl low_level_control_;
ros::Subscriber Odom_ ;
ros::Subscriber accel_ ;
ros::Publisher mav_actuators ;
ros::NodeHandle nh_, private_nh_ ;
bool odometry_received_ ;
};
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "low_level_control");

  ros::NodeHandle nh, private_nh("~");

  mav_fmpc_multi_robot::LowLevelControlNode low_level_control_node(nh, private_nh);
  
ros::Rate rate(100);

while (ros::ok())
{
ros::spinOnce();
rate.sleep();
}

  return 0;
}
