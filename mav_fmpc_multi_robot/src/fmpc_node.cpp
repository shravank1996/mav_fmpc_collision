#include <vector>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <mav_fmpc_multi_robot/fmpc_node.h>

namespace mav_fmpc_multi_robot
{
	FlatMPCNode::FlatMPCNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh): nh_(nh),private_nh_(private_nh), fmpc_trajectory_tracker_(nh,private_nh)
	{ 
		state_subscriber_ = nh_.subscribe("state",1, &FlatMPCNode::robotStateCallBack,this);
		trajectory_subscriber_ = nh_.subscribe("command/trajectory",1, &FlatMPCNode::commandTrajectoryCallBack,this);
		received_reference_=false;
		received_first_odometry_=false;
		command_angular_rate_thrust_=nh_.advertise<asl_msgs::AngularRateThrust>("command/angular_rate_thrust", 1);
		states_trajectory_=nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("reference/state_trajectory", 1);
		
		double sampling_time ;
		private_nh_.getParam("controller_sampling_time", sampling_time) ;
		
		mpc_timer_ = nh_.createTimer(ros::Duration(sampling_time),&FlatMPCNode::mpcTimerCallBack,this);
	}
	
    void FlatMPCNode::robotStateCallBack(const asl_msgs::Comms::ConstPtr& msg){
	if(!received_first_odometry_)
	 ROS_INFO_ONCE("FMPC: Received first odometry");
	
	Odometry current_odometry ;
	
	current_odometry.timestamp_ns=msg->header.stamp.toNSec();
	current_odometry.position_ =mav_msgs::vector3FromPointMsg(msg->Pose.pose.position);
	current_odometry.velocity_ =mav_msgs::vector3FromMsg(msg->Twist.twist.linear);
	current_odometry.acceleration_ =mav_msgs::vector3FromMsg(msg->Acceleration.Accel); 
	current_odometry.acceleration_(2)-= 9.81 ;
	
	Eigen::Quaterniond quaternion = mav_msgs::quaternionFromMsg(msg->Pose.pose.orientation);
    current_odometry.yaw_ = mav_msgs::yawFromQuaternion(quaternion);
     
    std::vector<double> covariance;
    covariance.resize(36,0);
    
	for(int i=0;i<3;i++)
	{for(int j=0;j<3;j++)
	  { covariance[i * 6 + j]=msg->Pose.covariance[i * 6 + j];
	    covariance[18 + i * 6 + j]=msg->Twist.covariance[i * 6 + j];
      }
    }
    fmpc_trajectory_tracker_.setOdometry(current_odometry,covariance);
    received_first_odometry_=true ;
  }
  
  void FlatMPCNode::commandTrajectoryCallBack(const asl_msgs::Trajectory::ConstPtr& trajectory){
	  if(!received_reference_)
	  ROS_INFO_ONCE("FMPC: Received trajectory to track");
	  
	  ROS_INFO("FMPC: In Trajectroy CallBack");
	  if(trajectory->points.empty())
	  {received_reference_=false;
	   return ;
      }
	  mav_msgs::EigenTrajectoryPointDeque reference_trajectory ;
	  std::cout<<"Trajectory to call is"<<trajectory->points[0].pose[0].position.x;
	  asl_msgs::eigenTrajectoryPointDequeFromTrajectory(*trajectory,&reference_trajectory);
	  fmpc_trajectory_tracker_.evaluateReferenceTrajectory(reference_trajectory);
	  received_reference_=true;  
	  ROS_INFO("FMPC: Completed Trajectory CallBack");
}
  
  void FlatMPCNode::setAngularRateThrust(){
	  	  if(!received_reference_ || !received_first_odometry_)
	  	  { ROS_INFO_ONCE("FMPC: No trajectory to track");
	  	   return;
	      }
	Eigen::VectorXd angular_rate_thrust; 
bool status = fmpc_trajectory_tracker_.calculateAngularVelocityThrust(&angular_rate_thrust);
	
	if(status)
	{asl_msgs::AngularRateThrust command_angular_rate_thrust;
	command_angular_rate_thrust.omega_1 = angular_rate_thrust(0);
	command_angular_rate_thrust.omega_2 = angular_rate_thrust(1);
	command_angular_rate_thrust.omega_3 = angular_rate_thrust(2);
	command_angular_rate_thrust.thrust = angular_rate_thrust(3);
	command_angular_rate_thrust.header.stamp=ros::Time::now();

	command_angular_rate_thrust_.publish(command_angular_rate_thrust);
	fmpc_trajectory_tracker_.updateStateTrajectory();
    }
    
	fmpc_trajectory_tracker_.preparationStep();
}

void FlatMPCNode::publishStateTrajectory(){
	
	if(states_trajectory_.getNumSubscribers()>0)
	{mav_msgs::EigenTrajectoryPointDeque state_trajectory ;
	fmpc_trajectory_tracker_.getStateTrajectory(&state_trajectory);
	
	trajectory_msgs::MultiDOFJointTrajectory trajectory;
	trajectory.header.frame_id="odom";
	trajectory.header.stamp = ros::Time::now();
	trajectory.points.clear();
	mav_msgs::msgMultiDofJointTrajectoryFromEigen(state_trajectory, &trajectory);
	states_trajectory_.publish(trajectory);
    }
}

void FlatMPCNode::mpcTimerCallBack(const ros::TimerEvent& event)
{
	setAngularRateThrust();
	publishStateTrajectory();
}

}//end mav_fmpc_multi_robot

int main(int argc, char** argv)
{
  ros::init(argc, argv, "FMPCNode");

  ros::NodeHandle nh, private_nh("~");
  
  mav_fmpc_multi_robot::FlatMPCNode fmpc_trajectory_tracker_node(nh,private_nh) ;
  
	ros::spin();
	
  return 0;
}
