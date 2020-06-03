#include <mav_msgs/default_topics.h>

#include <multi_robot_collision_detection/multi_robot_collision_detection_node.h>

namespace mav_fmpc_multi_robot
{
	MultiRobotCollisionDetectionNode::MultiRobotCollisionDetectionNode(const ros::NodeHandle nh,const ros::NodeHandle private_nh):nh_(nh),private_nh_(private_nh),multi_robot_collision_detection_(nh,private_nh),other_robot_trajectory_spinner_(1,&trajectory_call_back_queue_)
	{
		trajectory_subscriber_ = nh_.subscribe("reference/state_trajectory",1,&MultiRobotCollisionDetectionNode::trajectoryCallBack,this);
	   initializeParams();
	   
	   received_self_trajectory_ = false;
	   
	   other_robot_trajectory_spinner_.start();
	   
	   ros::TimerOptions timer_options(ros::Duration(0.02),boost::bind(&MultiRobotCollisionDetectionNode::detectCollisionsCallBack, this, _1),&trajectory_call_back_queue_);
	   collision_detection_timer_=nh_.createTimer(timer_options);
	}
	
	void MultiRobotCollisionDetectionNode::initializeParams()
	{
		if (!private_nh_.getParam("number_of_robots", number_of_robots_)) {
    ROS_ERROR(
        "robot radius for collision detection is not loaded from ros parameter server");
    abort();
  }
  
  std::string s;
private_nh_.param<std::string>("robot_name",s,"firefly");

  for(int i=1;i<=number_of_robots_ ;i++)
	{	std::string robot_name= s+std::to_string(i);	 	 
		std::string odometry="/" + robot_name + "/command/current_reference";
		other_robot_trajectory_subscriber_.push_back(nh_.subscribe(odometry, 10 ,&MultiRobotCollisionDetectionNode::otherRobotTrajectoryCallBack,this));
    }

   }

void MultiRobotCollisionDetectionNode::trajectoryCallBack(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& trajectory)
{
	ROS_INFO("In Robot Trajectory CallBack");
	mav_msgs::EigenTrajectoryPointDeque self_trajectory;
	mav_msgs::eigenTrajectoryPointDequeFromMsg(*trajectory,&self_trajectory);
	multi_robot_collision_detection_.setRobotTrajectory(self_trajectory);
	
	received_self_trajectory_=true;
}

void MultiRobotCollisionDetectionNode::otherRobotTrajectoryCallBack(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& trajectory)
{
	ROS_INFO("In Other Robot Trajectory CallBack");
	mav_msgs::EigenTrajectoryPointDeque robot_trajectory;
	mav_msgs::eigenTrajectoryPointDequeFromMsg(*trajectory,&robot_trajectory);
	multi_robot_collision_detection_.setOtherRobotTrajectory(robot_trajectory);
}

void MultiRobotCollisionDetectionNode::detectCollisionsCallBack(const ros::TimerEvent& event)
{
	ROS_INFO("Detecting Collisions");
	if(received_self_trajectory_ == false)
	  return;
	  
	multi_robot_collision_detection_.detectRobotCollision();
	multi_robot_collision_detection_.clearOtherTrajectoryQueue();
}

}

void otherRobotTrajectoryCallBack(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& trajectory)
{

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "multi_robot_collision_detection_node");

  ros::NodeHandle nh, private_nh("~");
  
  mav_fmpc_multi_robot::MultiRobotCollisionDetectionNode multi_robot_collision_detection_node(nh,private_nh) ;
    
ros::AsyncSpinner async_spinner(4); //for reading robot trajectories in parallel
async_spinner.start();
//ros::spin();
ros::waitForShutdown();

  return 0;
}
