#include <iostream>
#include <deque>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>

#include <mav_msgs/conversions.h>

#include <multi_robot_collision_detection/multi_robot_collision_detection.h>

namespace mav_fmpc_multi_robot
{class MultiRobotCollisionDetectionNode
{public:
MultiRobotCollisionDetectionNode(const ros::NodeHandle nh,const ros::NodeHandle private_nh);

void initializeParams();

void trajectoryCallBack(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& trajectory);
void otherRobotTrajectoryCallBack(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& trajectory);

void detectCollisionsCallBack(const ros::TimerEvent& event);

EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
ros::NodeHandle nh_, private_nh_;

//Collision Detection class
MultiRobotCollisionDetection multi_robot_collision_detection_ ;

//Subscribers
ros::Subscriber trajectory_subscriber_ ;
std::deque<ros::Subscriber> other_robot_trajectory_subscriber_ ;

//callBack queue
ros::CallbackQueue trajectory_call_back_queue_ ;

//AsyncSpinner for reading other robot's trajectories
ros::AsyncSpinner other_robot_trajectory_spinner_;

ros::Timer collision_detection_timer_ ;

//Number of robots
int number_of_robots_ ;

//boolean parameters to hold
bool received_self_trajectory_ ; 

};

}
