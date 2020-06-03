#include <iostream>
#include <deque>
#include <Eigen/Eigen>

#include <ros/ros.h>

#include <asl_msgs/AngularRateThrust.h>
#include <asl_msgs/CommsFull.h>
#include <mav_msgs/conversions.h>
#include <asl_msgs/conversions.h>
#include <asl_msgs/Trajectory.h>

#include <mav_fmpc_multi_robot/fmpc.h>

namespace mav_fmpc_multi_robot
{class FlatMPCNode
{public:
FlatMPCNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);

//ros subscribers callbacks
void robotStateCallBack(const asl_msgs::Comms::ConstPtr& msg);
void commandTrajectoryCallBack(const asl_msgs::Trajectory::ConstPtr& trajectory);

void setAngularRateThrust();
void publishStateTrajectory();

void mpcTimerCallBack(const ros::TimerEvent& event);

EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
ros::NodeHandle nh_, private_nh_ ;

FlatMPC fmpc_trajectory_tracker_ ;

//ros subscribers
ros::Subscriber state_subscriber_ ;
ros::Subscriber trajectory_subscriber_ ;

//ros publishers
ros::Publisher command_angular_rate_thrust_ ;
ros::Publisher states_trajectory_ ;

bool received_reference_ ;
bool received_first_odometry_ ;

ros::Timer mpc_timer_;

};

}//mav_fmpc_multi_robot

