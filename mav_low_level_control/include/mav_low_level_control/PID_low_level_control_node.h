#include <ros/ros.h>
#include <Eigen/Eigen>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <asl_msgs/AngularRateThrust.h>
#include <nav_msgs/Odometry.h>

#include <mav_low_level_control/PID_low_level_control.h>

namespace mav_fmpc_multi_robot
{
class PIDLowLevelControlNode
{public:
PIDLowLevelControlNode(const ros::NodeHandle& nh,const ros::NodeHandle& private_nh);

void odomCallBack(const nav_msgs::OdometryConstPtr& odometry_msg);
void CommandRollPitchYawRateThrustCallback(const mav_msgs::RollPitchYawrateThrustConstPtr& roll_pitch_yawrate_thrust_reference);

private:
ros::NodeHandle nh_;
ros::NodeHandle private_nh_;

PIDLowLevelControl low_level_control_ ;

//publishers
ros::Publisher angular_rate_thrust_sub_ ;

// subscribers
ros::Subscriber command_roll_pitch_yawrate_thrust_sub_;
ros::Subscriber odometry_sub_;

bool got_first_attitude_command_ ;

};
}

