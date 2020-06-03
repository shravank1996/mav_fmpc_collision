#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <asl_msgs/AngularRateThrust.h>
#include <mav_msgs/Actuators.h>

#include <mav_low_level_control/low_level_control.h>

namespace mav_fmpc_multi_robot
{
class LowLevelControlNode
{
private:
LowLevelControlNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);

void ImuCallBack(const sensor_msgs::Imu::ConstPtr& Accel);

};
}
