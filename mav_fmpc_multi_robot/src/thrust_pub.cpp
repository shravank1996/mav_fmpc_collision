#include <ros/ros.h>
#include <asl_msgs/AngularRateThrust.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "PIDAttitudeControllerNode");

  ros::NodeHandle nh;

  
  ros::Publisher thrust_pub = nh.advertise<asl_msgs::AngularRateThrust>("command/angular_rate_thrust", 1) ;
  
  asl_msgs::AngularRateThrust  thrust_command ;

ros::Rate rate(100);

  
  while (ros::ok())
{
thrust_command.omega_1 = 0.0;
  thrust_command.omega_2 = 0.0;
  thrust_command.omega_3 = 0.0;
  thrust_command.thrust = 36;
  thrust_pub.publish(thrust_command);
rate.sleep();
}

  return 0;
}
