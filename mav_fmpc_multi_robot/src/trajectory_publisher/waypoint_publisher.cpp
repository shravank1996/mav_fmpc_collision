#include <fstream>
#include <iostream>

#include <Eigen/Core>
#include <ros/ros.h>
#include <asl_msgs/Trajectory.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "waypoint_publisher");
  ros::NodeHandle nh;
  ros::Publisher trajectory_pub =
      nh.advertise<asl_msgs::Trajectory>("command/trajectory", 10);

  ROS_INFO("Started waypoint_publisher.");

  ros::V_string args;
  ros::removeROSArgs(argc, argv, args);

  double delay;

  if (args.size() == 5) {
    delay = 1.0;
  } else if (args.size() == 6) {
    delay = std::stof(args.at(5));
  } else {
    ROS_ERROR("Usage: waypoint_publisher <x> <y> <z> <yaw_deg> [<delay>]\n");
    return -1;
  }

  asl_msgs::Trajectory pose_msg;

  Eigen::Vector3d desired_position(std::stof(args.at(1)), std::stof(args.at(2)),
                                   std::stof(args.at(3)));

pose_msg.header.stamp = ros::Time::now();

asl_msgs::TrajectoryPoint msg;

geometry_msgs::Pose pose;
  pose.position.x=desired_position(0);
  pose.position.y=desired_position(1);
  pose.position.z=desired_position(2);
geometry_msgs::Vector3 dummy ;
dummy.x=0;
dummy.y=0;
dummy.z=0;
msg.pose.push_back(pose);
msg.velocity.push_back(dummy);
msg.acceleration.push_back(dummy);
msg.jerk.push_back(dummy);
msg.snap.push_back(dummy);
double pi=3.1416 ;

msg.heading.push_back(std::stof(args.at(4)) * pi/180);

msg.heading_rate.push_back(0);
msg.heading_acceleration.push_back(0);
//msg.time_from_start(delay);

pose_msg.points.push_back(msg);

  // Wait for some time to create the ros publisher.
  ros::Duration(delay).sleep();

  while (trajectory_pub.getNumSubscribers() == 0 && ros::ok()) {
    ROS_INFO("There is no subscriber available, trying again in 1 second.");
    ros::Duration(1.0).sleep();
  }

  ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",
           nh.getNamespace().c_str(),
           desired_position.x(),
           desired_position.y(),
           desired_position.z());

  trajectory_pub.publish(pose_msg);

  ros::spinOnce();
  ros::shutdown();
	
  return 0;
}
