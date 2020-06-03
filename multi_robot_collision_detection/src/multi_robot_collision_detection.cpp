#include <multi_robot_collision_detection/multi_robot_collision_detection.h>
#include <unsupported/Eigen/MatrixFunctions>

namespace mav_fmpc_multi_robot
{
MultiRobotCollisionDetection::MultiRobotCollisionDetection(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh):nh_(nh),private_nh_(private_nh)
{
	clearQueue();
	clearOtherTrajectoryQueue();
	initializeParams();
	collision_statuses_.clear();
}

void MultiRobotCollisionDetection::initializeParams()
{
	if (!private_nh_.getParam("radius", radius_)) {
    ROS_ERROR(
        "robot radius for collision detection is not loaded from ros parameter server");
    abort();
  }
  
  if (!private_nh_.getParam("height", height_)) {
    ROS_ERROR(
        "robot height for collision detection is not loaded from ros parameter server");
    abort();
  }
  
  if (!private_nh_.getParam("discretization", discretization_)) {
    ROS_ERROR(
        "discretization for collision detection is not loaded from ros parameter server");
    abort();
  }
  
  if (!private_nh_.getParam("collision_check_time", collision_check_time_)) {
    ROS_ERROR(
        "collision_check_time for collision detection is not loaded from ros parameter server");
    abort();
  }
  
    if (!private_nh_.getParam("acceleration_weight", acceleration_weight_)) {
    ROS_ERROR(
        "acceleration_weight for collision detection is not loaded from ros parameter server");
    abort();
  }
  
  if (!private_nh_.getParam("drag_offset", drag_offset_)) {
    ROS_ERROR(
        "collision_check_time for collision detection is not loaded from ros parameter server");
    abort();
  }
  
  if (!private_nh_.getParam("collision_sphere_radius", collision_sphere_radius_)) {
    ROS_ERROR(
        "collision_sphere_radius for collision detection is not loaded from ros parameter server");
    abort();
  }
  
  if (!private_nh_.getParam("safety_sphere_radius", safety_sphere_radius_)) {
    ROS_ERROR(
        "safety_sphere_radius for collision detection is not loaded from ros parameter server");
    abort();
  }
}

void MultiRobotCollisionDetection::setRobotTrajectory(mav_msgs::EigenTrajectoryPointDeque self_trajectory)
{clearQueue();
	setTrajectory(self_trajectory, &position_trajectory_,&velocity_trajectory_,&acceleration_trajectory_,&yaw_trajectory_ );
}

void MultiRobotCollisionDetection::setOtherRobotTrajectory(mav_msgs::EigenTrajectoryPointDeque robot_trajectory)
{Vector3dDeque robot_position_trajectory, robot_velocity_trajectory, robot_acceleration_trajectory ;
 std::deque<double> robot_yaw_trajectory;
	setTrajectory(robot_trajectory, &robot_position_trajectory,&robot_velocity_trajectory,&robot_acceleration_trajectory,&robot_yaw_trajectory);
	other_robots_position_trajectory_.push_back(robot_position_trajectory);
	other_robots_velocity_trajectory_.push_back(robot_velocity_trajectory);
	other_robots_acceleration_trajectory_.push_back(robot_acceleration_trajectory);
	other_robots_yaw_trajectory_.push_back(robot_yaw_trajectory);
}

void MultiRobotCollisionDetection::setOtherRobotTrajectory(DequeVector3dDeque position_trajectory,DequeVector3dDeque velocity_trajectory, DequeVector3dDeque acceleration_trajectory, std::deque<std::deque<double>> yaw_trajectory)
{clearOtherTrajectoryQueue();
 other_robots_position_trajectory_.assign(position_trajectory.begin(),position_trajectory.end());
 other_robots_velocity_trajectory_.assign(velocity_trajectory.begin(),velocity_trajectory.end());
 other_robots_acceleration_trajectory_.assign(acceleration_trajectory.begin(),acceleration_trajectory.end());
 other_robots_yaw_trajectory_.assign(yaw_trajectory.begin(),yaw_trajectory.end());
}

void MultiRobotCollisionDetection::setOtherRobotTrajectory(OdometryDeque robot_states)
{clearOtherTrajectoryQueue();
for(int i=0;i<robot_states.size();i++)
 { 
  Vector3dDeque robot_position_trajectory, robot_velocity_trajectory, robot_acceleration_trajectory ;
  std::deque<double> robot_yaw_trajectory;
  
  robot_position_trajectory.push_back(robot_states[i].position_);
  robot_velocity_trajectory.push_back(robot_states[i].velocity_);
  robot_acceleration_trajectory.push_back(robot_states[i].acceleration_);
  robot_yaw_trajectory.push_back(robot_states[i].yaw_);
	
	for(int j=1;j<position_trajectory_.size();j++)
	{Eigen::Vector3d position,velocity ;
	 
	 position = robot_position_trajectory[j-1] + robot_velocity_trajectory[j-1] * discretization_ + 0.5 * robot_states[i].acceleration_ * std::pow(discretization_,2);
	 velocity = robot_velocity_trajectory[j-1] +  robot_states[i].acceleration_ * discretization_;
	 
	 robot_position_trajectory.push_back(position);
     robot_velocity_trajectory.push_back(velocity);
     robot_acceleration_trajectory.push_back(robot_states[i].acceleration_);
     robot_yaw_trajectory.push_back(robot_states[i].yaw_);
	}
	
  other_robots_position_trajectory_.push_back(robot_position_trajectory);
  other_robots_velocity_trajectory_.push_back(robot_velocity_trajectory);
  other_robots_acceleration_trajectory_.push_back(robot_acceleration_trajectory);
  other_robots_yaw_trajectory_.push_back(robot_yaw_trajectory);
 }
}


void MultiRobotCollisionDetection::setOtherRobotSize(std::deque<double> other_robot_radius, std::deque<double> other_robot_height)
{  other_robot_radius_.clear();
   other_robot_height_.clear();
	other_robot_radius_= other_robot_radius;
	other_robot_height_ = other_robot_height;
}

void MultiRobotCollisionDetection::detectRobotCollision()
{   collision_statuses_.clear();
	Eigen::VectorXd position_trajectory;
	position_trajectory.resize(3 * position_trajectory_.size());
	for(int i=0; i<position_trajectory_.size();i++)
	{
		position_trajectory.segment<3>(3 * i)= position_trajectory_.at(i);
	}
	for(int i=0;i<other_robots_position_trajectory_.size();i++) //iterating over number of robots
	{Eigen::VectorXd other_robot_position_trajectory = Eigen::VectorXd::Zero(3 * position_trajectory_.size());
	 CollisionData collision_data;
	 collision_data.robot_number_ = i;
		for(int j=0;j<other_robots_position_trajectory_[i].size();j++)
		other_robot_position_trajectory.segment<3>(3 * i)=other_robots_position_trajectory_.at(i).at(j);
			
       Eigen::MatrixXd dummy = (position_trajectory - other_robot_position_trajectory) * (position_trajectory - other_robot_position_trajectory).transpose();
      for(int j=0;j<dummy.rows()/3;j++)
      {double distance =0;
		  for(int k=0;k<3;k++)
		  distance+=dummy(3 * j +k);
		if(distance <= safety_sphere_radius_ && distance > collision_sphere_radius_)
		{ CollisionStatus current_collision_status;
			current_collision_status.ellipsoid_size_.clear();
			
			current_collision_status.collision_type_='s';
			current_collision_status.horizon_stamp_ =j ;
			Eigen::Vector3d ellipsoid_size = computeEllipsoidSize(acceleration_trajectory_[j], yaw_trajectory_[j],other_robot_radius_[i],other_robot_height_[i],other_robots_acceleration_trajectory_.at(i).at(j),other_robots_yaw_trajectory_.at(i).at(j));
			current_collision_status.ellipsoid_size_.push_back(ellipsoid_size);
			collision_data.robot_collision_status_.push_back(current_collision_status);
		}
		if(distance <= collision_sphere_radius_)
		{  CollisionStatus current_collision_status;
			current_collision_status.ellipsoid_size_.clear();
			
			current_collision_status.collision_type_='c';
			current_collision_status.horizon_stamp_ =j ;
			Eigen::Vector3d ellipsoid_size = computeEllipsoidSize(acceleration_trajectory_[j], yaw_trajectory_[j],other_robot_radius_[i],other_robot_height_[i],other_robots_acceleration_trajectory_.at(i).at(j),other_robots_yaw_trajectory_.at(i).at(j));
			current_collision_status.ellipsoid_size_.push_back(ellipsoid_size);
			
			//If in collision sphere, then apply hard constraint for collision_check_time_ in future
			for(int k=1;std::ceil(collision_check_time_/discretization_);k++)
			{ if(j+k <dummy.rows()/3)
				{ellipsoid_size = computeEllipsoidSize(acceleration_trajectory_[j + k], yaw_trajectory_[j + k],other_robot_radius_[i],other_robot_height_[i],other_robots_acceleration_trajectory_.at(i).at(j + k),other_robots_yaw_trajectory_.at(i).at(j + k));
			     current_collision_status.ellipsoid_size_.push_back(ellipsoid_size);
		        }
		       else
		       break ; 
			}
		 collision_data.robot_collision_status_.push_back(current_collision_status);
		}
	  }
	  
	  collision_statuses_.push_back(collision_data);
	}
}

Eigen::Vector3d MultiRobotCollisionDetection::computeEllipsoidSize(Eigen::Vector3d self_acceleration,double self_yaw, double robot_radius, double robot_height, Eigen::Vector3d robot_acceleration, double robot_yaw)
{ Eigen::Matrix3d rotation_matrix = getRotationMatrix(self_acceleration,self_yaw);
  Eigen::Matrix3d robot_rotation_matrix = getRotationMatrix(robot_acceleration,robot_yaw);
  self_acceleration(2)+=kGravity;
 robot_acceleration(2)+=kGravity;
  
	double self_augmentation = acceleration_weight_ * (self_acceleration.norm() + drag_offset_);
	double robot_augmentation = acceleration_weight_ * (robot_acceleration.norm() + drag_offset_);
	
	Eigen::Vector3d self_size(radius_,radius_, height_ + self_augmentation);
	Eigen::Vector3d robot_size(robot_radius,robot_radius, robot_height + robot_augmentation);
	
	return rotation_matrix * self_size + robot_rotation_matrix * robot_size ;
}

Eigen::Matrix3d MultiRobotCollisionDetection::getRotationMatrix(Eigen::Vector3d acceleration, double yaw)
{
Eigen::Matrix3d rotation_matrix;
Eigen::Vector3d Z_b=acceleration ;
Z_b(2)+=kGravity ;
rotation_matrix.col(2) = Z_b.normalized();

Eigen::Vector3d X_c;
X_c<<std::cos(yaw),std::sin(yaw),0;
Eigen::Vector3d Y_b = Z_b.cross(X_c);
rotation_matrix.col(1)=Y_b.normalized();

rotation_matrix.col(0)=rotation_matrix.col(1).cross(rotation_matrix.col(2));

return rotation_matrix ;
}	

}
