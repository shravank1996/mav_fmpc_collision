#include <Eigen/Eigen>
#include <iostream>
#include <deque>

#include <ros/ros.h>

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>

namespace mav_fmpc_multi_robot
{
typedef std::deque<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > Vector3dDeque;
typedef std::deque<Vector3dDeque,Eigen::aligned_allocator<Vector3dDeque>> DequeVector3dDeque ;

struct Odometry
{Eigen::Vector3d position_ ;
Eigen::Vector3d velocity_ ;
Eigen::Vector3d acceleration_ ;
double yaw_ ;
int64_t timestamp_ns;
EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef std::deque<Odometry,Eigen::aligned_allocator<Odometry>> OdometryDeque ;


struct CollisionStatus
{
char collision_type_ ;
int horizon_stamp_ ;
Vector3dDeque ellipsoid_size_ ;
EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef std::deque<CollisionStatus,Eigen::aligned_allocator<CollisionStatus>> RobotCollisionStatus ;

struct CollisionData
{
int robot_number_ ; //robot position in deque
RobotCollisionStatus robot_collision_status_ ;
};

class MultiRobotCollisionDetection
{public:
MultiRobotCollisionDetection(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);

void initializeParams();

void setRobotTrajectory(mav_msgs::EigenTrajectoryPointDeque self_trajectory);
void setOtherRobotTrajectory(mav_msgs::EigenTrajectoryPointDeque robot_trajectory);
void setOtherRobotTrajectory(DequeVector3dDeque position_trajectory,DequeVector3dDeque velocity_trajectory, DequeVector3dDeque acceleration_trajectory, std::deque<std::deque<double>> yaw_trajectory);
void setOtherRobotTrajectory(OdometryDeque robot_states);

void setOtherRobotSize(std::deque<double> other_robot_radius, std::deque<double> other_robot_height);

void detectRobotCollision();

void clearOtherTrajectoryQueue()
{
other_robots_position_trajectory_.clear();
other_robots_velocity_trajectory_.clear();
other_robots_acceleration_trajectory_.clear();
other_robots_yaw_trajectory_.clear();
}


EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
static constexpr double kGravity = 9.8066;

//Nodehandle
ros::NodeHandle nh_, private_nh_ ;

//collisionCheck parameters
double collision_sphere_radius_ ;
double safety_sphere_radius_ ;

//robot size
double radius_ ;
double height_ ;

//temporal parameter
double discretization_ ;
double collision_check_time_ ;

//Ellipsoid size modifiers
double acceleration_weight_ ; //proportionality constant for thrust 
double drag_offset_ ;

//robot trajectory
Vector3dDeque position_trajectory_, velocity_trajectory_, acceleration_trajectory_ ;
std::deque<double> yaw_trajectory_ ;

//Other Robot trajectory
DequeVector3dDeque other_robots_position_trajectory_ , other_robots_velocity_trajectory_, other_robots_acceleration_trajectory_ ;
std::deque<std::deque<double>> other_robots_yaw_trajectory_ ;

//other Robot Size
std::deque<double> other_robot_radius_ ;
std::deque<double> other_robot_height_ ;

std::deque<CollisionData, Eigen::aligned_allocator<CollisionData>> collision_statuses_ ;

Eigen::Vector3d computeEllipsoidSize(Eigen::Vector3d self_acceleration,double self_yaw, double robot_radius, double robot_height, Eigen::Vector3d robot_acceleration, double robot_yaw);
Eigen::Matrix3d getRotationMatrix(Eigen::Vector3d acceleration, double yaw);

void clearQueue()
{position_trajectory_.clear();
velocity_trajectory_.clear();
acceleration_trajectory_.clear();
yaw_trajectory_.clear();

}

void setTrajectory(mav_msgs::EigenTrajectoryPointDeque trajectory, Vector3dDeque* position_trajectory,Vector3dDeque* velocity_trajectory, Vector3dDeque* acceleration_trajectory, std::deque<double>* yaw_trajectory)
{
for(int i=0;i<trajectory.size();i++)
	{position_trajectory->push_back(trajectory[i].position_W) ;
	 velocity_trajectory->push_back(trajectory[i].velocity_W) ;
	 acceleration_trajectory->push_back(trajectory[i].acceleration_W) ;
	 yaw_trajectory->push_back(trajectory[i].getYaw());
    }
}

};

}
