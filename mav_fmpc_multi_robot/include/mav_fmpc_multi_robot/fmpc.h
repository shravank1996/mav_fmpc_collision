#include <Eigen/Eigen>
#include <iostream>
#include <deque>

#include <ros/ros.h>

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>

#include <mav_fmpc_multi_robot/ekf_drag_estimator.h>
#include <mav_fmpc_multi_robot/mpc_queue.h>

namespace mav_fmpc_multi_robot
{
typedef std::deque<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > Vector3dDeque;

struct Odometry
{Eigen::Vector3d position_ ;
Eigen::Vector3d velocity_ ;
Eigen::Vector3d acceleration_ ;
double yaw_ ;
int64_t timestamp_ns;
EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class FlatMPC
{
public: 
FlatMPC(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
~FlatMPC();
void initializeParams();

void setOdometry(Odometry odometry, std::vector<double> state_covariance);
void evaluateReferenceTrajectory(mav_msgs::EigenTrajectoryPointDeque& references);

void preparationStep();
bool calculateAngularVelocityThrust(Eigen::VectorXd* angular_velocity_thrust);
void updateStateTrajectory();

void getStateTrajectory(mav_msgs::EigenTrajectoryPointDeque* state_trajectory) const; 


EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
static constexpr int kStateSize = 10;
static constexpr int kInputSize = 4;
static constexpr int kThrustSize = 6;
static constexpr int kJerkSize = 3;
static constexpr double kGravity = 9.8066;

//ROS nodehandles
ros::NodeHandle nh_, private_nh_ ;

//Physical parameters
double mass_ ;

//Drag coefficients
Eigen::Vector3d translation_drag_coefficients_ ;

//Temporal parameters for MPC
double discretization_ ;
int horizon_ ;
double sampling_time_ ;

//Dynamic limits
double thrust_max_ ;
double thrust_min_ ;
double angular_velocity_limit_ ;

//Reference trajectory
MPCQueue mpc_queue_ ;

//current state
Odometry current_odometry_ ;

//telescopic_matrices
Eigen::MatrixXd A_cap_ ;
Eigen::MatrixXd B_cap_ ;
Eigen::MatrixXd A_hat_ ;
Eigen::MatrixXd B_hat_knot_ ;

//constraint matrices
Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> A_inequality_;
Eigen::VectorXd thrust_limits_max_ ;
Eigen::VectorXd thrust_limits_min_ ;
Eigen::VectorXd jerk_limits_ ;
Eigen::VectorXd yaw_rate_limits_ ;

//Mapping matrices to convert observations
Eigen::SparseMatrix<double> C_thrust_ ;
Eigen::SparseMatrix<double> C_angular_velocity_ ;
Eigen::SparseMatrix<double> D_jerk_ ;
Eigen::SparseMatrix<double> D_jerk_knot_ ; //for knot inputs' constraints
Eigen::SparseMatrix<double> D_U_knot_ ; // Inputs for computing knot points

//Weights
Eigen::Vector3d q_position_ ;
Eigen::Vector3d q_velocity_ ;
Eigen::Vector3d q_acceleration_ ;
Eigen::Vector3d r_jerk_ ;
double q_yaw_ ;
double r_yaw_rate_ ;
Eigen::SparseMatrix<double> Q_cap_, R_cap_ ;


//Extended Kalman Filter for drag estimation
EKFDragEstimator drag_estimator_ ;

//Matrices for QP
Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> H_ ;
Eigen::VectorXd F_ ;

//previous trajectory
Vector3dDeque position_trajectory_ , velocity_trajectory_ , acceleration_trajectory_ ;
std::deque<double> yaw_trajectory_ ;

//computedinputs
Eigen::VectorXd inputs_ ;
Eigen::Vector3d acceleration_ ;
Eigen::Matrix<double, kInputSize,1> u_0_ , u_i_ , u_knot_, applied_input_ ;

//boolean parameters for one time operations
bool received_first_odometry_ ;
bool reveived_first_reference_ ;

void initializeConstraintObservations();
void clearTrajectory()
{
position_trajectory_.clear();
velocity_trajectory_.clear();
acceleration_trajectory_.clear();
yaw_trajectory_.clear();

position_trajectory_.resize(2 * horizon_);
velocity_trajectory_.resize(2 * horizon_);
acceleration_trajectory_.resize(2 * horizon_);
yaw_trajectory_.resize(2 * horizon_,0);
}

Eigen::Matrix3d getRotationMatrix(Eigen::Vector3d acceleration, Eigen::Vector3d velocity, double yaw);
void computeHermiteSimpsonTelescopicMatrix();
void computeTelescopicMatrix();
void lineariseConstraints();

void intializeSolver(); //for setting constraints during the first run


void computeCurrentStateInput();
};

}//End namespace
