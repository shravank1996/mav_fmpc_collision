#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <unsupported/Eigen/MatrixFunctions>

namespace mav_fmpc_multi_robot
{
class EKFDragEstimator
{public:
EKFDragEstimator(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);

void resetObserver(Eigen::Vector3d position, Eigen::Vector3d velocity, Eigen::Vector3d drag_coefficients);
void resetObserver(Eigen::Vector3d position, Eigen::Vector3d velocity);

Eigen::Vector3d getEstimatedPosition() const
  {
    if (initialized_)
      return state_.segment(0, 3);
    else
      return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d getEstimatedVelocity() const
  {
    if (initialized_)
      return state_.segment(3, 3);
    else
      return Eigen::Vector3d::Zero();
  }

Eigen::Vector3d getEstimatedDragCoefficients() const
  {
    if (initialized_)
      return drag_coefficients_ ;
    else
      return Eigen::Vector3d::Zero();
  }

void getCurrentState(Eigen::VectorXd* current_state) const;

void feedCurrentState(const Eigen::Matrix<double, 6,1>& state, std::vector<double> state_covariance);

bool updateEstimator(const Eigen::Vector3d& acceleration);


  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  static constexpr int kStateSize = 6;
  static constexpr int kKalmanStateSize = 9;

ros::NodeHandle nh_, private_nh_ ;

double mass_, sampling_time_ ;

//states
Eigen::Matrix<double, kStateSize,1> state_;
Eigen::Vector3d drag_coefficients_ ;
Eigen::Matrix<double, kKalmanStateSize, 1> augumented_state_ ;

Eigen::Matrix<double, kKalmanStateSize, kKalmanStateSize> system_covariance_  ;
Eigen::Matrix<double, kKalmanStateSize, 1> initial_covariance_ ;
Eigen::Matrix<double, kKalmanStateSize,1> process_covariance_ ;

Eigen::SparseMatrix<double> C_ ;
Eigen::Matrix<double, kStateSize,kStateSize> measurement_covariance_ ;
Eigen::Matrix<double, kStateSize,1> measurement_;

bool initialized_ ;

void initialize();
void predictEstimator(const Eigen::Vector3d& acceleration,double dt);

};
}
