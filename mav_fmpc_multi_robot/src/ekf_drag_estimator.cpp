#include <mav_fmpc_multi_robot/ekf_drag_estimator.h>

namespace mav_fmpc_multi_robot
{
	EKFDragEstimator::EKFDragEstimator(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh):nh_(nh), private_nh_(private_nh)
	{
		initialize();
	}
	
	void EKFDragEstimator::initialize()
	{  std::vector<double> temporary_system_noise, temporary_process_noise,drag_coeffs;
		if (!private_nh_.getParam("process_noise", temporary_process_noise )) {
    ROS_ERROR(
        "process noise for ekf_drag_estimator is not loaded from ros parameter server");
    abort();
  }
  
  if (!private_nh_.getParam("system_noise", temporary_system_noise )) {
    ROS_ERROR(
        "system noise for ekf_drag_estimator is not loaded from ros parameter server");
    abort();
    }
    
    if (!private_nh_.getParam("translation_drag_coefficients",drag_coeffs)) {
    ROS_ERROR(
        "translation_drag_coefficients for ekf_drag_estimator is not loaded from ros parameter server");
    abort();
  }
  
  if (!private_nh_.getParam("mass",mass_)) {
    ROS_ERROR(
        "mass for ekf_drag_estimator is not loaded from ros parameter server");
    abort();
  }
   
  if (!private_nh_.getParam("sampling_time", sampling_time_)) {
    ROS_ERROR("sampling_time for ekf_drag_estimator is not loaded from ros parameter server");
    abort();
  }
  
  Eigen::Map<Eigen::MatrixXd> system_noise(temporary_system_noise.data(), kKalmanStateSize, 1);
  Eigen::Map<Eigen::MatrixXd> process_noise(temporary_process_noise.data(),kKalmanStateSize, 1);
  system_covariance_=  system_noise.asDiagonal();
  initial_covariance_= system_noise;
  process_covariance_= process_noise ;
  drag_coefficients_<<drag_coeffs[0], drag_coeffs[1], drag_coeffs[2] ;
  state_.setZero();
  
  Eigen::Matrix<double,kStateSize, kKalmanStateSize> H;
  H.setZero();
  H.block<kStateSize,kStateSize>(0,0)=Eigen::MatrixXd::Identity(kStateSize,kStateSize);
  C_=H.sparseView();
  
  ROS_INFO("Initialized ekf_drag_estimator");
	
  initialized_=true;
   }
  
  void EKFDragEstimator::predictEstimator(const Eigen::Vector3d& acceleration, double dt){
	  
 	  augumented_state_.segment<kStateSize>(0)= state_ ;
	  augumented_state_.segment<3>(6)= drag_coefficients_ ;
	  
	  Eigen::Matrix<double,kKalmanStateSize, 1> B; //dummy vector to augument discretization
	  
	  Eigen::Vector3d velocity_predict = (acceleration - drag_coefficients_.asDiagonal() * state_.tail(3)) * dt;
	  Eigen::Vector3d position_predict = (0.5 * velocity_predict + augumented_state_.segment<3>(3) ) * dt ;
	  Eigen::Vector3d drag_predict;
	  drag_predict.setZero();
	  
	  B<<position_predict,velocity_predict,drag_predict;
	  augumented_state_= augumented_state_ + B; 
 }
 
 bool EKFDragEstimator::updateEstimator(const Eigen::Vector3d& acceleration){
	 
	 Eigen::Matrix<double, kKalmanStateSize, kKalmanStateSize> A_continuous;
	 A_continuous.setZero();
	 
	 A_continuous.block<3,3>(0,3)= Eigen::MatrixXd::Identity(3,3);
	 A_continuous.block<3,3>(3,3)= -1 * drag_coefficients_.asDiagonal() * Eigen::MatrixXd::Identity(3,3);
	 A_continuous.block<3,3>(3,6)= -1 * state_.tail(3).asDiagonal() * Eigen::MatrixXd::Identity(3,3);
	 
	 Eigen::SparseMatrix<double> A_discrete = (sampling_time_ * A_continuous).exp().sparseView();
	 
	 system_covariance_ = A_discrete * system_covariance_ * A_discrete.transpose();
     system_covariance_.diagonal() += process_covariance_;
	 
	 //predict state
	 predictEstimator(acceleration, sampling_time_);
	 
	 //Innovation matrix
	 Eigen::Matrix<double, kStateSize,kStateSize> innovation = C_ * system_covariance_ * C_.transpose() + measurement_covariance_	;
	 
	 //Kalman gain
	 Eigen::Matrix<double,kKalmanStateSize, kStateSize> kalman_gain = system_covariance_ * C_.transpose() * innovation.inverse();
	 
	 //update states
	 Eigen::Matrix<double,kKalmanStateSize,1>updated_state = augumented_state_ + kalman_gain * (measurement_  - C_ * augumented_state_);
	 
	 //update covariance
	 system_covariance_ = (Eigen::MatrixXd::Identity(kKalmanStateSize,kKalmanStateSize) - kalman_gain * C_ ) * system_covariance_ ;
	 
	 if (updated_state.allFinite() == false) {
    ROS_ERROR("The estimated state in ekf_drag has a non-finite element");
    return false;
  }
  
  state_= updated_state.segment<kStateSize>(0);
  drag_coefficients_ = updated_state.segment<3>(kStateSize);
  
  return true;
}

void EKFDragEstimator::feedCurrentState(const Eigen::Matrix<double, 6,1>& state, std::vector<double> state_covariance){
 measurement_ = state ;
 measurement_covariance_=Eigen::Map<Eigen::Matrix<double, kStateSize,kStateSize, Eigen::RowMajor>>(state_covariance.data(),kStateSize, kStateSize);
}

void EKFDragEstimator::resetObserver(Eigen::Vector3d position, Eigen::Vector3d velocity, Eigen::Vector3d drag_coefficients){
	
	state_.setZero();
	state_.segment<3>(0)=position;
	state_.segment<3>(3)=velocity;
	drag_coefficients_=drag_coefficients;
	system_covariance_ = initial_covariance_.asDiagonal();
}

void EKFDragEstimator::resetObserver(Eigen::Vector3d position, Eigen::Vector3d velocity){
	
	state_.setZero();
	state_.segment<3>(0)=position;
	state_.segment<3>(3)=velocity;
	
	std::vector<double> drag_coeffs;
	
	if (!private_nh_.getParam("linear_drag_coefficients",drag_coeffs)) {
    ROS_ERROR(
        "linear_drag_coefficients for ekf_drag_estimator is not loaded from ros parameter server");
    abort();
  }
  
  drag_coefficients_<<drag_coeffs[0], drag_coeffs[1], drag_coeffs[2] ;
  
	system_covariance_ = initial_covariance_.asDiagonal();
}  

void EKFDragEstimator::getCurrentState(Eigen::VectorXd* current_state) const{
	
	assert(current_state);
    assert(initialized_);
    
    current_state->resize(kKalmanStateSize);
    current_state->segment<kStateSize>(0)= state_;
    current_state->segment<3>(kStateSize)= drag_coefficients_ ;
}

}
