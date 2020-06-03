#include <ros/ros.h>

#include <mav_low_level_control/low_level_control.h>
#include <asl_msgs/AngularRateThrust.h>


namespace mav_fmpc_multi_robot
{
	LowLevelControl::LowLevelControl(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh):nh_(nh),private_nh_(private_nh)
	{
		initializeParameters();
	}
void LowLevelControl::initializeParameters()
	{
		std::vector<double> temporary_allocation_matrix;
		std::vector<double> temporary_system_noise, temporary_process_noise, inertia, drag_ceoffs; 
		if (!private_nh_.getParam("rotor_force_constant", motor_thrust_constant_)) {
    ROS_ERROR(
        "thrust constant for low level controller is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("rotor_moment_constant", drag_constant_)) {
    ROS_ERROR(
        "drag constant for low level controller is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("arm_length", arm_length_)) {
    ROS_ERROR("arm_length for low level controller is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("inertia", inertia)) {
    ROS_ERROR("inertia for low level controller is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("allocation_matrix", temporary_allocation_matrix )) {
    ROS_ERROR(
        "allocation matrix for low level controller is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("n_rotors", number_of_rotors_)) {
    ROS_ERROR("n_rotors for low level controller is not loaded from ros parameter server");
    abort();
  }

  if (temporary_allocation_matrix.size() != number_of_rotors_ * 4) {
    ROS_ERROR("Low level controller: dimension of allocation matrix is wrong.");
    abort();
  }

  if (!private_nh_.getParam("rotaional_drag_coefficients",drag_ceoffs)) {
    ROS_ERROR(
        "rotaional_drag_coefficients for low level controller is not loaded from ros parameter server");
    abort();
  }
  
  if (!private_nh_.getParam("omega1_gain", omega1_gain_)) {
    ROS_ERROR(
        "omega1_gain for low level controller is not loaded from ros parameter server");
    abort();
  }
  
    if (!private_nh_.getParam("omega2_gain", omega2_gain_)) {
    ROS_ERROR(
        "omega2_gain for low level controller is not loaded from ros parameter server");
    abort();
  }
  
    if (!private_nh_.getParam("omega3_gain", omega3_gain_)) {
    ROS_ERROR(
        "omega3_gain for low level controller is not loaded from ros parameter server");
    abort();
  }
  
  if (!private_nh_.getParam("process_noise", temporary_process_noise )) {
    ROS_ERROR(
        "process noise for low level controller is not loaded from ros parameter server");
    abort();
  }
  
  if (!private_nh_.getParam("system_noise", temporary_system_noise )) {
    ROS_ERROR(
        "system noise for low level controller is not loaded from ros parameter server");
    abort();
  }
  
  if (!private_nh_.getParam("sampling_time", sampling_time_)) {
    ROS_ERROR("sampling_time in KF is not loaded from ros parameter server");
    abort();
  }
  
  Eigen::Map<Eigen::MatrixXd> system_noise(temporary_system_noise.data(), 6, 1);
  Eigen::Map<Eigen::MatrixXd> process_noise(temporary_process_noise.data(), 6, 1);
  system_noise_= system_noise.asDiagonal();
  process_noise_= process_noise.asDiagonal();

  measurement_matrix_.setZero();
  measurement_matrix_.block<3,3>(0,0)=Eigen::MatrixXd::Identity(3, 3);
  
  moment_of_inertia_<<inertia[0], inertia[1], inertia[2] ;
  
  rotaional_drag_coefficients_<<drag_ceoffs[0], drag_ceoffs[1], drag_ceoffs[2] ; 
  
  Eigen::Map<Eigen::MatrixXd> allocation_matrix_map(temporary_allocation_matrix.data(), 4, number_of_rotors_);
  
  Eigen::MatrixXd allocation_matrix(4, number_of_rotors_);
  allocation_matrix = allocation_matrix_map;                                                  
  
  Eigen::Matrix4d I;
  I.setZero();
  I.block<3, 3>(0, 0) = moment_of_inertia_.asDiagonal();
  I(3, 3) = 1;
  
  Eigen::Matrix4d K;
  K.setZero();
  K(0, 0) = arm_length_ * motor_thrust_constant_;
  K(1, 1) = arm_length_ * motor_thrust_constant_;
  K(2, 2) = motor_thrust_constant_ * drag_constant_ ;
  K(3, 3) = motor_thrust_constant_ ;
  
  control_allocation_matrix_ = allocation_matrix.transpose()
      * (allocation_matrix * allocation_matrix.transpose()).inverse() * K.inverse() * I ;
      
  angular_velocity_.setZero();
  //std::cout<<"Drag coefficients:"<< rotaional_drag_coefficients_.transpose() <<std::endl ;
  //std::cout<<"Allocation_Matrix:"<< control_allocation_matrix_ <<std::endl ; 
  //std::cout<<"angular velocity:"<<angular_velocity_.transpose();
  //std::cout<<"Inertia Matrix:"<< moment_of_inertia_.asDiagonal() <<std::endl ;
  
 }
void LowLevelControl::computeRotorSpeed(Eigen::VectorXd* rotor_speeds, Eigen::Vector3d angular_velocity, Eigen::Matrix3d angular_velocity_covariance)
{
	
assert(rotor_speeds);
rotor_speeds->resize(number_of_rotors_);	

bool kalman_status = kalmanPredict(angular_velocity, angular_velocity_covariance) ;
//bool kalman_status = false;

if(kalman_status ==false)
 { //reseting by using current angular velocity estimate and initial drag coefficients
  ROS_ERROR("The estimated state in low level control's kalman filter has a non-finite element");
  angular_velocity_ = angular_velocity ;
  std::vector<double> drag_ceoffs;
  private_nh_.getParam("rotaional_drag_coefficients",drag_ceoffs);
  rotaional_drag_coefficients_<<drag_ceoffs[0], drag_ceoffs[1], drag_ceoffs[2] ;  
 }

//std::cout<<"angular velocity in the rotorspeed:"<<angular_velocity_ ;

computeAngularAcceleration(angular_velocity_);

std::cout<<"Angular Low level controls's thrust and  torque required is:\n "<<thrust_angular_acceleration_<<std::endl;

*rotor_speeds = control_allocation_matrix_ * thrust_angular_acceleration_ ;
*rotor_speeds = rotor_speeds->cwiseMax(Eigen::VectorXd::Zero(rotor_speeds->rows()));
*rotor_speeds = rotor_speeds->cwiseSqrt();

std::cout<<"Angular Low level controls's rotor speeds are:\n"<<*rotor_speeds<<std::endl;
}


bool LowLevelControl::kalmanPredict(Eigen::Vector3d angular_velocity, Eigen::Matrix3d angular_velocity_covariance)
{
		Eigen::Matrix<double, 6, 1> temporary_state;
		temporary_state.setZero();
		
		temporary_state.head(3)= (thrust_angular_acceleration_.tail(3)- rotaional_drag_coefficients_.asDiagonal() * angular_velocity_ - angular_velocity_.cross(moment_of_inertia_.asDiagonal() * angular_velocity_)) * sampling_time_ + angular_velocity_;
		temporary_state.tail(3)= rotaional_drag_coefficients_ ;
		
		Eigen::Matrix<double, 6, 6> state_jacobian=Eigen::Matrix<double, 6, 6>::Zero();
		
		for(int i=0;i<3;i++)
		  state_jacobian(i,i)= - rotaional_drag_coefficients_(i) / moment_of_inertia_(i);	
		
		state_jacobian(0,1)= -(moment_of_inertia_(2) - moment_of_inertia_(1)) * angular_velocity_(2) / moment_of_inertia_(0) ;
	    state_jacobian(0,2)= -(moment_of_inertia_(2) - moment_of_inertia_(1)) * angular_velocity_(1) / moment_of_inertia_(0) ;
	    state_jacobian(0,3)= - angular_velocity_(0) /  moment_of_inertia_(0) ;
	    state_jacobian(1,0)= -(moment_of_inertia_(0) - moment_of_inertia_(2)) * angular_velocity_(2) / moment_of_inertia_(1) ;
	    state_jacobian(1,2)= -(moment_of_inertia_(0) - moment_of_inertia_(2)) * angular_velocity_(0) / moment_of_inertia_(1) ;
	    state_jacobian(1,4)= - angular_velocity_(1) / moment_of_inertia_(1) ;
	    state_jacobian(2,0)= -(moment_of_inertia_(1) - moment_of_inertia_(0)) * angular_velocity_(1) / moment_of_inertia_(2) ;
	    state_jacobian(2,1)= -(moment_of_inertia_(1) - moment_of_inertia_(0)) * angular_velocity_(0) / moment_of_inertia_(2) ;
        state_jacobian(2,5)= - angular_velocity_(2) / moment_of_inertia_(2) ;
	 
	//state_jacobian= (sampling_time_ * state_jacobian).exp() ;
	//std::cout<<"discretized state matrix is:"<<state_jacobian<<std::endl;
	system_noise_ = state_jacobian * system_noise_ * state_jacobian.transpose() + process_noise_ ;
	bool kalman_status = kalmanUpdate(angular_velocity, angular_velocity_covariance, temporary_state);
	
	if(kalman_status==false)
	  return false;
	
	return true ;
}

bool LowLevelControl::kalmanUpdate(Eigen::Vector3d angular_velocity, Eigen::Matrix3d angular_velocity_covariance, Eigen::Matrix<double, 6, 1> predicted_state)
{
	Eigen::Vector3d measurement_update = angular_velocity - measurement_matrix_ *  predicted_state ; 
	Eigen::Matrix<double, 6, 1> angular_state_ ;
	
	//Innovation
	Eigen::Matrix<double, 3, 3> innovation = measurement_matrix_ * system_noise_ * measurement_matrix_.transpose() + angular_velocity_covariance;
	
	//Kalman Gain
	Eigen::Matrix<double, 6, 3>kalman_gain = system_noise_ * measurement_matrix_.transpose() * innovation.inverse() ;
	
	//update state
	angular_state_ = predicted_state + kalman_gain * angular_velocity ;
	
	//update covariance
	system_noise_ = (Eigen::Matrix<double, 6, 6>::Identity() - kalman_gain * measurement_matrix_ ) * system_noise_ ; 
	
	if(angular_state_.allFinite()==false)
	{
		return false;
	}
	angular_velocity_ = angular_state_.head(3);
    rotaional_drag_coefficients_ = angular_state_.tail(3);
}

void LowLevelControl::computeAngularAcceleration(Eigen::Vector3d angular_velocity)
{
	Eigen::Matrix3d gain_matrix=Eigen::Matrix3d::Zero();
	Eigen::Matrix3d dummy_inertia= moment_of_inertia_.asDiagonal();
		gain_matrix(0,0) = omega1_gain_ ;
		gain_matrix(1,1) = omega2_gain_ ;
		gain_matrix(2,2) = omega3_gain_ ;
		//std::cout<<"gain matrix:"<< gain_matrix <<std::endl;
		Eigen::Vector3d torque ;
		torque = dummy_inertia.inverse() * ( gain_matrix * (angular_velocity_reference_ - angular_velocity_ ) + angular_velocity_.cross(moment_of_inertia_.asDiagonal() * angular_velocity_)) ; //+ rotaional_drag_coefficients_.asDiagonal() * angular_velocity_
		thrust_angular_acceleration_.head(3)= torque ;
}
}

