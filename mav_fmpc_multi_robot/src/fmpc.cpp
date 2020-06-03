#include <ros/ros.h>

#include <mav_fmpc_multi_robot/fmpc.h>
#include <unsupported/Eigen/MatrixFunctions>
#include <vector>
#include <cmath>
#include <OsqpEigen/OsqpEigen.h>

//#include <qpOASES/QProblem.hpp>

namespace mav_fmpc_multi_robot
{
	constexpr double FlatMPC::kGravity ;
	constexpr int FlatMPC::kStateSize ;
	constexpr int FlatMPC::kInputSize ;
	constexpr int FlatMPC::kThrustSize ;
	constexpr int FlatMPC::kJerkSize ;

FlatMPC::FlatMPC(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh) :nh_(nh),private_nh_(private_nh),mpc_queue_(nh,private_nh), drag_estimator_(nh,private_nh)
{
initializeParams();

reveived_first_reference_=false;
received_first_odometry_=false;

mpc_queue_.initializeQueue(sampling_time_,discretization_,horizon_);
initializeConstraintObservations();
computeHermiteSimpsonTelescopicMatrix();

clearTrajectory();
}

FlatMPC::~FlatMPC()
{
}

void FlatMPC::initializeParams()
{ std::vector<double> q_position,q_velocity, q_acceleration, r_jerk, drag_coeffs;
	
	if (!private_nh_.getParam("mass", mass_)) {
    ROS_ERROR("mass of multirotor for Flat MPC is not loaded from ros parameter "
              "server");
    abort();
  }

  if (!private_nh_.getParam("q_position", q_position)) {
    ROS_ERROR(
        "position weight for Flat MPC is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("q_velocity", q_velocity)) {
    ROS_ERROR("velocity weight for Flat MPC is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("q_acceleration", q_acceleration)) {
    ROS_ERROR(
        "acceleration weight for Flat MPC is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("r_jerk", r_jerk)) {
    ROS_ERROR("input weight for Flat MPC is not loaded from ros parameter server");
    abort();
  }
  
    if (!private_nh_.getParam("q_yaw", q_yaw_)) {
    ROS_ERROR("yaw weight for Flat MPC is not loaded from ros parameter server");
    abort();
  }
    
    if (!private_nh_.getParam("r_yaw_rate", r_yaw_rate_)) {
    ROS_ERROR("input weight for Flat MPC is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("translation_drag_coefficients", drag_coeffs)) {
    ROS_ERROR(
        "Drag_coefficients for Flat MPC is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("thrust_max", thrust_max_)) {
    ROS_ERROR(
        "thrust max for for Flat MPC is not loaded from ros parameter server");
    abort();
  }
  
    if (!private_nh_.getParam("thrust_min", thrust_min_)) {
    ROS_ERROR(
        "thrust min for for Flat MPC is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("angular_velocity_limit",
                            angular_velocity_limit_)) {
    ROS_ERROR(
        "angular_velocity_limit for Flat MPC is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("discretization", discretization_)) {
    ROS_ERROR("discretization for Flat MPC is not loaded from ros parameter server. Setting default 0.2");
    discretization_ = 0.2;
  }
  
  if (!private_nh_.getParam("controller_sampling_time", sampling_time_)) {
    ROS_ERROR("controller sampling_time for Flat MPC is not loaded from ros parameter server. Setting default of 0.01");
    sampling_time_ = 0.01;
  }

  if (!private_nh_.getParam("time_horizon", horizon_)) {
    ROS_ERROR("time horizon for Flat MPC is not loaded from ros parameter server. Setting default of 15");
    horizon_ = 15 ;
  }
  
  thrust_min_ /=  mass_;
  thrust_max_ /=  mass_;
  thrust_min_ = std::pow(thrust_min_,2);
  thrust_max_ = std::pow(thrust_max_,2);
  
  q_position_<<q_position[0],q_position[1],q_position[2];
  q_acceleration_<<q_acceleration[0],q_acceleration[1],q_acceleration[2];
  q_velocity_<<q_velocity[0],q_velocity[1],q_velocity[2];
  r_jerk_<<r_jerk[0],r_jerk[1],r_jerk[2];
  translation_drag_coefficients_<<drag_coeffs[0],drag_coeffs[1],drag_coeffs[2];
   
  Eigen::MatrixXd Q_cap= Eigen::MatrixXd::Zero(kStateSize * horizon_, kStateSize * horizon_) ;
  Eigen::MatrixXd R_cap= Eigen::MatrixXd::Zero(kInputSize * (2 * horizon_ + 1), kInputSize * (2 * horizon_ +1)) ;

  Eigen::Matrix<double, kStateSize,1> Q;
  Q<<q_position_,q_velocity_,q_acceleration_, q_yaw_ ;
  Eigen::Matrix<double, kInputSize,1> R ;
  R<<r_jerk_, r_yaw_rate_ ;
  
  for(int i=0;i<horizon_;i++)
{Q_cap.block<kStateSize,kStateSize>(kStateSize * i,kStateSize * i)= Q.asDiagonal(); 
 R_cap.block<kInputSize,kInputSize>(kInputSize * 2 * i,kInputSize * 2 * i) = R.asDiagonal();
 R_cap.block<kInputSize,kInputSize>(kInputSize * (2 * i + 1),kInputSize * (2 * i + 1)) = 4 * R.asDiagonal();
}

Q_cap_= Q_cap.sparseView();
R_cap_= R_cap.sparseView();

    
	thrust_limits_max_.resize(2 * horizon_);
	thrust_limits_min_.resize(2 * horizon_);
	jerk_limits_.resize(4 * horizon_ );
	yaw_rate_limits_.resize(2 * horizon_) ;
	inputs_.resize(kInputSize * (2 * horizon_ + 1));
	A_inequality_ =Eigen::MatrixXd::Zero(6 * horizon_ + 2, kInputSize * (2 * horizon_ +1));
	
   A_cap_ = Eigen::MatrixXd::Zero(kStateSize * horizon_, kStateSize);
   B_cap_ = Eigen::MatrixXd::Zero(kStateSize * horizon_, kInputSize * (2 * horizon_ + 1));
   //B_cap_ = Eigen::MatrixXd::Zero(kStateSize * horizon_, kInputSize *  horizon_ );
   A_hat_ = Eigen::MatrixXd::Zero(kStateSize * horizon_, kStateSize * (horizon_ +1));
   B_hat_knot_ = Eigen::MatrixXd::Zero(kStateSize * horizon_, kInputSize * (horizon_ +1));
}

void FlatMPC::initializeConstraintObservations(){
		
	Eigen::Matrix<double,kThrustSize,kStateSize> C_thrust;
	C_thrust.setZero();
	C_thrust(0,1)=1;
    C_thrust(1,2)=1;
    C_thrust(2,4)=1;
    C_thrust(3,5)=1;
    C_thrust(4,7)=1;
    C_thrust(5,8)=1;
    Eigen::Matrix<double,kJerkSize,kStateSize> C_jerk;
    C_jerk.setZero();
    C_jerk(0,2)=1;
    C_jerk(1,5)=1;
    C_jerk(2,8)=1;
    Eigen::Matrix<double,kJerkSize, kInputSize> D_jerk;
    D_jerk.setZero();
    D_jerk.block<kJerkSize,kJerkSize>(0,0)=Eigen::Matrix3d::Identity();
    
    Eigen::MatrixXd C_= Eigen::MatrixXd::Zero(kThrustSize * horizon_, kStateSize * horizon_);
    Eigen::MatrixXd C= Eigen::MatrixXd::Zero(kJerkSize * horizon_, kStateSize * horizon_);
    Eigen::MatrixXd D= Eigen::MatrixXd::Zero(kJerkSize *  horizon_ +1, kInputSize * (2 * horizon_ +1));
    Eigen::MatrixXd D_knot= Eigen::MatrixXd::Zero(kJerkSize * horizon_, kInputSize * (2 * horizon_ +1));
    Eigen::MatrixXd D_U= Eigen::MatrixXd::Zero(kInputSize * (horizon_ +1),kInputSize * (2 * horizon_ +1));
    for(int i=0;i<horizon_;i++)
   {	 
    C_.block<kThrustSize,kStateSize>(kThrustSize * i, kStateSize * i)=C_thrust;	
    C.block<kJerkSize,kStateSize>(kJerkSize * i, kStateSize * i)=C_jerk;
    D.block<kJerkSize, kInputSize>(kJerkSize * i , kInputSize * (2 *(i +1)))= D_jerk;
    D_knot.block<kJerkSize, kInputSize>(kJerkSize * i , kInputSize * (2 *i + 1))=D_jerk ; 
    D_U.block<kInputSize,kInputSize>(kInputSize * i,kInputSize * 2*i)=Eigen::MatrixXd::Identity(kInputSize,kInputSize);
   }
   
   D_U.block<kInputSize,kInputSize>(kInputSize * horizon_,kInputSize * 2*horizon_)=Eigen::MatrixXd::Identity(kInputSize,kInputSize);
      
  C_thrust_=C_.sparseView();
  C_angular_velocity_ =C.sparseView();
  D_jerk_ = D.sparseView();
  D_jerk_knot_ = D_knot.sparseView();
  D_U_knot_=D_U.sparseView();
}

void FlatMPC::preparationStep()
{  
	bool ekf_status = drag_estimator_.updateEstimator(acceleration_);
	if(!ekf_status)
	{
	 drag_estimator_.resetObserver(current_odometry_.position_,current_odometry_.velocity_,translation_drag_coefficients_);
	}
	
	//translation_drag_coefficients_ = drag_estimator_.getEstimatedDragCoefficients();
	//computeHermiteSimpsonTelescopicMatrix();
	lineariseConstraints();
}

void FlatMPC::setOdometry(Odometry odometry, std::vector<double> state_covariance){
	if(!received_first_odometry_)
	{current_odometry_.position_=odometry.position_;
	 current_odometry_.velocity_=odometry.velocity_;
	 current_odometry_.acceleration_=odometry.acceleration_;
	 current_odometry_.yaw_=odometry.yaw_;
	 
	 Eigen::Matrix<double,6,1> measured_state;
	 measured_state<<odometry.position_,odometry.velocity_ ;
	 drag_estimator_.feedCurrentState(measured_state,state_covariance) ; 
	 
	 received_first_odometry_ =true;
	 //intializeSolver();
	 return;
	}
	
	current_odometry_.position_=odometry.position_;
	current_odometry_.velocity_=odometry.velocity_;
	current_odometry_.acceleration_=odometry.acceleration_;
	current_odometry_.yaw_=odometry.yaw_;
	
	Eigen::Matrix<double,6,1> measured_state;
	measured_state<<odometry.position_,odometry.velocity_ ;
	drag_estimator_.feedCurrentState(measured_state,state_covariance) ;
}

void FlatMPC::evaluateReferenceTrajectory(mav_msgs::EigenTrajectoryPointDeque& references)
{	//ros::WallTime starting_time = ros::WallTime::now();
	if(!reveived_first_reference_)
	{   //ROS_INFO("FMPC: Evaluating Trajectory ");
		mpc_queue_.insertReferenceTrajectory(references);
		//ROS_INFO("FMPC: Completed Evaluating Trajectory, Initializing Constraints");
		intializeSolver();
	    reveived_first_reference_=true;
	    //double solve_time =(ros::WallTime::now() - starting_time).toSec() * 1000.0;
      //ROS_INFO_STREAM("evaluating reference trajectory took "<<solve_time<<" milliseconds");
	    return;	
	}
	
	if(references.size() <1)
	{
	  ROS_INFO("No references to track for Flat MPC");
	  return;
	}
	
	mpc_queue_.insertReferenceTrajectory(references);
	//double solve_time =(ros::WallTime::now() - starting_time).toSec() * 1000.0;
    //ROS_INFO_STREAM("evaluating reference trajectory took "<<solve_time<<" milliseconds");
}

void FlatMPC::intializeSolver()
{ 
 mpc_queue_.getQueue(position_trajectory_,velocity_trajectory_,acceleration_trajectory_,yaw_trajectory_);
 lineariseConstraints();
}

void FlatMPC::computeTelescopicMatrix()
{
	Eigen::Matrix<double, kStateSize,kStateSize> A_continuous ;
	Eigen::Matrix<double, kStateSize,kInputSize> B_continuous ;
	
	ros::WallTime starting_time = ros::WallTime::now();

	A_continuous.setZero();
    B_continuous.setZero();
	
	
	for(int i=0;i<3;i++)
	{
		A_continuous(3 * i, 3 * i+1)=1;
		A_continuous(3 * i+1,3 * i+2)=1;
		A_continuous(3 * i +1, 3 * i+1)=translation_drag_coefficients_(i);
		A_continuous(3 * i + 2, 3 * i+2)=translation_drag_coefficients_(i);
		B_continuous(3 * i + 2,i)=1;
	}
	B_continuous(9,3)=1;
	
	Eigen::Matrix<double, kStateSize,kStateSize> A_disc ;
	Eigen::Matrix<double, kStateSize,kInputSize> B_disc ;
	A_disc=(discretization_ * A_continuous).exp();
	B_disc=A_disc * A_continuous.inverse() * B_continuous ;

for(int i=0;i<horizon_;i++)
{ if(i==0)
	A_cap_.block<kStateSize,kStateSize>(kStateSize * i,0)= A_disc;
  else if(i>0)
	A_cap_.block<kStateSize,kStateSize>(kStateSize * i,0)= A_cap_.block<kStateSize,kStateSize>(10 * (i-1),0) * A_disc;
	
	for(int j=0;j<=i;j++)
	{
		if(i ==j){
      	B_cap_.block<kStateSize,kInputSize>(kStateSize * i,kInputSize * j) +=  B_disc;
	    }
		else {
		B_cap_.block<kStateSize,kInputSize>(kStateSize * i,kInputSize * j) += A_cap_.block<kStateSize,kStateSize>(10 * (i-j -1),0) *  B_disc ;
	   }
	}
	
	H_ = 0.5 * B_cap_.transpose() * Q_cap_ * B_cap_  ;
}
	
}

void FlatMPC::computeHermiteSimpsonTelescopicMatrix(){
	Eigen::Matrix<double, kStateSize,kStateSize> A_continuous ;
	Eigen::Matrix<double, kStateSize,kInputSize> B_continuous ;
	
	ros::WallTime starting_time = ros::WallTime::now();

	A_continuous.setZero();
    B_continuous.setZero();
	
	
	for(int i=0;i<3;i++)
	{
		A_continuous(3 * i, 3 * i+1)=1;
		A_continuous(3 * i+1,3 * i+2)=1;
		A_continuous(3 * i +1, 3 * i+1)=translation_drag_coefficients_(i);
		A_continuous(3 * i + 2, 3 * i+2)=translation_drag_coefficients_(i);
		B_continuous(3 * i + 2,i)=1;
	}
	B_continuous(9,3)=1;
	
Eigen::MatrixXd Z=(Eigen::MatrixXd::Identity(kStateSize,kStateSize) + std::pow(discretization_,2) * A_continuous.pow(2)/12 - A_continuous * discretization_/2  ).inverse(); 

Eigen::Matrix<double,kStateSize,kStateSize> A = Z * (Eigen::MatrixXd::Identity(kStateSize,kStateSize) + std::pow(discretization_,2) * A_continuous.pow(2)/12 + A_continuous * discretization_/2 );

Eigen::Matrix<double, kStateSize,kInputSize> B = Z * 2 * B_continuous * discretization_/3 ; // u_{k + 0.5}
Eigen::Matrix<double, kStateSize,kInputSize> B_ = Z * ( B_continuous * discretization_/6 + std::pow(discretization_,2)/12 * A_continuous * B_continuous) ; // u_k
Eigen::Matrix<double, kStateSize,kInputSize> B_i = Z * ( B_continuous * discretization_/6 - std::pow(discretization_,2)/12 * A_continuous * B_continuous) ; // u_{k+1} 

for(int i=0;i<horizon_;i++)
{ if(i==0)
	A_cap_.block<kStateSize,kStateSize>(kStateSize * i,0)= A;
  else if(i>0)
	A_cap_.block<kStateSize,kStateSize>(kStateSize * i,0)= A_cap_.block<kStateSize,kStateSize>(10 * (i-1),0) * A;
	//A_cap_.block<kStateSize,kStateSize>(kStateSize * i,0)= A.pow(i+1);
	for(int j=0;j<=i;j++)
	{   //Eigen::Matrix<double,kStateSize,kStateSize> A_pow = A.pow(i-j);
		if(i ==j){
      	B_cap_.block<kStateSize,kInputSize>(kStateSize * i,kInputSize * 2 * j) +=  B_ ;
		B_cap_.block<kStateSize,kInputSize>(kStateSize * i,kInputSize * 2 * j + kInputSize) +=  B ;
		B_cap_.block<kStateSize,kInputSize>(kStateSize * i,kInputSize * 2 * j + 2 * kInputSize) +=  B_i ;
	    }
		else {
		B_cap_.block<kStateSize,kInputSize>(kStateSize * i,kInputSize * 2 * j) += A_cap_.block<kStateSize,kStateSize>(10 * (i-j -1),0) *  B_ ;
		B_cap_.block<kStateSize,kInputSize>(kStateSize * i,kInputSize * 2 * j + kInputSize) +=  A_cap_.block<kStateSize,kStateSize>(10 * (i-j -1),0) * B ;
		B_cap_.block<kStateSize,kInputSize>(kStateSize * i,kInputSize * 2 * j + 2 * kInputSize) +=  A_cap_.block<kStateSize,kStateSize>(10 * (i-j -1),0) * B_i ;
	   }
	}
	A_hat_.block<kStateSize,kStateSize>(kStateSize * i, kStateSize * i)=Eigen::MatrixXd::Identity(kStateSize,kStateSize)/2 + A_continuous * discretization_/8;
    A_hat_.block<kStateSize,kStateSize>(kStateSize * i, kStateSize * (i+1))=Eigen::MatrixXd::Identity(kStateSize,kStateSize)/2 - A_continuous * discretization_/8;
    B_hat_knot_.block<kStateSize,kInputSize>(kStateSize * i, kInputSize * i)=B_continuous * discretization_/8 ;
    B_hat_knot_.block<kStateSize,kInputSize>(kStateSize * i, kInputSize * (i+1))=-B_continuous * discretization_/8 ;
}

Eigen::MatrixXd B_hat = Eigen::MatrixXd::Zero(kStateSize * (horizon_ +1), kInputSize * (2 * horizon_ + 1));
B_hat.block(kStateSize,0,kStateSize * horizon_, kInputSize * (2 * horizon_ + 1)) = B_cap_ ;

//std::cout<<"The matrices compared are: "<<((A_hat_ * B_hat).transpose() * Q_cap_ * (A_hat_ * B_hat)).isApprox(B_hat.transpose() * A_hat_.transpose() * Q_cap_ * A_hat_ * B_hat);

H_ = 0.5 * (B_cap_.transpose() + 4 * (A_hat_ * B_hat).transpose()) * Q_cap_ * (B_cap_ + (A_hat_ * B_hat)) ;
H_ +=0.5 * R_cap_ ;
H_ +=2 * D_U_knot_.transpose() * B_hat_knot_.transpose() * Q_cap_ * B_hat_knot_ * D_U_knot_ + 4 * (A_hat_ * B_hat).transpose() * Q_cap_ * B_hat_knot_ * D_U_knot_ ;
H_ *= discretization_/6 ;

   double solve_time =(ros::WallTime::now() - starting_time).toSec() * 1000.0;
   
   ROS_INFO_STREAM("Computation time of Hermite Simpson is: "<<solve_time<<" ms");
}

void FlatMPC::lineariseConstraints(){
	Eigen::MatrixXd thrust_limits_map = Eigen::MatrixXd::Zero(horizon_,kThrustSize * horizon_);
	Eigen::MatrixXd thrust_limits_map_knot = Eigen::MatrixXd::Zero(horizon_,kThrustSize * horizon_);
	Eigen::MatrixXd	jerk_limits_map = Eigen::MatrixXd::Zero(4 * (horizon_ +1), kJerkSize * (2 * horizon_ + 1));
	Eigen::MatrixXd	jerk_limits_map_knot = Eigen::MatrixXd::Zero(2 * (horizon_ +1), kJerkSize * (2 * horizon_ + 1));
	Eigen::Vector3d gravity_vector;
	 gravity_vector<<0,0,kGravity;
	 
	for(int i=0;i<horizon_;i++)
	{Eigen::Matrix<double,6,1> current_velocity, current_velocity_knot, linearization, linearization_knot;	
	 Eigen::Matrix3d rotation_matrix=getRotationMatrix(acceleration_trajectory_[2 * i],velocity_trajectory_[2 * i],yaw_trajectory_[2 * i]);
	 jerk_limits_map.block<kJerkSize,1>(2 * i,kJerkSize * i)= rotation_matrix.col(1).transpose();
	 jerk_limits_map.block<kJerkSize,1>(2 * i +1,kJerkSize * i)= rotation_matrix.col(0).transpose();
	 Eigen::Vector3d Y_c;
	 Y_c<<-std::sin(yaw_trajectory_[i]),std::cos(yaw_trajectory_[i]),0 ;
	 Eigen::Vector3d X_c;
	 X_c<<std::cos(yaw_trajectory_[i]),std::sin(yaw_trajectory_[i]),0 ;
	 yaw_rate_limits_(2 * i)= angular_velocity_limit_/std::sqrt(3) * (Y_c.transpose() * rotation_matrix.col(2) + (Y_c.transpose().cross(rotation_matrix.col(2))).norm())/(X_c.transpose() * rotation_matrix.col(1));
	 
	 rotation_matrix=getRotationMatrix(acceleration_trajectory_[2 * i +1],velocity_trajectory_[2 * i +1],yaw_trajectory_[2 * i +1]);
	 jerk_limits_map_knot.block<kJerkSize,1>(2 * i,kJerkSize * i)= rotation_matrix.col(1).transpose();
	 jerk_limits_map_knot.block<kJerkSize,1>(2 * i +1,kJerkSize * i)= rotation_matrix.col(0).transpose();
	 Y_c<<-std::sin(yaw_trajectory_[2 * i +1]),std::cos(yaw_trajectory_[2 * i +1]),0 ;
	 X_c<<std::cos(yaw_trajectory_[2 * i +1]),std::sin(yaw_trajectory_[2 * i +1]),0 ;
	 yaw_rate_limits_(2 * i+1)= angular_velocity_limit_/std::sqrt(3) * (Y_c.transpose() * rotation_matrix.col(2) + (Y_c.transpose().cross(rotation_matrix.col(2))).norm())/(X_c.transpose() * rotation_matrix.col(1));

		for(int j=1;j<3;j++)
		{
		 linearization(2 * j)= 2 * translation_drag_coefficients_(j) * velocity_trajectory_[2 * i](j);
		 linearization(2 * j + 1)= 2 * acceleration_trajectory_[2 * i](j);
		 linearization_knot(2 * j)= 2 * translation_drag_coefficients_(j) * velocity_trajectory_[2 * i +1](j);
		 linearization_knot(2 * j + 1)= 2 * acceleration_trajectory_[2 * i + 1](j);
		
		 current_velocity(2 * j)=velocity_trajectory_[i](j);
		 current_velocity(2 * j+1)=acceleration_trajectory_[i](j);
		 current_velocity_knot(2 * j)=velocity_trajectory_[2 * i + 1](j);
		 current_velocity_knot(2 * j+1)=acceleration_trajectory_[2 * i + 1](j);  
	    }
	        
	 thrust_limits_map.block<kThrustSize,1>(i,kThrustSize*i)=linearization; 
	 thrust_limits_map_knot.block<kThrustSize,1>(i,kThrustSize*i)=linearization_knot;
	    
	 double current_thrust = (acceleration_trajectory_[2 * i] +  translation_drag_coefficients_.asDiagonal() * velocity_trajectory_[2 * i] + gravity_vector).squaredNorm();
	 thrust_limits_min_(i)= thrust_min_ - current_thrust + linearization.transpose() * current_velocity ;
	 thrust_limits_max_(i)= thrust_max_ - current_thrust + linearization.transpose() * current_velocity ;
	 jerk_limits_(2 * i)=  angular_velocity_limit_/std::sqrt(3) * std::sqrt(current_thrust) ;
	 jerk_limits_(2 * i + 1)=  angular_velocity_limit_/std::sqrt(3) * std::sqrt(current_thrust) ;
	
	 current_thrust = (acceleration_trajectory_[2 * i + 1] +  translation_drag_coefficients_.asDiagonal() * velocity_trajectory_[2 * i + 1] + gravity_vector).squaredNorm();
	 thrust_limits_min_(horizon_ + i)= thrust_min_ - current_thrust + linearization_knot.transpose() * current_velocity_knot ;
	 thrust_limits_max_(horizon_ + i)= thrust_max_ - current_thrust + linearization_knot.transpose() * current_velocity_knot ;
	 jerk_limits_(2 * (horizon_ + i))=  std::abs(angular_velocity_limit_/std::sqrt(3) * std::sqrt(current_thrust)) ;
	 jerk_limits_(2 * (horizon_ + i) + 1)=  std::abs(angular_velocity_limit_/std::sqrt(3) * std::sqrt(current_thrust)) ;
	}
	
	/*thrust_limits_map_=thrust_limits_map.sparseView();
	knot_thrust_limits_map_=thrust_limits_map_knot.sparseView();
	jerk_limits_map_=jerk_limits_map.block(0,0, 2 * horizon_, kJerkSize * horizon_).sparseView();
	knot_jerk_limits_map_=jerk_limits_map.block(2 * horizon_,0, 2 * horizon_, kJerkSize * horizon_).sparseView();*/
	
		ros::WallTime starting_time = ros::WallTime::now();
	Eigen::MatrixXd B_hat = Eigen::MatrixXd::Zero(kStateSize * (horizon_ +1), kInputSize * (2 * horizon_ + 1));
    B_hat.block(kStateSize,0,kStateSize * horizon_, kInputSize * (2 * horizon_ + 1)) = B_cap_ ;
	double solve_time =(ros::WallTime::now() - starting_time).toSec() * 1000.0;
		
	//std::cout<<"thrust_limits\n"<<thrust_limits_map<<std::endl;
	//std::cout<<"thrust_limits\n"<<thrust_limits_map_knot<<std::endl;
	
    //Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> A_inequality =Eigen::MatrixXd::Zero(2 * horizon_ ,kInputSize * (2 * horizon_ +1));
    A_inequality_.block(0,0,horizon_,kInputSize * (2 * horizon_ + 1)) = thrust_limits_map * C_thrust_ * B_cap_ ;
    A_inequality_.block(horizon_,0,horizon_,kInputSize * (2 * horizon_ + 1)) = thrust_limits_map_knot * C_thrust_ * A_hat_ * B_hat ;
    A_inequality_.block(2 * horizon_,0,2 * horizon_,kInputSize * (2 * horizon_ + 1)) = jerk_limits_map * C_angular_velocity_ * B_cap_ ;
    A_inequality_.block(2 * horizon_,0,2 * horizon_,kInputSize * (2 * horizon_ + 1))+= jerk_limits_map * C_angular_velocity_ * D_jerk_ ;
    A_inequality_.block(4 * horizon_,0,2 * horizon_,kInputSize * (2 * horizon_ + 1)) = jerk_limits_map_knot * C_angular_velocity_ * A_hat_ * B_cap_  ;
    A_inequality_.block(4 * horizon_,0,2 * horizon_,kInputSize * (2 * horizon_ + 1)) += jerk_limits_map_knot * C_angular_velocity_ * A_hat_ * D_jerk_knot_ ;
}

Eigen::Matrix3d FlatMPC::getRotationMatrix(Eigen::Vector3d acceleration, Eigen::Vector3d velocity, double yaw){

Eigen::Matrix3d rotation_matrix;
Eigen::Vector3d Z_b=acceleration + this->translation_drag_coefficients_.asDiagonal() * velocity ;
Z_b(2)+=kGravity ;
rotation_matrix.col(2) = Z_b.normalized();

Eigen::Vector3d X_c;
X_c<<std::cos(yaw),std::sin(yaw),0;
Eigen::Vector3d Y_b = Z_b.cross(X_c);
rotation_matrix.col(1)=Y_b.normalized();

rotation_matrix.col(0)=rotation_matrix.col(1).cross(rotation_matrix.col(2));

return rotation_matrix ;
}

bool FlatMPC::calculateAngularVelocityThrust(Eigen::VectorXd* angular_velocity_thrust)
{   
	ros::WallTime starting_time = ros::WallTime::now();
    Eigen::Matrix<double, kStateSize, 1> current_state;
	for(int i=0;i<3;i++)
	{
     	current_state(3 * i)=current_odometry_.position_(i);
		current_state(3 * i +1)=current_odometry_.velocity_(i);
		current_state(3 * i +2)=current_odometry_.acceleration_(i);
    }
	current_state(kStateSize-1)=current_odometry_.yaw_;
	
	Eigen::VectorXd state_reference, input_reference ;
	//get reshaped reference
	mpc_queue_.updateQueue();
	mpc_queue_.getThirdOrderQueue(&state_reference, &input_reference,false);
	
	F_ = (A_cap_ * current_state - state_reference).transpose() * (Q_cap_ * B_cap_ ) - input_reference.transpose() *  R_cap_ ;
	
	mpc_queue_.getThirdOrderQueue(&state_reference, &input_reference,true);
    Eigen::MatrixXd B_hat = Eigen::MatrixXd::Zero(kStateSize * (horizon_ +1), kInputSize * (2 * horizon_ + 1));
    B_hat.block(kStateSize,0,kStateSize * horizon_, kInputSize * (2 * horizon_ + 1)) = B_cap_ ;
    Eigen::MatrixXd A_knot = Eigen::MatrixXd::Zero(kStateSize * (horizon_ +1),kStateSize);
    A_knot.block<kStateSize,kStateSize>(0,0)=Eigen::MatrixXd::Identity(kStateSize,kStateSize);
    A_knot.block(kStateSize,0,kStateSize * horizon_,kStateSize)=A_cap_;
    
    Eigen::MatrixXd Q_consol = (Q_cap_ * A_hat_ * B_hat);
    //ros::WallTime start_time = ros::WallTime::now();
    F_ += 4 * ((A_hat_ * A_knot * current_state-state_reference).transpose() * Q_consol + (A_hat_ * A_knot * current_state-state_reference).transpose() * (Q_cap_ * B_hat_knot_)  * D_U_knot_) ;
    F_ *= discretization_/6 ;   
      
    thrust_limits_max_.segment(0,horizon_) -= C_thrust_ * A_cap_ * current_state ;
    thrust_limits_min_.segment(0,horizon_) -= C_thrust_ * A_cap_ * current_state ;
    thrust_limits_max_.segment(horizon_,horizon_) -= C_thrust_ * A_hat_ * A_cap_ * current_state ;
    thrust_limits_min_.segment(horizon_,horizon_) -= C_thrust_ * A_hat_ * A_cap_ * current_state ;
    jerk_limits_.segment(0,2 * horizon_) += C_angular_velocity_ * A_cap_ * current_state ;
    jerk_limits_.segment(2 * horizon_,2 * horizon_) += C_angular_velocity_ * A_hat_ * A_cap_ * current_state ;
    
    Eigen::Matrix3d rotation_matrix=getRotationMatrix(current_odometry_.acceleration_,current_odometry_.velocity_,current_odometry_.yaw_);
    Eigen::Vector3d gravity_vector;
    gravity_vector<<0,0,kGravity;
    
    double thrust = (current_odometry_.acceleration_ +  translation_drag_coefficients_.asDiagonal() * current_odometry_.velocity_ + gravity_vector).norm();
    Eigen::Vector2d first_angular_velocity_limit;
    first_angular_velocity_limit<<angular_velocity_limit_ * thrust - rotation_matrix.col(1).transpose() * translation_drag_coefficients_.asDiagonal() * current_odometry_.acceleration_, angular_velocity_limit_ * thrust - rotation_matrix.col(0).transpose() * translation_drag_coefficients_.asDiagonal() * current_odometry_.acceleration_ ;
    A_inequality_.block<3,1>(4 * horizon_,0)=rotation_matrix.col(1).transpose();
    A_inequality_.block<3,1>(4 * horizon_ + 1,0)=rotation_matrix.col(0).transpose();
        
    Eigen::VectorXd upper_bound = Eigen::VectorXd::Zero(6 * horizon_ + 2);
    upper_bound<<thrust_limits_max_,jerk_limits_,first_angular_velocity_limit ;
    //upper_bound<<jerk_limits_,first_angular_velocity_limit ;
    Eigen::VectorXd lower_bound= Eigen::VectorXd::Zero(6 * horizon_ + 2);
   lower_bound<<thrust_limits_min_,-jerk_limits_,-first_angular_velocity_limit; 
   //lower_bound<<-jerk_limits_,-first_angular_velocity_limit;
    
 /*   qpOASES::real_t Hessian[(2 * horizon_ +1) * kInputSize * (2 * horizon_ +1) * kInputSize] ;
    qpOASES::real_t A_ineq[A_inequality_.rows() * A_inequality_.cols()] ;
    qpOASES::real_t Jacobian[A_inequality_.cols()] ;
    qpOASES::real_t ineq_ub[A_inequality_.rows()];
    qpOASES::real_t ineq_lb[A_inequality_.rows()];
    qpOASES::real_t ub[A_inequality_.cols()];
    qpOASES::real_t lb[A_inequality_.cols()];
    
    Eigen::Vector3d Y_c;
    Y_c<<-std::sin(current_odometry_.yaw_),std::cos(current_odometry_.yaw_),0;
    Eigen::Vector3d X_c;
    X_c<<std::cos(current_odometry_.yaw_),std::sin(current_odometry_.yaw_),0;
    ub[0] = angular_velocity_limit_/std::sqrt(3) * (Y_c.transpose() * rotation_matrix.col(2) + (Y_c.transpose().cross(rotation_matrix.col(2))).norm())/(X_c.transpose() * rotation_matrix.col(1)) ;
    lb[0] = -ub[0];
    
    ROS_INFO("FMPC: Started Mapping to real_t");
    Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(Hessian), A_inequality_.cols() , A_inequality_.cols()) = 0.5 * H_ ;
    Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(A_ineq), A_inequality_.rows(), A_inequality_.cols()) = A_inequality_ ;
    Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(Jacobian), A_inequality_.cols() , 1) = F_ ;
    Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(ineq_ub), A_inequality_.rows(), 1) = upper_bound ;
    Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(ineq_lb), A_inequality_.rows(), 1) = lower_bound ;
    ROS_INFO("FMPC: Finished Mapping to real_t");
    
    for(int i=1;i< 2 * horizon_ + 1;i++){
    ub[kInputSize * i]=100;
	ub[kInputSize * i +1]=100;
	ub[kInputSize * i + 2]=100;
	ub[kInputSize * i +3]=yaw_rate_limits_(i);
	lb[kInputSize * i]=-100;
	lb[kInputSize * i +1]=-100;
	lb[kInputSize * i + 2]=-100;
	lb[kInputSize * i +3]=-yaw_rate_limits_(i);	
	}
    
    ros::WallTime time_before_solving = ros::WallTime::now();
       
    qpOASES::QProblem QP(A_inequality_.cols(),A_inequality_.rows());
    //QP.setPrintLevel( qpOASES::PL_LOW );
    qpOASES::int_t nWSR= 5 * A_inequality_.rows();
    int solved = (int) QP.init(Hessian,Jacobian,A_ineq,ineq_lb,ineq_ub,lb,ub,nWSR);
    //qpOASES::returnValue retVal =  QP.init(Hessian,Jacobian,lb,ub,nWSR);
     qpOASES::real_t solution[(2 * horizon_ +1) * kInputSize];
   QP.getPrimalSolution(solution);
  inputs_=Eigen::Map<Eigen::VectorXd>(solution, (2 * horizon_ +1) * kInputSize, 1)  ;*/
   
    
 ros::WallTime time_before_solving = ros::WallTime::now();

        OsqpEigen::Solver solver;
        solver.settings()->setPolish(true);
        
        solver.data()->setNumberOfVariables(A_inequality_.cols());
        solver.data()->setNumberOfConstraints(A_inequality_.rows());

Eigen::SparseMatrix<double> hessian=H_.sparseView();
Eigen::SparseMatrix<double> A_inequal= A_inequality_.sparseView();

  bool hess_status = solver.data()->setHessianMatrix(hessian);
   hess_status =  solver.data()->setGradient(F_); 
    hess_status = solver.data()->setLinearConstraintsMatrix(A_inequal); 
    hess_status = solver.data()->setLowerBound(lower_bound) ;
    hess_status = solver.data()->setUpperBound(upper_bound) ;
 
bool initailised = solver.initSolver();
bool solved;
if(initailised)
solved = solver.solve() ;
else
{
    ROS_WARN_STREAM("FMPC: Solver failed to initialize");
    ROS_WARN("applying next input from the previous solution");
    intializeSolver();
    ROS_WARN("Finished initialising from the previous solution");
    return false;	
}

 double solve_time =(ros::WallTime::now() - time_before_solving).toSec() * 1000.0;
 
   ROS_INFO_STREAM("Time to solve NLP is:"<<solve_time<<" ms");
   
   inputs_ = solver.getSolution();

   u_0_ = inputs_.segment<kInputSize>(0) ;
   u_knot_ = inputs_.segment<kInputSize>(kInputSize) ;
   u_i_ = inputs_.segment<kInputSize>(2 * kInputSize) ;

   computeCurrentStateInput();
   
   if(applied_input_.allFinite() == false || solved ==0 || initailised==false)
   {
    ROS_WARN_STREAM("FMPC: Solver failed with status: " << solved);
    ROS_WARN("applying next input from the previous solution");
    intializeSolver();
    ROS_WARN("Finished initialising from the previous solution");
    return false;
    }
   
   *angular_velocity_thrust=applied_input_;
   	double total_time_in_loop = (ros::WallTime::now()- starting_time).toSec() * 1000.0;
   	ROS_INFO_STREAM("time to calculate thrust is:"<<total_time_in_loop<<" ms");
   	//std::cin>>solve_time;
   	return true; 
}

void FlatMPC::computeCurrentStateInput()
{
	Eigen::Matrix<double, kStateSize, 1> current_state;
	for(int i=0;i<3;i++)
	{
     	current_state(3 * i)=current_odometry_.position_(i);
		current_state(3 * i +1)=current_odometry_.velocity_(i);
		current_state(3 * i +2)=current_odometry_.acceleration_(i);
    }
	current_state(kStateSize-1)=current_odometry_.yaw_;
	
 Eigen::Matrix<double, kStateSize, 1> alpha1, alpha2, predicted_state;
	Eigen::Matrix<double, kStateSize,kStateSize> A_continuous ;
	Eigen::Matrix<double, kStateSize,kInputSize> B_continuous ;
	
	A_continuous.setZero();
    B_continuous.setZero();
	
	for(int i=0;i<3;i++)
	{
		A_continuous(3 * i, 3 * i+1)=1;
		A_continuous(3 * i+1,3 * i+2)=1;
		A_continuous(3 * i +1, 3 * i+1)=translation_drag_coefficients_(i);
		A_continuous(3 * i + 2, 3 * i+2)=translation_drag_coefficients_(i);
		B_continuous(3 * i + 2,i)=1;
    }
	B_continuous(kStateSize-1,kInputSize-1)=1;
	
	Eigen::MatrixXd Z=(Eigen::MatrixXd::Identity(kStateSize,kStateSize) + 4 * std::pow(discretization_,2) * A_continuous.pow(2)/48 - A_continuous * discretization_/2  ).inverse(); /*

    Eigen::Matrix<double,kStateSize,kStateSize> A = Z * (Eigen::MatrixXd::Identity(kStateSize,kStateSize) + 4 * std::pow(discretization_,2) * A_continuous.pow(2)/48 + A_continuous * discretization_/2 );

    Eigen::Matrix<double, kStateSize,kInputSize> B = Z * 4 * B_continuous ; // u_{k + 0.5}
    Eigen::Matrix<double, kStateSize,kInputSize> B_ = Z * ( B_continuous * discretization_/6 + std::pow(discretization_,2)/12 * A_continuous * B_continuous) ; // u_k
    Eigen::Matrix<double, kStateSize,kInputSize> B_i = Z * ( B_continuous * discretization_/6 - std::pow(discretization_,2)/12 * A_continuous * B_continuous) ; // u_{k+1} 
	
	Eigen::Matrix<double, kInputSize,1> current_input = u_0_ - sampling_time_/discretization_ * (3 * u_0_ - 4 * u_knot_ + u_i_) + std::pow(sampling_time_,2) * 2/std::pow(discretization_,2) * (u_0_ - 2 * u_knot_ + u_i_)  ;
 
	alpha1 = A * current_state + B * u_knot_ + B_ * u_0_ + B_i * u_i_ ;
	alpha2 = (discretization_ * A_continuous/8 + Eigen::MatrixXd::Identity(kStateSize,kStateSize)/2) * current_state + (-discretization_ * A_continuous/8 + Eigen::MatrixXd::Identity(kStateSize,kStateSize)/2) * alpha1 ;
	predicted_state = current_state + (A_continuous * current_state + B_continuous * u_0_) * sampling_time_ - sampling_time_ * sampling_time_/(2 * discretization_) * (3 * (A_continuous * current_state + B_continuous * u_0_) - 4 * (A_continuous * alpha2 + B_continuous * u_knot_) + (A_continuous * alpha1 + B_continuous * u_i_)) + 2 * std::pow(sampling_time_,3)/std::pow(discretization_,2) * ((A_continuous * current_state + B_continuous * u_0_) - 2 * (A_continuous * alpha2 + B_continuous * u_knot_) + (A_continuous * alpha1 + B_continuous * u_i_)) ;*/
	
	predicted_state = A_continuous * current_state + B_continuous * u_0_ ;
	Eigen::Matrix<double, kInputSize,1> current_input = u_0_;
	
	Eigen::Matrix<double,3,kStateSize> C_accel ;
    C_accel.setZero();
    C_accel(0,2)=1;
    C_accel(1,5)=1;
    C_accel(2,8)=1;
	acceleration_= C_accel * predicted_state;
	
//	std::cout<<"Computed jerk and yaw is: \n"<<current_input<<std::endl;
	
	Eigen::Vector3d velocity ;
	velocity<<predicted_state(1),predicted_state(4),predicted_state(7);
	Eigen::Vector3d gravity_vector; 
	gravity_vector<<0,0,kGravity;
		
	Eigen::Matrix3d rotation_matrix=getRotationMatrix(acceleration_,velocity,predicted_state(kStateSize-1));
	Eigen::Vector3d Y_c;
	Y_c<<-std::sin(predicted_state(kStateSize-1)),std::cos(predicted_state(kStateSize-1)),0 ;
	Eigen::Vector3d X_c;
	X_c<<std::cos(predicted_state(kStateSize-1)),std::sin(predicted_state(kStateSize-1)),0;
	
	double computed_thrust = (acceleration_ + translation_drag_coefficients_.asDiagonal() * velocity + gravity_vector).dot(rotation_matrix.col(2)) ;

	
	Eigen::Matrix3d angular_velocity_inversion;
	angular_velocity_inversion.setZero();
	angular_velocity_inversion(0,0)=-computed_thrust;
	angular_velocity_inversion(1,1)=computed_thrust;
	angular_velocity_inversion(2,1)=Y_c.transpose() * rotation_matrix.col(2) ;
	angular_velocity_inversion(2,2)=(Y_c.transpose().cross(rotation_matrix.col(2))).norm() ;
	
	Eigen::Vector3d map_b, current_angular_velocity;
	map_b(0)=rotation_matrix.col(1).transpose() * (current_input.segment<3>(0) + translation_drag_coefficients_.asDiagonal() * acceleration_) ;
	map_b(1)=rotation_matrix.col(0).transpose() * (current_input.segment<3>(0) + translation_drag_coefficients_.asDiagonal() * acceleration_) ;
	map_b(2)= current_input(3)  * X_c.transpose() * rotation_matrix.col(0);
	//map_b(2) *= ;
	
	current_angular_velocity= angular_velocity_inversion.inverse() * map_b;
	applied_input_.segment<3>(0)=current_angular_velocity;
	applied_input_(3)=mass_ * computed_thrust ;
}

void FlatMPC::updateStateTrajectory(){
Eigen::VectorXd state_trajectory, state_knot_trajectory, augumented_state ;
	state_trajectory.resize(kStateSize * horizon_);
	state_knot_trajectory.resize(kStateSize * horizon_);
	augumented_state.resize(kStateSize * (horizon_ + 1));
	
	Eigen::Matrix<double, kStateSize, 1> current_state;
	for(int i=0;i<3;i++)
	{
     	current_state(3 * i)=current_odometry_.position_(i);
		current_state(3 * i +1)=current_odometry_.velocity_(i);
		current_state(3 * i +2)=current_odometry_.acceleration_(i);
    }
	current_state(kStateSize-1)=current_odometry_.yaw_;
	
	state_trajectory = A_cap_ * current_state +  B_cap_ * inputs_ ;
	augumented_state.segment(0,kStateSize) = current_state;
	augumented_state.segment(kStateSize,kStateSize * horizon_) = state_trajectory;
	state_knot_trajectory = A_hat_ * augumented_state ; 
	
	clearTrajectory();
	for(int i=0;i<horizon_;i++)
	{for(int j=0;j<3;j++)
		{
			position_trajectory_[2 * i](j) = state_trajectory(i * kStateSize + 3 * j) ;
			velocity_trajectory_[2 * i](j) = state_trajectory(i * kStateSize + 3 * j + 1) ;
			acceleration_trajectory_[2 * i](j) = state_trajectory(i * kStateSize + 3 * j + 2) ;
			yaw_trajectory_[2 * i] = state_trajectory((i+1) * kStateSize - 1) ;
			
			position_trajectory_[2 * i + 1](j) = state_knot_trajectory(i * kStateSize + 3 * j) ;
			velocity_trajectory_[2 * i + 1](j) = state_knot_trajectory(i * kStateSize + 3 * j + 1) ;
			acceleration_trajectory_[2 * i + 1](j) = state_knot_trajectory(i * kStateSize + 3 * j + 2) ;
			yaw_trajectory_[2 * i + 1] = state_knot_trajectory((i+1) * kStateSize - 1) ;	
		}
	}
}

void FlatMPC::getStateTrajectory(mav_msgs::EigenTrajectoryPointDeque* state_trajectory) const{
	
	assert(state_trajectory !=nullptr);
	state_trajectory->clear();
	
	for(int i=0;i<position_trajectory_.size();i++)
	{mav_msgs::EigenTrajectoryPoint pnt;
    pnt.position_W = position_trajectory_[i];
    pnt.velocity_W = velocity_trajectory_[i];
	pnt.acceleration_W = velocity_trajectory_[i];
	pnt.setFromYaw(yaw_trajectory_[i]);
	
	pnt.time_from_start_ns = static_cast<int64_t>(i) *
                           static_cast<int64_t>(sampling_time_ * 1000000000.0);
    pnt.timestamp_ns = current_odometry_.timestamp_ns + pnt.time_from_start_ns;
    
    state_trajectory->push_back(pnt);
    }
}

}// end mav_fmpc_multi_robot
