#include <Eigen/Eigen>
#include <unsupported/Eigen/MatrixFunctions>

#include <iostream>
#include <deque>
#include <algorithm>
#include <stdio.h> 
#include <stdlib.h> 

#include <chrono>
#include <thread>

int main(){
int horizon_;
double discretization_;

std::cout<<"Enter future steps to predict\n";
std::cin>>horizon_;

std::cout<<"Enter discretization\n";
std::cin>>discretization_;

static constexpr int kStateSize = 10;
static constexpr int kInputSize = 4;
static constexpr int kThrustSize = 6;
static constexpr int kJerkSize = 3;
static constexpr double kGravity = 9.8066;

srand(time(0)); 

Eigen::Matrix<double, 10, 1> odometry_ , Oodometry_;

auto start = std::chrono::steady_clock::now(); 

Eigen::Vector3d translation_drag_coefficients_ ;
translation_drag_coefficients_<<0.1,0.01,0;

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
	B_continuous(9,3)=1;

/*std::cout<<"System Matrix is:"<<A_continuous<<"\n";
std::cout<<"input Matrix is:"<<B_continuous<<"\n";*/
	
Eigen::MatrixXd Z=(Eigen::MatrixXd::Identity(kStateSize,kStateSize) + 4 * std::pow(discretization_,2) * A_continuous.pow(2)/48 - A_continuous * discretization_/2  ).inverse(); 

Eigen::Matrix<double,kStateSize,kStateSize> A = Z * (Eigen::MatrixXd::Identity(kStateSize,kStateSize) + 4 * std::pow(discretization_,2) * A_continuous.pow(2)/48 + A_continuous * discretization_/2 );

Eigen::Matrix<double, kStateSize,kInputSize> B = Z * 4 * B_continuous ; // u_{k + 0.5}
Eigen::Matrix<double, kStateSize,kInputSize> B_ = Z * ( B_continuous * discretization_/6 + std::pow(discretization_,2)/12 * A_continuous * B_continuous) ; // u_k
Eigen::Matrix<double, kStateSize,kInputSize> B_i = Z * ( B_continuous * discretization_/6 - std::pow(discretization_,2)/12 * A_continuous * B_continuous) ; // u_{k+1} 

/*std::cout<<"Inverse Matrix is:"<<Z<<"\n";
std::cout<<"Discrete System  Matrix is:"<<A<<"\n";
std::cout<<"Discrete Matrix is:"<<B_<<"\n";
std::cout<<"Discrete knot Matrix is:"<<B<<"\n";
std::cout<<"Discrete u +1 Matrix is:"<<B_i<<"\n";*/

Eigen::MatrixXd A_cap_ = Eigen::MatrixXd::Zero(kStateSize * horizon_, kStateSize);
Eigen::MatrixXd B_cap_ = Eigen::MatrixXd::Zero(kStateSize * horizon_, kInputSize * (2 * horizon_ + 1));
Eigen::MatrixXd A_hat_ = Eigen::MatrixXd::Zero(kStateSize * horizon_, kStateSize * (horizon_ +1));

for(int i=0;i<horizon_;i++)
{   //auto initial = std::chrono::steady_clock::now();
	A_cap_.block<kStateSize,kStateSize>(kStateSize * i,0)= A.pow(i+1);
	//auto stop = std::chrono::steady_clock::now();
	//auto time_to_compute = stop-initial ;
	std::cout<<i<<std::endl;
	//std::this_thread::sleep_for(std::chrono::seconds(2));
	//std::cout <<"time taken to compute power\t"<<std::chrono::duration <double, std::micro>(time_to_compute).count()<<" microseconds"<<std::endl;
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
	A_hat_.block<kStateSize,kStateSize>(kStateSize * i, kStateSize * i)=Eigen::MatrixXd::Identity(kStateSize,kStateSize)/2 + A_continuous * 0.1/8;
    A_hat_.block<kStateSize,kStateSize>(kStateSize * i, kStateSize * (i+1))=Eigen::MatrixXd::Identity(kStateSize,kStateSize)/2 - A_continuous * 0.1/8;	
}

Eigen::MatrixXd B_hat = Eigen::MatrixXd::Zero(kStateSize * (horizon_ +1), kInputSize * (2 * horizon_ + 1));
B_hat.block(kStateSize,0,kStateSize * horizon_, kInputSize * (2 * horizon_ + 1)) = B_cap_ ;

Eigen::MatrixXd H_ = (B_cap_.transpose() + 4 * (A_hat_ * B_hat).transpose()) * (B_cap_ + 4 * (A_hat_ * B_hat)) ;
//H_ +=R_cap_ ; 
H_ *= discretization_/6 ;

auto stop = std::chrono::steady_clock::now();
auto time_taken = stop - start;

Eigen::SparseMatrix<double> dummy=H_.sparseView();

std::cout<<"computation time\t"<<std::chrono::duration <double, std::micro>(time_taken).count()<<" microseconds"<<std::endl;
std::cout<<"sparse amtrix is:"<<dummy<<std::endl;

/*Eigen::Matrix<double,6,10> constraint ;
constraint(0,1)=1;
constraint(1,2)=1;
constraint(2,4)=1;
constraint(3,5)=1;
constraint(4,7)=1;
constraint(5,8)=1;

C_= Eigen::MatrixXd::Zero(6 * N, 10 * N);

for(int i=0;i<N;i++)
{ 
 C_.block<6,10>(6 * i, 10 * i)=constraint;	
}*/

//Eigen::VectorXd X_ = A_ * odometry_ + B_ * U_ ;

//Eigen::VectorXd Y_ = A_ * Oodometry_ + B_ * V_ ;

//Eigen::VectorXd diff= ((C * (X_ -Y_)) * (C * (X_ -Y_)).transpose()).diagonal();


/*for(int i=0;i<N;i++)
{double distance=0;
	for(int j=0;j<3;j++)
	distance+=diff(3 * i +j);
if(distance <=1.2)
{
	std::cout<<"Robot in collision sphere\n";
}
else if (distance <=5 && distance > 1.2)
{
	std::cout<<"Robot in safety sphere\n";
}
else
{
	std::cout<<"Robot outside safety sphere\n";
}
} 

start = std::chrono::steady_clock::now();
time_taken =start - stop;

std::cout <<"collision check time\t"<<std::chrono::duration <double, std::micro>(time_taken).count()<<" microseconds"<<std::endl*/;

}
