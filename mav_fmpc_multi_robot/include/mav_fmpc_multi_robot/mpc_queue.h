#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <deque>
#include <chrono>
#include <thread>
#include <mutex>
#include <memory>
#include <visualization_msgs/Marker.h>

namespace mav_fmpc_multi_robot
{typedef std::deque<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > Vector3dDeque;
class MPCQueue
{
 public:
  MPCQueue(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
  ~MPCQueue();

	void initializeQueue(const mav_msgs::EigenTrajectoryPoint& point,
			double controller_sampling_time, double prediction_sampling_time,int mpc_queue_size);

	void initializeQueue(const mav_msgs::EigenOdometry& odometry,
			double controller_sampling_time, double prediction_sampling_time,int mpc_queue_size);

	void initializeQueue(double controller_sampling_time,
			double prediction_sampling_time, int mpc_queue_size);

void insertReference(const mav_msgs::EigenTrajectoryPoint& point);
void insertReferenceTrajectory(const mav_msgs::EigenTrajectoryPointDeque& queue);

void getQueue(Vector3dDeque& position_reference, Vector3dDeque& velocity_reference,Vector3dDeque& acceleration_reference,Vector3dDeque& jerk_reference, std::deque<double>& yaw_reference,std::deque<double>& yaw_rate_reference,bool knot);
void getQueue(Vector3dDeque& position_reference, Vector3dDeque& velocity_reference,Vector3dDeque& acceleration_reference, std::deque<double>& yaw_reference);
void getThirdOrderQueue(Eigen::VectorXd* state_reference, Eigen::VectorXd* input_reference, bool knot);

void updateQueue();

bool empty() const { return current_queue_size_ == 0; }

private:
  static constexpr int kStateSize = 10;
  static constexpr int kInputSize = 4;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  int minimum_queue_size_;
  int mpc_queue_size_;
  const int maximum_queue_size_;
  int current_queue_size_;
  bool initialized_;

  double prediction_sampling_time_;
  double queue_dt_;

  Vector3dDeque position_reference_, velocity_reference_, acceleration_reference_, jerk_reference_ ;
  std::deque<double> yaw_reference_, yaw_rate_reference_ ;

  double queue_start_time_;

  void clearQueue();
  void fillQueueWithPoint(const mav_msgs::EigenTrajectoryPoint& point);
  void pushBackPoint(const mav_msgs::EigenTrajectoryPoint& point);
  void popFrontPoint();
  void popBackPoint();
  void getLastPoint(mav_msgs::EigenTrajectoryPoint* point);
  void shrinkQueueToMinimum();

  //interpolate the reference queue to the controller update rate
  void linearInterpolateTrajectory(const mav_msgs::EigenTrajectoryPointDeque& input_queue,  mav_msgs::EigenTrajectoryPointDeque& output_queue);


  void printQueue();

  ros::Publisher trajectory_reference_vis_publisher_;
  void publishQueueMarker(const ros::TimerEvent&);
  ros::Timer publish_queue_marker_timer_;
  std::string reference_frame_id_;
};
}
