#include <ros/ros.h>




#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <eigen3/Eigen/Dense>
geometry_msgs::PoseWithCovarianceStamped svo_pose;
sensor_msgs::Imu imu_data;

void svo_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg){
  svo_pose = *msg;
}

void imu_cb(const sensor_msgs::Imu::ConstPtr &msg){
  imu_data = *msg;
}

void correct(){
  double alpha = 0;
  double kappa = 0;
  double beta = 0;
  double lambda_ = 0;
  const int STATE_SIZE = 15;
  std::vector<Eigen::VectorXd> sigmaPoints_;
  std::vector<double> stateWeights_;
  std::vector<double> covarWeights_;
  Eigen::MatrixXd weightedCovarSqrt_;
  Eigen::MatrixXd estimateErrorCovariance_;



  float sigmaCount = (STATE_SIZE << 1) +1;
  sigmaPoints_.resize(sigmaCount, Eigen::VectorXd(STATE_SIZE));

  //Prepare constants
  lambda_ = alpha * alpha * (STATE_SIZE + kappa) - STATE_SIZE;
  stateWeights_.resize(sigmaCount);
  covarWeights_.resize(sigmaCount);

  stateWeights_[0] = lambda_ / (STATE_SIZE + lambda_);
  covarWeights_[0] = stateWeights_[0] + (1 - (alpha * alpha) + beta);
  sigmaPoints_[0].setZero();

  for (size_t i = 1; i < sigmaCount; ++i)
  {
    sigmaPoints_[i].setZero();
    stateWeights_[i] =  1 / (2 * (STATE_SIZE + lambda_));
    covarWeights_[i] = stateWeights_[i];
  }

  //correct

  weightedCovarSqrt_ = ((STATE_SIZE + lambda_) * estimateErrorCovariance_).llt().matrixL();


}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ukf_estimate");
  ros::NodeHandle nh;
  ros::Subscriber svo_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/svo/pose_imu2", 10, svo_cb);
  ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, imu_cb);
  correct();
  ROS_INFO("Hello world!");
}
