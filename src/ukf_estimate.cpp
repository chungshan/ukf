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

typedef struct Measurement
{
  // The measurement and its associated covariance;
  Eigen::VectorXd measurement_;
  Eigen::MatrixXd covariance_;
  int updatesize;
  double mahalanobisThresh_;

}Measurement;
Measurement measurement;
Eigen::VectorXd state_;


enum StateMembers
{
  StateMemberX = 0,
  StateMemberY,
  StateMemberZ,
  StateMemberRoll,
  StateMemberPitch,
  StateMemberYaw,
  StateMemberVx,
  StateMemberVy,
  StateMemberVz,
  StateMemberVroll,
  StateMemberVpitch,
  StateMemberVyaw,
  StateMemberAx,
  StateMemberAy,
  StateMemberAz
};

bool checkMahalanobisThreshold(const Eigen::VectorXd &innovation,
                               const Eigen::MatrixXd &invCovariance,
                               const double nsigmas)
{
  double sqMahalanobis = innovation.dot(invCovariance * innovation);
  double threshold = nsigmas * nsigmas;

  if (sqMahalanobis >= threshold)
  {
    return false;
  }

  return true;
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
  bool uncorrected_;




  float sigmaCount = (STATE_SIZE << 1) +1;
  sigmaPoints_.resize(sigmaCount, Eigen::VectorXd(STATE_SIZE));

  //Prepare constants
  //lamda, Wi_c,
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
  if (!uncorrected_)
  {
    // root square of (L+lamda)*Px
    weightedCovarSqrt_ = ((STATE_SIZE + lambda_) * estimateErrorCovariance_).llt().matrixL();
    // First sigma point is the current state
    sigmaPoints_[0] = state_;
    // Generate the sigma points
    // x_i = x + weightedCovarSqrt_ , i = 1, ..., L
    // x_i = x - weightedCovarSqrt_ , i = L+1, ..., 2L
    for (size_t sigmaInd = 0; sigmaInd < STATE_SIZE; ++sigmaInd)
     {
      sigmaPoints_[sigmaInd + 1] = state_ + weightedCovarSqrt_.col(sigmaInd);
      sigmaPoints_[sigmaInd + 1 + STATE_SIZE] = state_ - weightedCovarSqrt_.col(sigmaInd);
     }
  }


  // We don't want to update everything, so we need to build matrices that only update
  // the measured parts of our state vector

  // First, determine how many state vector values we're updating

  size_t updateSize = measurement.updatesize;

  // Now set up the relevant matrices
  Eigen::VectorXd stateSubset(updateSize);                              // x (in most literature)
  Eigen::VectorXd measurementSubset(updateSize);                        // y
  Eigen::MatrixXd measurementCovarianceSubset(updateSize, updateSize);  // R
  Eigen::MatrixXd stateToMeasurementSubset(updateSize, STATE_SIZE);     // H
  Eigen::MatrixXd kalmanGainSubset(STATE_SIZE, updateSize);             // K
  Eigen::VectorXd innovationSubset(updateSize);                         // z - Hx
  Eigen::VectorXd predictedMeasurement(updateSize);
  Eigen::VectorXd sigmaDiff(updateSize);
  Eigen::MatrixXd predictedMeasCovar(updateSize, updateSize);
  Eigen::MatrixXd crossCovar(STATE_SIZE, updateSize);

  std::vector<Eigen::VectorXd> sigmaPointMeasurements(sigmaPoints_.size(), Eigen::VectorXd(updateSize));


  stateSubset.setZero();
  measurementSubset.setZero();
  measurementCovarianceSubset.setZero();
  stateToMeasurementSubset.setZero();
  kalmanGainSubset.setZero();
  innovationSubset.setZero();
  predictedMeasurement.setZero();
  predictedMeasCovar.setZero();
  crossCovar.setZero();

  // Now build the sub-matrices from the full-sized matrices
  for (size_t i = 0; i < updateSize; ++i){
    measurementSubset(i) = measurement.measurement_(i);
    stateSubset(i) = state_(i);
    for (size_t j = 0; j < updateSize; ++j){
      measurementCovarianceSubset(i,j) = measurement.covariance_(i);
    }
   }

  // The state-to-measurement function, h, will now be a measurement_size x full_state_size
  // matrix, with ones in the (i, i) locations of the values to be updated
  for (size_t i = 0; i < updateSize; ++i)
  {
    stateToMeasurementSubset(i, i) = 1;
  }

  // (1) Generate sigma points, use them to generate a predicted measurement
  for (size_t sigmaInd = 0; sigmaInd < sigmaPoints_.size(); ++sigmaInd)
  {
    sigmaPointMeasurements[sigmaInd] = stateToMeasurementSubset * sigmaPoints_[sigmaInd];
    // y = sum of (wi*yi)
    predictedMeasurement.noalias() += stateWeights_[sigmaInd] * sigmaPointMeasurements[sigmaInd];
  }

  // (2) Use the sigma point measurements and predicted measurement to compute a predicted
  // measurement covariance matrix P_yy and a state/measurement cross-covariance matrix P_xy.
  for (size_t sigmaInd = 0; sigmaInd < sigmaPoints_.size(); ++sigmaInd)
  {
    sigmaDiff = sigmaPointMeasurements[sigmaInd] - predictedMeasurement;
    predictedMeasCovar.noalias() += covarWeights_[sigmaInd] * (sigmaDiff * sigmaDiff.transpose());
    crossCovar.noalias() += covarWeights_[sigmaInd] * ((sigmaPoints_[sigmaInd] - state_) * sigmaDiff.transpose());
  }

  // (3) Compute the Kalman gain, making sure to use the actual measurement covariance: K = P_xz * (P_zz + R)^-1
  Eigen::MatrixXd invInnovCov = (predictedMeasCovar + measurementCovarianceSubset).inverse();
  kalmanGainSubset = crossCovar * invInnovCov;

  // (4) Apply the gain to the difference between the actual and predicted measurements: x = x + K(y - y_hat)
  // y - y_hat

  innovationSubset = (measurementSubset - predictedMeasurement);

  // Wrap angles in the innovation

  // (5) Check Mahalanobis distance of innovation

  if (checkMahalanobisThreshold(innovationSubset, invInnovCov, measurement.mahalanobisThresh_))
  {
    // x = x + K*(y - y_hat)
    state_.noalias() += kalmanGainSubset * innovationSubset;

    // (6) Compute the new estimate error covariance P = P - (K * P_yy * K')
    estimateErrorCovariance_.noalias() -= (kalmanGainSubset * predictedMeasCovar * kalmanGainSubset.transpose());

    //wrapStateAngles();

    // Mark that we need to re-compute sigma points for successive corrections
    uncorrected_ = false;
  }
}

void predict(const double referenceTime, const double delta)
{
  Eigen::MatrixXd transferFunction_;
  double roll = state_(StateMemberRoll);
  double pitch = state_(StateMemberPitch);
  double yaw = state_(StateMemberYaw);

  // We'll need these trig calculations a lot.
  double sp = ::sin(pitch);
  double cp = ::cos(pitch);

  double sr = ::sin(roll);
  double cr = ::cos(roll);

  double sy = ::sin(yaw);
  double cy = ::cos(yaw);
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
