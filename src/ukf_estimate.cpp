#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <eigen3/Eigen/Dense>
#include <tf/transform_datatypes.h>
geometry_msgs::PoseWithCovarianceStamped svo_pose;
sensor_msgs::Imu imu_data;

void svo_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg){
  svo_pose = *msg;
}

void imu_cb(const sensor_msgs::Imu::ConstPtr &msg){
  imu_data = *msg;
}

struct Measurement
{
  // The measurement and its associated covariance
  std::vector<float> measurement_;
  Eigen::MatrixXd covariance_;
  int updatesize;
  double mahalanobisThresh_;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};
Measurement measurement;

// Global variable
Eigen::VectorXd state_(15); //x
Eigen::MatrixXd weightedCovarSqrt_; // square root of (L+lamda)*P_k-1
Eigen::MatrixXd estimateErrorCovariance_(15,15); // P_k-1
std::vector<Eigen::VectorXd> sigmaPoints_;
std::vector<double> stateWeights_;
std::vector<double> covarWeights_;
bool uncorrected_;

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

void initialize(){
  double alpha = 1;
  double kappa = 2;
  double beta = 3;
  double lambda_ = 0.01;
  const int STATE_SIZE = 15;
  float sigmaCount = (STATE_SIZE << 1) +1; //2L + 1 = 31(15 states)
  sigmaPoints_.resize(sigmaCount, Eigen::VectorXd(STATE_SIZE));

  //Prepare constants
  //lamda, Wi_c, Wi_m
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

  // Initialize Px
  estimateErrorCovariance_(0,0) = 0.01;// x
  estimateErrorCovariance_(1,1) = 0.02;// y
  estimateErrorCovariance_(2,2) = 0.03;// z
  estimateErrorCovariance_(3,3) = 0.04;// roll
  estimateErrorCovariance_(4,4) = 0.05;// pitch
  estimateErrorCovariance_(5,5) = 0.01;// yaw
  estimateErrorCovariance_(6,6) = 0.01;// Vx
  estimateErrorCovariance_(7,7) = 0.01;// Vy
  estimateErrorCovariance_(8,8) = 0.01;// Vz
  estimateErrorCovariance_(9,9) = 0.01;// Vroll
  estimateErrorCovariance_(10,10) = 0.01;// Vpitch
  estimateErrorCovariance_(11,11) = 0.01;// Vyaw
  estimateErrorCovariance_(12,12) = 0.01;// Ax
  estimateErrorCovariance_(13,13) = 0.01;// Ay
  estimateErrorCovariance_(14,14) = 0.01;// Az
  // Initialize state by using first measurement
  state_(StateMemberX) = svo_pose.pose.pose.position.x;
  state_(StateMemberY) = svo_pose.pose.pose.position.y;
  state_(StateMemberZ) = svo_pose.pose.pose.position.z;
  state_(StateMemberAx) = imu_data.linear_acceleration.x;
  state_(StateMemberAy) = imu_data.linear_acceleration.y;
  state_(StateMemberAz) = (imu_data.linear_acceleration.z - 9.8);
}


void quaternionToRPY(){
  tf::Quaternion quat(imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w);
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  geometry_msgs::Vector3 rpy;
  rpy.x = roll = 5;
  rpy.y = pitch = 5;
  rpy.z = yaw = 5;
  state_[StateMemberRoll] = rpy.x;
  state_[StateMemberPitch] = rpy.y;
  state_[StateMemberYaw] = rpy.z;

}

void writeInMeasurement(){
  measurement.measurement_.resize(15);
  measurement.measurement_[StateMemberX] = svo_pose.pose.pose.position.x;
  measurement.measurement_[StateMemberY] = svo_pose.pose.pose.position.y;
  measurement.measurement_[StateMemberZ] = svo_pose.pose.pose.position.z;
  measurement.measurement_[StateMemberAx] = imu_data.linear_acceleration.x;
  measurement.measurement_[StateMemberAy] = imu_data.linear_acceleration.y;
  measurement.measurement_[StateMemberAz] = imu_data.linear_acceleration.z;

}

void correct(){
  double lambda_ = 0;
  const int STATE_SIZE = 15;



  //correct, calculate sigma points, if uncorrected = true ,than this loop won't be run.
  if (!uncorrected_)
  {
    // caculate square root of (L+lamda)*P_k-1
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

  size_t updateSize = measurement.updatesize ;

  // Now set up the relevant matrices
  Eigen::VectorXd stateSubset(updateSize);                              // x (in most literature)
  Eigen::VectorXd measurementSubset(STATE_SIZE);                        // y
  Eigen::MatrixXd measurementCovarianceSubset(STATE_SIZE, STATE_SIZE);  // Py
  Eigen::MatrixXd stateToMeasurementSubset(STATE_SIZE, STATE_SIZE);     // H
  Eigen::MatrixXd kalmanGainSubset(STATE_SIZE, updateSize);             // K
  Eigen::VectorXd innovationSubset(updateSize);                         // y - Hx
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

  for (size_t i = 0; i < STATE_SIZE; ++i){
    measurementSubset(i) = measurement.measurement_[i];
    //stateSubset(i) = state_(i);
    //measurementCovarianceSubset(i,i) = measurement.covariance_(i,i);

   }

  ROS_INFO("%f",float(measurementSubset[0]));
  // The state-to-measurement function, H, will now be a measurement_size x full_state_size
  // matrix, with ones in the (i, i) locations of the values to be updated
  stateToMeasurementSubset(0,0) = 1;
  stateToMeasurementSubset(1,1) = 1;
  stateToMeasurementSubset(2,2) = 1;
  stateToMeasurementSubset(3,3) = 0;
  stateToMeasurementSubset(4,4) = 0;
  stateToMeasurementSubset(5,5) = 0;
  stateToMeasurementSubset(6,6) = 0;
  stateToMeasurementSubset(7,7) = 0;
  stateToMeasurementSubset(8,8) = 0;
  stateToMeasurementSubset(9,9) = 0;
  stateToMeasurementSubset(10,10) = 0;
  stateToMeasurementSubset(11,11) = 0;
  stateToMeasurementSubset(12,12) = 1;
  stateToMeasurementSubset(13,13) = 1;
  stateToMeasurementSubset(14,14) = 1;

  //The measurecovariance subset

  measurementCovarianceSubset(0,0) = 0.01;
  measurementCovarianceSubset(1,1) = 0.01;
  measurementCovarianceSubset(2,2) = 0.01;
  measurementCovarianceSubset(12,12) = 0.01;
  measurementCovarianceSubset(13,13) = 0.01;
  measurementCovarianceSubset(14,14) = 0.01;

  // (5) Generate sigma points, use them to generate a predicted measurement,y_k_hat-
  for (size_t sigmaInd = 0; sigmaInd < sigmaPoints_.size(); ++sigmaInd)
  {
    sigmaPointMeasurements[sigmaInd] = stateToMeasurementSubset * sigmaPoints_[sigmaInd];
    // y = sum of (wi*yi)
    predictedMeasurement.noalias() += stateWeights_[sigmaInd] * sigmaPointMeasurements[sigmaInd];
  }

  // (6) Use the sigma point measurements and predicted measurement to compute a predicted
  // measurement covariance matrix P_yy and a state/measurement cross-covariance matrix P_xy.
  for (size_t sigmaInd = 0; sigmaInd < sigmaPoints_.size(); ++sigmaInd)
  {
    sigmaDiff = sigmaPointMeasurements[sigmaInd] - predictedMeasurement;//Y(i)_k|k-1 - y_k_hat-
    predictedMeasCovar.noalias() += covarWeights_[sigmaInd] * (sigmaDiff * sigmaDiff.transpose());//P_y_k~_y_k_~
    crossCovar.noalias() += covarWeights_[sigmaInd] * ((sigmaPoints_[sigmaInd] - state_) * sigmaDiff.transpose());//P_x_k_y_k
  }

  // (7) Compute the Kalman gain, making sure to use the actual measurement covariance: K = P_x_k_y_k * (P_y_k~_y_k_~ + R)^-1
  // kalman gain :https://dsp.stackexchange.com/questions/2347/how-to-understand-kalman-gain-intuitively
  Eigen::MatrixXd invInnovCov = (predictedMeasCovar + measurementCovarianceSubset).inverse();
  kalmanGainSubset = crossCovar * invInnovCov;


  // (8) Apply the gain to the difference between the actual and predicted measurements: x = x + K(y - y_hat)
  // y - y_hat

  innovationSubset = (measurementSubset - predictedMeasurement);

  // Wrap angles in the innovation

  // (8.1) Check Mahalanobis distance of innovation

  if (checkMahalanobisThreshold(innovationSubset, invInnovCov, measurement.mahalanobisThresh_))
  {
    // x = x + K*(y - y_hat)
    state_.noalias() += kalmanGainSubset * innovationSubset;

    // (9) Compute the new estimate error covariance P = P - (K * P_yy * K')
    estimateErrorCovariance_.noalias() -= (kalmanGainSubset * predictedMeasCovar * kalmanGainSubset.transpose());

    //wrapStateAngles();

    // Mark that we need to re-compute sigma points for successive corrections
    uncorrected_ = false;
  }

}

void predict(const double referenceTime, const double delta)
{
  Eigen::MatrixXd transferFunction_(15,15);

  const int STATE_SIZE = 15;
  double lambda_ = 0.01;
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

  // Prepare the transfer function
  transferFunction_(StateMemberX, StateMemberVx) = cy * cp * delta;
  transferFunction_(StateMemberX, StateMemberVy) = (cy * sp * sr - sy * cr) * delta;
  transferFunction_(StateMemberX, StateMemberVz) = (cy * sp * cr + sy * sr) * delta;
  transferFunction_(StateMemberX, StateMemberAx) = 0.5 * transferFunction_(StateMemberX, StateMemberVx) * delta;
  transferFunction_(StateMemberX, StateMemberAy) = 0.5 * transferFunction_(StateMemberX, StateMemberVy) * delta;
  transferFunction_(StateMemberX, StateMemberAz) = 0.5 * transferFunction_(StateMemberX, StateMemberVz) * delta;
  transferFunction_(StateMemberY, StateMemberVx) = sy * cp * delta;
  transferFunction_(StateMemberY, StateMemberVy) = (sy * sp * sr + cy * cr) * delta;
  transferFunction_(StateMemberY, StateMemberVz) = (sy * sp * cr - cy * sr) * delta;
  transferFunction_(StateMemberY, StateMemberAx) = 0.5 * transferFunction_(StateMemberY, StateMemberVx) * delta;
  transferFunction_(StateMemberY, StateMemberAy) = 0.5 * transferFunction_(StateMemberY, StateMemberVy) * delta;
  transferFunction_(StateMemberY, StateMemberAz) = 0.5 * transferFunction_(StateMemberY, StateMemberVz) * delta;
  transferFunction_(StateMemberZ, StateMemberVx) = -sp * delta;
  transferFunction_(StateMemberZ, StateMemberVy) = cp * sr * delta;
  transferFunction_(StateMemberZ, StateMemberVz) = cp * cr * delta;
  transferFunction_(StateMemberZ, StateMemberAx) = 0.5 * transferFunction_(StateMemberZ, StateMemberVx) * delta;
  transferFunction_(StateMemberZ, StateMemberAy) = 0.5 * transferFunction_(StateMemberZ, StateMemberVy) * delta;
  transferFunction_(StateMemberZ, StateMemberAz) = 0.5 * transferFunction_(StateMemberZ, StateMemberVz) * delta;
  transferFunction_(StateMemberRoll, StateMemberVroll) = transferFunction_(StateMemberX, StateMemberVx);
  transferFunction_(StateMemberRoll, StateMemberVpitch) = transferFunction_(StateMemberX, StateMemberVy);
  transferFunction_(StateMemberRoll, StateMemberVyaw) = transferFunction_(StateMemberX, StateMemberVz);
  transferFunction_(StateMemberPitch, StateMemberVroll) = transferFunction_(StateMemberY, StateMemberVx);
  transferFunction_(StateMemberPitch, StateMemberVpitch) = transferFunction_(StateMemberY, StateMemberVy);
  transferFunction_(StateMemberPitch, StateMemberVyaw) = transferFunction_(StateMemberY, StateMemberVz);
  transferFunction_(StateMemberYaw, StateMemberVroll) = transferFunction_(StateMemberZ, StateMemberVx);
  transferFunction_(StateMemberYaw, StateMemberVpitch) = transferFunction_(StateMemberZ, StateMemberVy);
  transferFunction_(StateMemberYaw, StateMemberVyaw) = transferFunction_(StateMemberZ, StateMemberVz);
  transferFunction_(StateMemberVx, StateMemberAx) = delta;
  transferFunction_(StateMemberVy, StateMemberAy) = delta;
  transferFunction_(StateMemberVz, StateMemberAz) = delta;

  // (1) Take the square root of a small fraction of the estimateErrorCovariance_ using LL' decomposition
  // caculate square root of (L+lamda)*P_k-1
  // This will be a diagonal matrix (15*15)
  weightedCovarSqrt_ = ((0*STATE_SIZE + lambda_) * estimateErrorCovariance_).llt().matrixL();


  // (2) Compute sigma points *and* pass them through the transfer function to save
  // the extra loop

  // First sigma point(through transferfunction) is the current state
  // x_k|k-1(0)
  // sigmaPoint_[0][0~14]
  sigmaPoints_[0] = transferFunction_ * state_;



  // Next STATE_SIZE sigma points are state + weightedCovarSqrt_[ith column]
  // STATE_SIZE sigma points after that are state - weightedCovarSqrt_[ith column]
  for (size_t sigmaInd = 0; sigmaInd < STATE_SIZE; ++sigmaInd)
  {
    sigmaPoints_[sigmaInd + 1] = transferFunction_ * (state_ + weightedCovarSqrt_.col(sigmaInd));
    sigmaPoints_[sigmaInd + 1 + STATE_SIZE] = transferFunction_ * (state_ - weightedCovarSqrt_.col(sigmaInd));
  }





  // (3) Sum the weighted sigma points to generate a new state prediction
  // x_k_hat- = w_im * x_k|k-1
  state_.setZero();
  for (size_t sigmaInd = 0; sigmaInd < sigmaPoints_.size(); ++sigmaInd)
  {
    state_.noalias() += stateWeights_[sigmaInd] * sigmaPoints_[sigmaInd];
  }


  // (4) Now us the sigma points and the predicted state to compute a predicted covariance P_k-
  estimateErrorCovariance_.setZero();
  Eigen::VectorXd sigmaDiff(STATE_SIZE);
  for (size_t sigmaInd = 0; sigmaInd < sigmaPoints_.size(); ++sigmaInd)
  {
    sigmaDiff = (sigmaPoints_[sigmaInd] - state_);
    estimateErrorCovariance_.noalias() += covarWeights_[sigmaInd] * (sigmaDiff * sigmaDiff.transpose());
  }
  // Mark that we can keep these sigma points
      uncorrected_ = true;

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ukf_estimate");
  ros::NodeHandle nh;
  ros::Subscriber svo_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/svo/pose_imu2", 10, svo_cb);
  ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, imu_cb);
  initialize();
  ros::Rate rate(50);
  while(ros::ok()){
    quaternionToRPY();
    writeInMeasurement();
    predict(1,0.1);
    correct();

  }
}
