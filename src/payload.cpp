#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <eigen3/Eigen/Dense>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/VFR_HUD.h>
#include <UKF/output.h>
#include <std_msgs/Float64.h>
#include <string>
using namespace std;
geometry_msgs::PoseWithCovarianceStamped svo_pose;
geometry_msgs::PoseStamped mocap_pose;
sensor_msgs::Imu imu_data;
nav_msgs::Odometry filterd;
mavros_msgs::VFR_HUD vfr_data;
UKF::output output_data;
geometry_msgs::PoseStamped measure_data;
/*
void svo_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg){
  svo_pose = *msg;
}

void mocap_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
  mocap_pose = *msg;
}

void imu_cb(const sensor_msgs::Imu::ConstPtr &msg){
  imu_data = *msg;//test
}

void vfr_cb(const mavros_msgs::VFR_HUD::ConstPtr &msg){
  vfr_data = *msg;//test
}
*/
void measure_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
  measure_data = *msg;
}
void output_cb(const UKF::output::ConstPtr &msg){
  output_data = *msg;
}
struct Measurement
{
  // The measurement and its associated covariance
  std::vector<float> measurement_;
  Eigen::MatrixXd covariance_;
  int updatesize;
  double mahalanobisThresh_;


};
Measurement measurement;

// Global variable
const int STATE_SIZE = 28;
Eigen::VectorXd state_(STATE_SIZE); //x
Eigen::MatrixXd weightedCovarSqrt_(STATE_SIZE,STATE_SIZE); // square root of (L+lamda)*P_k-1
Eigen::MatrixXd estimateErrorCovariance_(STATE_SIZE,STATE_SIZE); // P_k-1
Eigen::VectorXd process_noise(STATE_SIZE);
std::vector<Eigen::VectorXd> sigmaPoints_;
std::vector<double> stateWeights_;
std::vector<double> covarWeights_;
double lambda_;
bool uncorrected_;
int flag;
int flag2;
int flag3;
float payload_pitch, payload_yaw, payload_roll;
float thrust;
float a_g;
float imu_ax_bias;
float imu_ay_bias;
/*test variable*/




enum StateMembers
{
  x_c = 0,
  y_c,
  z_c,
  Vx_c,
  Vy_c,
  Vz_c,
  pitch_c,
  yaw_c,
  roll_c,
  Vpitch_c,
  Vyaw_c,
  Vroll_c,
  pitch_p,
  yaw_p,
  roll_p,
  Fx_f,
  Fy_f,
  Fz_f,
  Fx_l,
  Fy_l,
  Fz_l,
  Ax_f,
  Ay_f,
  Az_f,
  a_g_x,
  a_g_y,
  a_g_z,
  Vpitch_p
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
  ROS_INFO("initilaize");
  /*test variable*/
  /*
  svo_pose.pose.pose.position.x = 0.1;
  svo_pose.pose.pose.position.y = 0.2;
  svo_pose.pose.pose.position.z = 0.3;
  imu_data.linear_acceleration.x = 0.1;
  imu_data.linear_acceleration.y = 0.2;
  imu_data.linear_acceleration.z = 9.9;
  */
  /*test variable*/
  double alpha = 1e-3;
  double kappa = 0;
  double beta = 2;
  //const int STATE_SIZE = 19;
  float sigmaCount = (STATE_SIZE << 1) +1; //2L + 1 = 37(19 states)
  sigmaPoints_.resize(sigmaCount, Eigen::VectorXd(STATE_SIZE));

  //Prepare constants
  //lamda,
  lambda_ = alpha * alpha * (STATE_SIZE + kappa) - STATE_SIZE;
  //ROS_INFO("lamda = %f", lambda_);
  stateWeights_.resize(sigmaCount);
  covarWeights_.resize(sigmaCount);

  // Wi_c, Wi_m
  stateWeights_[0] = lambda_ / (STATE_SIZE + lambda_);
  //stateWeights_[0] = 1 / (sigmaCount);
  covarWeights_[0] =  stateWeights_[0] + (1 - (alpha * alpha) + beta);
  //covarWeights_[0] = 1 / (sigmaCount);
  sigmaPoints_[0].setZero();
  //ROS_INFO("stateWeights = %f", stateWeights_[0]);
  //ROS_INFO("covarWeights[0] = %f", covarWeights_[0]);

  for (size_t i = 1; i < sigmaCount; ++i)
  {
    sigmaPoints_[i].setZero();
    stateWeights_[i] =  1 / (2 * (STATE_SIZE + lambda_));
    //stateWeights_[i] = 1 / (sigmaCount);
    covarWeights_[i] = stateWeights_[i];
  }
/*
  float sum = 0;
  for(size_t i = 0; i< sigmaCount;i++)
  {
    sum = sum + stateWeights_[i];
  }
  ROS_INFO("sum of weight = %f", sum);
  //ROS_INFO("stateWeights[i] = %f", stateWeights_[1]);

  for (int i = 0; i < sigmaCount; i++)
  {
    printf("%f\n", stateWeights_[i]);
  }
*/
  // Initialize Px,P_k-1
  estimateErrorCovariance_(0,0) = 1e-02;// x_c
  estimateErrorCovariance_(1,1) = 1e-02;// y_c
  estimateErrorCovariance_(2,2) = 1e-02;// z_c
  estimateErrorCovariance_(3,3) = 1e-06;// Vx_c
  estimateErrorCovariance_(4,4) = 1e-06;// Vy_c
  estimateErrorCovariance_(5,5) = 1e-06;// Vz_c
  estimateErrorCovariance_(6,6) = 1e-06;// pitch_c
  estimateErrorCovariance_(7,7) = 1e-06;// yaw_c
  estimateErrorCovariance_(8,8) = 1e-06;// roll_c
  estimateErrorCovariance_(9,9) = 1e-06;// Vpitch_c
  estimateErrorCovariance_(10,10) = 1e-06;// Vyaw_c
  estimateErrorCovariance_(11,11) = 1e-06;// Vroll_c
  estimateErrorCovariance_(12,12) = 1e-02;// pitch_p
  estimateErrorCovariance_(13,13) = 1e-02;// yaw_p
  estimateErrorCovariance_(14,14) = 1e-02;// roll_p
  estimateErrorCovariance_(15,15) = 1e-06;// Vpitch_p
  estimateErrorCovariance_(16,16) = 1e-06;// Vyaw_p
  estimateErrorCovariance_(17,17) = 1e-06;// Vroll_p
  estimateErrorCovariance_(18,18) = 1e-02;//F_x_f
  estimateErrorCovariance_(19,19) = 1e-06;//F_y_f
  estimateErrorCovariance_(20,20) = 1e-06;//F_z_f
  estimateErrorCovariance_(21,21) = 1e-06;//F_x_l
  estimateErrorCovariance_(22,22) = 1e-06;//F_y_l
  estimateErrorCovariance_(23,23) = 1e-06;//F_z_l
  estimateErrorCovariance_(24,24) = 1e-06;//a_g_x
  estimateErrorCovariance_(25,25) = 1e-06;//a_g_y
  estimateErrorCovariance_(26,26) = 1e-06;//a_g_z

/*
  //process noise
  for(int i = 0; i < 19; i++){
    process_noise[i] = 0.0001;
  }



  process_noise[6] = 0.005;
  process_noise[7] = 0.005;
  process_noise[8] = 0.005;
*/




/*
  for (int i = 0; i < 19; i++){
    for (int j = 0; j < 19; j++){

        printf("%.3f ", estimateErrorCovariance_(i,j));


    }
    printf("\n");
  }
*/

  // Initialize state by using first measurement x_0
  state_.setZero();

  uncorrected_ = false;

  measurement.mahalanobisThresh_ = 8;





}

double clamRotation(double rotation)
{
  const double PI = 3.141592653589793;
  const double TAU = 6.283185307179587;
  while (rotation > PI)
  {
    rotation -= TAU;
  }

  while (rotation < -PI)
  {
    rotation += TAU;
  }

  return rotation;

}

void checkData(){
  if(measure_data.pose.position.x != 0 && output_data.force.x != 0 && output_data.Af.x != 0){
    flag = 1;
  }
}

void writeInMeasurement(){

  measurement.measurement_.resize(STATE_SIZE);
  float roll, pitch , yaw;
  float theta_c = 0;
  //const float imu_ax_bias = -0.077781;
  //const float imu_ay_bias = 0.083215;
  Eigen::Matrix3f Rx, Ry, Rz;
  Eigen::Vector3f a_g_inertial;
  Eigen::Vector3f a_g_body;
  Rx.setZero();
  Ry.setZero();
  Rz.setZero();
  a_g_inertial.setZero();

  a_g_inertial(0) = 0;
  a_g_inertial(1) = 0;
  a_g_inertial(2) = a_g;
  //read payload attitude from topic
  roll = payload_roll;
  pitch = payload_pitch;
  yaw = payload_yaw;
  Rx(0,0) = 1;
  Rx(1,0) = 0;
  Rx(2,0) = 0;
  Rx(0,1) = 0;
  Rx(1,1) = cos(roll);
  Rx(1,2) = -sin(roll);
  Rx(2,0) = 0;
  Rx(2,1) = sin(roll);
  Rx(2,2) = cos(roll);

  Ry(0,0) = cos(pitch);
  Ry(1,0) = 0;
  Ry(2,0) = sin(pitch);
  Ry(0,1) = 0;
  Ry(1,1) = 1;
  Ry(1,2) = 0;
  Ry(2,0) = -sin(pitch);
  Ry(2,1) = 0;
  Ry(2,2) = cos(pitch);

  Rz(0,0) = cos(yaw);
  Rz(1,0) = -sin(yaw);
  Rz(2,0) = 0;
  Rz(0,1) = sin(yaw);
  Rz(1,1) = cos(yaw);
  Rz(1,2) = 0;
  Rz(2,0) = 0;
  Rz(2,1) = 0;
  Rz(2,2) = 1;

  a_g_body = Ry*Rx*Rz*a_g_inertial;
  //a_g_body(0) = (sin(yaw)*sin(roll) + cos(yaw)*sin(pitch)*cos(roll)) * 9.8;
  a_g_body(0) = sin(pitch)*cos(roll)*a_g;

  measurement.measurement_[x_c] = measure_data.pose.position.x;
  measurement.measurement_[y_c] = measure_data.pose.position.y;
  measurement.measurement_[z_c] = measure_data.pose.position.z;

  theta_c = atan2(output_data.force.z, output_data.force.x);

  measurement.measurement_[pitch_c] = theta_c;
  /*
  measurement.measurement_[yaw_c] = ;
  measurement.measurement_[roll_c] = ;
  */
  /*
  measurement.measurement_[pitch_p] = ;
  measurement.measurement_[yaw_p] = ;
  measurement.measurement_[roll_p] = ;
  */

  measurement.measurement_[Fx_f] = output_data.force.x;
  measurement.measurement_[Fy_f] = output_data.force.y;
  measurement.measurement_[Fz_f] = output_data.force.z;




  state_[Ax_f] = output_data.Af.x;
  state_[Ay_f] = output_data.Af.y;
  state_[Az_f] = output_data.Af.z;
  //ROS_INFO("Ax_f = %f, Ay_f = %f, Az_f = %f", state_[Ax_f], state_[Ay_f], state_[Az_f]);

  state_[a_g_x] = a_g_body(0);
  state_[a_g_y] = a_g_body(1);
  state_[a_g_z] = -a_g_body(2);

  //ROS_INFO("ax = %f", measurement.measurement_[StateMemberAz]);
  /*printf measurement_[i]
  for (int i = 0; i < 19 ; i++)
  {
    printf("%f ",measurement.measurement_[i]);
  }
  printf("\n");
*/
  //ROS_INFO("meas_x = %f, mea_y = %f", measurement.measurement_[StateMemberX], measurement.measurement_[StateMemberY]);

}

void correct(){
  //ROS_INFO("---correct start---\n");


  //const int STATE_SIZE = 19;
  const double PI = 3.141592653589793;
  const double TAU = 6.283195307179587;

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

  //size_t updateSize = 19 ;

  // Now set up the relevant matrices
  //Eigen::VectorXd stateSubset(updateSize);                              // x (in most literature)
  Eigen::VectorXd measurementSubset(STATE_SIZE);                        // y
  Eigen::MatrixXd measurementCovarianceSubset(STATE_SIZE, STATE_SIZE);  // Py
  Eigen::MatrixXd stateToMeasurementSubset(STATE_SIZE, STATE_SIZE);     // H
  Eigen::MatrixXd kalmanGainSubset(STATE_SIZE, STATE_SIZE);             // K
  Eigen::VectorXd innovationSubset(STATE_SIZE);                         // y - Hx
  Eigen::VectorXd predictedMeasurement(STATE_SIZE);
  Eigen::VectorXd sigmaDiff(STATE_SIZE);
  Eigen::MatrixXd predictedMeasCovar(STATE_SIZE, STATE_SIZE);
  Eigen::MatrixXd crossCovar(STATE_SIZE, STATE_SIZE);

  std::vector<Eigen::VectorXd> sigmaPointMeasurements(sigmaPoints_.size(), Eigen::VectorXd(STATE_SIZE));


  //stateSubset.setZero();
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

  // The state-to-measurement function, H, will now be a measurement_size x full_state_size
  // matrix, with ones in the (i, i) locations of the values to be updated
  stateToMeasurementSubset(0,0) = 1;
  stateToMeasurementSubset(1,1) = 1;
  stateToMeasurementSubset(2,2) = 1;
  stateToMeasurementSubset(3,3) = 0;
  stateToMeasurementSubset(4,4) = 0;
  stateToMeasurementSubset(5,5) = 0;
  stateToMeasurementSubset(6,6) = 1;
  stateToMeasurementSubset(7,7) = 1;
  stateToMeasurementSubset(8,8) = 1;
  stateToMeasurementSubset(9,9) = 0;
  stateToMeasurementSubset(10,10) = 0;
  stateToMeasurementSubset(11,11) = 0;
  stateToMeasurementSubset(12,12) = 1;
  stateToMeasurementSubset(13,13) = 1;
  stateToMeasurementSubset(14,14) = 1;
  stateToMeasurementSubset(15,15) = 1;
  stateToMeasurementSubset(16,16) = 1;
  stateToMeasurementSubset(17,17) = 1;
  stateToMeasurementSubset(18,18) = 0;
  stateToMeasurementSubset(19,19) = 0;
  stateToMeasurementSubset(20,20) = 0;
  stateToMeasurementSubset(21,21) = 0;
  stateToMeasurementSubset(22,22) = 0;
  stateToMeasurementSubset(23,23) = 0;
  stateToMeasurementSubset(24,24) = 0;
  stateToMeasurementSubset(25,25) = 0;
  stateToMeasurementSubset(26,26) = 0;
  stateToMeasurementSubset(27,27) = 0;



  //The measurecovariance subset R

  measurementCovarianceSubset(0,0) = 0.2;
  measurementCovarianceSubset(1,1) = 0.2;
  measurementCovarianceSubset(2,2) = 0.2;
  measurementCovarianceSubset(6,6) = 0.2;
  measurementCovarianceSubset(7,7) = 0.2;
  measurementCovarianceSubset(8,8) = 0.2;
  measurementCovarianceSubset(12,12) = 0.2;
  measurementCovarianceSubset(13,13) = 0.2;
  measurementCovarianceSubset(14,14) = 0.2;
  measurementCovarianceSubset(15,15) = 0.2;
  measurementCovarianceSubset(16,16) = 0.2;
  measurementCovarianceSubset(17,17) = 0.2;
  measurementCovarianceSubset(3,3) = measurementCovarianceSubset(4,4) = measurementCovarianceSubset(5,5) = measurementCovarianceSubset(9,9) = measurementCovarianceSubset(10,10) = measurementCovarianceSubset(11,11) = measurementCovarianceSubset(18,18) = measurementCovarianceSubset(19,19) = measurementCovarianceSubset(20,20) = measurementCovarianceSubset(21,21) = measurementCovarianceSubset(22,22) =  measurementCovarianceSubset(23,23) = measurementCovarianceSubset(24,24) = measurementCovarianceSubset(25,25) = measurementCovarianceSubset(26,26) = measurementCovarianceSubset(27,27) = 0.4;

  // (5) Generate sigma points, use them to generate a predicted measurement,y_k_hat-
  for (size_t sigmaInd = 0; sigmaInd < sigmaPoints_.size(); ++sigmaInd)
  {
    sigmaPointMeasurements[sigmaInd] = stateToMeasurementSubset * sigmaPoints_[sigmaInd];
    // y = sum of (wi*yi)
    predictedMeasurement.noalias() += stateWeights_[sigmaInd] * sigmaPointMeasurements[sigmaInd];
  }
/*
  printf("---sigma_Measurements---\n");
  for(int i = 0; i < 37; i++){
    for(int j = 0; j < 19; j++){
      printf("%f ", sigmaPointMeasurements[i][j]);
    }
    printf("\n");
  }
  printf("\n");
*/
/*
  printf("---y_k_hat----\n");
  for(int i = 0; i < 19; i++){
    printf("%f ", predictedMeasurement[i]);

  }
  printf("\n");
*/
  // (6) Use the sigma point measurements and predicted measurement to compute a predicted
  // measurement covariance matrix P_yy and a state/measurement cross-covariance matrix P_xy.

  for (size_t sigmaInd = 0; sigmaInd < sigmaPoints_.size(); ++sigmaInd)
  {
    sigmaDiff = sigmaPointMeasurements[sigmaInd] - predictedMeasurement;//Y(i)_k|k-1 - y_k_hat-
    predictedMeasCovar.noalias() += covarWeights_[sigmaInd] * (sigmaDiff * sigmaDiff.transpose());//P_y_k~_y_k_~
    crossCovar.noalias() += covarWeights_[sigmaInd] * ((sigmaPoints_[sigmaInd] - state_) * sigmaDiff.transpose());//P_x_k_y_k
  }

  //check p_yy
  for (int i = 3; i < 12 ; i++){
    for (int j = 0; j < 3 ; j++){
      predictedMeasCovar(i,j) = 0;
    }
  }
  for (int i = 0; i < 28; i++){
    for (int j = 3; j < 12; j++){
      predictedMeasCovar(i,j) = 0;
    }
  }
  for (int i = 3; i < 12; i++){
    for (int j = 12; j < 18; j++){
      predictedMeasCovar(i,j) = 0;
    }
  }
  for (int i = 0;i < 28; i++){
    for (int j = 18; j < 28; j++){
      predictedMeasCovar(i,j) = 0;
    }
  }
  //check p_xy
  for (int i = 0; i < 28; i++){
    for (int j =3; j < 12; j++){
      crossCovar(i,j) = 0;
    }
  }
  for (int i = 0; i < 28; i++){
    for (int j = 18; j <28; j++){
      crossCovar(i,j) = 0;
    }
  }
  /*
  Eigen::MatrixXd crossCovar1(STATE_SIZE, updateSize);
  for (size_t sigmaInd = 0; sigmaInd < sigmaPoints_.size(); ++sigmaInd)
  {
    crossCovar1 = ((sigmaPoints_[sigmaInd] - state_) * sigmaDiff.transpose());
    for(int i = 0; i < 19; i++){
      for(int j = 0 ; j < 19; j++){
        printf("%f \n", crossCovar(i,j));
      }
      printf("---next crossCovar---\n");
    }

  }
*/



/*
  printf("---sigmaDiff of y---\n");
  for(int i = 0; i < 19; i++){
    printf("%f ", sigmaDiff[i]);
  }
  printf("\n");
  */
/*
  printf("---P_y_k~_y_k_~---\n");
  for(int i = 0; i < 19; i++){
    for(int j = 0 ; j < 19; j++){
      printf("%f ", predictedMeasCovar(i,j));
    }
    printf("\n");
  }
  printf("\n");


  printf("---P_x_k_y_k---\n");
  for(int i = 0; i < 19; i++){
    for(int j = 0 ; j < 19; j++){
      printf("%f ", crossCovar(i,j));
    }
    printf("\n");
  }
  printf("\n");
*/
  //ROS_INFO("sigmaPointMeasurements = %f", sigmaPointMeasurements[0][0]);
  //ROS_INFO("predictedMeasCovar = %f", predictedMeasCovar(0,0));
  //ROS_INFO("sigmadiff = %f", sigmaDiff(0,0));
  //ROS_INFO("crossCovar = %f",crossCovar(10,10));

  // (7) Compute the Kalman gain, making sure to use the actual measurement covariance: K = P_x_k_y_k * (P_y_k~_y_k_~ + R)^-1
  // kalman gain :https://dsp.stackexchange.com/questions/2347/how-to-understand-kalman-gain-intuitively
  Eigen::MatrixXd invInnovCov = (predictedMeasCovar + measurementCovarianceSubset).inverse();
  //Eigen::MatrixXd inv_test = predictedMeasCovar + measurementCovarianceSubset;
  //ROS_INFO("invInnovCov = %f", invInnovCov(0,0));
  kalmanGainSubset = crossCovar * invInnovCov;
  //ROS_INFO("kalmanGain = %f", kalmanGainSubset(5,5));
/*
  for(int i = 0;i < 19; i ++){
    for(int j = 12; j < 15; j++){
      if(kalmanGainSubset(i,j) > 0.1)
        kalmanGainSubset(i,j) = 0.1;
      else if(kalmanGainSubset(i,j) < -0.1)
        kalmanGainSubset(i,j) = -0.1;
    }
  }
*/
/*
  printf("---kalmanGain---\n");
  for(int i = 0;i<19;i++){
    for(int j = 0; j < 19; j++){
      printf("%f ", kalmanGainSubset(i,j));
    }
    printf("\n");
  }
  printf("\n");
*/

  // (8) Apply the gain to the difference between the actual and predicted measurements: x = x + K(y - y_hat)
  // y - y_hat
  //ROS_INFO("measure = %f",measurementSubset[0]);
  //ROS_INFO("predic_measure = %f", predictedMeasurement[0]);

  innovationSubset = (measurementSubset - predictedMeasurement);

  //ROS_INFO("innovationSubset = %f", innovationSubset[3]);
  //Eigen::MatrixXd test = kalmanGainSubset * innovationSubset;
  //ROS_INFO("%f", test(0,0));
  //ROS_INFO("state = %f", state_[0]);
/*
  printf("---(y-y_hat)---\n");
  for(int i = 0; i < 19; i++){
    printf("%f ", innovationSubset[i]);
  }
  printf("\n");
*/
  // Wrap angles in the innovation
  while (innovationSubset(roll_p) < -PI)
   {
   innovationSubset(roll_p) += TAU;
   }

   while (innovationSubset(roll_p) > PI)
   {
    innovationSubset(roll_p) -= TAU;
   }

   while (innovationSubset(yaw_p) < -PI)
    {
    innovationSubset(yaw_p) += TAU;
    }

    while (innovationSubset(yaw_p) > PI)
    {
     innovationSubset(yaw_p) -= TAU;
    }

    while (innovationSubset(pitch_p) < -PI)
     {
     innovationSubset(pitch_p) += TAU;
     }

     while (innovationSubset(pitch_p) > PI)
     {
      innovationSubset(pitch_p) -= TAU;
     }
     double sqMahalanobis = innovationSubset.dot(invInnovCov * innovationSubset);
     double threshold = 1 * 1;
     //ROS_INFO("sq = %f", sqMahalanobis);


  // (8.1) Check Mahalanobis distance of innovation
  if (checkMahalanobisThreshold(innovationSubset, invInnovCov, measurement.mahalanobisThresh_))
  {
    // x = x + K*(y - y_hat)
    state_.noalias() += kalmanGainSubset * innovationSubset;
    //ROS_INFO("Vc: x = %f, y = %f, z = %f", state_[Vx_c], state_[Vy_c], state_[Vz_c]);

/*
    if(abs(output.force.x) > 0.3){
      ROS_INFO("Fx is larger than 0.3.");
      ROS_INFO("Fx = %f", output.force.x);
    }
*/
    /*
    if(abs(output.force.y) > 0.3){
      ROS_INFO("Fy is larger than 0.3.");
      ROS_INFO("Fy = %f", output.force.y);
    }
    */
/*
    if(abs(output.force.z) > 0.3){
      ROS_INFO("Fz is larger than 0.3.");
      ROS_INFO("Fz = %f", output.force.z);
    }
*/


  //ROS_INFO("Thrust = %f", state_[StateMemberThrust]);



    // (9) Compute the new estimate error covariance P = P - (K * P_yy * K')
    estimateErrorCovariance_.noalias() -= (kalmanGainSubset * predictedMeasCovar * kalmanGainSubset.transpose());

    //ROS_INFO("predict::estimate = %f",estimateErrorCovariance_(0,0));
    //ROS_INFO("kalman gain = %f", kalmanGainSubset(0,0));
    //ROS_INFO("predictedMeas = %f", predictedMeasCovar(0,0));
/*
    printf("---estimateErrorCov---\n");
    for(int i = 0; i < 19; i++){
      for(int j = 0; j < 19; j++){
        printf("%f ", estimateErrorCovariance_(i,j));
      }
      printf("\n");
    }
    printf("\n");
*/
    //wrapStateAngles();
    state_(roll_p) = clamRotation(state_(roll_p));
    state_(yaw_p) = clamRotation(state_(yaw_p));
    state_(pitch_p) = clamRotation(state_(pitch_p));

    // Mark that we need to re-compute sigma points for successive corrections
    uncorrected_ = false;
    measurement.mahalanobisThresh_ = 2;
    //ROS_INFO("---correct success!!---\n");

  }
  else{
    measurement.mahalanobisThresh_ = 8;
    //ROS_INFO("---correct fail!!---\n");
  }
  //ROS_INFO("thrshold = %f", measurement.mahalanobisThresh_);

}

void predict(const double referenceTime, const double delta)
{
  //ROS_INFO("---Predict start---");
  Eigen::MatrixXd transferFunction_(STATE_SIZE,STATE_SIZE);
  Eigen::MatrixXd process_noise_m(STATE_SIZE,STATE_SIZE);
  double m = 0.6,m_p = 0.6;
  //const int STATE_SIZE = 19;
  float k_drag_x = 0.12;
  float k_drag_y = 0.12;
  float k_drag_z = 0.22;



  double roll = state_(roll_p);
  double pitch = state_(pitch_p);
  double yaw = state_(yaw_p);

  // We'll need these trig calculations a lot.
  double sp = ::sin(pitch);
  double cp = ::cos(pitch);

  double sr = ::sin(roll);
  double cr = ::cos(roll);

  double sy = ::sin(yaw);
  double cy = ::cos(yaw);

  double spi = ::sin(-pitch);
  double cpi = ::cos(-pitch);

  double sri = ::sin(-roll);
  double cri = ::cos(-roll);

  double syi = ::sin(-yaw);
  double cyi = ::cos(-yaw);

  double vpitch = state_[Vpitch_c];
  //l
  double l_x,l_z;
  double r_cp;
  double alpha_p = 0;
  double theta_p = state_[pitch_p];
  double theta_c = state_[pitch_c];
  //transfer function

  //p_c_dot = v_c
  transferFunction_(x_c,x_c) = transferFunction_(y_c,y_c) = transferFunction_(z_c,z_c) = 1;
  transferFunction_(x_c,Vx_c) = cy * cp * delta;
  transferFunction_(x_c,Vy_c) = (cy * sp * sr - sy * cr) * delta;
  transferFunction_(x_c,Vz_c) = (cy * sp * cr + sy * sr) * delta;
  transferFunction_(y_c,Vx_c) = sy * cp * delta;
  transferFunction_(y_c,Vy_c) = (sy * sp * sr + cy * cr) * delta;
  transferFunction_(y_c,Vz_c) = (sy * sp * cr - cy * sr) * delta;
  transferFunction_(z_c,Vx_c) = -sp * delta;
  transferFunction_(z_c,Vy_c) = cp * sr * delta;
  transferFunction_(z_c,Vz_c) = cp * cr * delta;

  //v_c_dot = a_c = a_f + ...
  transferFunction_(Vx_c,Vx_c) = transferFunction_(Vy_c,Vy_c) = transferFunction_(Vz_c,Vz_c) = 1;
  transferFunction_(Vx_c,Ax_f) = 1*delta;
  transferFunction_(Vx_c,Vpitch_c) = vpitch*l_x*delta;
  transferFunction_(Vz_c,Az_f) = 1*delta;
  transferFunction_(Vz_c,Vpitch_c) = -1*vpitch*l_z*delta;
  //theta_c_dot = omega_c
  transferFunction_(pitch_c,pitch_c) = 1;
  transferFunction_(pitch_c,Vpitch_c) = delta;
  //omega_dot = 0
  transferFunction_(Vpitch_c,Vpitch_c) = 1;
  //theta_p_dot = omega_p
  transferFunction_(pitch_p,pitch_p) = 1;
  transferFunction_(pitch_p,Vpitch_p) = 1*delta;
  //omeag_p_dot = alpha_p
  transferFunction_(Vpitch_p,Vpitch_p) = 1 + vpitch*cos(theta_p);
  transferFunction_(Vpitch_p,Fx_f) = 1/(r_cp*m_p)*sin(theta_p)*delta;
  transferFunction_(Vpitch_p,Fx_l) = 1/(r_cp*m_p)*sin(theta_p)*delta;
  transferFunction_(Vpitch_p,Ax_f) = -1*r_cp*delta;
  transferFunction_(Vpitch_p,Vpitch_c) = -1*vpitch*sin(theta_c);

  //F_l
  transferFunction_(Fx_l,Fx_l) = 1;
  transferFunction_(Fy_l,Fy_l) = 1;
  transferFunction_(Fz_l,Fz_l) = 1;

  process_noise_m(0,0) = 0.05;//x_c
  process_noise_m(1,1) = 0.05;//y_c
  process_noise_m(2,2) = 0.06;//z_c
  process_noise_m(3,3) = 0.3;//Vx_c
  process_noise_m(4,4) = 0.3;//Vy_c
  process_noise_m(5,5) = 0.6;//Vz_c
  process_noise_m(6,6) = 0.05;//pitch_c
  process_noise_m(7,7) = 0.05;//yaw_c
  process_noise_m(8,8) = 0.04;//roll_c
  process_noise_m(9,9) = 0.5;//Vpitch_c
  process_noise_m(10,10) = 0.4;//Vyaw_c
  process_noise_m(11,11) = 0.4;//Vroll_c
  process_noise_m(12,12) = 0.05;//pitch_p
  process_noise_m(13,13) = 0.05;//yaw_p
  process_noise_m(14,14) = 0.08;//roll_p
  process_noise_m(15,15) = 0.05;//Fx_f
  process_noise_m(16,16) = 0.06;//Fy_f
  process_noise_m(17,17) = 0.04;//Fz_f
  process_noise_m(18,18) = 0.5;//Fx_l
  process_noise_m(19,19) = 0.5;//Fy_l
  process_noise_m(20,20) = 0.4;//Fz_l
  process_noise_m(21,21) = 0.01;//Ax_f
  process_noise_m(22,22) = 0.01;//Ay_f
  process_noise_m(23,23) = 0.01;//Az_f
  process_noise_m(24,24) = 0.01;//a_g_x
  process_noise_m(25,25) = 0.01;//a_g_y
  process_noise_m(26,26) = 0.01;//a_g_z
   //print transfer function
 /*
  printf("---transfer function---\n");
  for (int i = 0;i < 19; i++){
    for (int j = 0; j < 19; j++){
      printf("%f ", transferFunction_(i,j));
    }

    printf("\n");
  }
  printf("\n");
*/
  // (1) Take the square root of a small fraction of the estimateErrorCovariance_ using LL' decomposition
  // caculate square root of (L+lamda)*P_k-1
  // This will be a diagonal matrix (19*19)

  weightedCovarSqrt_ = ((STATE_SIZE + lambda_) * estimateErrorCovariance_).llt().matrixL();
  /*
  Eigen::MatrixXd L = estimateErrorCovariance_.llt().matrixL();
  Eigen::MatrixXd L_ = L*L.transpose();
  printf("---L---\n");
  for(int i = 0;i < 19;i++){
    for(int j = 0 ; j<19;j++){
      printf("%f ", L(i,j));
    }
    printf("\n");
  }
  printf("\n");
  printf("---L_---\n");
  for(int i = 0;i < 19;i++){
    for(int j = 0 ; j<19;j++){
      printf("%f ", L_(i,j));
    }
    printf("\n");
  }
  printf("\n");
*/

 // printf weightedCovarSqrt
/*
 printf("---weightedCovarSqrt---\n");
  for (int i = 0; i < 19; i++){
    for (int j = 0 ; j < 19; j++){
      printf("%f ", weightedCovarSqrt_(i,j));
    }
    printf("\n");
  }
  printf("\n");
*/
  // (2) Compute sigma points *and* pass them through the transfer function to save
  // the extra loop

  // First sigma point(through transferfunction) is the current state
  // x_k|k-1(0)
  // sigmaPoint_[0][0~14]
  //ROS_INFO("state_x = %f", state_[6]);
  sigmaPoints_[0] = transferFunction_ * state_;
  //ROS_INFO("%f", sigmaPoints_[0][0]);
/*
  for (int j = 0; j < 19; j++){
    printf("%f ", sigmaPoints_[0][j]);
  }
  printf("\n");
*/
/*
  printf("---state---\n");
  for (int i = 0; i < 19; i++){
    printf("%f ", state_[i]);
  }
  printf("\n");
*/


  // Next STATE_SIZE sigma points are state + weightedCovarSqrt_[ith column]
  // STATE_SIZE sigma points after that are state - weightedCovarSqrt_[ith column]
  for (size_t sigmaInd = 0; sigmaInd < STATE_SIZE; ++sigmaInd)
  {
    sigmaPoints_[sigmaInd + 1] = transferFunction_ * (state_ + weightedCovarSqrt_.col(sigmaInd));
    sigmaPoints_[sigmaInd + 1 + STATE_SIZE] = transferFunction_ * (state_ - weightedCovarSqrt_.col(sigmaInd));
  }
  //ROS_INFO("sigma = %f", sigmaPoints_[2][1]);
  //print state_ + weightCovarSqrt


  //print sigmaPoints
/*
printf("---sigmaPoints---\n");
  for (int i = 0; i < 37 ; i++){
    for (int j = 0; j< 19 ; j++){
      printf("%f ", sigmaPoints_[i][j]);
    }
    printf("\n");
  }
printf("\n");
*/

  // (3) Sum the weighted sigma points to generate a new state prediction
  // x_k_hat- = w_im * x_k|k-1
  state_.setZero();
  for (size_t sigmaInd = 0; sigmaInd < sigmaPoints_.size(); ++sigmaInd)
  {
    state_.noalias() += stateWeights_[sigmaInd] * sigmaPoints_[sigmaInd];
  }


/*
  printf("---state before adding noise---\n");

    for (int i = 0; i < 19; i++){
      printf("%f ", state_[i]);
    }
    printf("\n");
*/

/*
  printf("---state adding noise---\n");

    for (int i = 0; i < 19; i++){
      printf("%f ", state_[i]);
    }
    printf("\n");
*/
  //ROS_INFO("state = %f",state_[6]);

/*
  ROS_INFO("initial covariance");
  for (int i = 0; i < 19; i++){
    for (int j = 0 ; j<19; j++){
      printf("%f ", estimateErrorCovariance_(i,j));
    }
    printf("\n");
  }
  printf("\n");
  */
  // (4) Now us the sigma points and the predicted state to compute a predicted covariance P_k-
  estimateErrorCovariance_.setZero();
 /*
  printf("---sigmaPoints---\n");
  for (int j = 0; j < 19; j++){
    printf("%f ", sigmaPoints_[30][j]);

  }
  printf("\n");
  printf("---state_---\n");
  for (int j = 0; j < 19; j++){
    printf("%f ", state_[j]);
  }
  printf("\n");
*/

  Eigen::VectorXd sigmaDiff(STATE_SIZE);
  for (size_t sigmaInd = 0; sigmaInd < sigmaPoints_.size(); ++sigmaInd)
  {
    sigmaDiff = (sigmaPoints_[sigmaInd] - state_);
    //ROS_INFO("sigmapoint = %f", sigmaPoints_[0][0]);
    //ROS_INFO("sigmaDiff = %f", sigmaDiff[0]);
    estimateErrorCovariance_.noalias() += covarWeights_[sigmaInd] * (sigmaDiff * sigmaDiff.transpose());
  }
  estimateErrorCovariance_ = estimateErrorCovariance_ + process_noise_m;

/*
  printf("---sigmaDiff---\n");
  for(int i = 0; i < 19; i++){
      printf("%f ", sigmaDiff[i]);
  }
*/
  //ROS_INFO("estimateErrorCov = %f", estimateErrorCovariance_(0,0));
/*
  printf("---predicted estimate covariance---\n");
  for (int i = 0; i < 19; i++){
    for (int j = 0 ; j<19; j++){
      printf("%f ", estimateErrorCovariance_(i,j));
    }
    printf("\n");
  }
  printf("\n");
*/
// Mark that we can keep these sigma points
      uncorrected_ = true;
      //ROS_INFO("Vx = %f, Vy = %f, Vz = %f", state_[6], state_[7], state_[8]);
      //ROS_INFO("---Predict end---");

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "payload");
  ros::NodeHandle nh;
  //get param
  string topic_measure;


  //ros::Subscriber svo_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/svo/pose_imu", 10, svo_cb);
  //ros::Subscriber mocap_sub = nh.subscribe<geometry_msgs::PoseStamped>(topic_mocap, 2, mocap_cb);
  //ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>(topic_imu, 2, imu_cb);
  //ros::Subscriber vfr_sub = nh.subscribe<mavros_msgs::VFR_HUD>(topic_thrust, 2, vfr_cb);
  //ros::Publisher output_pub = nh.advertise<UKF::output>("/output", 10);
  ros::Subscriber measurement_sub = nh.subscribe<geometry_msgs::PoseStamped>(topic_measure, 2, measure_cb);
  ros::Subscriber output_sub = nh.subscribe<UKF::output>("/output", 2, output_cb);
  initialize();
  int count = 0;
  ros::Rate rate(50);
  while(ros::ok()){
    //output_data.header.stamp = ros::Time::now();

    //imu_data.header.stamp = ros::Time::now();
    //svo_pose.header.stamp = ros::Time::now();
    //measure_data.pose.position.x = 1;
    checkData();


    if(flag == 1)
    {
    writeInMeasurement();
    predict(1,0.02);
    correct();
    //output_pub.publish(output);
    }



    ros::spinOnce();
    rate.sleep();


  }
}
