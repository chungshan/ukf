// reference: Estimating unknown object dynamics in human-robot manipulation tasks

#include <ros/ros.h>
#include <Eigen/Dense>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_datatypes.h>
#include "lpf2.h"
lpf2 lpaverage(0.01,0.02);
lpf2 lpf_leader_y(6, 0.02);
lpf2 lpf_leader_z(6, 0.02);
lpf2 lpf_follower_y(6, 0.02);
lpf2 lpf_follower_z(6, 0.02);
lpf2 lpf_alpha_x(2, 0.02);
lpf2 lpf_alpha_y(2, 0.02);
lpf2 lpf_alpha_z(2, 0.02);
geometry_msgs::Point follower_force, pose;
geometry_msgs::Point m, J, J_ref, p, p_ref, J_product, J_product_ref;
void follower_force_cb(const geometry_msgs::Point::ConstPtr& msg){
  follower_force = *msg;
}

geometry_msgs::Point leader_force;

void leader_force_cb(const geometry_msgs::Point::ConstPtr& msg){
  leader_force = *msg;
}


sensor_msgs::Imu leader_imu;
void leader_imu_cb(const sensor_msgs::Imu::ConstPtr &msg){
  leader_imu = *msg;
}
sensor_msgs::Imu follower_imu;
geometry_msgs::Point follower_imu_angular_v;
void follower_imu_cb(const sensor_msgs::Imu::ConstPtr &msg){
  follower_imu = *msg;
  follower_imu_angular_v.x = follower_imu.angular_velocity.x;
  follower_imu_angular_v.y = follower_imu.angular_velocity.y;
  follower_imu_angular_v.z = follower_imu.angular_velocity.z;
}

geometry_msgs::PoseStamped imu1_mocap;

void imu1_mocap_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
  imu1_mocap = *msg;
}
geometry_msgs::PoseStamped leader_mocap;
void leader_mocap_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
  leader_mocap = *msg;
}
geometry_msgs::PoseStamped follower_mocap;
void follower_mocap_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
  follower_mocap = *msg;
}
geometry_msgs::PoseStamped drone_follower_mocap;
void drone_follower_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
  drone_follower_mocap = *msg;
}
Eigen::VectorXd u_r_o;
int rate_r;
int flag2;
Eigen::VectorXd theta_hat(10);
Eigen::MatrixXd P(10,10);
Eigen::MatrixXd phi_sum(10,10);
int count;
double delta;
double first_time, time_now;
geometry_msgs::Vector3 rpy_mocap;
Eigen::VectorXd sigma_e2(6);
Eigen::VectorXd sigma_v2(6);
Eigen::MatrixXd sigma_q2(6,6);
Eigen::MatrixXd R(6,6);
void quaternionToRPY(){

  if(imu1_mocap.pose.orientation.w == 0)
  {
    imu1_mocap.pose.orientation.w = 1;
    flag2 = 0;
  }
  if(imu1_mocap.pose.orientation.w != 0 && imu1_mocap.pose.orientation.w != 1){
    flag2 = 1;
  }

  //ROS_INFO("imu.x = %f", imu_data.orientation.x);

  //ROS_INFO("flag = %d", flag);
  //ROS_INFO("imu = %f", imu_data.orientation.w);
  tf::Quaternion quat1(imu1_mocap.pose.orientation.x, imu1_mocap.pose.orientation.y, imu1_mocap.pose.orientation.z, imu1_mocap.pose.orientation.w);

  double roll, pitch, yaw;
  double yaw_bias;
  double roll_mocap, pitch_mocap, yaw_mocap;

  tf::Matrix3x3(quat1).getRPY(roll_mocap, pitch_mocap, yaw_mocap);




  rpy_mocap.x = roll_mocap;
  rpy_mocap.y = pitch_mocap;
  rpy_mocap.z = yaw_mocap;


}
double residual;
Eigen::MatrixXd sum_PE(10,10);
void estimation_model(){
  Eigen::VectorXd u_l(6);
  Eigen::VectorXd u_f(6);
  Eigen::VectorXd u_l_f(12);
  Eigen::MatrixXd G_r_h(6,6);
  Eigen::MatrixXd G_r_h_T(6,6);
  Eigen::VectorXd u_r_o(6);
  Eigen::VectorXd u_r_o_hat(6);
  Eigen::MatrixXd H(6,12);
  Eigen::MatrixXd phi(6,10);
  Eigen::MatrixXd phi_square(10,10);
  Eigen::Vector3d v_f_dot;
  Eigen::Vector3d v_f_dot_body;
  Eigen::Vector3d g_r;
  Eigen::Vector3d g;
  Eigen::Matrix3d w_o_dot_skew;
  Eigen::Matrix3d w_o_skew;
  Eigen::MatrixXd dot_w_dot(3,6);
  Eigen::MatrixXd dot_w(3,6);
  Eigen::Matrix3d w_o_skew_double;
  Eigen::Matrix3d g_v_skew;
  Eigen::MatrixXd two_term(3,6);
  Eigen::MatrixXd K(10,6);
  Eigen::MatrixXd I6(6,6);
  Eigen::MatrixXd I10(10,10);
  Eigen::VectorXd e(6);
  Eigen::VectorXd e2(6);
  Eigen::MatrixXd q(6,6);
  Eigen::MatrixXd q2(6,6);

  Eigen::VectorXd sigma_e(6);
  Eigen::VectorXd sigma_v(6);
  Eigen::MatrixXd sigma_q(6,6);
  Eigen::VectorXd lambda(6);
  Eigen::VectorXd lambda_u(6);
  Eigen::VectorXd lambda_d(6);

  Eigen::MatrixXd PE(10,10);
  double w_o_dot_x, w_o_dot_y, w_o_dot_z;
  double last_w_o_x, last_w_o_y, last_w_o_z;
  double dt;
  double alpha, beta;
  double phi_n;
  double p;
  residual = 0;
  p = 1.2;
  alpha = 0.997395;
  beta = 0.99913;
  I6 << 1.0, 0, 0, 0, 0, 0,
        0, 1.0, 0, 0, 0, 0,
        0, 0, 1.0, 0, 0, 0,
        0, 0, 0, 1.0, 0, 0,
        0, 0, 0, 0, 1.0, 0,
        0, 0, 0, 0, 0, 1.0;
  I10 << 1.0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 1.0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 1.0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 1.0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 1.0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 1.0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0;
  u_l.setZero();
  u_f.setZero();
  u_l_f.setZero();

  double leader_y, leader_z, follower_y, follower_z;

  leader_y = lpf_leader_y.filter(leader_force.y);
  leader_z = lpf_leader_z.filter(leader_force.z);
  follower_y = lpf_follower_y.filter(follower_force.y);
  follower_z = lpf_follower_z.filter(follower_force.z);

  u_l << -leader_force.x, -leader_y, -leader_z, 0, 0, 0;
  u_f << -follower_force.x, -follower_y, -follower_z, 0, 0, 0;
  u_l_f << u_f, u_l;




  G_r_h_T << 1, 0, 0, 0, (follower_mocap.pose.position.z - leader_mocap.pose.position.z), -(follower_mocap.pose.position.y - leader_mocap.pose.position.y),
             0, 1, 0, -(follower_mocap.pose.position.z - leader_mocap.pose.position.z), 0, (follower_mocap.pose.position.x - leader_mocap.pose.position.x),
             0, 0, 1, (follower_mocap.pose.position.y - leader_mocap.pose.position.y), -(follower_mocap.pose.position.x - leader_mocap.pose.position.x), 0,
             0, 0, 0, 1, 0, 0,
             0, 0, 0, 0, 1, 0,
             0, 0, 0, 0, 0, 1;

  G_r_h = G_r_h_T.transpose();
  //[I6 rGh]
  H << 1, 0, 0, 0, 0, 0, G_r_h(0,0), G_r_h(0,1), G_r_h(0,2), G_r_h(0,3), G_r_h(0,4), G_r_h(0,5),
       0, 1, 0, 0, 0, 0, G_r_h(1,0), G_r_h(1,1), G_r_h(1,2), G_r_h(1,3), G_r_h(1,4), G_r_h(1,5),
       0, 0, 1, 0, 0, 0, G_r_h(2,0), G_r_h(2,1), G_r_h(2,2), G_r_h(2,3), G_r_h(2,4), G_r_h(2,5),
       0, 0, 0, 1, 0, 0, G_r_h(3,0), G_r_h(3,1), G_r_h(3,2), G_r_h(3,3), G_r_h(3,4), G_r_h(3,5),
       0, 0, 0, 0, 1, 0, G_r_h(4,0), G_r_h(4,1), G_r_h(4,2), G_r_h(4,3), G_r_h(4,4), G_r_h(4,5),
       0, 0, 0, 0, 0, 1, G_r_h(5,0), G_r_h(5,1), G_r_h(5,2), G_r_h(5,3), G_r_h(5,4), G_r_h(5,5);

  u_r_o = H * u_l_f;
  //remove gravity in acceleration
  double roll, pitch , yaw;
  //const float imu_ax_bias = -0.077781;
  //const float imu_ay_bias = 0.083215;
  Eigen::Matrix3d Rx, Ry, Rz;
  Eigen::Vector3d a_g_inertial;
  Eigen::Vector3d a_g_body;
  Rx.setZero();
  Ry.setZero();
  Rz.setZero();
  a_g_inertial.setZero();

  a_g_inertial(0) = 0;
  a_g_inertial(1) = 0;
  a_g_inertial(2) = 9.81;
  roll = rpy_mocap.x;
  pitch = rpy_mocap.y;
  yaw = rpy_mocap.z;
  Rx(0,0) = 1;
  Rx(1,0) = 0;
  Rx(2,0) = 0;
  Rx(0,1) = 0;
  Rx(1,1) = cos(roll);
  Rx(1,2) = -sin(roll);
  Rx(0,2) = 0;
  Rx(2,1) = sin(roll);
  Rx(2,2) = cos(roll);

  Ry(0,0) = cos(pitch);
  Ry(1,0) = 0;
  Ry(2,0) = sin(pitch);
  Ry(0,1) = 0;
  Ry(1,1) = 1;
  Ry(1,2) = 0;
  Ry(0,2) = -sin(pitch);
  Ry(2,1) = 0;
  Ry(2,2) = cos(pitch);

  Rz(0,0) = cos(yaw);
  Rz(1,0) = -sin(yaw);
  Rz(2,0) = 0;
  Rz(0,1) = sin(yaw);
  Rz(1,1) = cos(yaw);
  Rz(1,2) = 0;
  Rz(0,2) = 0;
  Rz(2,1) = 0;
  Rz(2,2) = 1;

  a_g_body = Ry*Rx*Rz*a_g_inertial;

  v_f_dot_body(0) = (follower_imu.linear_acceleration.x - a_g_body(0));
  v_f_dot_body(1) = (follower_imu.linear_acceleration.y + a_g_body(1));
  v_f_dot_body(2) = (follower_imu.linear_acceleration.z - a_g_body(2));
  v_f_dot = (Rz*Rx*Ry).inverse()*v_f_dot_body;
  g << 0, 0, -9.8;
  //follower_imu.angular_velocity.x = 0;
  g_r = Ry*Rx*Rz*g;
  w_o_dot_x = lpf_alpha_x.filter((follower_imu.angular_velocity.x - last_w_o_x) * rate_r);
  w_o_dot_y = lpf_alpha_y.filter((follower_imu.angular_velocity.y - last_w_o_y) * rate_r);
  w_o_dot_z = lpf_alpha_z.filter((follower_imu.angular_velocity.z - last_w_o_z) * rate_r);

  last_w_o_x = follower_imu.angular_velocity.x;
  last_w_o_y = follower_imu.angular_velocity.y;
  last_w_o_z = follower_imu.angular_velocity.z;



  w_o_dot_skew << 0, -w_o_dot_z, w_o_dot_y,
                  w_o_dot_z, 0, -w_o_dot_x,
                 -w_o_dot_y, w_o_dot_x, 0;

  w_o_skew << 0, -follower_imu.angular_velocity.z, follower_imu.angular_velocity.y,
              follower_imu.angular_velocity.z, 0, -follower_imu.angular_velocity.x,
              -follower_imu.angular_velocity.y, follower_imu.angular_velocity.x, 0;

  dot_w_dot << w_o_dot_x, w_o_dot_y, w_o_dot_z, 0, 0, 0,
               0, w_o_dot_x, 0, w_o_dot_y, w_o_dot_z, 0,
               0, 0, w_o_dot_x, 0, w_o_dot_y, w_o_dot_z;
  dot_w << follower_imu.angular_velocity.x, follower_imu.angular_velocity.y, follower_imu.angular_velocity.z, 0, 0, 0,
           0, follower_imu.angular_velocity.x, 0, follower_imu.angular_velocity.y, follower_imu.angular_velocity.z, 0,
           0, 0, follower_imu.angular_velocity.x, 0, follower_imu.angular_velocity.y, follower_imu.angular_velocity.z;

  w_o_skew_double = w_o_skew * w_o_skew;

  //g_v_skew << 0, -(g_r(2) - v_f_dot(2)), (g_r(1) - v_f_dot(1)),(g_r(2) - v_f_dot(2));
  //g_r(2) - v_f_dot(2)
  //v_f_dot(2) - g_r(2)
  two_term = dot_w_dot + w_o_skew*dot_w;
  phi << v_f_dot(0) - g_r(0), w_o_dot_skew(0,0) + w_o_skew_double(0,0), w_o_dot_skew(0,1) + w_o_skew_double(0,1), w_o_dot_skew(0,2) + w_o_skew_double(0,2), 0, 0, 0, 0, 0, 0,
         v_f_dot(1) - g_r(1), w_o_dot_skew(1,0) + w_o_skew_double(1,0), w_o_dot_skew(1,1) + w_o_skew_double(1,1), w_o_dot_skew(1,2) + w_o_skew_double(1,2), 0, 0, 0, 0, 0, 0,
         v_f_dot(2) - g_r(2), w_o_dot_skew(2,0) + w_o_skew_double(2,0), w_o_dot_skew(2,1) + w_o_skew_double(2,1), w_o_dot_skew(2,2) + w_o_skew_double(2,2), 0, 0, 0, 0, 0, 0,
         0, 0, -(g_r(2) - v_f_dot(2)), (g_r(1) - v_f_dot(1)), two_term(0,0), two_term(0,1), two_term(0,2), two_term(0,3), two_term(0,4), two_term(0,5),
         0, (g_r(2) - v_f_dot(2)), 0, -(g_r(0) - v_f_dot(0)), two_term(1,0), two_term(1,1), two_term(1,2), two_term(1,3), two_term(1,4), two_term(1,5),
         0, -(g_r(1) - v_f_dot(1)), (g_r(0) - v_f_dot(0)), 0, two_term(2,0), two_term(2,1), two_term(2,2), two_term(2,3), two_term(2,4), two_term(2,5);
/*
  PE = phi.transpose()*phi;
  sum_PE = sum_PE + PE;

  std::cout << "---PE---" << std::endl;
  std::cout << sum_PE << std::endl;
*/

  /*
  phi_square = phi.transpose()*phi;
  phi_sum += phi_square;
  count += 1;
  if(count == 20){
    std::cout << "---------phi_sum------------" << std::endl;
    std::cout << phi_sum << std::endl;
    phi_sum.setZero();
    count = 0;
  }
  */
  u_r_o_hat = phi*theta_hat;
  //H.transpose()*(H*H.transpose()).inverse()*
  //forgetting factor

  e = u_r_o - u_r_o_hat;
/*
  e2 << e(0)*e(0), e(1)*e(1), e(2)*e(2), e(3)*e(3), e(4)*e(4), e(5)*e(5);
  q = phi*P*phi.transpose();

  q2 << q(0,0)*q(0,0), q(0,1)*q(0,1), q(0,2)*q(0,2), q(0,3)*q(0,3), q(0,4)*q(0,4), q(0,5)*q(0,5),
        q(1,0)*q(1,0), q(1,1)*q(1,1), q(1,2)*q(1,2), q(1,3)*q(1,3), q(1,4)*q(1,4), q(1,5)*q(1,5),
        q(2,0)*q(2,0), q(2,1)*q(2,1), q(2,2)*q(2,2), q(2,3)*q(2,3), q(2,4)*q(2,4), q(2,5)*q(2,5),
        q(3,0)*q(3,0), q(3,1)*q(3,1), q(3,2)*q(3,2), q(3,3)*q(3,3), q(3,4)*q(3,4), q(3,5)*q(3,5),
        q(4,0)*q(4,0), q(4,1)*q(4,1), q(4,2)*q(4,2), q(4,3)*q(4,3), q(4,4)*q(4,4), q(4,5)*q(4,5),
        q(5,0)*q(5,0), q(5,1)*q(5,1), q(5,2)*q(5,2), q(5,3)*q(5,3), q(5,4)*q(5,4), q(5,5)*q(5,5);

  sigma_e2 = alpha * sigma_e2 + (1 - alpha)*e2;
  sigma_q2 = alpha * sigma_q2 + (1 - alpha)*q2;
  sigma_v2 = beta * sigma_v2 + (1 - beta)*e2;

  sigma_e << sqrt(sigma_e2(0)), sqrt(sigma_e2(1)), sqrt(sigma_e2(2)), sqrt(sigma_e2(3)), sqrt(sigma_e2(4)), sqrt(sigma_e2(5));
  sigma_q <<  sqrt(q2(0,0)), sqrt(q2(0,1)), sqrt(q2(0,2)), sqrt(q2(0,3)), sqrt(q2(0,4)), sqrt(q2(0,5)),
              sqrt(q2(1,0)), sqrt(q2(1,1)), sqrt(q2(1,2)), sqrt(q2(1,3)), sqrt(q2(1,4)), sqrt(q2(1,5)),
              sqrt(q2(2,0)), sqrt(q2(2,1)), sqrt(q2(2,2)), sqrt(q2(2,3)), sqrt(q2(2,4)), sqrt(q2(2,5)),
              sqrt(q2(3,0)), sqrt(q2(3,1)), sqrt(q2(3,2)), sqrt(q2(3,3)), sqrt(q2(3,4)), sqrt(q2(3,5)),
              sqrt(q2(4,0)), sqrt(q2(4,1)), sqrt(q2(4,2)), sqrt(q2(4,3)), sqrt(q2(4,4)), sqrt(q2(4,5)),
              sqrt(q2(5,0)), sqrt(q2(5,1)), sqrt(q2(5,2)), sqrt(q2(5,3)), sqrt(q2(5,4)), sqrt(q2(5,5));
  sigma_v << sqrt(sigma_v2(0)), sqrt(sigma_v2(1)), sqrt(sigma_v2(2)), sqrt(sigma_v2(3)), sqrt(sigma_v2(4)), sqrt(sigma_v2(5));

  lambda_u = sigma_q*sigma_v;
  lambda_d = sigma_e - sigma_v;

  std::cout << "---sigma_q*sigma_v---" << std::endl;

  std::cout << lambda_u << std::endl;

  lambda(0) = lambda_u(0)/(1e-8+lambda_d(0));
  lambda(1) = lambda_u(1)/(1e-8+lambda_d(1));
  lambda(2) = lambda_u(2)/(1e-8+lambda_d(2));
  lambda(3) = lambda_u(3)/(1e-8+lambda_d(3));
  lambda(4) = lambda_u(4)/(1e-8+lambda_d(4));
  lambda(5) = lambda_u(5)/(1e-8+lambda_d(5));


  std::cout << "----lambda----" << std::endl;
  std::cout << lambda << std::endl;


  delta = lambda(0);
  if(delta > 0.99){
    delta = 0.99;
  }


  else if(delta < 0.95){
    delta = 0.95;
  }
  */
  //delta += 3.267*1e-5;
/*
  std::cout << fabs(e(0)) << std::endl;
  std::cout << "---" << std::endl;
  phi_n = p*pow(fabs(e(0)), 0.2)/((1 + pow(fabs(e(0)), 1.2)) * fabs(e(0)));
  std::cout << phi_n << std::endl;
  residual = e.dot(e);
*/
  //K = phi_n*P*phi.transpose()*(delta*R+phi_n*phi*P*phi.transpose()).inverse();
  K = P*phi.transpose()*(delta*R+phi*P*phi.transpose()).inverse();
/*
  std::cout << "-----K------" << std::endl;
  std::cout << K << std::endl;
  std::cout << "-------residual-------" << std::endl;

  std::cout << (u_r_o_hat - u_r_o) << std::endl;
  std::cout << "----------------" << std::endl;
*/
  P = (1/delta)*(I10 - K*phi)*P;



  theta_hat = theta_hat - K*(u_r_o_hat - u_r_o);

  /*
  if(theta_hat(7) > 0.256){
    ROS_INFO("success!");
  }
  std::cout << u_r_o_hat - u_r_o << std::endl;
  time_now = ros::Time::now().toSec();
  dt = time_now - first_time;
  */
  //ROS_INFO("dt = %f", dt);
  /*
  if(dt > 15){
    delta = 0.999;
  }
*/
}









int main(int argc, char **argv)
{
  ros::init(argc, argv, "estimate_payload_v2");
  ros::NodeHandle nh;
  ros::Subscriber follower_force_sub = nh.subscribe<geometry_msgs::Point>("/follower_ukf/force_estimate", 1, follower_force_cb);
  ros::Subscriber leader_force_sub = nh.subscribe<geometry_msgs::Point>("/leader_ukf/force_estimate", 1, leader_force_cb);
  ros::Subscriber leader_imu_sub = nh.subscribe<sensor_msgs::Imu>("/drone3_imu/mavros/imu/data", 1, leader_imu_cb);
  ros::Subscriber follower_imu_sub = nh.subscribe<sensor_msgs::Imu>("/imu1/mavros/imu/data", 1, follower_imu_cb);
  ros::Subscriber imu1_mocap_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/RigidBody1/pose", 1, imu1_mocap_cb);
  ros::Subscriber leader_mocap_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/RigidBody4/pose", 1, leader_mocap_cb);
  ros::Subscriber follower_mocap_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/RigidBody1/pose", 1, follower_mocap_cb);
  ros::Subscriber drone_follower_mocap_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/RigidBody2/pose", 1, drone_follower_cb);
  ros::Publisher m_pub = nh.advertise<geometry_msgs::Point>("/m", 2);
  ros::Publisher J_pub = nh.advertise<geometry_msgs::Point>("/J", 2);
  ros::Publisher J_ref_pub = nh.advertise<geometry_msgs::Point>("/J_ref", 2);
  ros::Publisher J_product_pub = nh.advertise<geometry_msgs::Point>("/J_product", 2);
  ros::Publisher J_product_ref_pub = nh.advertise<geometry_msgs::Point>("/J_product_ref", 2);
  ros::Publisher p_pub = nh.advertise<geometry_msgs::Point>("/p", 2);
  ros::Publisher p_ref_pub = nh.advertise<geometry_msgs::Point>("/p_ref", 2);
  ros::Publisher follower_imu_angular_v_pub = nh.advertise<geometry_msgs::Point>("/follower_angular_v", 2);
  ros::Publisher pose_pub = nh.advertise<geometry_msgs::Point>("/pose", 2);
  rate_r = 50;
  ros::Rate rate(rate_r);
  theta_hat << 1, 1, 1, 1, 1, 1, 1, 1, 1, 1;
  P << 1.0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
       0, 1.0, 0, 0, 0, 0, 0, 0, 0, 0,
       0, 0, 1.0, 0, 0, 0, 0, 0, 0, 0,
       0, 0, 0, 1.0, 0, 0, 0, 0, 0, 0,
       0, 0, 0, 0, 1.0, 0, 0, 0, 0, 0,
       0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0,
       0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0,
       0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0,
       0, 0, 0, 0, 0, 0, 0, 0, 1.0, 0,
       0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0;
  R << 1, 0, 0, 0, 0, 0,
       0, 1, 0, 0, 0, 0,
       0, 0, 1, 0, 0, 0,
       0, 0, 0, 1, 0, 0,
       0, 0, 0, 0, 1, 0,
       0, 0, 0, 0, 0, 1;
  delta = 0.97;
  first_time = ros::Time::now().toSec();
  sigma_v2.setZero();
  int average_flag;
  double average, average_m, average_Jyy, average_Jxx;

  double sum, sum_m, sum_Jyy, sum_Jxx;

  double running_time;
  int count = 1;
  while(ros::ok()){
    quaternionToRPY();
    double filtertt;
    if(flag2 == 1){
    estimation_model();
    m.x = theta_hat(0);
    m.y = 0.3;
    m.z = average_m;
    J.x = average_Jyy;//theta_hat(4)
    J.y = theta_hat(7);//theta_hat(7)
    J.z = theta_hat(9);//theta_hat(9)
    J_product.x = theta_hat(5);
    J_product.y = theta_hat(6);
    J_product.z = theta_hat(8);
    J_product_ref.x = 0;
    J_product_ref.y = 0;
    J_product_ref.z = 0;
    J_ref.x = 2.2*1e-5;
    J_ref.y = 0.192;
    J_ref.z = 0.192;//0.256

    p_ref.x = 0.75;
    p_ref.y = 0.0;
    p_ref.z = 0.0;

    //pose.x = follower_force.z;
    //pose.y = leader_mocap.pose.position.z;
    //pose.z = leader_mocap.pose.position.z;
    pose_pub.publish(pose);
    m_pub.publish(m);
    J_pub.publish(J);
    J_ref_pub.publish(J_ref);
    J_product_pub.publish(J_product);
    J_product_ref_pub.publish(J_product_ref);
    p_pub.publish(p);
    p_ref_pub.publish(p_ref);
    follower_imu_angular_v_pub.publish(follower_imu_angular_v);
    running_time = ros::Time::now().toSec() - first_time;
    if(running_time > 5){
      average_flag = 1;

    }
    if(running_time > 15){

      delta = 0.97;

    }
    if(average_flag == 1){
    sum = sum + theta_hat(9);
    sum_m = sum_m + theta_hat(0);
    sum_Jyy = sum_Jyy + theta_hat(7);
    sum_Jxx = sum_Jxx + theta_hat(4);
    average = sum/count;
    average_m = sum_m/count;
    average_Jyy = sum_Jyy/count;
    average_Jxx = sum_Jxx/count;
    p.x = -theta_hat(1)/average_m;
    p.y = 0.5*(leader_mocap.pose.position.x - follower_mocap.pose.position.x);//theta_hat(2)/average_m
    p.z = -leader_force.x;;//theta_hat(3)/average_m
    count+=1;
    }
    filtertt = lpaverage.filter(theta_hat(9));
    ROS_INFO("m = %f", theta_hat(0));
    ROS_INFO("tx = %f, ty = %f, tz = %f", p.x,p.y,p.z);
    ROS_INFO("J_xx = %f, J_yy = %f, J_zz = %f", theta_hat(4), theta_hat(7), theta_hat(9));
    ROS_INFO("average_Jzz = %f, average_Jyy = %f, average_m = %f", average, average_Jyy, average_m);

}


    ros::spinOnce();
    rate.sleep();
  }
  return 0;

}
