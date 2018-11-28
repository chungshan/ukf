#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
geometry_msgs::TwistStamped vel1, vel2;
geometry_msgs::Point test;
void vel1_cb(const geometry_msgs::TwistStamped::ConstPtr &msg){
  vel1 = *msg;
}
void vel2_cb(const geometry_msgs::TwistStamped::ConstPtr &msg){
  vel2 = *msg;
}
sensor_msgs::Imu imu1, imu2;
void imu1_cb(const sensor_msgs::Imu::ConstPtr &msg){
  imu1 = *msg;
}
void imu2_cb(const sensor_msgs::Imu::ConstPtr &msg){
  imu2 = *msg;
}

geometry_msgs::PoseStamped pose1, pose2;
void pose1_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
  pose1 = *msg;
}
void pose2_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
  pose2 = *msg;
}

geometry_msgs::Point follower_force, leader_force;
void follower_force_cb(const geometry_msgs::Point::ConstPtr &msg){
  follower_force = *msg;
}
void leader_force_cb(const geometry_msgs::Point::ConstPtr &msg){
  leader_force = *msg;
}


int flag;


void check(){
  if(vel1.twist.linear.x != 0){
    flag = 1;
  }
}
double dij;
Eigen::Vector2d K;
double P;
Eigen::Vector2d yij_old;
Eigen::Vector2d uf; //zij_dot
Eigen::Vector2d yf; // yij
Eigen::Vector2d zijf; // zij_dot_f
std::vector<double> uf_a;
std::vector<double> yf_a;
std::vector<double> yij_a;

Eigen::VectorXd uf_aa;
Eigen::VectorXd yf_aa;
Eigen::VectorXd yij_aa;
int n = 2;
int signn;
void est_zij(){
  Eigen::Vector2d vcl;
  Eigen::Vector2d vcf;
  Eigen::Vector2d zij_dot;
  Eigen::Vector2d zij_dot_hat;
  Eigen::Vector2d zij_dot_T;
  Eigen::Vector2d yij;
  Eigen::Vector2d yij_dot;
  Eigen::Vector2d result;
  double result_1;
  double uf_norm;
  double yij_norm;
  double rate_r;
  double zij_dot_norm;
  double theta;
  double delta;
  double sign;
  delta = 0.95;
  rate_r = 50;
  vcf << vel1.twist.linear.x, vel1.twist.linear.y;
  vcl << vel2.twist.linear.x, vel2.twist.linear.y;
  zij_dot = vcl - vcf;
  zij_dot_norm = sqrt(zij_dot(0)*zij_dot(0)+zij_dot(1)*zij_dot(1));

  zij_dot_T << -zij_dot(1), zij_dot(0);
  yij = zij_dot_T / (sqrt(zij_dot(0)*zij_dot(0)+zij_dot(1)*zij_dot(1)));

  uf = uf + 0.5*(zij_dot - uf);
  yf = yf + 0.5*(yij - yf);
  /*
  zijf = zijf + 0.5*(zij_dot - zijf);
  sign = zijf.dot(yij-yf);
  if(sign > 0){
    signn = 1;
  }
  else{
    signn = -1;
  }
  */
/*
  uf_a.push_back(uf(0));
  uf_a.push_back(uf(1));
  yf_a.push_back(yf(0));
  yf_a.push_back(yf(1));
  yij_a.push_back(yij(0));
  yij_a.push_back(yij(1));
  uf_aa.resize(n);
  yf_aa.resize(n);
  yij_aa.resize(n);
*/
  uf_norm = uf(0) * uf(0) + uf(1) * uf(1);
  yij_norm = (yij(0) - yf(0)) * (yij(0) - yf(0)) + (yij(1) - yf(1)) * (yij(1) - yf(1));

  //result = sqrt(uf_norm/(0.25*yij_norm)) * yij;
  result_1 =  sqrt(uf_norm/(0.25*yij_norm));
  std::cout << result << std::endl;
  //test.x = result_1;
  //test.y = 1.6;
/*
  for (int i = 0; i < n ; i++){
    uf_aa(i) = uf_a[i];
    yf_aa(i) = yf_a[i];
    yij_aa(i) = yij_a[i];
  }
  n += 2;
  theta = uf_aa.dot(0.5*(yij_aa - yf_aa))/(uf_aa.dot(uf_aa));


  dij = 1/(theta);
  ROS_INFO("---------------------------------");
  ROS_INFO("dij = %f", dij);
  test.x = fabs(result(0));
*/
/*
  yij_dot = 50*(yij - yij_old);
  yij_old = yij;
  if(zij_dot_norm < 0.5){

  zij_dot_hat = dij * yij_dot;




  K(0) = P * yij_dot(0)/((delta+yij_dot.dot(P*yij_dot)));
  K(1) = P * yij_dot(1)/((delta+yij_dot.dot(P*yij_dot)));

  P = (1 - K.dot(yij_dot))*P/delta;
  dij = dij + K.dot(zij_dot - zij_dot_hat);
  }
  else {
    dij = dij;
  }
*/
}

void est_w(){
  Eigen::Vector2d vcl;
  Eigen::Vector2d vcf;
  Eigen::Vector2d zij_dot;
  Eigen::Vector2d zij_dot_T;
  Eigen::Vector2d zij;
  double omega;
  double omega_n;
  double omega_n2;
  double err1, err2;
  double err3, err4;
  vcf << vel1.twist.linear.x, vel1.twist.linear.y;
  vcl << vel2.twist.linear.x, vel2.twist.linear.y;

  zij_dot = vcl - vcf;
  zij_dot_T << -zij_dot(1), zij_dot(0);

  zij << (pose2.pose.position.x - pose1.pose.position.x), (pose2.pose.position.y - pose1.pose.position.y);

  omega = -(zij.dot(zij_dot_T))/(zij.dot(zij));

  err1 = fabs(omega - imu1.angular_velocity.z);
  err2 = fabs(-omega - imu1.angular_velocity.z);
  if(err1 > err2){
    omega_n = -omega;
  }
  else{
    omega_n = omega;
  }

  err3 = fabs(omega * zij(0) + zij_dot_T(0));
  err4 = fabs(-omega * zij(0) + zij_dot_T(0));

  if(err3 > err4){
    omega_n2 = -omega;
  }
  else{
    omega_n2 = omega;
  }

  //test.x = omega;
  //test.y = imu1.angular_velocity.z;
}
double yf_w, uf_z;
void est_J(){
  double omega;
  double J;
  Eigen::Vector2d zl, zf;
  double z_sum;

  zl << pose2.pose.position.x - 0.5*(pose2.pose.position.x + pose1.pose.position.x), pose2.pose.position.y - 0.5*(pose2.pose.position.y + pose1.pose.position.y);
  zf << pose1.pose.position.x - 0.5*(pose2.pose.position.x + pose1.pose.position.x), pose1.pose.position.y - 0.5*(pose2.pose.position.y + pose1.pose.position.y);
  z_sum = zl(0) * zl(0) + zl(1) * zl(1) + zf(0) * zf(0) + zf(1) * zf(1);
  omega = imu1.angular_velocity.z;
  yf_w = yf_w + 0.5*(omega - yf_w);
  uf_z = uf_z + 0.5*(z_sum - uf_z);

  J = uf_z/(0.25*(omega - yf_w)); // Jz = 1.4*1e-6

  //test.x = follower_force.x;//J
  //test.y = -(pose1.pose.position.y - 0.5*(pose2.pose.position.y + pose1.pose.position.y));//1.4*1e-6
  //ROS_INFO("J = %f", J);
}

Eigen::Vector2d yf_vc;
Eigen::Vector2d uf_f;

void est_m(){
  Eigen::Vector2d v_c;
  Eigen::Vector2d v_cf;
  Eigen::Vector2d r;
  Eigen::Vector2d force_sum;
  double force_sum1;
  double omega;
  double m;
  v_cf << vel1.twist.linear.x, vel1.twist.linear.y;
  omega = imu1.angular_velocity.z;
  r << -(0.5*(pose2.pose.position.y + pose1.pose.position.y) - pose1.pose.position.y), 0.5*(pose2.pose.position.x + pose1.pose.position.x) - pose1.pose.position.x;
  force_sum << leader_force.x + follower_force.x, -(leader_force.y + follower_force.y);
  force_sum1 = force_sum(0)*force_sum(0) +force_sum(1)*force_sum(1);
  v_c = v_cf - omega * r;
  uf_f = uf_f + 0.9*(force_sum - uf_f); // n*f_mean
  yf_vc = yf_vc + 0.9*(v_c - yf_vc); // vc
  std::cout << v_cf << std::endl;
  m = (uf_f(0) * uf_f(0) + uf_f(1) * uf_f(1)) / (0.81*((yf_vc(0) - v_c(0))*(yf_vc(0) - v_c(0)) + (yf_vc(1) - v_c(1))*(yf_vc(1) - v_c(1))));
  ROS_INFO("m = %f", m);
  test.x = m;
  test.y = 0.3;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "estimate_payload_v3");
  ros::NodeHandle nh;
  ros::Subscriber vel1_sub = nh.subscribe<geometry_msgs::TwistStamped>("/imu1/mavros/local_position/velocity", 2, vel1_cb);
  ros::Subscriber vel2_sub = nh.subscribe<geometry_msgs::TwistStamped>("/drone3_imu/mavros/local_position/velocity", 2, vel2_cb);


  ros::Subscriber imu1_sub = nh.subscribe<sensor_msgs::Imu>("/imu1/mavros/imu/data", 2, imu1_cb);
  ros::Subscriber imu2_sub = nh.subscribe<sensor_msgs::Imu>("/drone3_imu/mavros/imu/data", 2, imu2_cb);

  ros::Subscriber pose1_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/RigidBody1/pose", 2, pose1_cb);
  ros::Subscriber pose2_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/RigidBody4/pose", 2, pose2_cb);
  ros::Subscriber follower_force_sub = nh.subscribe<geometry_msgs::Point>("/follower_ukf/force", 2, follower_force_cb);
  ros::Subscriber leader_force_sub = nh.subscribe<geometry_msgs::Point>("/leader_ukf/force", 2, leader_force_cb);
  ros::Publisher test_pub = nh.advertise<geometry_msgs::Point>("/test", 2);
  ros::Rate rate(50);
  flag = 0;
  P = 1;
  while(ros::ok()){
    check();
    if(flag==1){
      est_zij();
      est_w();
      est_J();
      est_m();
    }
    test_pub.publish(test);
    ros::spinOnce();
    rate.sleep();
  }

}
