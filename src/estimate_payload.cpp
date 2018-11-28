#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include "ceres/ceres.h"
#include "glog/logging.h"
#include "gflags/gflags.h"
#include "stdio.h"
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <UKF/output.h>
using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using namespace std;

int rate_r;
//transformation
Eigen::Vector4d qji_r, qji_d;
double qji_r_w, qji_r_x, qji_r_y, qji_r_z;
double qji_d_w, qji_d_x, qji_d_y, qji_d_z;

double qji_r_w_k, qji_r_x_k, qji_r_y_k, qji_r_z_k;
double qji_d_w_k, qji_d_x_k, qji_d_y_k, qji_d_z_k;
Eigen::VectorXd qji_k(8), inertia_k(6);
//Eigen::VectorXd qji(8);
//vector<double*> qji;
//double qji[8];
Eigen::VectorXd transform_hkxk(8);
Eigen::MatrixXd h_g(8,8);

Eigen::MatrixXd P_k(8,8), P_k_inverse(8,8), R_k_inverse(8,8), P_k_inverse_i(6,6), R_k_inverse_i(3,3);
Eigen::MatrixXd R_k(8,8);
Eigen::VectorXd y_k(8);
Eigen::MatrixXd P_k_inertia(6,6);

//transformation

int flag = 0;
geometry_msgs::Point plot_x;

geometry_msgs::TwistStamped leader_v;
void leader_v_cb(const geometry_msgs::TwistStamped::ConstPtr &msg){
  leader_v = *msg;
}
geometry_msgs::TwistStamped follower_v;
void follower_v_cb(const geometry_msgs::TwistStamped::ConstPtr &msg){
  follower_v = *msg;
}

sensor_msgs::Imu leader_imu;
void leader_imu_cb(const sensor_msgs::Imu::ConstPtr &msg){
  leader_imu = *msg;
}
sensor_msgs::Imu follower_imu;
void follower_imu_cb(const sensor_msgs::Imu::ConstPtr &msg){
  follower_imu = *msg;
}

//mass

Eigen::Vector3d H_m;
Eigen::Vector3d y_m;
Eigen::Vector3d hkxk_m;
Eigen::Matrix3d R_k_m;
Eigen::Matrix3d R_k_m_inverse;
double m_k;
double P_k_m;
double P_k_m_inverse;



//imu1_mocap
geometry_msgs::PoseStamped imu1_mocap;

void imu1_mocap_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
  imu1_mocap = *msg;
}
int flag2;
geometry_msgs::Vector3 rpy_mocap;

//force

geometry_msgs::Point follower_force;

void follower_force_cb(const geometry_msgs::Point::ConstPtr& msg){
  follower_force = *msg;
}

geometry_msgs::Point leader_force;

void leader_force_cb(const geometry_msgs::Point::ConstPtr& msg){
  leader_force = *msg;
}

void initial(){
  R_k << 0.1, 0, 0, 0, 0, 0, 0, 0,
         0, 0.1, 0, 0, 0, 0, 0, 0,
         0, 0, 0.1, 0, 0, 0, 0, 0,
         0, 0, 0, 0.1, 0, 0, 0, 0,
         0, 0, 0, 0, 0.1, 0, 0, 0,
         0, 0, 0, 0, 0, 0.1, 0, 0,
         0, 0, 0, 0, 0, 0, 0.1, 0,
         0, 0, 0, 0, 0, 0, 0, 0.1;
  P_k << 1, 0, 0, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0, 0, 0,
         0, 0, 1, 0, 0, 0, 0, 0,
         0, 0, 0, 1, 0, 0, 0, 0,
         0, 0, 0, 0, 1, 0, 0, 0,
         0, 0, 0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 0, 0, 1;

  P_k_inertia << 1, 0, 0, 0, 0, 0,
                 0, 1, 0, 0, 0, 0,
                 0, 0, 1, 0, 0, 0,
                 0, 0, 0, 1, 0, 0,
                 0, 0, 0, 0, 1, 0,
                 0, 0, 0, 0, 0, 1;
  R_k_inverse_i << 10, 0, 0,
                   0, 10, 0,
                   0, 0, 10;
  R_k_inverse = R_k.inverse();
  ROS_INFO("ok");

  //y_k.setZero();
  qji_k << 1, 0, 0, 0, 0, 0, 0, 0;
  m_k = 1;
  P_k_m = 0.5;
  R_k_m << 0.1, 0.0, 0.0,
           0.0, 0.1, 0.0,
           0.0, 0.0, 0.1;
      /*
  qji[0][0] = 1.0;
  qji[0][1] = 0.0;
  qji[0][2] = 0.0;
  qji[0][3] = 0.0;
  qji[0][4] = 0.0;
  qji[0][5] = 2.0;
  qji[0][6] = 0.0;
  qji[0][7] = 0.0;
*/
}

void checkmeasure(){
  if(leader_v.twist.linear.x!=0 && leader_imu.angular_velocity.x!=0 && follower_v.twist.linear.x!=0 && follower_imu.angular_velocity.x!=0){
    flag = 1;
  }
}


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

void transformation_model(){
  double leader_vx, leader_vy, leader_vz;
  double follower_vx, follower_vy, follower_vz;
  double leader_wx, leader_wy, leader_wz;
  double follower_wx, follower_wy, follower_wz;
  Eigen::VectorXd y(8);
  double lamda = 1.004;
  leader_vx = leader_v.twist.linear.x;
  leader_vy = leader_v.twist.linear.y;
  leader_vz = leader_v.twist.linear.z;
  follower_vx = follower_v.twist.linear.x;
  follower_vy = follower_v.twist.linear.y;
  follower_vz = follower_v.twist.linear.z;

  leader_wx = leader_imu.angular_velocity.x;
  leader_wy = leader_imu.angular_velocity.y;
  leader_wz = leader_imu.angular_velocity.z;
  follower_wx = follower_imu.angular_velocity.x;
  follower_wy = follower_imu.angular_velocity.y;
  follower_wz = follower_imu.angular_velocity.z;


  //ROS_INFO("leader_vx = %f, leader_vy = %f, leader_vz = %f", leader_vx, leader_vy, leader_vz);
  h_g.resize(8,8);
  h_g.setZero();
  h_g << 0, leader_wx - follower_wx, leader_wy - follower_wy, leader_wz - follower_wz, 0, 0, 0, 0,
         follower_wx - leader_wx, 0, -(leader_wz + follower_wz), (leader_wy + follower_wy), 0, 0, 0, 0,
         follower_wy - leader_wy, (leader_wz + follower_wz), 0, -(leader_wx + follower_wx), 0, 0, 0, 0,
         follower_wz - leader_wz, -(leader_wy + follower_wy), (leader_wx + follower_wx), 0, 0, 0, 0, 0,
         0, leader_vx - follower_vx, leader_vy - follower_vy, leader_vz - follower_vz, 0, leader_wx - follower_wx, leader_wy - follower_wy, leader_wz - follower_wz,
         follower_vx - leader_vx, 0, -(leader_vz + follower_vz), (leader_vy + follower_vy), follower_wx - leader_wx, 0, -(leader_wz + follower_wz), (leader_wy + follower_wy),
         follower_vy - leader_vy, (leader_vz + follower_vz), 0, -(leader_vx + follower_vx), follower_wy - leader_wy, (leader_wz + follower_wz), 0, -(leader_wx + follower_wx),
         follower_vz - leader_vz, -(leader_vy + follower_vy), (leader_vx + follower_vx), 0, follower_wz - leader_wz, -(leader_wy + follower_wy), (leader_wx + follower_wx), 0;
  //ROS_INFO("qji_k[5] = %f", qji_k(5));
  /*
  ROS_INFO("--------h_g------------");
  std::cout << h_g << std::endl;

  transform_hkxk = h_g * qji_k;
  ROS_INFO("--------transformhkxk------------");
  std::cout << transform_hkxk << std::endl;
  */
  //P_k.inverse() + h_g.transpose()*R_k.inverse*h_g
  P_k = lamda*(P_k.inverse() + h_g.transpose()*R_k.inverse()*h_g).inverse();

  /*
  Eigen::MatrixXd S(8,8);
  Eigen::MatrixXd K(8,8);
  Eigen::MatrixXd I(8,8);
  I << 1, 0, 0, 0, 0, 0, 0, 0,
       0, 1, 0, 0, 0, 0, 0, 0,
       0, 0, 1, 0, 0, 0, 0, 0,
       0, 0, 0, 1, 0, 0, 0, 0,
       0, 0, 0, 0, 1, 0, 0, 0,
       0, 0, 0, 0, 0, 1, 0, 0,
       0, 0, 0, 0, 0, 0, 1, 0,
       0, 0, 0, 0, 0, 0, 0, 1;
  S = h_g*P_k*h_g.transpose() + R_k;
  K = P_k*h_g.transpose()*S.inverse();
  P_k = (I - K*h_g)*P_k;
  std::cout << P_k << std::endl;
*/
  P_k_inverse = P_k.inverse();
  /*
  ROS_INFO("------------------------------");
  std::cout << P_k_inverse << std::endl;
*/
}

void alpha_model(){
  double leader_alpha_x, leader_alpha_y, leader_alpha_z;
  double leader_a_x, leader_a_y, leader_a_z;
  double follower_alpha_x, follower_alpha_y, follower_alpha_z;
  double follower_a_x, follower_a_y, follower_a_z;
  Eigen::Vector3d leader_a, follower_a;
  Eigen::Vector3d y;
  leader_a << leader_a_x, leader_a_y, leader_a_z;
  follower_a << follower_a_x, follower_a_y, follower_a_z;
  //y = follower_a - Rji*leader_a;
}

void model(){

}

void p_model(){
  double F_x, F_y, F_z;
  double alpha_x, alpha_y, alpha_z;
  double w_x, w_y, w_z;
  double v_x, v_y, v_z;
  double a_x, a_y, a_z;
  double p_x, p_y, p_z;
  Eigen::MatrixXd h_pc(2,3);
  Eigen::MatrixXd F_i_T(2,3);
  Eigen::Matrix3d alpha_i_x;
  Eigen::Vector3d w_i;
  Eigen::Matrix3d w_i_x2;
  Eigen::Vector3d v_i;
  Eigen::Vector3d a_i;
  Eigen::Vector2d y_pc;
  Eigen::Vector3d p_ic;
  F_i_T << 1, -1, (-1*F_x + 1*F_y)/F_z,
           -1, 1, (1*F_x - 1*F_y)/F_z;
  alpha_i_x << 0, -alpha_z, alpha_y,
               alpha_z, 0, -alpha_x,
               -alpha_y, alpha_x, 0;
  w_i_x2 << -w_y*w_y-w_z*w_z, w_x*w_y, w_x*w_z,
            w_x*w_y, -w_x*w_x-w_z*w_z, w_y*w_z,
            w_x*w_z, w_y*w_z, -w_x*w_x - w_y*w_y;
  v_i << v_x, v_y, v_z;
  a_i << a_x, a_y, a_z;
  w_i << w_x, w_y, w_z;
  h_pc = F_i_T*(alpha_i_x + w_i_x2);
  y_pc = -F_i_T*(w_i.cross(v_i) + a_i);
  y_pc = h_pc * p_ic;
}
  Eigen::Vector3d y_I;
  Eigen::Vector3d hI_Ii;
void inertia_model(){
  double alpha_x, alpha_y, alpha_z;
  double w_x, w_y, w_z;
  double F_x, F_y, F_z;
  double p_x, p_y, p_z;
  double I_xx, I_yy, I_zz, I_xy, I_xz, I_yz;

  Eigen::VectorXd I(6);
  Eigen::MatrixXd H_I(3,6);
  Eigen::Vector3d F_i;
  Eigen::Vector3d p_ic;

  double lamda = 1.004;
  I << I_xx, I_yy, I_zz, I_xy, I_xz, I_yz;
  H_I << alpha_x, -w_y*w_z, w_y*w_z, alpha_y - w_x*w_z, alpha_z+w_x*w_y, w_y*w_y-w_z*w_z,
         w_x*w_z, alpha_y, -w_x*w_z, alpha_x + w_y*w_z, w_z*w_z-w_x*w_x, alpha_z - w_x*w_y,
        -w_x*w_y, w_x*w_y, alpha_z, w_x*w_x-w_y*w_y, alpha_x - w_y*w_z, alpha_y + w_x*w_z;
  F_i << F_x, F_y, F_z;
  p_ic << p_x, p_y, p_z;

  P_k = lamda*(P_k_inertia.inverse() + H_I.transpose()*R_k_inverse_i.inverse()*H_I).inverse();
  y_I = F_i.cross(p_ic);
  hI_Ii = H_I * I;
}
double w_x_old, w_y_old, w_z_old;
void m_model(){
  double a_x, a_y, a_z;
  double alpha_x, alpha_y, alpha_z;
  double p_x, p_y, p_z;
  double w_x, w_y, w_z;
  double v_x, v_y, v_z;
  double lamda = 1.004;
  Eigen::Vector3d F_i;
  Eigen::Vector3d a_i;
  Eigen::Vector3d alpha_i;
  Eigen::Vector3d p_ic;
  Eigen::Vector3d w_i;
  Eigen::Vector3d v_i;
  //a from body to inertia
  double roll, pitch , yaw;
  Eigen::Matrix3d Rx, Ry, Rz;
  Eigen::Vector3d a_inertial;
  Eigen::Vector3d a_body;
  Rx.setZero();
  Ry.setZero();
  Rz.setZero();
  a_body.setZero();
  a_body(0) = follower_imu.linear_acceleration.x;
  a_body(1) = follower_imu.linear_acceleration.y;
  a_body(2) = follower_imu.linear_acceleration.z;
  roll = rpy_mocap.x;
  pitch = rpy_mocap.y;
  yaw = rpy_mocap.z;
  Rx << 1, 0, 0,
        0, cos(roll), -sin(roll),
        0, sin(roll), cos(roll);
  Ry << cos(pitch), 0, sin(pitch),
        0, 1, 0,
        -sin(pitch), 0, cos(pitch);
  Rz << cos(yaw), -sin(yaw), 0,
        sin(yaw), cos(yaw), 0,
        0, 0, 1;

  a_inertial = (Rz*Rx*Ry).inverse()*a_body;

  a_inertial(2) = a_inertial(2) - 9.8;

  a_x = a_inertial(0);
  a_y = a_inertial(1);
  a_z = a_inertial(2);
  //omega
  w_x = follower_imu.angular_velocity.x;
  w_y = follower_imu.angular_velocity.y;
  w_z = follower_imu.angular_velocity.z;
  //v
  v_x = follower_v.twist.linear.x;
  v_y = follower_v.twist.linear.y;
  v_z = follower_v.twist.linear.z;
  //p
  p_x = 0.8;
  p_y = 0;
  p_z = 0;
  //alpha
  alpha_x = (w_x - w_x_old)*rate_r;
  alpha_y = (w_y - w_y_old)*rate_r;
  alpha_z = (w_z - w_z_old)*rate_r;

  R_k_m_inverse = R_k_m.inverse();
  a_i << a_x, a_y, a_z;
  alpha_i << alpha_x, alpha_y, alpha_z;
  p_ic << p_x, p_y, p_z;
  v_i << v_x, v_y, v_z;
  w_i << w_x, w_y, w_z;
  H_m = a_i + alpha_i.cross(p_ic) + w_i.cross(w_i.cross(p_ic) + v_i);
  hkxk_m = H_m * m_k;
  y_m << follower_force.x + leader_force.x,  0,  follower_force.z + leader_force.z;
  P_k_m = lamda*(1/((1/P_k_m) + H_m.transpose()*R_k_m.inverse()*H_m));

  P_k_m_inverse = (1/P_k_m);

  //ROS_INFO("P_k_m_inverse = %f", P_k_m_inverse);
  w_x_old = w_x;
  w_y_old = w_y;
  w_z_old = w_z;


}

void p_I_m_model(){
  double F_x, F_y, F_z;
  double alpha_x, alpha_y, alpha_z;
  double w_x, w_y, w_z;
  double v_x, v_y, v_z;
  double a_x, a_y, a_z;
  Eigen::MatrixXd F_i_T(2,3);
  Eigen::Matrix3d alpha_i_x;
  Eigen::Matrix3d w_i_x2;
  Eigen::Matrix3d F_i_x;
  Eigen::MatrixXd H_I(3,6);
  Eigen::Vector3d F_i;
  Eigen::Vector3d w_i, v_i, a_i;
  Eigen::VectorXd y_D(8);
  Eigen::VectorXd y_D_U(2);
  Eigen::VectorXd y_D_M(3);
  Eigen::VectorXd y_D_D(3);

  Eigen::MatrixXd H_D(8,10);
  Eigen::MatrixXd H_D_LU(2,3);
  Eigen::Matrix3d H_D_LD;
  F_i_T << 1, -1, (-1*F_x + 1*F_y)/F_z,
           -1, 1, (1*F_x - 1*F_y)/F_z;
  alpha_i_x << 0, -alpha_z, alpha_y,
               alpha_z, 0, -alpha_x,
               -alpha_y, alpha_x, 0;
  w_i_x2 << -w_y*w_y-w_z*w_z, w_x*w_y, w_x*w_z,
            w_x*w_y, -w_x*w_x-w_z*w_z, w_y*w_z,
            w_x*w_z, w_y*w_z, -w_x*w_x - w_y*w_y;
  F_i_x << 0, -F_z, F_y,
           F_z, 0, -F_x,
          -F_y, F_x, 0;
  H_I << alpha_x, -w_y*w_z, w_y*w_z, alpha_y - w_x*w_z, alpha_z+w_x*w_y, w_y*w_y-w_z*w_z,
         w_x*w_z, alpha_y, -w_x*w_z, alpha_x + w_y*w_z, w_z*w_z-w_x*w_x, alpha_z - w_x*w_y,
        -w_x*w_y, w_x*w_y, alpha_z, w_x*w_x-w_y*w_y, alpha_x - w_y*w_z, alpha_y + w_x*w_z;
  F_i << F_x, F_y, F_z;
  w_i << w_x, w_y, w_z;
  v_i << v_x, v_y, v_z;
  a_i << a_x, a_y, a_z;

  H_D_LU = F_i_T*(alpha_i_x + w_i_x2);
  H_D_LD = -alpha_i_x - w_i_x2;
  H_D << H_D_LU(0,0), H_D_LU(0,1), H_D_LU(0,2), 0, 0, 0, 0, 0, 0, 0,
         H_D_LU(1,0), H_D_LU(1,1), H_D_LU(2,2), 0, 0, 0, 0, 0, 0, 0,
         -F_i_x(0,0), -F_i_x(0,1), -F_i_x(0,2), H_I(0,0), H_I(0,1), H_I(0,2), H_I(0,3), H_I(0,4), H_I(0,5), 0,
         -F_i_x(1,0), -F_i_x(1,1), -F_i_x(1,2), H_I(1,0), H_I(1,1), H_I(1,2), H_I(1,3), H_I(1,4), H_I(1,5), 0,
         -F_i_x(2,0), -F_i_x(2,1), -F_i_x(2,2), H_I(2,0), H_I(2,1), H_I(2,2), H_I(2,3), H_I(2,4), H_I(2,5), 0,
         H_D_LD(0,0), H_D_LD(0,1), H_D_LD(0,2), 0, 0, 0, 0, 0, 0, F_i(0),
         H_D_LD(1,0), H_D_LD(1,1), H_D_LD(1,2), 0, 0, 0, 0, 0, 0, F_i(1),
         H_D_LD(2,0), H_D_LD(2,1), H_D_LD(2,2), 0, 0, 0, 0, 0, 0, F_i(2);
  y_D_U = -F_i_x*(w_i.cross(v_i) + a_i);
  y_D_M << 0, 0, 0;
  y_D_D = w_i.cross(v_i) + a_i;
  y_D << y_D_U(0), y_D_U(1), y_D_U(2), y_D_M(0), y_D_M(1), y_D_M(2), y_D_D(0), y_D_D(1), y_D_D(2);

}

void filer(Eigen::VectorXd &y_k, Eigen::MatrixXd &H_k, Eigen::MatrixXd &P_k, Eigen::MatrixXd &R_k, Eigen::VectorXd *x_k_1){



}

struct CostFunctor {
   template <typename T>
   bool operator()(const T* const x, T* residual) const {
     residual[0] = T(10.0) - x[0];
     return true;
   }
};

struct F1 {
   template <typename T>
   bool operator()(const T* const x, T* residual) const {
     residual[0] = (sqrt(P_k_inverse(5,5))) * (x[0] - (qji_d_x_k));
     return true;
   }
};

struct F2 {
   template <typename T>
   bool operator()(const T* const x, T* residual) const {
     residual[0] = T(R_k_inverse(5,5))*(T(0) - T(transform_hkxk(5)));
     return true;
   }
};


struct F1_v {
   template <typename T>
   bool operator()(const T* const x, T* residual) const {
     residual[0] = ((x[0]-qji_k(0))*P_k_inverse(0,0) + (x[1]-qji_k(1))*P_k_inverse(1,0) + (x[2]-qji_k(2))*P_k_inverse(2,0) + (x[3]-qji_k(3))*P_k_inverse(3,0) + (x[4]-qji_k(4))*P_k_inverse(4,0) + (x[5]-qji_k(5))*P_k_inverse(5,0) + (x[6]-qji_k(6))*P_k_inverse(6,0) + (x[7]-qji_k(7))*P_k_inverse(7,0))*(x[0] - qji_k(0))
                  +((x[0]-qji_k(0))*P_k_inverse(0,1) + (x[1]-qji_k(1))*P_k_inverse(1,1) + (x[2]-qji_k(2))*P_k_inverse(2,1) + (x[3]-qji_k(3))*P_k_inverse(3,1) + (x[4]-qji_k(4))*P_k_inverse(4,1) + (x[5]-qji_k(5))*P_k_inverse(5,1) + (x[6]-qji_k(6))*P_k_inverse(6,1) + (x[7]-qji_k(7))*P_k_inverse(7,1))*(x[1] - qji_k(1))
                  +((x[0]-qji_k(0))*P_k_inverse(0,2) + (x[1]-qji_k(1))*P_k_inverse(1,2) + (x[2]-qji_k(2))*P_k_inverse(2,2) + (x[3]-qji_k(3))*P_k_inverse(3,2) + (x[4]-qji_k(4))*P_k_inverse(4,2) + (x[5]-qji_k(5))*P_k_inverse(5,2) + (x[6]-qji_k(6))*P_k_inverse(6,2) + (x[7]-qji_k(7))*P_k_inverse(7,2))*(x[2] - qji_k(2))
                  +((x[0]-qji_k(0))*P_k_inverse(0,3) + (x[1]-qji_k(1))*P_k_inverse(1,3) + (x[2]-qji_k(2))*P_k_inverse(2,3) + (x[3]-qji_k(3))*P_k_inverse(3,3) + (x[4]-qji_k(4))*P_k_inverse(4,3) + (x[5]-qji_k(5))*P_k_inverse(5,3) + (x[6]-qji_k(6))*P_k_inverse(6,3) + (x[7]-qji_k(7))*P_k_inverse(7,3))*(x[3] - qji_k(3))
                  +((x[0]-qji_k(0))*P_k_inverse(0,4) + (x[1]-qji_k(1))*P_k_inverse(1,4) + (x[2]-qji_k(2))*P_k_inverse(2,4) + (x[3]-qji_k(3))*P_k_inverse(3,4) + (x[4]-qji_k(4))*P_k_inverse(4,4) + (x[5]-qji_k(5))*P_k_inverse(5,4) + (x[6]-qji_k(6))*P_k_inverse(6,4) + (x[7]-qji_k(7))*P_k_inverse(7,4))*(x[4] - qji_k(4))
                  +((x[0]-qji_k(0))*P_k_inverse(0,5) + (x[1]-qji_k(1))*P_k_inverse(1,5) + (x[2]-qji_k(2))*P_k_inverse(2,5) + (x[3]-qji_k(3))*P_k_inverse(3,5) + (x[4]-qji_k(4))*P_k_inverse(4,5) + (x[5]-qji_k(5))*P_k_inverse(5,5) + (x[6]-qji_k(6))*P_k_inverse(6,5) + (x[7]-qji_k(7))*P_k_inverse(7,5))*(x[5] - qji_k(5))
                  +((x[0]-qji_k(0))*P_k_inverse(0,6) + (x[1]-qji_k(1))*P_k_inverse(1,6) + (x[2]-qji_k(2))*P_k_inverse(2,6) + (x[3]-qji_k(3))*P_k_inverse(3,6) + (x[4]-qji_k(4))*P_k_inverse(4,6) + (x[5]-qji_k(5))*P_k_inverse(5,6) + (x[6]-qji_k(6))*P_k_inverse(6,6) + (x[7]-qji_k(7))*P_k_inverse(7,6))*(x[6] - qji_k(6))
                  +((x[0]-qji_k(0))*P_k_inverse(0,7) + (x[1]-qji_k(1))*P_k_inverse(1,7) + (x[2]-qji_k(2))*P_k_inverse(2,7) + (x[3]-qji_k(3))*P_k_inverse(3,7) + (x[4]-qji_k(4))*P_k_inverse(4,7) + (x[5]-qji_k(5))*P_k_inverse(5,7) + (x[6]-qji_k(6))*P_k_inverse(6,7) + (x[7]-qji_k(7))*P_k_inverse(7,7))*(x[7] - qji_k(7))
                  +((0-transform_hkxk(0))*R_k_inverse(0,0) + (0-transform_hkxk(1))*R_k_inverse(1,0) + (0-transform_hkxk(2))*R_k_inverse(2,0) + (0-transform_hkxk(3))*R_k_inverse(3,0) + (0-transform_hkxk(4))*R_k_inverse(4,0) + (0-transform_hkxk(5))*R_k_inverse(5,0) + (0-transform_hkxk(6))*R_k_inverse(6,0) + (0-transform_hkxk(7))*R_k_inverse(7,0))*(0 - transform_hkxk(0))
                  +((0-transform_hkxk(0))*R_k_inverse(0,1) + (0-transform_hkxk(1))*R_k_inverse(1,1) + (0-transform_hkxk(2))*R_k_inverse(2,1) + (0-transform_hkxk(3))*R_k_inverse(3,1) + (0-transform_hkxk(4))*R_k_inverse(4,1) + (0-transform_hkxk(5))*R_k_inverse(5,1) + (0-transform_hkxk(6))*R_k_inverse(6,1) + (0-transform_hkxk(7))*R_k_inverse(7,1))*(0 - transform_hkxk(1))
                  +((0-transform_hkxk(0))*R_k_inverse(0,2) + (0-transform_hkxk(1))*R_k_inverse(1,2) + (0-transform_hkxk(2))*R_k_inverse(2,2) + (0-transform_hkxk(3))*R_k_inverse(3,2) + (0-transform_hkxk(4))*R_k_inverse(4,2) + (0-transform_hkxk(5))*R_k_inverse(5,2) + (0-transform_hkxk(6))*R_k_inverse(6,2) + (0-transform_hkxk(7))*R_k_inverse(7,2))*(0 - transform_hkxk(2))
                  +((0-transform_hkxk(0))*R_k_inverse(0,3) + (0-transform_hkxk(1))*R_k_inverse(1,3) + (0-transform_hkxk(2))*R_k_inverse(2,3) + (0-transform_hkxk(3))*R_k_inverse(3,3) + (0-transform_hkxk(4))*R_k_inverse(4,3) + (0-transform_hkxk(5))*R_k_inverse(5,3) + (0-transform_hkxk(6))*R_k_inverse(6,3) + (0-transform_hkxk(7))*R_k_inverse(7,3))*(0 - transform_hkxk(3))
                  +((0-transform_hkxk(0))*R_k_inverse(0,4) + (0-transform_hkxk(1))*R_k_inverse(1,4) + (0-transform_hkxk(2))*R_k_inverse(2,4) + (0-transform_hkxk(3))*R_k_inverse(3,4) + (0-transform_hkxk(4))*R_k_inverse(4,4) + (0-transform_hkxk(5))*R_k_inverse(5,4) + (0-transform_hkxk(6))*R_k_inverse(6,4) + (0-transform_hkxk(7))*R_k_inverse(7,4))*(0 - transform_hkxk(4))
                  +((0-transform_hkxk(0))*R_k_inverse(0,5) + (0-transform_hkxk(1))*R_k_inverse(1,5) + (0-transform_hkxk(2))*R_k_inverse(2,5) + (0-transform_hkxk(3))*R_k_inverse(3,5) + (0-transform_hkxk(4))*R_k_inverse(4,5) + (0-transform_hkxk(5))*R_k_inverse(5,5) + (0-transform_hkxk(6))*R_k_inverse(6,5) + (0-transform_hkxk(7))*R_k_inverse(7,5))*(0 - transform_hkxk(5))
                  +((0-transform_hkxk(0))*R_k_inverse(0,6) + (0-transform_hkxk(1))*R_k_inverse(1,6) + (0-transform_hkxk(2))*R_k_inverse(2,6) + (0-transform_hkxk(3))*R_k_inverse(3,6) + (0-transform_hkxk(4))*R_k_inverse(4,6) + (0-transform_hkxk(5))*R_k_inverse(5,6) + (0-transform_hkxk(6))*R_k_inverse(6,6) + (0-transform_hkxk(7))*R_k_inverse(7,6))*(0 - transform_hkxk(6))
                  +((0-transform_hkxk(0))*R_k_inverse(0,7) + (0-transform_hkxk(1))*R_k_inverse(1,7) + (0-transform_hkxk(2))*R_k_inverse(2,7) + (0-transform_hkxk(3))*R_k_inverse(3,7) + (0-transform_hkxk(4))*R_k_inverse(4,7) + (0-transform_hkxk(5))*R_k_inverse(5,7) + (0-transform_hkxk(6))*R_k_inverse(6,7) + (0-transform_hkxk(7))*R_k_inverse(7,7))*(0 - transform_hkxk(7));
     return true;
   }
};

struct F1_inertia {
   template <typename T>
   bool operator()(const T* const x, T* residual) const {
     residual[0] = ((x[0]-inertia_k(0))*P_k_inverse_i(0,0) + (x[1]-inertia_k(1))*P_k_inverse_i(1,0) + (x[2]-inertia_k(2))*P_k_inverse_i(2,0) + (x[3]-inertia_k(3))*P_k_inverse_i(3,0) + (x[4]-inertia_k(4))*P_k_inverse_i(4,0) + (x[5]-inertia_k(5))*P_k_inverse_i(5,0))
                  +((x[0]-inertia_k(0))*P_k_inverse_i(0,1) + (x[1]-inertia_k(1))*P_k_inverse_i(1,1) + (x[2]-inertia_k(2))*P_k_inverse_i(2,1) + (x[3]-inertia_k(3))*P_k_inverse_i(3,1) + (x[4]-inertia_k(4))*P_k_inverse_i(4,1) + (x[5]-inertia_k(5))*P_k_inverse_i(5,1))
                  +((x[0]-inertia_k(0))*P_k_inverse_i(0,2) + (x[1]-inertia_k(1))*P_k_inverse_i(1,2) + (x[2]-inertia_k(2))*P_k_inverse_i(2,2) + (x[3]-inertia_k(3))*P_k_inverse_i(3,2) + (x[4]-inertia_k(4))*P_k_inverse_i(4,2) + (x[5]-inertia_k(5))*P_k_inverse_i(5,2))
                  +((x[0]-inertia_k(0))*P_k_inverse_i(0,3) + (x[1]-inertia_k(1))*P_k_inverse_i(1,3) + (x[2]-inertia_k(2))*P_k_inverse_i(2,3) + (x[3]-inertia_k(3))*P_k_inverse_i(3,3) + (x[4]-inertia_k(4))*P_k_inverse_i(4,3) + (x[5]-inertia_k(5))*P_k_inverse_i(5,3))
                  +((x[0]-inertia_k(0))*P_k_inverse_i(0,4) + (x[1]-inertia_k(1))*P_k_inverse_i(1,4) + (x[2]-inertia_k(2))*P_k_inverse_i(2,4) + (x[3]-inertia_k(3))*P_k_inverse_i(3,4) + (x[4]-inertia_k(4))*P_k_inverse_i(4,4) + (x[5]-inertia_k(5))*P_k_inverse_i(5,4))
                  +((x[0]-inertia_k(0))*P_k_inverse_i(0,5) + (x[1]-inertia_k(1))*P_k_inverse_i(1,5) + (x[2]-inertia_k(2))*P_k_inverse_i(2,5) + (x[3]-inertia_k(3))*P_k_inverse_i(3,5) + (x[4]-inertia_k(4))*P_k_inverse_i(4,5) + (x[5]-inertia_k(5))*P_k_inverse_i(5,5))
                  +((y_I(0)-hI_Ii(0))*R_k_inverse_i(0,0) + (y_I(1)-hI_Ii(1))*R_k_inverse_i(1,0) + (y_I(2)-hI_Ii(2))*R_k_inverse_i(2,0))
                  +((y_I(0)-hI_Ii(0))*R_k_inverse_i(0,1) + (y_I(1)-hI_Ii(1))*R_k_inverse_i(1,1) + (y_I(2)-hI_Ii(2))*R_k_inverse_i(2,1))
                  +((y_I(0)-hI_Ii(0))*R_k_inverse_i(0,2) + (y_I(1)-hI_Ii(1))*R_k_inverse_i(1,2) + (y_I(2)-hI_Ii(2))*R_k_inverse_i(2,2));

     return true;
   }
};

struct F1_m {
   template <typename T>
   bool operator()(const T* const x, T* residual) const {
     residual[0] = ((x[0] - m_k) * P_k_m_inverse * (x[0] - m_k))
                   +(((y_m(0)-hkxk_m(0))*R_k_m_inverse(0,0) + (y_m(1) - hkxk_m(1))*R_k_m_inverse(1,0) + (y_m(2) - hkxk_m(2))*R_k_m_inverse(2,0)) * (y_m(0) - hkxk_m(0))
                   +((y_m(0)-hkxk_m(0))*R_k_m_inverse(0,1) + (y_m(1) - hkxk_m(1))*R_k_m_inverse(1,1) + (y_m(2) - hkxk_m(2)*R_k_m_inverse(2,1))) * (y_m(1) - hkxk_m(1))
                   +((y_m(0)-hkxk_m(0))*R_k_m_inverse(0,2) + (y_m(1) - hkxk_m(1))*R_k_m_inverse(1,2) + (y_m(2) - hkxk_m(2)*R_k_m_inverse(2,2))) * (y_m(2) - hkxk_m(2)))*0.5;

     return true;
   }
};
/*
struct F1_m1 {
   template <typename T>
   bool operator()(const T* const x, T* residual) const {
     residual[0] = 0*x[0]+((y_m(0)-hkxk_m(0))*R_k_m_inverse(0,0) + (y_m(1) - hkxk_m(1))*R_k_m_inverse(1,0) + (y_m(2) - hkxk_m(2))*R_k_m_inverse(2,0)) * (y_m(0) - hkxk_m(0))
                   +((y_m(0)-hkxk_m(0))*R_k_m_inverse(0,1) + (y_m(1) - hkxk_m(1))*R_k_m_inverse(1,1) + (y_m(2) - hkxk_m(2)*R_k_m_inverse(2,1))) * (y_m(1) - hkxk_m(1))
                   +((y_m(0)-hkxk_m(0))*R_k_m_inverse(0,2) + (y_m(1) - hkxk_m(1))*R_k_m_inverse(1,2) + (y_m(2) - hkxk_m(2)*R_k_m_inverse(2,2))) * (y_m(2) - hkxk_m(2));
     return true;
   }
};
*/
/*
struct F2_v {
   template <typename T>
   bool operator()(const T* const x, T* residual) const {
     residual[0] = (R_k_inverse.llt().matrixL())*(y_k - (transform_hkxk));
     return true;
   }
};
*/
DEFINE_string(minimizer, "trust_region",
              "Minimizer type to use, choices are: line_search & trust_region");
int main(int argc, char **argv)
{
  ros::init(argc, argv, "estimate_payload");
  ros::NodeHandle nh;
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  google::InitGoogleLogging(argv[0]);

  ros::Subscriber leader_v_sub = nh.subscribe<geometry_msgs::TwistStamped>("/drone3_imu/mavros/local_position/velocity", 1, leader_v_cb);
  ros::Subscriber follower_v_sub = nh.subscribe<geometry_msgs::TwistStamped>("/imu1/mavros/local_position/velocity", 1, follower_v_cb);
  ros::Subscriber leader_imu_sub = nh.subscribe<sensor_msgs::Imu>("/drone3_imu/mavros/imu/data", 1, leader_imu_cb);
  ros::Subscriber follower_imu_sub = nh.subscribe<sensor_msgs::Imu>("/imu1/mavros/imu/data", 1, follower_imu_cb);
  ros::Subscriber imu1_mocap_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/RigidBody1/pose", 1, imu1_mocap_cb);
  ros::Subscriber follower_force_sub = nh.subscribe<geometry_msgs::Point>("/follower_ukf/force", 1, follower_force_cb);
  ros::Subscriber leader_force_sub = nh.subscribe<geometry_msgs::Point>("/leader_force", 1, leader_force_cb);
  ros::Publisher plot_pub = nh.advertise<geometry_msgs::Point>("/plot", 1);

  initial();
  double qji_5_old, qji_0_old, qji_1_old, qji_2_old, qji_3_old, qji_4_old, qji_6_old, qji_7_old;
  double qji_5_initial, qji_0_initial, qji_1_initial, qji_2_initial, qji_3_initial, qji_4_initial, qji_6_initial, qji_7_initial;

  double inertia_5_old, inertia_0_old, inertia_1_old, inertia_2_old, inertia_3_old, inertia_4_old;
  double inertia_5_initial, inertia_0_initial, inertia_1_initial, inertia_2_initial, inertia_3_initial, inertia_4_initial;


  // The variable to solve for with its initial value.
  //initial qji
  double x1[] = {1.0, 0.2, 0.3, 0.2, 0.0, 3.0, 1.0, 1.0};

  double qji[] = {1.0, 0.0, 0.0, 0.0, 0.0, 3.0, 1.0, 1.0};
  double inertia[] = {0.6, 0.5, 0.4, 0.3, 0.2, 0.1};
  double m = 0.1;
  double m_old, m_initial;
  int count_m = 0;
  double m_final;
  m_old = 0.1;
  int flag3 = 0;
  ROS_INFO("qji_initial = %f", qji[5]);
  ROS_INFO("qji_k_initial = %f", qji_k(5));
  //ROS_INFO("m_k_initial = %f", m_k);
  //ROS_INFO("m_initial = %f", m);
    // Build the problem.
    //Problem problem_m;

    rate_r = 5;
    ros::Rate rate(rate_r);
    while(ros::ok()){

    checkmeasure();
    quaternionToRPY();
    if(flag ==1 && flag2 ==1){
    transformation_model();
    m_model();
    inertia_model();
    Problem problem;
    Problem problem_m;
    Problem problem_inertia;
    // ----------------------estimate mass---------------------------------

    problem_m.AddResidualBlock(new AutoDiffCostFunction<F1_m, 1, 1>(new F1_m),
                               NULL,
                               &m);


    problem_m.SetParameterUpperBound(&m, 0, 1);
    problem_m.SetParameterLowerBound(&m, 0, 0);
    Solver::Options options_m;

    /*
    LOG_IF(FATAL, !ceres::StringToMinimizerType(FLAGS_minimizer,
                                                &options_m.minimizer_type))
        << "Invalid minimizer: " << FLAGS_minimizer
        << ", valid options are: trust_region and line_search.";
        */
    options_m.max_num_iterations = 20;
    options_m.linear_solver_type = ceres::DENSE_QR;
    options_m.minimizer_progress_to_stdout = true;
    //options.check_gradients = true;
    options_m.use_nonmonotonic_steps = true;
    Solver::Summary summary_m;
    //Solve(options_m, &problem_m, &summary_m);

/*
    if(summary.final_cost < 1e-24){
      ROS_INFO("final m = %f", m);
      ros::shutdown();
    }
*/
    //std::cout << summary_m.BriefReport() << "\n";

    //ROS_INFO("m = %.8f", m);
    //ROS_INFO("m_old = %.8f", m_old);
/*
    plot_x.x = m;
    plot_x.y = 0.3;
    plot_pub.publish(plot_x);
*/
/*
    if(abs(m - m_old) < 1e-5){
      count_m += 1;
    }
    if(count_m > 100){
      ROS_INFO("final m = %f", m);

      ros::shutdown();
    }
      */

    m_old = m;
    m_k = m;
    m = 0.1;
    flag3 = 1;

/*
    if(m < m_old || m == 1){
      m_initial = m + 0.5*(m - m_old);
    }
    else if(m == 0){
      m_initial = m + 0.05;
    }
    else{
      m_initial = m + 0.5*(m - m_old);
    }
    m_old = m;
    m = m_initial;
    */

 //-------------------------------------------------------------------------------
 // --------------------------estimate length---------------------------

    // Set up the only cost function (also known as residual). This uses
    // auto-differentiation to obtain the derivative (jacobian).

    int i = 5;
    double up = 3;
    double low = 1;
    vector<int> qji4_const_v;
    qji4_const_v.insert(qji4_const_v.end(), 0);
    qji4_const_v.insert(qji4_const_v.end(), 1);
    qji4_const_v.insert(qji4_const_v.end(), 2);
    qji4_const_v.insert(qji4_const_v.end(), 3);
    qji4_const_v.insert(qji4_const_v.end(), 4);
    problem.AddResidualBlock(new AutoDiffCostFunction<F1_v, 1, 8>(new F1_v),
                               NULL,
                               qji);
    ceres::SubsetParameterization *qji4_const = new ceres::SubsetParameterization(8, qji4_const_v);
    problem.SetParameterization(qji, qji4_const);
    problem.SetParameterUpperBound(qji, i, up);
    problem.SetParameterLowerBound(qji, i, low);

    //problem.SetParameterBlockConstant(qji);
    /*
    problem.AddResidualBlock(new AutoDiffCostFunction<F2_v, 1, 8>(new F2_v),
                               NULL,
                             qji);
*/
    // Run the solver!

    Solver::Options options;
    /*
    LOG_IF(FATAL, !ceres::StringToMinimizerType(FLAGS_minimizer,
                                                &options.minimizer_type))
        << "Invalid minimizer: " << FLAGS_minimizer
        << ", valid options are: trust_region and line_search.";
        */
    options.max_num_iterations = 100;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    //options.use_nonmonotonic_steps = true;
    //options.trust_region_strategy_type = ceres::DOGLEG;
    //options.use_nonmonotonic_steps = true;
    //options.function_tolerance = 1e-9;
    //options.max_line_search_step_contraction = 1e-1;
    Solver::Summary summary;
    //Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
    /*
    std::cout << "qji_x : " << qji_k
              << " -> " << qji << "\n";
*/
    //qji_d_x_k = qji_d_x;
    //qji_k = qji;
/*
    ROS_INFO("------------------------------------");
    ROS_INFO("tx = %f, ty = %f, tz = %f", qji[5], qji[6], qji[7]);
    ROS_INFO("qw = %f, qx = %f, qy = %f, qz = %f", qji[0],qji[1],qji[2],qji[3]);
    ROS_INFO("------------------------------------");
*/
    plot_x.x = qji[5];
    plot_x.y = 1.6;
    plot_pub.publish(plot_x);
    //ROS_INFO("---------------------");
    //ROS_INFO("qjir_w = %f, qjir_x = %f, qjir_y = %f, qjir_z = %f", qji[0], qji[1], qji[2], qji[3]);
    //ROS_INFO("qjid_w = %f, qjid_x = %f, qjid_y = %f, qjid_z = %f", qji[4], qji[5], qji[6], qji[7]);
    qji_k(0) = qji[0];
    qji_k(1) = qji[1];
    qji_k(2) = qji[2];
    qji_k(3) = qji[3];
    qji_k(4) = qji[4];
    qji_k(5) = qji[5];
    qji_k(6) = qji[6];
    qji_k(7) = qji[7];




    if(qji[0] < qji_0_old){
      qji_0_initial = qji[0] - 0.1;
    }
    else{
      qji_0_initial = qji[0] + 0.1;
    }

    if(qji[1] < qji_1_old){
      qji_1_initial = qji[1] - 0.1;
    }
    else{
      qji_1_initial = qji[1] + 0.1;
    }
    if(qji[2] < qji_2_old){
      qji_2_initial = qji[2] - 0.1;
    }
    else{
      qji_2_initial = qji[2] + 0.1;
    }

    if(qji[3] < qji_3_old){
      qji_3_initial = qji[3] - 0.1;
    }
    else{
      qji_3_initial = qji[3] + 0.1;
    }
    if(qji[4] < qji_4_old){
      qji_4_initial = qji[4] - 0.1;
    }
    else{
      qji_4_initial = qji[4] + 0.1;
    }

    if(qji[5] < qji_5_old){
      qji_5_initial = qji[5] - 0.1;
    }
    else{
      qji_5_initial = qji[5] + 0.1;
    }
    if(qji[6] < qji_6_old){
      qji_6_initial = qji[6] - 0.1;
    }
    else{
      qji_6_initial = qji[6] + 0.1;
    }

    if(qji[7] < qji_7_old){
      qji_7_initial = qji[7] - 0.1;
    }
    else{
      qji_7_initial = qji[7] + 0.1;
    }

    qji_0_old = qji[0];
    qji_1_old = qji[1];
    qji_2_old = qji[2];
    qji_3_old = qji[3];
    qji_4_old = qji[4];
    qji_5_old = qji[5];
    qji_6_old = qji[6];
    qji_7_old = qji[7];

    qji[0] = 1.0;
    qji[1] = 0.0;
    qji[2] = 0.0;
    qji[3] = 0.0;
    qji[4] = 0.0;
    qji[5] = 3.0;
    qji[6] = 1.0;
    qji[7] = 1.0;
    // ----------------------------------------------------------------------------------------------
    // --------------------------estimate inertia---------------------------

       // Set up the only cost function (also known as residual). This uses
       // auto-differentiation to obtain the derivative (jacobian).

       vector<int> inertia_const_v;
       inertia_const_v.insert(inertia_const_v.end(), 0);
       inertia_const_v.insert(inertia_const_v.end(), 1);
       inertia_const_v.insert(inertia_const_v.end(), 2);
       inertia_const_v.insert(inertia_const_v.end(), 3);
       inertia_const_v.insert(inertia_const_v.end(), 4);
       problem_inertia.AddResidualBlock(new AutoDiffCostFunction<F1_inertia, 1, 6>(new F1_inertia),
                                  NULL,
                                  inertia);
       //ceres::SubsetParameterization *qji4_const = new ceres::SubsetParameterization(8, qji4_const_v);
       //problem.SetParameterization(qji, qji4_const);
       //problem.SetParameterUpperBound(inertia, i, up);
       //problem.SetParameterLowerBound(qji, i, low);

       //problem.SetParameterBlockConstant(qji);
       /*
       problem.AddResidualBlock(new AutoDiffCostFunction<F2_v, 1, 8>(new F2_v),
                                  NULL,
                                qji);
   */
       // Run the solver!

       Solver::Options options_inertia;
       /*
       LOG_IF(FATAL, !ceres::StringToMinimizerType(FLAGS_minimizer,
                                                   &options.minimizer_type))
           << "Invalid minimizer: " << FLAGS_minimizer
           << ", valid options are: trust_region and line_search.";
           */
       options_inertia.max_num_iterations = 100;
       options_inertia.linear_solver_type = ceres::DENSE_QR;
       options_inertia.minimizer_progress_to_stdout = true;
       //options.use_nonmonotonic_steps = true;
       //options.trust_region_strategy_type = ceres::DOGLEG;
       //options.use_nonmonotonic_steps = true;
       //options.function_tolerance = 1e-9;
       //options.max_line_search_step_contraction = 1e-1;
       Solver::Summary summary_inertia;
       Solve(options_inertia, &problem_inertia, &summary_inertia);

       std::cout << summary_inertia.BriefReport() << "\n";

       ROS_INFO("Jzz = %f", inertia[0]);

       inertia_k(0) = inertia[0];
       inertia_k(1) = inertia[1];
       inertia_k(2) = inertia[2];
       inertia_k(3) = inertia[3];
       inertia_k(4) = inertia[4];
       inertia_k(5) = inertia[5];




       if(inertia[0] < inertia_0_old){
         inertia_0_initial = inertia[0] - 0.1;
       }
       else{
         inertia_0_initial = inertia[0] + 0.1;
       }

       if(inertia[1] < inertia_1_old){
         inertia_1_initial = inertia[1] - 0.1;
       }
       else{
         inertia_1_initial = inertia[1] + 0.1;
       }
       if(inertia[2] < inertia_2_old){
         inertia_2_initial = inertia[2] - 0.1;
       }
       else{
         inertia_2_initial = inertia[2] + 0.1;
       }

       if(inertia[3] < inertia_3_old){
         inertia_3_initial = inertia[3] - 0.1;
       }
       else{
         inertia_3_initial = inertia[3] + 0.1;
       }
       if(inertia[4] < inertia_4_old){
         inertia_4_initial = inertia[4] - 0.1;
       }
       else{
         inertia_4_initial = inertia[4] + 0.1;
       }

       if(inertia[5] < inertia_5_old){
         inertia_5_initial = inertia[5] - 0.1;
       }
       else{
         qji_5_initial = qji[5] + 0.1;
       }


       inertia_0_old = inertia[0];
       inertia_1_old = inertia[1];
       inertia_2_old = inertia[2];
       inertia_3_old = inertia[3];
       inertia_4_old = inertia[4];
       inertia_5_old = inertia[5];


       inertia[0] = 0.6;
       inertia[1] = 0.5;
       inertia[2] = 0.4;
       inertia[3] = 0.3;
       inertia[4] = 0.2;
       inertia[5] = 0.1;

       // ----------------------------------------------------------------------------------------------
    }
    ros::spinOnce();
    rate.sleep();

}
    return 0;

}
