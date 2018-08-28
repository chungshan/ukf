#include <ros/ros.h>
#include <eigen3/Eigen/Dense>


Eigen::Vector4d qji_r, qji_d;
Eigen::VectorXd qji(8);

void transformation_model(){
  double leader_vx, leader_vy, leader_vz;
  double follower_vx, follower_vy, follower_vz;
  double leader_wx, leader_wy, leader_wz;
  double follower_wx, follower_wy, follower_wz;
  Eigen::MatrixXd h_g(8,8);
  Eigen::VectorXd y(8);
  h_g << 0, leader_wx - follower_wx, leader_wy - follower_wy, leader_wz - follower_wz, 0, 0, 0, 0,
         follower_wx - leader_wx, 0, -(leader_wz + follower_wz), (leader_wy + follower_wy), 0, 0, 0, 0,
         follower_wy - leader_wy, (leader_wz + follower_wz), 0, -(leader_wx + follower_wx), 0, 0, 0, 0,
         follower_wz - leader_wz, -(leader_wy + follower_wy), (leader_wx + follower_wx), 0, 0, 0, 0, 0,
         0, leader_vx - follower_vx, leader_vy - follower_vy, leader_vz - follower_vz, 0, leader_wx - follower_wx, leader_wy - follower_wy, leader_wz - follower_wz,
         follower_vx - leader_vx, 0, -(leader_vz + follower_vz), (leader_vy + follower_vy), follower_wx - leader_wx, 0, -(leader_wz + follower_wz), (leader_wy + follower_wy),
         follower_vy - leader_vy, (leader_vz + follower_vz), 0, -(leader_vx + follower_vx), follower_wy - leader_wy, (leader_wz + follower_wz), 0, -(leader_wx + follower_wx),
         follower_vz - leader_vz, -(leader_vy + follower_vy), (leader_vx + follower_vx), 0, follower_wz - leader_wz, -(leader_wy + follower_wy), (leader_wx + follower_wx);
  y = h_g * qji;

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
  Eigen::Vector3d y_I;

  I << I_xx, I_yy, I_zz, I_xy, I_xz, I_yz;
  H_I << alpha_x, -w_y*w_z, w_y*w_z, alpha_y - w_x*w_z, alpha_z+w_x*w_y, w_y*w_y-w_z*w_z,
         w_x*w_z, alpha_y, -w_x*w_z, alpha_x + w_y*w_z, w_z*w_z-w_x*w_x, alpha_z - w_x*w_y,
        -w_x*w_y, w_x*w_y, alpha_z, w_x*w_x-w_y*w_y, alpha_x - w_y*w_z, alpha_y + w_x*w_z;
  F_i << F_x, F_y, F_z;
  p_ic << p_x, p_y, p_z;

  y_I = F_i.cross(p_ic);
  y_I = H_I * I;

}

void m_model(){
  double a_x, a_y, a_z;
  double alpha_x, alpha_y, alpha_z;
  double p_x, p_y, p_z;
  double w_x, w_y, w_z;
  double v_x, v_y, v_z;

  Eigen::Vector3d H_m;
  Eigen::Vector3d y_m;
  Eigen::Vector3d F_i;
  Eigen::Vector3d a_i;
  Eigen::Vector3d alpha_i;
  Eigen::Vector3d p_ic;
  Eigen::Vector3d w_i;
  Eigen::Vector3d v_i;

  a_i << a_x, a_y, a_z;
  alpha_i << alpha_x, alpha_y, alpha_z;
  p_ic << p_x, p_y, p_z;


}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "estimate_payload");
  ros::NodeHandle nh;

  ROS_INFO("Hello world!");
}
