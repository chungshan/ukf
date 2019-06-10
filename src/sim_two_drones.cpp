#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/TwistStamped.h>
#include <Eigen/Dense>
#include <tf/tf.h>
#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#define pi 3.1415926
#include <geometry_msgs/Point.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Imu.h>
#include "lpf2.h"
geometry_msgs::Point uav1_pose_z,uav2_pose_z,uav1_theta_hat_1_p,uav1_theta_hat_1_meas_p,uav1_force_est, uav2_force_est, uav1_rope_angle, uav2_rope_angle, uav2_theta_hat,uav2_theta_hat_meas, uav2_psi_hat, uav2_psi_hat_meas,uav2_E_dot_hat_theta,uav1_E_dot_hat_theta, uav1_control_input,uav2_control_input, uav1_theta;
lpf2 lpf2_E_hat_dot(10,0.0333);
lpf2 lpf2_E_f_dot(10,0.0333);
lpf2 lpf2_E_hat_dot_psi(10,0.0333);
lpf2 lpf2_E_f_dot_psi(10,0.0333);
lpf2 lpf2_E_psi(10,0.0333);
lpf2 lpf2_psi_dot(10,0.0333);
lpf2 lpf2_uav1_alpha(10,0.0333);
bool jumping_rope_control = false, start_observer = false;

//payload's imu cb
sensor_msgs::Imu payload_imu;
void payload_imu_cb(const sensor_msgs::Imu::ConstPtr& msg){
  payload_imu = *msg;
}


//uav1 cb
mavros_msgs::State uav1_current_state;
void uav1_state_cb(const mavros_msgs::State::ConstPtr& msg) {
    uav1_current_state = *msg;
}
geometry_msgs::PoseStamped uav1_host_mocap;
geometry_msgs::PoseStamped uav1_initial_pose;
bool uav1_init = false;
double uav1_roll, uav1_pitch, uav1_yaw;
Eigen::Vector4d uav1_mavatt_;
Eigen::Matrix3d uav1_RIB;
void uav1_host_pos(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
        uav1_host_mocap = *msg;
        if(uav1_init==false)
        {
        uav1_initial_pose = uav1_host_mocap;
        uav1_init = true;
        }
        tf::Quaternion Q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);

        tf::Matrix3x3(Q).getRPY(uav1_roll,uav1_pitch,uav1_yaw);

        double x = msg->pose.orientation.x;
        double y = msg->pose.orientation.y;
        double z = msg->pose.orientation.z;
        double w = msg->pose.orientation.w;
        uav1_mavatt_ << w,x,y,z;
        tf::Matrix3x3 R_IB_q;

        tf::Quaternion drone_q(x,y,z,w);
        R_IB_q.setRotation(drone_q);

        uav1_RIB<< w*w+x*x-y*y-z*z  , 2*x*y-2*w*z ,            2*x*z+2*w*y,
            2*x*y +2*w*z           , w*w-x*x+y*y-z*z    ,2*y*z-2*w*x,
            2*x*z -2*w*y          , 2*y*z+2*w*x        ,w*w-x*x-y*y+z*z;
        uav1_pose_z.z = uav1_host_mocap.pose.position.z;

}

geometry_msgs::TwistStamped uav1_host_mocapvel;
void uav1_host_vel(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
        uav1_host_mocapvel = *msg;

}

void uav1_force_cb(const geometry_msgs::Point::ConstPtr& msg)
{
        uav1_force_est = *msg;

}
geometry_msgs::WrenchStamped uav1_wrench;
void uav1_wrench_cb(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
        uav1_wrench = *msg;
        uav1_wrench.wrench.force.x = uav1_wrench.wrench.force.x;
        uav1_wrench.wrench.force.y = -uav1_wrench.wrench.force.y;
        uav1_wrench.wrench.force.z = -uav1_wrench.wrench.force.z;
}

sensor_msgs::Imu uav1_imu;
Eigen::Vector3d uav1_acc_inertia;
void uav1_imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
  Eigen::Vector3d uav1_acc_body;
  uav1_imu = *msg;
  uav1_acc_body << uav1_imu.linear_acceleration.x,uav1_imu.linear_acceleration.y,uav1_imu.linear_acceleration.z;
  uav1_acc_inertia = uav1_RIB*uav1_acc_body;
  uav1_acc_inertia(2) = uav1_acc_inertia(2) - 9.81;
}
//uav2 cb
mavros_msgs::State uav2_current_state;
void uav2_state_cb(const mavros_msgs::State::ConstPtr& msg) {
    uav2_current_state = *msg;
}
geometry_msgs::PoseStamped uav2_host_mocap;
geometry_msgs::PoseStamped uav2_initial_pose;
bool uav2_init = false;
double uav2_roll, uav2_pitch, uav2_yaw;
Eigen::Vector4d uav2_mavatt_;
Eigen::Matrix3d uav2_RIB;
void uav2_host_pos(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
        uav2_host_mocap = *msg;
        if(uav2_init==false)
        {
        uav2_initial_pose = uav2_host_mocap;
        uav2_init = true;
        }
        tf::Quaternion Q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);

        tf::Matrix3x3(Q).getRPY(uav2_roll,uav2_pitch,uav2_yaw);

        double x = msg->pose.orientation.x;
        double y = msg->pose.orientation.y;
        double z = msg->pose.orientation.z;
        double w = msg->pose.orientation.w;
        uav2_mavatt_ << w,x,y,z;
        tf::Matrix3x3 R_IB_q;

        tf::Quaternion drone_q(x,y,z,w);
        R_IB_q.setRotation(drone_q);

        uav2_RIB<< w*w+x*x-y*y-z*z  , 2*x*y-2*w*z ,            2*x*z+2*w*y,
            2*x*y +2*w*z           , w*w-x*x+y*y-z*z    ,2*y*z-2*w*x,
            2*x*z -2*w*y          , 2*y*z+2*w*x        ,w*w-x*x-y*y+z*z;

        uav2_pose_z.z = uav2_host_mocap.pose.position.z;

}



geometry_msgs::TwistStamped uav2_host_mocapvel;
void uav2_host_vel(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
        uav2_host_mocapvel = *msg;

}


void uav2_force_cb(const geometry_msgs::Point::ConstPtr& msg)
{
        uav2_force_est = *msg;

}
geometry_msgs::WrenchStamped uav2_wrench;
void uav2_wrench_cb(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
        uav2_wrench = *msg;

        uav2_wrench.wrench.force.x = -uav2_wrench.wrench.force.x;
        uav2_wrench.wrench.force.y = -uav2_wrench.wrench.force.y;
        uav2_wrench.wrench.force.z = -uav2_wrench.wrench.force.z;
}

sensor_msgs::Imu uav2_imu;
Eigen::Vector3d uav2_acc_inertia;
void uav2_imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
  Eigen::Vector3d uav2_acc_body;
  uav2_imu = *msg;
  uav2_acc_body << uav2_imu.linear_acceleration.x,uav2_imu.linear_acceleration.y,uav2_imu.linear_acceleration.z;
  uav2_acc_inertia = uav2_RIB*uav2_acc_body;
  uav2_acc_inertia(2) = uav2_acc_inertia(2) - 9.81;
}
typedef struct
{
    float roll;
    float x;
    float y;
    float z;
}vir;
/*
 * Taken from
 * http://stackoverflow.com/questions/421860/capture-characters-from-standard-input-without-waiting-for-enter-to-be-pressed
 *
 * @return the character pressed.
 */
char getch()
{
    int flags = fcntl(0, F_GETFL, 0);
    fcntl(0, F_SETFL, flags | O_NONBLOCK);

    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0) {
        perror("tcsetattr()");
    }
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0) {
        perror("tcsetattr ICANON");
    }
    if (read(0, &buf, 1) < 0) {
        //perror ("read()");
    }
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0) {
        perror ("tcsetattr ~ICANON");
    }
    return (buf);
}

//att controller
Eigen::Vector4d rot2Quaternion(Eigen::Matrix3d R){
  Eigen::Vector4d quat;
  double tr = R.trace();
  if (tr > 0.0) {
    double S = sqrt(tr + 1.0) * 2.0; // S=4*qw
    quat(0) = 0.25 * S;
    quat(1) = (R(2, 1) - R(1, 2)) / S;
    quat(2) = (R(0, 2) - R(2, 0)) / S;
    quat(3) = (R(1, 0) - R(0, 1)) / S;
  } else if ((R(0, 0) > R(1, 1)) & (R(0, 0) > R(2, 2))) {
    double S = sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0; // S=4*qx
    quat(0) = (R(2, 1) - R(1, 2)) / S;
    quat(1) = 0.25 * S;
    quat(2) = (R(0, 1) + R(1, 0)) / S;
    quat(3) = (R(0, 2) + R(2, 0)) / S;
  } else if (R(1, 1) > R(2, 2)) {
    double S = sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0; // S=4*qy
    quat(0) = (R(0, 2) - R(2, 0)) / S;
    quat(1) = (R(0, 1) + R(1, 0)) / S;
    quat(2) = 0.25 * S;
    quat(3) = (R(1, 2) + R(2, 1)) / S;
  } else {
    double S = sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0; // S=4*qz
    quat(0) = (R(1, 0) - R(0, 1)) / S;
    quat(1) = (R(0, 2) + R(2, 0)) / S;
    quat(2) = (R(1, 2) + R(2, 1)) / S;
    quat(3) = 0.25 * S;
  }
  return quat;
}

Eigen::Vector4d acc2quaternion(Eigen::Vector3d vector_acc, double yaw) {
  Eigen::Vector4d quat;
  Eigen::Vector3d zb_des, yb_des, xb_des, yc;
  Eigen::Matrix3d rotmat;
  yc = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())*Eigen::Vector3d::UnitY();
  zb_des = vector_acc / vector_acc.norm();
  xb_des = yc.cross(zb_des) / ( yc.cross(zb_des) ).norm();

  rotmat << xb_des(0), yb_des(0), zb_des(0),
            xb_des(1), yb_des(1), zb_des(1),
            xb_des(2), yb_des(2), zb_des(2);
  quat = rot2Quaternion(rotmat);
  yb_des = zb_des.cross(xb_des) / (zb_des.cross(xb_des)).norm();

  return quat;
}
Eigen::Vector4d quatMultiplication(Eigen::Vector4d &q, Eigen::Vector4d &p) {
   Eigen::Vector4d quat;
  quat << p(0) * q(0) - p(1) * q(1) - p(2) * q(2) - p(3) * q(3),
          p(0) * q(1) + p(1) * q(0) - p(2) * q(3) + p(3) * q(2),
          p(0) * q(2) + p(1) * q(3) + p(2) * q(0) - p(3) * q(1),
          p(0) * q(3) - p(1) * q(2) + p(2) * q(1) + p(3) * q(0);
  return quat;
}
Eigen::Matrix3d quat2RotMatrix(Eigen::Vector4d q){
  Eigen::Matrix3d rotmat;
  rotmat << q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3),
    2 * q(1) * q(2) - 2 * q(0) * q(3),
    2 * q(0) * q(2) + 2 * q(1) * q(3),

    2 * q(0) * q(3) + 2 * q(1) * q(2),
    q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3),
    2 * q(2) * q(3) - 2 * q(0) * q(1),

    2 * q(1) * q(3) - 2 * q(0) * q(2),
    2 * q(0) * q(1) + 2 * q(2) * q(3),
    q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
  return rotmat;
}
const double norm_thrust_const_ = 0.06;
const double attctrl_tau_ = 0.15;
Eigen::Vector4d attcontroller(Eigen::Vector4d &ref_att, Eigen::Vector3d &ref_acc, Eigen::Vector4d &curr_att){
  Eigen::Vector4d ratecmd;
  Eigen::Vector4d qe, q_inv, inverse;
  Eigen::Matrix3d rotmat;
  Eigen::Vector3d zb;
  inverse << 1.0, -1.0, -1.0, -1.0;
  q_inv = inverse.asDiagonal() * curr_att;
  qe = quatMultiplication(q_inv, ref_att);
  ratecmd(0) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(1);
  ratecmd(1) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(2);
  ratecmd(2) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(3);
  rotmat = quat2RotMatrix(curr_att);
  zb = rotmat.col(2);
  ratecmd(3) = std::max(0.0, std::min(1.0, norm_thrust_const_ * ref_acc.dot(zb))); //Calculate thrust
  return ratecmd;
}

//controller

double uav1_sumx,uav1_sumy,uav1_sumz;
double KPx=5, KPy=5, KPz=0.5;
double KVx=1.3, KVy=1.3, KVz=0.2;
double KIx=0.33, KIy=0.33, KIz=0.05;
double KPyaw = 1;
double m_rope = 0.2;
double limit_contol_factor = 0.2;

double uav1_last_omega;
double total_air;
void follow_omega1(vir& vir, geometry_msgs::PoseStamped& host_mocap, geometry_msgs::TwistStamped& host_mocapvel, mavros_msgs::AttitudeTarget* pose,double theta,double theta_2,double omega,double omega_2)
{
float errx, erry, errz, err_roll;
float errvx , errvy ,errvz ;
float ux, uy, uz, uroll, upitch, uyaw, uz_test;
float ux_f, uy_f;
tf::Quaternion Q;
double E,E_z, E_des, E_des_z;
double J_rope, r_g;
const double g = 9.81;
double omega_des;
const double k_rope = 50;
const double ng = 0.4*9.81;
const double mp = 0.6;
const double mq = 1.5;
const double L = 0.5;
double jumping_rope_uy;
double alpha, alpha_filt;
double uz_f;
Eigen::Vector3d total_thrust;
Eigen::Vector3d e3;
std::vector<double> root_;
double theta_des;
double E_dot_l;
double theta_z;
double l_star;
const double c_d = 0.82;
const double Area = 1.5*0.05;
double E_dot_comp;
double theta_des_sign;
double v_c;
e3 << 0,0,1;
r_g = 0.5;
J_rope = m_rope * r_g * r_g;
omega_des = g/r_g;
theta_des = 135/57.29577951;
l_star = 0.5;
v_c = l_star*omega;
E = m_rope*g*r_g*(cos(theta) - 1)  + 0.5*J_rope*(omega)*(omega);
// + 0.5*J_rope*(omega)*(omega)
//E = m_rope*g*l_star*(cos(theta) - 1);
alpha = (omega - uav1_last_omega)*30;
alpha_filt = lpf2_uav1_alpha.filter(alpha);
//E_des = 0.5*J_rope*omega_des;
//E_des = 0;
//E_des = m_rope*g*r_g*(cos(theta_des) - 1)+ 0.5*J_rope*(1.5*omega)*(1.5*omega);//for compress swinging motion

theta_z = -(theta + pi);
if(omega>0){
  theta_des_sign = -1;
}
else{
  theta_des_sign = 1;
}
E_des = m_rope*g*l_star*(cos(theta_des) - 1) + 0.5*c_d*Area*1.2*v_c*v_c*l_star*(theta_des_sign*theta_des-theta);
E_des_z = m_rope*g*l_star*(1-cos(pi-theta_des)) + 0.5*c_d*Area*1.2*v_c*v_c*l_star*(theta_des_sign*theta_des-theta);//for analysis only
E_z = m_rope*g*l_star*(1-cos(theta_z)) + 0.5*J_rope*(omega)*(omega);//for analysis only
E_dot_l = -m_rope*l_star*(-omega)*cos(theta_z)*uav1_acc_inertia(0)/2;

uav1_E_dot_hat_theta.x = E_des_z;
uav1_E_dot_hat_theta.y = E_z;
uav1_E_dot_hat_theta.z = E_des_z - E_z;

uav1_theta.x = theta_z;
uav1_theta.y = pi - theta_des;
errx = -(host_mocap.pose.position.x - vir.x);
erry = -(host_mocap.pose.position.y - vir.y);
errz = -(host_mocap.pose.position.z - vir.z);
uav1_pose_z.x = fabs(errz);
uav1_pose_z.y = vir.z;
errvx = -(host_mocapvel.twist.linear.x - 0);
errvy = -(host_mocapvel.twist.linear.y - 0);
errvz = -(host_mocapvel.twist.linear.z - 0);

uav1_sumx += KIx*errx*0.02;
uav1_sumy += KIy*erry*0.02;
uav1_sumz += KIz*errz*0.02;

if(uav1_sumz>0.2){
  uav1_sumz = 0.2;
}
else if(uav1_sumz<-0.2){
  uav1_sumz = -0.2;
}
if(uav1_sumx>0.6){
  uav1_sumx = 0.6;
}
else if(uav1_sumx<-0.6){
  uav1_sumx = -0.6;
}
if(uav1_sumy>0.6){
  uav1_sumy = 0.6;
}
else if(uav1_sumy<-0.6){
  uav1_sumy = -0.6;
}
err_roll = vir.roll - uav1_yaw;
if(err_roll>pi)
err_roll = err_roll - 2*pi;
else if(err_roll<-pi)
err_roll = err_roll + 2*pi;

//ROS_INFO("err: x = %.3f, y = %.3f, z = %.3f, yaw = %.3f",errx,erry,errz,err_roll);
//ROS_INFO("err: %.3f,%.3f,%.3f",errvx,errvy,errvz);
Eigen::Vector3d att_rate_global;
Eigen::Vector3d att_rate_body;
Eigen::Vector3d g_;
g_ << 0,0,-9.8;
Eigen::Vector3d a_des;
Eigen::Vector3d a_ref;
Eigen::Vector3d a_fb;
Eigen::Vector4d q_des;
Eigen::Vector4d cmdbodyrate_;

 a_fb <<  6*errx + 1.5*errvx + uav1_sumx, 6*erry + 1.5*errvy+ uav1_sumy, 10*errz + 3.33*errvz;
 a_ref << 0, 0, 0;
 a_des << a_ref + a_fb - g_;
 q_des = acc2quaternion(a_des,uav1_yaw);
 cmdbodyrate_ = attcontroller(q_des,a_des,uav1_mavatt_);
 pose->body_rate.x = cmdbodyrate_(0);
 pose->body_rate.y = cmdbodyrate_(1);
 pose->body_rate.z = cmdbodyrate_(2);
 pose->thrust = cmdbodyrate_(3);

 //ROS_INFO("u: %.4f,%.4f,%.4f,%.4f",cmdbodyrate_(0),cmdbodyrate_(1),cmdbodyrate_(2),cmdbodyrate_(3));
 if(jumping_rope_control == true){
   //ux = 0.4*(E-E_des)*omega*cos(theta) + 0.1 * copysign(1,host_mocap.pose.position.x) * log(1-fabs(host_mocap.pose.position.x)/1);
   E_dot_comp = 0.5*c_d*Area*1.2*v_c*v_c*v_c;
   ux = 4*(E-E_des)*omega*cos(theta) - 2*E_dot_comp/(m_rope*l_star*omega*cos(theta));
   if(ux > limit_contol_factor*ng){
     ux =  limit_contol_factor*ng;
   }
   if(ux < - limit_contol_factor*ng){
     ux =  - limit_contol_factor*ng;
   }
   uz = - 0*(0.5*m_rope)*l_star*alpha_filt*sin(theta) - (0.5*m_rope)*l_star*omega*omega*cos(theta);
   //5*errz + 1.33*errvz;
   a_fb <<  0, 9*erry + 2.25*errvy+ uav1_sumy, 5*errz + 1.33*errvz;
   a_ref << ux, 0,0;//uz/1.5
   a_des << a_ref + a_fb - g_;
   q_des = acc2quaternion(a_des,uav1_yaw);
   cmdbodyrate_ = attcontroller(q_des,a_des,uav1_mavatt_);
   pose->body_rate.x = cmdbodyrate_(0);
   pose->body_rate.y = cmdbodyrate_(1);
   pose->body_rate.z = cmdbodyrate_(2);
   pose->thrust = cmdbodyrate_(3);
   uav1_control_input.x = ux;
   uav1_control_input.z = uz/1.5;
 }



}
double uav2_sumx,uav2_sumy,uav2_sumz;
double E_theta_hat_last, E_theta_hat_last_psi;
double a_f_theta_filt;
void follow_omega2(vir& vir, geometry_msgs::PoseStamped& host_mocap, geometry_msgs::TwistStamped& host_mocapvel, mavros_msgs::AttitudeTarget* pose,double theta,double theta_psi,double omega,double omega_psi)
{
float errx, erry, errz, err_roll;
float errvx , errvy ,errvz ;
float ux, uy, uz, uroll, upitch, uyaw, uz_test;
float ux_f, uy_f;
tf::Quaternion Q;
double E, E_des;
double J_rope, r_g;
const double g = 9.81;
double omega_des;
const double k_rope = 50;
const double ng = 0.4*9.81;
const double mp = 0.6;
const double mq = 1.5;
const double L = 0.5;
double jumping_rope_uy;
double alpha;
double uz_f;
Eigen::Vector3d total_thrust;
Eigen::Vector3d e3;
std::vector<double> root_;
double phi_theta;
double E_dot_f,E_dot_f_psi, E_dot_l, E_dot_l_psi;
double E_theta_hat, E_uav2_theta_hat_psi, E_uav2_theta_hat_psi_filt;
double E_theta_hat_dot, E_theta_hat_dot_psi;
double l_star;
double dt;
double E_theta_hat_dot_filt, E_dot_f_filt, E_theta_hat_dot_filt_psi, E_dot_f_filt_psi, E_psi_filt;
double delta_E_psi, copysign_;
double E_dot_comp;
const double c_d = 0.82;
const double Area = 1.5*0.05;
double v_c;
dt = 0.03333;
e3 << 0,0,1;
r_g = 0.51;
l_star = 0.50;
J_rope = m_rope * r_g * r_g;
omega_des = g/r_g;
E = 0.5*J_rope*omega*omega + m_rope*g*r_g*(cos(theta) - 1);
//E_des = 0.5*J_rope*omega_des;
E_des = 0;
v_c = l_star*omega;
E_dot_comp = 0.5*c_d*Area*1.2*v_c*v_c*v_c;

//For theta oscillation
E_dot_f = -m_rope*l_star*omega*cos(theta)*uav2_acc_inertia(0)/2;
E_theta_hat = 0.5*m_rope*l_star*l_star*omega*omega + m_rope*g*l_star*(1-cos(theta));
E_theta_hat_dot = (E_theta_hat- E_theta_hat_last)/dt;
E_theta_hat_dot_filt = lpf2_E_hat_dot.filter(E_theta_hat_dot);
E_dot_f_filt = lpf2_E_f_dot.filter(E_dot_f);
E_dot_l = E_theta_hat_dot_filt - E_dot_f_filt + E_dot_comp;
E_theta_hat_last = E_theta_hat;
//For psi oscillation
E_dot_f_psi = -m_rope*omega_psi*cos(theta_psi)*uav2_acc_inertia(0)/2;
E_uav2_theta_hat_psi = 0.5*m_rope*l_star*l_star*omega_psi*omega_psi + m_rope*g*l_star*(1-cos(theta_psi));
E_psi_filt = lpf2_E_psi.filter(E_uav2_theta_hat_psi);
//E_theta_hat_dot_psi = (E_uav2_theta_hat_psi- E_theta_hat_last_psi)/dt;
//E_theta_hat_dot_filt_psi = lpf2_E_hat_dot_psi.filter(E_theta_hat_dot_psi);
//E_dot_f_filt_psi = lpf2_E_f_dot_psi.filter(E_dot_f_psi);
//E_dot_l_psi = E_theta_hat_dot_filt_psi - E_dot_f_filt_psi;
//E_theta_hat_last_psi = E_uav2_theta_hat_psi;

//Store variable to topic

uav2_E_dot_hat_theta.x = E_dot_l;
uav2_E_dot_hat_theta.y = E_theta_hat_dot_filt;


//position control

errx = -(host_mocap.pose.position.x - vir.x);
erry = -(host_mocap.pose.position.y - vir.y);
errz = -(host_mocap.pose.position.z - vir.z);
uav2_pose_z.x = fabs(errz);
errvx = -(host_mocapvel.twist.linear.x - 0);
errvy = -(host_mocapvel.twist.linear.y - 0);
errvz = -(host_mocapvel.twist.linear.z - 0);

uav2_sumx += KIx*errx*0.02;
uav2_sumy += KIy*erry*0.02;
uav2_sumz += KIz*errz*0.02;

if(uav2_sumz>0.2){
  uav2_sumz = 0.2;
}
else if(uav2_sumz<-0.2){
  uav2_sumz = -0.2;
}
if(uav2_sumx>0.6){
  uav2_sumx = 0.6;
}
else if(uav2_sumx<-0.6){
  uav2_sumx = -0.6;
}
if(uav2_sumy>0.6){
  uav2_sumy = 0.6;
}
else if(uav2_sumy<-0.6){
  uav2_sumy = -0.6;
}
err_roll = vir.roll - uav2_yaw;
if(err_roll>pi)
err_roll = err_roll - 2*pi;
else if(err_roll<-pi)
err_roll = err_roll + 2*pi;

//ROS_INFO("err: x = %.3f, y = %.3f, z = %.3f, yaw = %.3f",errx,erry,errz,err_roll);
//ROS_INFO("err: %.3f,%.3f,%.3f",errvx,errvy,errvz);
Eigen::Vector3d att_rate_global;
Eigen::Vector3d att_rate_body;
Eigen::Vector3d g_;
g_ << 0,0,-9.8;
Eigen::Vector3d a_des;
Eigen::Vector3d a_ref;
Eigen::Vector3d a_fb;
Eigen::Vector4d q_des;
Eigen::Vector4d cmdbodyrate_;
if(jumping_rope_control == false){
 a_fb <<  6*errx + 1.5*errvx + uav2_sumx, 6*erry + 1.5*errvy+ uav2_sumy, 10*errz + 3.33*errvz;
 a_ref << 0, 0, 0;
 a_des << a_ref + a_fb - g_;
 q_des = acc2quaternion(a_des,uav2_yaw);
 cmdbodyrate_ = attcontroller(q_des,a_des,uav2_mavatt_);
 pose->body_rate.x = cmdbodyrate_(0);
 pose->body_rate.y = cmdbodyrate_(1);
 pose->body_rate.z = cmdbodyrate_(2);
 pose->thrust = cmdbodyrate_(3);
}
 //ROS_INFO("u: %.4f,%.4f,%.4f,%.4f",cmdbodyrate_(0),cmdbodyrate_(1),cmdbodyrate_(2),cmdbodyrate_(3));

if(jumping_rope_control == true){
  double omega_zero_theta,omega_zero_psi, omega_theta,omega_psi_n,phi_psi, delta_u, delta_l, a_dis,a_dis_psi, a_f_theta, r_A1_theta, r_A1_psi;
  omega_zero_theta = sqrt(g/l_star);
  omega_zero_psi = 2*omega_zero_theta;
  omega_theta = omega_zero_theta*(1-(fabs(theta))/pi);
  phi_theta = atan2(-omega/omega_zero_theta, theta);
  delta_u = 0.1;
  delta_l = -0.1;
  a_f_theta = 4;
  //For theta-oscillation
  if(E_dot_l > delta_u){
    a_dis = a_f_theta;
  }else if(E_dot_l < delta_l){
    a_dis = -a_f_theta;
  }else{
    a_dis = 0;
  }
  if((a_dis - a_f_theta_filt) == 0){
    copysign_ = 0;
  }else
  {
    copysign_ = copysign(1,a_dis - a_f_theta_filt);
  }
  a_f_theta_filt = a_f_theta_filt + dt*(10*copysign_);
  //ROS_INFO("a_f_theta_filt = %f", a_f_theta_filt);//s
  r_A1_theta = a_dis*omega_theta*omega_theta*sin(phi_theta);
  if(r_A1_theta > limit_contol_factor*0.5*ng){
    r_A1_theta = limit_contol_factor*0.5*ng;
  }
  if(r_A1_theta < -limit_contol_factor*0.5*ng){
    r_A1_theta = -limit_contol_factor*0.5*ng;
  }
  //ROS_INFO("uav2_r_A1_theta = %f", r_A1_theta);
  //For psi-oscillation
  phi_psi = atan2(-omega_psi/omega_zero_psi, theta_psi);
  omega_psi_n = omega_zero_psi*(1-(fabs(theta_psi))/pi);
  delta_E_psi = 0 - E_psi_filt;
  if(fabs(delta_E_psi) > 0.25){
    a_dis_psi =a_f_theta*copysign(1,delta_E_psi);//Always negative
  }else{
    a_dis_psi = (a_f_theta/0.25)*delta_E_psi;
  }
  r_A1_psi = a_dis_psi*omega_psi_n*omega_psi_n*sin(phi_psi);//probelm in sin(phi_psi), the psi_omega is inaccurate.
  //ROS_INFO("uav2_r_A1_psi = %f", r_A1_psi);
      //ux = 0.4*(E-E_des)*omega*cos(theta);

  ux = r_A1_theta + r_A1_psi;
  if(ux > limit_contol_factor*ng){
    ux =  limit_contol_factor*ng;
  }
  if(ux < - limit_contol_factor*ng){
    ux =  - limit_contol_factor*ng;
  }

  //ROS_INFO("uav2_ux = %f", ux);
  a_fb <<  0, 9*erry + 2.25*errvy+ uav2_sumy, 10*errz + 3.33*errvz;
  a_ref << ux, 0, 0;
  a_des << a_ref + a_fb - g_;
  q_des = acc2quaternion(a_des,uav2_yaw);
  cmdbodyrate_ = attcontroller(q_des,a_des,uav2_mavatt_);
  pose->body_rate.x = cmdbodyrate_(0);
  pose->body_rate.y = cmdbodyrate_(1);
  pose->body_rate.z = cmdbodyrate_(2);
  pose->thrust = cmdbodyrate_(3);
  uav2_control_input.x = E_dot_f;
  uav2_control_input.y = theta;
  uav2_control_input.z = uav2_acc_inertia(0);
  //ROS_INFO("phi_psi = %f", phi_psi);

  uav2_psi_hat.z = cmdbodyrate_(3);
}


}
//non-linear observer
Eigen::Vector2d uav2_theta_hat_1, uav1_theta_hat_1;
Eigen::Vector2d uav2_theta_hat_1_meas, uav1_theta_hat_1_meas, uav2_theta_hat_psi_meas;
Eigen::Vector2d uav2_theta_hat_psi, uav2_theta_hat_psi_last;
double uav2_theta_psi_dot, uav2_theta_psi_l;

Eigen::Vector2d non_linear_observer(Eigen::Vector2d& x, double& y)
{
  Eigen::Vector2d x_k_1_;
  double dt;
  Eigen::Vector2d L_theta;
  double y_hat;
  Eigen::Vector2d C;
  Eigen::Vector2d f_x;
  double g;
  double l_star;

  g = -9.81;
  l_star = 0.50;
  dt = 0.0333;
  //L_theta << 4.669, 2;//For force sensor
  L_theta << 4.669, 1;
  C << 1, 0;
  y_hat = (C.transpose()*x).value();
  f_x << x(1), -g/l_star*sin(x(0));
  //- 1/l_star*cos(x(0))*(uav1_control_input.x+uav2_control_input.x)/2
  // - 1/l_star*cos(x(0))*(uav1_acc_inertia(0)+uav2_acc_inertia(0))/2
  //ROS_INFO("y = %f, x_hat = %f", y, x(0));
  x_k_1_ = x + dt*(f_x + L_theta*(y-y_hat));
  return x_k_1_;
}
/*
void non_linear_observer_psi(Eigen::Vector2d& x, double& y)
{
  Eigen::Vector2d x_k_2_;
  double dt;
  Eigen::Vector2d L_psi;
  double y_hat;
  Eigen::Vector2d C;
  Eigen::Vector2d f_x;
  double g;
  double l_star;

  g = -9.81;
  l_star = 0.50;
  dt = 0.0333;
  L_psi << 4.669, 10;
  C << 1, 0;
  y_hat = (C.transpose()*x).value();
  f_x << x(1), -g/l_star*sin(x(0));
  ROS_INFO("y = %f, y-y_hat = %f", y, y-y_hat);
  //std::cout << "f_x:" << f_x << std::endl;
  x_k_2_ = x + dt*(f_x + L_psi*(y-y_hat));
  uav2_theta_hat_psi = x_k_2_;
}
*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "sim_two_drones");
  ros::NodeHandle nh;
  //uav1
  ros::Subscriber uav1_state_sub = nh.subscribe<mavros_msgs::State>
                              ("/uav1/mavros/state", 10, uav1_state_cb);
  ros::ServiceClient uav1_arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                                     ("/uav1/mavros/cmd/arming");
  ros::ServiceClient uav1_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                                       ("/uav1/mavros/set_mode");
  ros::Subscriber uav1_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/uav1/mavros/local_position/pose", 10, uav1_host_pos);
  ros::Publisher uav1_attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/uav1/mavros/setpoint_raw/attitude", 2);
  ros::Subscriber uav1_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/uav1/mavros/local_position/velocity_local", 1, uav1_host_vel);
  //uav2
  ros::Subscriber uav2_state_sub = nh.subscribe<mavros_msgs::State>
                              ("/uav2/mavros/state", 10, uav2_state_cb);
  ros::ServiceClient uav2_arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                                     ("/uav2/mavros/cmd/arming");
  ros::ServiceClient uav2_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                                       ("/uav2/mavros/set_mode");
  ros::Subscriber uav2_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/uav2/mavros/local_position/pose", 10, uav2_host_pos);
  ros::Publisher uav2_attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/uav2/mavros/setpoint_raw/attitude", 2);
  ros::Subscriber uav2_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/uav2/mavros/local_position/velocity_local", 1, uav2_host_vel);
  ros::Subscriber uav1_force_sub = nh.subscribe<geometry_msgs::Point>("/uav1/ttt", 1, uav1_force_cb);
  ros::Subscriber uav2_force_sub = nh.subscribe<geometry_msgs::Point>("/uav2/qqq", 1, uav2_force_cb);
  ros::Publisher uav1_rope_angle_pub = nh.advertise<geometry_msgs::Point>("/uav1rope_angle", 2);
  ros::Publisher uav2_rope_angle_pub = nh.advertise<geometry_msgs::Point>("/uav2rope_angle2", 2);
  ros::Subscriber uav1_wrench_sub = nh.subscribe<geometry_msgs::WrenchStamped>("/uav1_ft_sensor", 2, uav1_wrench_cb);
  ros::Subscriber uav2_wrench_sub = nh.subscribe<geometry_msgs::WrenchStamped>("/uav2_ft_sensor", 2, uav2_wrench_cb);
  ros::Publisher uav2_theta_hat_pub = nh.advertise<geometry_msgs::Point>("/uav2_theta_hat", 2);
  ros::Publisher uav2_theta_hat_meas_pub = nh.advertise<geometry_msgs::Point>("/uav2_theta_hat_meas", 2);
  ros::Publisher uav2_psi_hat_pub = nh.advertise<geometry_msgs::Point>("/uav2_psi_hat", 2);
  ros::Publisher uav2_psi_hat_meas_pub = nh.advertise<geometry_msgs::Point>("/uav2_psi_hat_meas", 2);

  ros::Publisher uav2_E_dot_hat_theta_pub = nh.advertise<geometry_msgs::Point>("/uav2_E_dot_hat_theta", 2);
  ros::Publisher uav2_control_input_pub = nh.advertise<geometry_msgs::Point>("/uav2_control_input", 2);
  ros::Publisher uav1_E_dot_hat_theta_pub = nh.advertise<geometry_msgs::Point>("/uav1_E_dot_hat_theta", 2);
  ros::Subscriber uav2_imu_sub = nh.subscribe<sensor_msgs::Imu>("/uav2/mavros/imu/data",2,uav2_imu_cb);
  ros::Subscriber uav1_imu_sub = nh.subscribe<sensor_msgs::Imu>("/uav1/mavros/imu/data",2,uav1_imu_cb);
  ros::Publisher uav1_theta_pub = nh.advertise<geometry_msgs::Point>("/uav1_theta", 2);
  ros::Publisher uav1_theta_hat_1_pub = nh.advertise<geometry_msgs::Point>("/uav1_theta_hat_1", 2);
  ros::Publisher uav1_theta_hat_1_meas_pub = nh.advertise<geometry_msgs::Point>("/uav1_theta_hat_1_meas", 2);
  ros::Publisher uav1_pose_z_pub = nh.advertise<geometry_msgs::Point>("/uav1_pose", 2);
  ros::Publisher uav2_pose_z_pub = nh.advertise<geometry_msgs::Point>("/uav2_pose", 2);
  ros::Publisher uav1_control_input_pub = nh.advertise<geometry_msgs::Point>("uav1_control_input", 2);
  ros::Rate rate(30);

  while (ros::ok() && uav1_current_state.connected && uav2_current_state.connected) {
      ros::spinOnce();
      rate.sleep();
  }

  vir vir1;

  vir1.x = 0.0;
  vir1.y = 0.4;
  vir1.z = 1.0;
  vir1.roll = 0;

  vir vir2;

  vir2.x = 0.0;
  vir2.y = -0.4;
  vir2.z = 1.0;
  vir2.roll = 0;

  mavros_msgs::AttitudeTarget uav1_pose, uav2_pose;

  uav1_pose.thrust = 0;
  uav1_pose.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
  uav1_pose.body_rate.x = 0;
  uav1_pose.body_rate.y = 0;
  uav1_pose.body_rate.z = 0;

  uav2_pose.thrust = 0;

  uav2_pose.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
  uav2_pose.body_rate.x = 0;
  uav2_pose.body_rate.y = 0;
  uav2_pose.body_rate.z = 0;

  //send a few setpoints before starting
 for(int i = 200; ros::ok() && i > 0; --i){
      uav1_attitude_pub.publish(uav1_pose);
      uav1_attitude_pub.publish(uav2_pose);

      ros::spinOnce();
      rate.sleep();
  }

 mavros_msgs::SetMode uav1_offb_set_mode, uav2_offb_set_mode;
 uav1_offb_set_mode.request.custom_mode = "OFFBOARD";
 uav2_offb_set_mode.request.custom_mode = "OFFBOARD";

 mavros_msgs::CommandBool uav1_arm_cmd, uav2_arm_cmd;
 uav1_arm_cmd.request.value = true;
 uav2_arm_cmd.request.value = true;

 ros::Time uav1_last_request = ros::Time::now();
 ros::Time uav2_last_request = ros::Time::now();
 while (ros::ok()) {
   if (uav1_current_state.mode != "OFFBOARD" &&
           (ros::Time::now() - uav1_last_request > ros::Duration(5.0))) {
       if( uav1_set_mode_client.call(uav1_offb_set_mode) &&
               uav1_offb_set_mode.response.mode_sent) {
           ROS_INFO("Offboard enabled11111111111111");
       }
       uav1_last_request = ros::Time::now();
   } else {

       if (!uav1_current_state.armed &&
               (ros::Time::now() - uav1_last_request > ros::Duration(5.0))) {
           if( uav1_arming_client.call(uav1_arm_cmd) &&
                   uav1_arm_cmd.response.success) {
               ROS_INFO("Vehicle armed1111111111111111");
           }
           uav1_last_request = ros::Time::now();
       }
   }
   if (uav2_current_state.mode != "OFFBOARD" &&
           (ros::Time::now() - uav2_last_request > ros::Duration(5.0))) {
       if( uav2_set_mode_client.call(uav2_offb_set_mode) &&
               uav2_offb_set_mode.response.mode_sent) {
           ROS_INFO("Offboard enabled222222222222222");
       }
       uav2_last_request = ros::Time::now();
   } else {

       if (!uav2_current_state.armed &&
               (ros::Time::now() - uav2_last_request > ros::Duration(5.0))) {
           if( uav2_arming_client.call(uav2_arm_cmd) &&
                   uav2_arm_cmd.response.success) {
               ROS_INFO("Vehicle armed22222222222222");
           }
           uav2_last_request = ros::Time::now();
       }
   }

   int c = getch();
   //ROS_INFO("C: %d",c);
   if (c != EOF) {
       switch (c) {
       case 65:    // key up
           vir1.z += 0.05;
           vir2.z += 0.05;
           break;
       case 66:    // key down
           vir1.z += -0.05;
           vir2.z += -0.05;
           break;
       case 67:    // key CW(->)
           vir1.roll -= 0.03;
           break;
       case 68:    // key CCW(<-)
           vir1.roll += 0.03;
           break;
       case 119:    // key foward
           vir1.x += 0.05;
           vir2.x += 0.05;
           break;
       case 49: // keyboard 1
           jumping_rope_control = true;
           break;
       case 50:
         start_observer = true;
         break;
       case 120:    // key back
           vir1.x += -0.05;
           vir2.x += -0.05;
           break;
       case 97:    // key left
           vir1.y += 0.05;
           vir2.y += 0.05;
           break;
       case 100:    // key right
           vir1.y -= 0.05;
           vir2.y -= 0.05;
           break;
       case 108:    // close arming
                   {
                   uav1_offb_set_mode.request.custom_mode = "MANUAL";
                   uav1_set_mode_client.call(uav1_offb_set_mode);
                   uav1_arm_cmd.request.value = false;
                   uav1_arming_client.call(uav1_arm_cmd);

                   uav2_offb_set_mode.request.custom_mode = "MANUAL";
                   uav2_set_mode_client.call(uav2_offb_set_mode);
                   uav2_arm_cmd.request.value = false;
                   uav2_arming_client.call(uav2_arm_cmd);
       break;
                   }
       case 115: // keyboard s
           vir1.z = 0.2;
           vir2.z = 0.2;
           break;
       case 63:
           return 0;
           break;
       }
   }
   if(vir1.roll>pi)
   vir1.roll = vir1.roll - 2*pi;
   else if(vir1.roll<-pi)
   vir1.roll = vir1.roll + 2*pi;

   if(vir2.roll>pi)
   vir2.roll = vir2.roll - 2*pi;
   else if(vir2.roll<-pi)
   vir2.roll = vir2.roll + 2*pi;


   //Calculate rope angle

   double uav1_rope_theta,uav2_rope_theta, uav1_rope_theta_sensor, uav2_rope_theta_sensor;

   //From force estimate
    //uav1
   if(uav1_force_est.z < 0){
     uav1_rope_theta = atan2(uav1_force_est.z,uav1_force_est.x) + 2*pi;
   }
   else{
     uav1_rope_theta = atan2(uav1_force_est.z,uav1_force_est.x);
   }
   if(uav1_force_est.x > 0 && uav1_force_est.z > 0){
     uav1_rope_theta = uav1_rope_theta + 1.5*pi;
   }
   else{
   uav1_rope_theta = uav1_rope_theta - pi/2;
   }

   uav1_rope_theta = -uav1_rope_theta;
   uav1_rope_angle.x = uav1_rope_theta;
    //uav2
   if(uav2_force_est.z < 0){
     uav2_rope_theta = atan2(uav2_force_est.z,uav2_force_est.x) + 2*pi;
   }
   else{
     uav2_rope_theta = atan2(uav2_force_est.z,uav2_force_est.x);
   }
   if(uav2_force_est.x > 0 && uav2_force_est.z > 0){
     uav2_rope_theta = uav2_rope_theta + 1.5*pi;
   }
   else{
   uav2_rope_theta = uav2_rope_theta - pi/2;
   }

   uav2_rope_theta = -uav2_rope_theta;
   //For theta = zero is downward
   //uav2_rope_theta = -(uav2_rope_theta + pi);




   uav2_rope_angle.x = -(uav2_rope_theta + pi);

   //From force sensor
   if(uav1_wrench.wrench.force.x < 0 && uav1_wrench.wrench.force.z > 0){
     uav1_rope_theta_sensor = atan2(uav1_wrench.wrench.force.z,uav1_wrench.wrench.force.x);
     uav1_rope_theta_sensor = -2.5*3.1415926+uav1_rope_theta_sensor;
   }
   else{
   uav1_rope_theta_sensor = atan2(uav1_wrench.wrench.force.z,uav1_wrench.wrench.force.x) - 3.1415926/2;
}
   uav1_rope_angle.y = uav1_rope_theta_sensor;


   if(uav2_wrench.wrench.force.x < 0 && uav2_wrench.wrench.force.z > 0){
     uav2_rope_theta_sensor = atan2(uav2_wrench.wrench.force.z,uav2_wrench.wrench.force.x);
     uav2_rope_theta_sensor = -2.5*3.1415926+uav2_rope_theta_sensor;
   }
   else{
   uav2_rope_theta_sensor = atan2(uav2_wrench.wrench.force.z,uav2_wrench.wrench.force.x) - 3.1415926/2;
}
   uav2_rope_angle.y = -(uav2_rope_theta_sensor+pi);

   //non-linear observer x(k+1) = non_linear_observer(x(k),y(k))

   //uav2


//   uav2_theta_psi_dot = (uav2_theta_hat_psi(0) - uav2_theta_psi_l)/0.03333;
//   uav2_theta_psi_l = uav2_theta_hat_psi(0);
    //psi
   double psi_meas, psi_meas_meas;
   psi_meas_meas = ((-(uav2_rope_theta_sensor+pi)) - (-(uav2_theta_hat_1(0)+pi)));
   psi_meas = uav2_rope_theta - uav2_theta_hat_1(0);
   //ROS_INFO("psi_meas = %f", psi_meas);
   psi_meas = psi_meas;
   uav2_theta_hat_psi(0) = -uav2_theta_hat_psi(0);
   uav2_theta_hat_psi_meas(0) = -uav2_theta_hat_psi_meas(0);
   uav2_theta_hat_psi = non_linear_observer(uav2_theta_hat_psi, psi_meas);
   uav2_theta_hat_psi_meas = non_linear_observer(uav2_theta_hat_psi_meas, psi_meas_meas);

    //theta
   uav2_theta_hat_1 = non_linear_observer(uav2_theta_hat_1,uav2_rope_theta);
   uav2_theta_hat_1_meas = non_linear_observer(uav2_theta_hat_1_meas,uav2_rope_theta_sensor);

   //ROS_INFO("y = %f, x_hat = %f", y, x(0));

   //uav1
   uav1_theta_hat_1 = non_linear_observer(uav1_theta_hat_1,uav1_rope_theta);
   uav1_theta_hat_1_meas = non_linear_observer(uav1_theta_hat_1_meas,uav1_rope_theta_sensor);


   uav1_theta_hat_1_p.x = -(uav1_theta_hat_1(0)+pi);
   uav1_theta_hat_1_p.y = uav1_theta_hat_1(1);
   uav1_theta_hat_1_meas_p.x = -(uav1_theta_hat_1_meas(0)+pi);
   uav1_theta_hat_1_meas_p.y = -uav1_theta_hat_1_meas(1);

   uav2_theta_hat.x = -(uav2_theta_hat_1(0) + pi);
   uav2_theta_hat.y = -uav2_theta_hat_1(1);
   uav2_theta_hat_meas.x = -(uav2_theta_hat_1_meas(0) + pi);
   uav2_theta_hat_meas.y = -uav2_theta_hat_1_meas(1);
   uav2_psi_hat.x =  -(uav2_theta_hat_psi(0));
   uav2_psi_hat.y = - uav2_theta_hat_psi(1);
   uav2_psi_hat_meas.x =  -(uav2_theta_hat_psi_meas(0));
   uav2_psi_hat_meas.y = - uav2_theta_hat_psi_meas(1);
   ROS_INFO("uav2_psi_hat_dot = %f", uav2_theta_hat_psi(1));
   double psi_filt;
   psi_filt = lpf2_psi_dot.filter(uav2_theta_psi_dot);
   //uav2_psi_hat.z =psi_filt;
   uav1_control_input_pub.publish(uav1_control_input);
   uav1_pose_z_pub.publish(uav1_pose_z);
   uav2_pose_z_pub.publish(uav2_pose_z);
   uav1_theta_hat_1_pub.publish(uav1_theta_hat_1_p);
   uav1_theta_hat_1_meas_pub.publish(uav1_theta_hat_1_meas_p);
   uav1_theta_pub.publish(uav1_theta);
   uav2_theta_hat_pub.publish(uav2_theta_hat);
   uav2_theta_hat_meas_pub.publish(uav2_theta_hat_meas);
   uav2_psi_hat_pub.publish(uav2_psi_hat);
   uav2_psi_hat_meas_pub.publish(uav2_psi_hat_meas);
   uav1_rope_angle_pub.publish(uav1_rope_angle);
   uav2_rope_angle_pub.publish(uav2_rope_angle);
   uav2_control_input_pub.publish(uav2_control_input);
   uav2_E_dot_hat_theta_pub.publish(uav2_E_dot_hat_theta);
   uav1_E_dot_hat_theta_pub.publish(uav1_E_dot_hat_theta);
   follow_omega1(vir1,uav1_host_mocap, uav1_host_mocapvel,&uav1_pose, uav1_theta_hat_1(0), 0, uav1_theta_hat_1(1),  0);
   follow_omega2(vir2,uav2_host_mocap, uav2_host_mocapvel,&uav2_pose,-(uav2_theta_hat_1(0) + pi), -uav2_theta_hat_psi(0),-uav2_theta_hat_1(1),  uav2_theta_hat_psi(1));
   uav1_attitude_pub.publish(uav1_pose);
   uav2_attitude_pub.publish(uav2_pose);
   ros::spinOnce();
   rate.sleep();
 }
return 0;
}



