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
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <geometry_msgs/Wrench.h>
#include <gazebo_msgs/LinkStates.h>

gazebo_msgs::ApplyBodyWrench drone_apply_force;
geometry_msgs::Wrench apply_wrench;
geometry_msgs::Point uav1_pose_z,uav2_pose_z,uav1_theta_hat_1_p,uav1_theta_hat_1_meas_p,uav1_force_est, uav2_force_est, uav1_rope_angle, uav2_rope_angle, uav2_theta_hat,uav2_theta_hat_meas, uav2_psi_hat, uav2_psi_hat_meas,uav2_E_dot_hat_theta,uav1_E_dot_hat_theta, uav1_control_input,uav2_control_input, uav1_theta;
lpf2 lpf2_E_hat_dot(10,0.0333);
lpf2 lpf2_E_f_dot(10,0.0333);
lpf2 lpf2_E_hat_dot_psi(10,0.0333);
lpf2 lpf2_E_f_dot_psi(10,0.0333);
lpf2 lpf2_E_psi(10,0.0333);
lpf2 lpf2_psi_dot(10,0.0333);
lpf2 lpf2_uav1_alpha(10,0.0333);
lpf2 lpf2_uav2_alpha(10,0.0333);
lpf2 lpf2_uav1_omega(10,0.0333);
lpf2 lpf2_uav2_omega(10,0.0333);
bool swing_up_control = false, start_observer = false, apply_wrench_flag = false;

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

geometry_msgs::Point groundtruth;
void ground_cb(const geometry_msgs::Point::ConstPtr& msg)
{
  groundtruth = *msg;
}

gazebo_msgs::LinkStates link_states;
geometry_msgs::Point payload_angular_v;
void link_cb(const gazebo_msgs::LinkStates::ConstPtr& msg){
   link_states = *msg;
   if(link_states.name.size()>0){
       for (unsigned int i=0;i<link_states.name.size();i++) {
         /*
           if(link_states.name[i].compare("payload::payload_link1_box")==0){
                con_1_pose<< link_states.pose[i].position.x,
                            link_states.pose[i].position.y,
                            link_states.pose[i].position.z;
                con_1_vel<< link_states.twist[i].linear.x,
                            link_states.twist[i].linear.y,
                            link_states.twist[i].linear.z;
                con1_vel.x = link_states.twist[i].linear.x;
                con1_vel.y = link_states.twist[i].linear.y;
                con1_vel.z = link_states.twist[i].linear.z;

           }
           if(link_states.name[i].compare("payload::payload_link2_box")==0){
               con_2_pose<< link_states.pose[i].position.x,
                           link_states.pose[i].position.y,
                           link_states.pose[i].position.z;
               con_2_vel<< link_states.twist[i].linear.x,
                           link_states.twist[i].linear.y,
                           link_states.twist[i].linear.z;
               con2_vel.x = link_states.pose[i].position.x;
               con2_vel.y = link_states.pose[i].position.y;
               con2_vel.z = link_states.pose[i].position.z;
           }
*/
         if(link_states.name[i].compare("payload::payload_rec")==0){
             payload_angular_v.x = link_states.twist[i].angular.x;
             payload_angular_v.y = -link_states.twist[i].angular.y;
             payload_angular_v.z = link_states.twist[i].angular.z;

         }

       }
   }

}
typedef struct
{
    float yaw;
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
double m_p = 0.5;//Payload's mass
double limit_contol_factor = 0.2;//To limit the quadrotor's control input. a_lim = limit_contol_factor*0.4*g.
double sw_Kpz = 40, sw_Kvz = 1.25;// Height control gain sw_Kpz = 25, sw_Kvz = 1.25
double uav1_last_omega;
double total_air;
double air_density = 1.2;

double last_groundtruth_theta;
void leader_controller(vir& vir, geometry_msgs::PoseStamped& host_mocap, geometry_msgs::TwistStamped& host_mocapvel, mavros_msgs::AttitudeTarget* pose,double theta,double theta_2,double omega,double omega_2)
{
float errx, erry, errz, err_yaw;
float errvx , errvy ,errvz ;
float ux, uz;
tf::Quaternion Q;
double E,E_z, E_des, E_des_z;
double J_p;
const double g = 9.81;
double omega_des;
const double ng = 0.4*9.81;
double alpha, alpha_filt;
Eigen::Vector3d e3;
double theta_des, theta_des_z;
double E_dot_l;
double theta_z, theta_n;
double l_star;
const double c_d = 0.82;
const double Area = 1.5*0.05;
double theta_des_sign, theta_control_sign;
double v_p;
double uav1_omega_filt;
double k_sw;
double uz_l;
double omega_ground_theta;

omega_ground_theta = (groundtruth.z - last_groundtruth_theta)*30;
last_groundtruth_theta = groundtruth.z;

e3 << 0,0,1;
l_star = 0.5;
J_p = m_p * l_star * l_star;//Payload's inertia
omega_des = g/l_star;
theta_des = 135/57.29577951;
v_p = l_star*(-omega);
// + 0.5*J_p*(omega)*(omega)
//E = m_p*g*l_star*(cos(theta) - 1);

//Calculate angular acceleration
uav1_omega_filt = lpf2_uav1_omega.filter(omega);
alpha = (uav1_omega_filt - uav1_last_omega)*30;
uav1_last_omega = uav1_omega_filt;

alpha_filt = lpf2_uav1_alpha.filter(alpha);//Angular accleration of theta (second order low pass filter)

//E_des = 0.5*J_p*omega_des;
//E_des = 0;
//E_des = m_p*g*l_star*(cos(theta_des) - 1)+ 0.5*J_p*(1.5*omega)*(1.5*omega);//for compress swinging motion

//Conver theta = 0 at upright position to downward position
theta_z = -(theta + pi);
theta_des_z = pi - theta_des;

//payload energy estimate
if(omega>0){
  theta_des_sign = -1;
}
else{
  theta_des_sign = 1;
}
//Convert theta to +pi ~ -pi
if(theta > -pi){
  theta_n = theta;
}

if(theta < -pi){
  theta_n = (theta + 2*pi);
}
E = m_p*g*l_star*(cos(theta) - 1)  + 0.5*J_p*(omega)*(omega);
E_des = m_p*g*l_star*(cos(theta_des) - 1) + 0.5*c_d*Area*air_density*v_p*v_p*l_star*(theta_des_z-theta_des_sign*theta_z);
//E_des_z and E_z is only for analysis(theta = 0 at downward position).
E_des_z = m_p*g*l_star*(1-cos(pi-theta_des)) + 0.5*c_d*Area*air_density*v_p*v_p*l_star*(theta_des_z-theta_des_sign*theta_z);
E_z = m_p*g*l_star*(1-cos(theta_z)) + 0.5*J_p*(omega)*(omega);

E_dot_l = -m_p*l_star*(-omega)*cos(theta_z)*uav1_acc_inertia(0)/2;

//Publish data

uav1_E_dot_hat_theta.x = E_des_z;
uav1_E_dot_hat_theta.y = E_z;
uav1_E_dot_hat_theta.z = 0.5*J_p*(payload_angular_v.y)*(payload_angular_v.y) + m_p*g*l_star*(1-cos(groundtruth.z));




uav1_theta.x = theta_z;
uav1_theta.y = pi - theta_des;
uav1_theta.z = groundtruth.z;
//Position, veloctiy error

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
err_yaw = vir.yaw - uav1_yaw;
if(err_yaw>pi)
err_yaw = err_yaw - 2*pi;
else if(err_yaw<-pi)
err_yaw = err_yaw + 2*pi;

//ROS_INFO("err: x = %.3f, y = %.3f, z = %.3f, yaw = %.3f",errx,erry,errz,err_yaw);
//ROS_INFO("err: %.3f,%.3f,%.3f",errvx,errvy,errvz);

//Geometric controller for position control

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

 //Swing-up controller

 if(swing_up_control == true){
   //ux = 0.4*(E-E_des)*omega*cos(theta) + 0.1 * copysign(1,host_mocap.pose.position.x) * log(1-fabs(host_mocap.pose.position.x)/1);

   k_sw = 4;
   /*Test for limited path
   if(host_mocap.pose.position.x < -0.5 && host_mocap.pose.position.x > -1){
     if(omega*cos(theta) < 0){
     k_sw = 4*(1-log(1-fabs(host_mocap.pose.position.x)/1));
     }else{
       k_sw = 4*(log(fabs(host_mocap.pose.position.x)/1));
     }
     k_sw = fabs(k_sw);
   }else if(host_mocap.pose.position.x < -1){
     if(omega*cos(theta) < 0){
     k_sw = 4*(log(fabs(host_mocap.pose.position.x)/1-1));
     }else{
       k_sw = 4*(log(fabs(host_mocap.pose.position.x)/1));
     }
     k_sw = fabs(k_sw);
   }
 */
   double k_a = air_density*c_d*Area;
   double k = 0.8;
   //leader's swing-up controller
   if(omega>0){
     theta_control_sign = 1;
   }
   else{
     theta_control_sign = 0;
   }


   ux = k_sw*(E-E_des)*omega*cos(theta) - 2*k_a*(l_star*l_star*alpha*(theta_des_z-theta_des_sign*theta_z) - theta_control_sign*v_p*v_p)/((1+k)*m_p*cos(theta));
   //saturation function
   if(ux > limit_contol_factor*ng){
     ux =  limit_contol_factor*ng;
   }
   if(ux < - limit_contol_factor*ng){
     ux =  - limit_contol_factor*ng;
   }
/*
   ROS_INFO("ux = %f", ux);
   ROS_INFO("====end====");
   */

   //height controller
   //uz = - 0*(0.5*m_p)*l_star*alpha_filt*sin(theta) - 0.5*(0.5*m_p)*l_star*uav1_omega_filt*uav1_omega_filt*cos(theta);

   //Admittance controller
   double M = 2;
   uz_l = (uav1_force_est.z + sw_Kpz*errz + sw_Kvz*errvz)/M;
   //uz_l = (3*errz + 0.3*errvz);Without height control
   a_fb <<  0, 9*erry + 2.25*errvy+ uav1_sumy, 0;//10*errz + 3.33*errvz (without using heigh control, these control gains also have a good performance)
   a_ref << ux, 0, uz_l;//

   uav1_control_input.x = ux;
   uav1_control_input.z = uz_l;
 }
 //Geometric controller: Conver acceleration command to body rate and thrust command
 a_des << a_ref + a_fb - g_;
 q_des = acc2quaternion(a_des,uav1_yaw);
 cmdbodyrate_ = attcontroller(q_des,a_des,uav1_mavatt_);
 pose->body_rate.x = cmdbodyrate_(0);
 pose->body_rate.y = cmdbodyrate_(1);
 pose->body_rate.z = cmdbodyrate_(2);
 pose->thrust = cmdbodyrate_(3);


}
double uav2_sumx,uav2_sumy,uav2_sumz;
double E_theta_hat_last, E_theta_hat_last_psi;
double a_f_theta_filt;
double uav2_last_omega, last_groundtruth_psi;

void follower_controller(vir& vir, geometry_msgs::PoseStamped& host_mocap, geometry_msgs::TwistStamped& host_mocapvel, mavros_msgs::AttitudeTarget* pose,double theta,double theta_psi,double omega,double omega_psi)
{
float errx, erry, errz, err_yaw;
float errvx , errvy ,errvz ;
float ux;
double E;
double J_p, l_star;
const double g = 9.81;
double omega_des;
const double ng = 0.4*9.81;
double uz_f;
Eigen::Vector3d e3;
double phi_theta;
double E_dot_f,E_dot_f_psi, E_dot_l;
double E_theta_hat, E_uav2_theta_hat_psi;
double E_theta_hat_dot;
double dt;
double E_theta_hat_dot_filt, E_dot_f_filt,E_psi_filt;
double delta_E_psi;
double E_dot_AD;
const double c_d = 0.82;
const double Area = 1.5*0.05;
double v_p;
double alpha, alpha_filt;
double uav2_omega_filt;
double omega_ground_psi;
double F_AD;
double E_psi_meas;
//Ground truth for theta_psi and omega_psi
//theta_psi = groundtruth.x;
omega_ground_psi = (groundtruth.x - last_groundtruth_psi)*30;
last_groundtruth_psi = groundtruth.x;
uav2_psi_hat.z = omega_ground_psi;
/*
omega_psi = -omega_ground_psi;
*/
dt = 0.03333;
e3 << 0,0,1;
l_star = 0.50;
J_p = m_p * l_star * l_star;

//E_dot_AD
v_p = l_star*omega;
F_AD = 0.5*c_d*Area*air_density*v_p*v_p;
E_dot_AD = F_AD * l_star*omega;

//Angular accleration calculate
uav2_omega_filt = lpf2_uav2_omega.filter(omega);
alpha = (uav2_omega_filt - uav2_last_omega)*30;
uav2_last_omega = uav2_omega_filt;
alpha_filt = lpf2_uav2_alpha.filter(alpha);

//Estimate leader's energy rate
E_dot_f = -m_p*l_star*omega*cos(theta)*uav2_acc_inertia(0)/2;
E_theta_hat = 0.5*m_p*l_star*l_star*omega*omega + m_p*g*l_star*(1-cos(theta));
E_theta_hat_dot = (E_theta_hat- E_theta_hat_last)/dt;
E_theta_hat_dot_filt = lpf2_E_hat_dot.filter(E_theta_hat_dot);
E_dot_f_filt = lpf2_E_f_dot.filter(E_dot_f);
E_dot_l = E_theta_hat_dot_filt - E_dot_f_filt + E_dot_AD;
E_theta_hat_last = E_theta_hat;

//Esimtae psi energy of payload
E_dot_f_psi = -m_p*omega_psi*cos(theta_psi)*uav2_acc_inertia(0)/2;
E_uav2_theta_hat_psi = 0.5*m_p*l_star*l_star*omega_psi*omega_psi + m_p*g*l_star*(1-cos(theta_psi));
E_psi_filt = lpf2_E_psi.filter(E_uav2_theta_hat_psi);
//E_theta_hat_dot_psi = (E_uav2_theta_hat_psi- E_theta_hat_last_psi)/dt;
//E_theta_hat_dot_filt_psi = lpf2_E_hat_dot_psi.filter(E_theta_hat_dot_psi);
//E_dot_f_filt_psi = lpf2_E_f_dot_psi.filter(E_dot_f_psi);
//E_dot_l_psi = E_theta_hat_dot_filt_psi - E_dot_f_filt_psi;
//E_theta_hat_last_psi = E_uav2_theta_hat_psi;
E_psi_meas = 0.5*m_p*l_star*l_star*omega_ground_psi*omega_ground_psi + m_p*g*l_star*(1-cos(groundtruth.x));
//Publish data

uav2_E_dot_hat_theta.x = E_dot_l;
uav2_E_dot_hat_theta.y = E_psi_meas;
uav2_E_dot_hat_theta.z = E_psi_filt;

//position and velocity error

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
err_yaw = vir.yaw - uav2_yaw;
if(err_yaw>pi)
err_yaw = err_yaw - 2*pi;
else if(err_yaw<-pi)
err_yaw = err_yaw + 2*pi;

//Geometri controller for position control

Eigen::Vector3d g_;
g_ << 0,0,-9.8;
Eigen::Vector3d a_des;
Eigen::Vector3d a_ref;
Eigen::Vector3d a_fb;
Eigen::Vector4d q_des;
Eigen::Vector4d cmdbodyrate_;
if(swing_up_control == false){
 a_fb <<  6*errx + 1.5*errvx + uav2_sumx, 6*erry + 1.5*errvy+ uav2_sumy, 10*errz + 3.33*errvz;
 a_ref << 0, 0, 0;
}
 //ROS_INFO("u: %.4f,%.4f,%.4f,%.4f",cmdbodyrate_(0),cmdbodyrate_(1),cmdbodyrate_(2),cmdbodyrate_(3));

//Follower's controller

if(swing_up_control == true){
  double omega_zero_theta,omega_zero_psi, omega_theta,omega_psi_n,phi_psi, delta_u, delta_l, a_dis,a_dis_psi, a_f_theta, ux_theta, ux_psi;
  omega_zero_theta = sqrt(g/l_star);
  omega_zero_psi = 2*omega_zero_theta;
  omega_theta = omega_zero_theta*(1-(fabs(theta))/pi);
  phi_theta = atan2(-omega/omega_zero_theta, theta);
  delta_u = 0.1;
  delta_l = -0.1;
  a_f_theta = 4;

  //For theta-control

  if(E_dot_l > delta_u){
/*Test for limited path
    if(host_mocap.pose.position.x < -0.1 && host_mocap.pose.position.x > -1){

      if(omega_theta*omega_theta*sin(phi_theta) > 0){
      a_f_theta = 40*(1-log(1-fabs(host_mocap.pose.position.x)/1));
      }else{
        a_f_theta = 0.4*(log(fabs(host_mocap.pose.position.x)/1));
      }
      a_f_theta = fabs(a_f_theta);

    }else if(host_mocap.pose.position.x < -1){
      if(omega_theta*omega_theta*sin(phi_theta) > 0){
        a_f_theta = 40*(log(fabs(host_mocap.pose.position.x)/1-1));
      }else{
        a_f_theta = 0.4*(log(fabs(host_mocap.pose.position.x)/1));
      }
      a_f_theta = fabs(a_f_theta);
    }
*/
    a_dis = a_f_theta;
  }else if(E_dot_l < delta_l){
/*Test for limited path
    if(host_mocap.pose.position.x < -0.1 && host_mocap.pose.position.x > -1){
      if(omega_theta*omega_theta*sin(phi_theta) < 0){
      a_f_theta = 40*(1-log(1-fabs(host_mocap.pose.position.x)/1));
      }else{
        a_f_theta = 0.4*(log(fabs(host_mocap.pose.position.x)/1));
      }
      a_f_theta = fabs(a_f_theta);

    }else if(host_mocap.pose.position.x < -1){
      if(omega_theta*omega_theta*sin(phi_theta) < 0){
        a_f_theta = 40*(log(fabs(host_mocap.pose.position.x)/1-1));
      }else{
        a_f_theta = 0.4*(log(fabs(host_mocap.pose.position.x)/1));
      }
      a_f_theta = fabs(a_f_theta);
    }
*/
    a_dis = -a_f_theta;
  }else{
    a_dis = 0;
  }
  /*Test for limited path
  if((a_dis - a_f_theta_filt) == 0){
    copysign_ = 0;
  }else
  {
    copysign_ = copysign(1,a_dis - a_f_theta_filt);
  }
  a_f_theta_filt = a_f_theta_filt + dt*(10*copysign_);
  */

  ux_theta = a_dis*omega_theta*omega_theta*sin(phi_theta);

  if(ux_theta > limit_contol_factor*0.5*ng){
    ux_theta = limit_contol_factor*0.5*ng;
  }
  if(ux_theta < -limit_contol_factor*0.5*ng){
    ux_theta = -limit_contol_factor*0.5*ng;
  }

//ROS_INFO("x = %f, ux_theta = %f, a_dis = %f", host_mocap.pose.position.x, ux_theta, a_dis);
  //For psi-control
  phi_psi = atan2(-omega_psi/omega_zero_psi, theta_psi);
  omega_psi_n = omega_zero_psi*(1-(fabs(theta_psi))/pi);
  delta_E_psi = 0 - E_psi_filt;
/*Test for limited path
  if(host_mocap.pose.position.x < -0.1 && host_mocap.pose.position.x > -1){
    ROS_INFO("=====x < -0.1 =====");
    if(omega_psi_n*omega_psi_n*sin(phi_psi) < 0){
    a_f_theta = 40*(1-log(1-fabs(host_mocap.pose.position.x)/1));
    }else{
      a_f_theta = 0.4*(log(fabs(host_mocap.pose.position.x)/1));
    }
    a_f_theta = fabs(a_f_theta);

  }else if(host_mocap.pose.position.x < -1){
    ROS_INFO("=====x < -1 =====");
    if(omega_psi_n*omega_psi_n*sin(phi_psi) < 0){
    a_f_theta = 40*(log(fabs(host_mocap.pose.position.x)/1-1));
    //a_f_theta = 40;
    }else{
      a_f_theta = 0.4*(log(fabs(host_mocap.pose.position.x)/1));
    //a_f_theta = 0.4;
    }
    a_f_theta = fabs(a_f_theta);
  }
*/
  a_f_theta = 4;

  if(fabs(delta_E_psi) > 0.5){//0.25
    a_dis_psi =a_f_theta*copysign(1,delta_E_psi);//Always negative
  }else{
    a_dis_psi = (a_f_theta/0.5)*delta_E_psi;
  }
  ux_psi = a_dis_psi*omega_psi_n*omega_psi_n*sin(phi_psi);//probelm in sin(phi_psi), the psi_omega is inaccurate.

  ux = ux_theta + ux_psi;
  //ux = 2*ux_theta + 0*ux_psi;//without psi control

  if(ux > limit_contol_factor*ng){
    ux =  limit_contol_factor*ng;
  }
  if(ux < - limit_contol_factor*ng){
    ux =  - limit_contol_factor*ng;
  }

  //Height controller
  //uz = -0*(0.5*m_p)*l_star*alpha_filt*sin(theta) + 0.5*(0.5*m_p)*l_star*uav2_omega_filt*uav2_omega_filt*cos(theta);
  //Admittance control
  double M = 2;
  uz_f = (uav2_force_est.z + sw_Kpz*errz + 0.8*sw_Kvz*errvz)/M;
  //uz_f = (3*errz + 0.3*errvz);without height control
  a_fb <<  0, 9*erry + 2.25*errvy+ uav2_sumy, 0;
  a_ref << ux, 0, uz_f;
  uav2_control_input.x = theta_psi;
  uav2_control_input.y = omega_psi;
  uav2_control_input.z = ux_psi;
  //ROS_INFO("phi_psi = %f", phi_psi);
}
//Geometric controller: Convert accleration command to body rate and thrust command
a_des << a_ref + a_fb - g_;
q_des = acc2quaternion(a_des,uav2_yaw);
cmdbodyrate_ = attcontroller(q_des,a_des,uav2_mavatt_);
pose->body_rate.x = cmdbodyrate_(0);
pose->body_rate.y = cmdbodyrate_(1);
pose->body_rate.z = cmdbodyrate_(2);
pose->thrust = cmdbodyrate_(3);

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
  double v_p;
  double F_AD;
  double Area = 1.5*0.05;
  g = -9.81;
  l_star = 0.50;
  v_p = l_star*x(1);
  F_AD = 0.5*1.2*v_p*v_p*0.82*Area;
  dt = 0.0333;
  //L_theta << 4.669, 2;//For force sensor
  L_theta << 4.669, 0.8;//4.669, 0.8(First:Too small diverge, second: too large (>1) diverge
  C << 1, 0;
  y_hat = (C.transpose()*x).value();
  f_x << x(1), -g/l_star*sin(x(0));//-F_AD/(m_p*l_star)
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
  ros::Subscriber uav1_state_sub = nh.subscribe<mavros_msgs::State>("/uav1/mavros/state", 2, uav1_state_cb);
  ros::ServiceClient uav1_arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/uav1/mavros/cmd/arming");
  ros::ServiceClient uav1_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/uav1/mavros/set_mode");
  ros::Subscriber uav1_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/uav1/mavros/local_position/pose", 10, uav1_host_pos);
  ros::Publisher uav1_attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/uav1/mavros/setpoint_raw/attitude", 2);
  ros::Subscriber uav1_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/uav1/mavros/local_position/velocity_local", 2, uav1_host_vel);
  //uav2
  ros::Subscriber uav2_state_sub = nh.subscribe<mavros_msgs::State>("/uav2/mavros/state", 2, uav2_state_cb);
  ros::ServiceClient uav2_arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/uav2/mavros/cmd/arming");
  ros::ServiceClient uav2_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/uav2/mavros/set_mode");
  ros::Subscriber uav2_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/uav2/mavros/local_position/pose", 10, uav2_host_pos);
  ros::Publisher uav2_attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/uav2/mavros/setpoint_raw/attitude", 2);
  ros::Subscriber uav2_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/uav2/mavros/local_position/velocity_local", 2, uav2_host_vel);
  //Force sub (force estimate and force sensor)
  ros::Subscriber uav1_force_sub = nh.subscribe<geometry_msgs::Point>("/uav1/force_nobias", 2, uav1_force_cb);
  ros::Subscriber uav2_force_sub = nh.subscribe<geometry_msgs::Point>("/uav2/force_nobias", 2, uav2_force_cb);
  ros::Subscriber uav1_wrench_sub = nh.subscribe<geometry_msgs::WrenchStamped>("/uav1_ft_sensor", 2, uav1_wrench_cb);
  ros::Subscriber uav2_wrench_sub = nh.subscribe<geometry_msgs::WrenchStamped>("/uav2_ft_sensor", 2, uav2_wrench_cb);
  //IMU sub
  ros::Subscriber uav2_imu_sub = nh.subscribe<sensor_msgs::Imu>("/uav2/mavros/imu/data",2,uav2_imu_cb);
  ros::Subscriber uav1_imu_sub = nh.subscribe<sensor_msgs::Imu>("/uav1/mavros/imu/data",2,uav1_imu_cb);
  //Ground truth sub
  ros::Subscriber theta_ground_sub = nh.subscribe<geometry_msgs::Point>("/theta_groundtruth", 2, ground_cb);
  //Gazebo link state
  ros::Subscriber link_sub = nh.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 3, link_cb);


  //Data pub
  ros::Publisher uav1_rope_angle_pub = nh.advertise<geometry_msgs::Point>("/uav1rope_angle", 2);
  ros::Publisher uav2_rope_angle_pub = nh.advertise<geometry_msgs::Point>("/uav2rope_angle2", 2);

  ros::Publisher uav2_theta_hat_pub = nh.advertise<geometry_msgs::Point>("/uav2_theta_hat", 2);
  ros::Publisher uav2_theta_hat_meas_pub = nh.advertise<geometry_msgs::Point>("/uav2_theta_hat_meas", 2);
  ros::Publisher uav2_psi_hat_pub = nh.advertise<geometry_msgs::Point>("/uav2_psi_hat", 2);
  ros::Publisher uav2_psi_hat_meas_pub = nh.advertise<geometry_msgs::Point>("/uav2_psi_hat_meas", 2);

  ros::Publisher uav2_E_dot_hat_theta_pub = nh.advertise<geometry_msgs::Point>("/uav2_E_dot_hat_theta", 2);
  ros::Publisher uav2_control_input_pub = nh.advertise<geometry_msgs::Point>("/uav2_control_input", 2);
  ros::Publisher uav1_E_dot_hat_theta_pub = nh.advertise<geometry_msgs::Point>("/uav1_E_dot_hat_theta", 2);

  ros::Publisher uav1_theta_pub = nh.advertise<geometry_msgs::Point>("/uav1_theta", 2);
  ros::Publisher uav1_theta_hat_1_pub = nh.advertise<geometry_msgs::Point>("/uav1_theta_hat_1", 2);
  ros::Publisher uav1_theta_hat_1_meas_pub = nh.advertise<geometry_msgs::Point>("/uav1_theta_hat_1_meas", 2);
  ros::Publisher uav1_pose_z_pub = nh.advertise<geometry_msgs::Point>("/uav1_pose", 2);
  ros::Publisher uav2_pose_z_pub = nh.advertise<geometry_msgs::Point>("/uav2_pose", 2);
  ros::Publisher uav1_control_input_pub = nh.advertise<geometry_msgs::Point>("uav1_control_input", 2);
  ros::Publisher payload_angular_v_pub = nh.advertise<geometry_msgs::Point>("/payload_angular_v", 2);
  //Apply force service
  ros::ServiceClient apply_force_client = nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");

  ros::Rate rate(30);

  while (ros::ok() && uav1_current_state.connected && uav2_current_state.connected) {
      ros::spinOnce();
      rate.sleep();
  }
 //Set initial desired position
  vir vir1;

  vir1.x = 0.0;
  vir1.y = 0.4;
  vir1.z = 1.0;
  vir1.yaw = 0;

  vir vir2;

  vir2.x = 0.0;
  vir2.y = -0.4;
  vir2.z = 1.0;
  vir2.yaw = 0;

  //Initialization of body rate and thrust command
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
 //Set mode to OFFBOARD and arming
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

//keyboard control

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
           vir1.yaw -= 0.03;
           break;
       case 68:    // key CCW(<-)
           vir1.yaw += 0.03;
           break;
       case 119:    // key foward
           vir1.x += 0.05;
           vir2.x += 0.05;
           break;
       case 49: // keyboard 1
           swing_up_control = true;
           break;
       case 50:
         start_observer = true;
         break;
       case 53:
         apply_wrench_flag = true;
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
   // Conver yaw to +pi ~ -pi
   if(vir1.yaw>pi)
   vir1.yaw = vir1.yaw - 2*pi;
   else if(vir1.yaw<-pi)
   vir1.yaw = vir1.yaw + 2*pi;

   if(vir2.yaw>pi)
   vir2.yaw = vir2.yaw - 2*pi;
   else if(vir2.yaw<-pi)
   vir2.yaw = vir2.yaw + 2*pi;


   //Apply wrench for payload
/*
   if(apply_wrench_flag){
     ros::Duration duration_(0.8);
     //
     drone_apply_force.request.body_name="payload::payload_rec";
     drone_apply_force.request.duration = duration_;
     //drone_apply_force.request.reference_point = ref_point;
     apply_wrench.force.x = 2;
     apply_wrench.force.y = 0;
     apply_wrench.force.z = 0;
     drone_apply_force.request.wrench = apply_wrench;
     apply_force_client.call(drone_apply_force);

     apply_wrench_flag = false;
   }
*/

   //Calculate rope angle

   double uav1_rope_theta,uav2_rope_theta, uav1_rope_theta_sensor, uav2_rope_theta_sensor;

   //By force estimate
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
   uav1_rope_angle.x = -(uav1_rope_theta+pi);
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

   uav2_rope_theta = -uav2_rope_theta;//Theta = 0 is at upright position.
   //For theta = 0 is downward
   //uav2_rope_theta = -(uav2_rope_theta + pi);




   //uav2_rope_angle.x = -(uav2_rope_theta + pi);

   //By force sensor
   if(uav1_wrench.wrench.force.x < 0 && uav1_wrench.wrench.force.z > 0){
     uav1_rope_theta_sensor = atan2(uav1_wrench.wrench.force.z,uav1_wrench.wrench.force.x);
     uav1_rope_theta_sensor = -2.5*3.1415926+uav1_rope_theta_sensor;
   }
   else{
   uav1_rope_theta_sensor = atan2(uav1_wrench.wrench.force.z,uav1_wrench.wrench.force.x) - 3.1415926/2;
}
   uav1_rope_angle.y = -(uav1_rope_theta_sensor+pi);


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
   //theta_psi estimate
   double psi_meas, psi_meas_meas;
   psi_meas_meas = ((-(uav2_rope_theta_sensor+pi)) - (-(uav2_theta_hat_1(0)+pi)));
   psi_meas = uav2_rope_theta - uav2_theta_hat_1(0);
   psi_meas = psi_meas;
   uav2_theta_hat_psi(0) = -uav2_theta_hat_psi(0);//This conversion is due to the direction of psi estimate
   uav2_theta_hat_psi_meas(0) = -uav2_theta_hat_psi_meas(0);
   uav2_theta_hat_psi = non_linear_observer(uav2_theta_hat_psi, psi_meas);
   uav2_theta_hat_psi_meas = non_linear_observer(uav2_theta_hat_psi_meas, psi_meas_meas);

    //theta_theta estimate
   uav2_theta_hat_1 = non_linear_observer(uav2_theta_hat_1,uav2_rope_theta);
   uav2_theta_hat_1_meas = non_linear_observer(uav2_theta_hat_1_meas,uav2_rope_theta_sensor);


   //uav1

   //theta_theta estimate
   uav1_theta_hat_1 = non_linear_observer(uav1_theta_hat_1,uav1_rope_theta);
   uav1_theta_hat_1_meas = non_linear_observer(uav1_theta_hat_1_meas,uav1_rope_theta_sensor);


   uav1_theta_hat_1_p.x = -(uav1_theta_hat_1(0)+pi);
   uav1_theta_hat_1_p.y = -uav1_theta_hat_1(1);
   uav1_theta_hat_1_meas_p.x = -(uav1_theta_hat_1_meas(0)+pi);
   uav1_theta_hat_1_meas_p.y = -uav1_theta_hat_1_meas(1);

   //publish data

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
   payload_angular_v_pub.publish(payload_angular_v);
   //Controller
   //For leader's controller, theta = 0 is at upright positon.
   leader_controller(vir1,uav1_host_mocap, uav1_host_mocapvel,&uav1_pose, uav1_theta_hat_1(0), 0, uav1_theta_hat_1(1),  0);
   uav1_attitude_pub.publish(uav1_pose);
   //For follower's controller, theta = 0 is at downwared position. Acoordingly, it requires the conversion from theta = 0 at upright positon to theta = 0 at downward postion.
   follower_controller(vir2,uav2_host_mocap, uav2_host_mocapvel,&uav2_pose,-(uav2_theta_hat_1(0) + pi), uav2_theta_hat_psi(0),-uav2_theta_hat_1(1),  uav2_theta_hat_psi(1));
   uav2_attitude_pub.publish(uav2_pose);
   ros::spinOnce();
   rate.sleep();
 }
return 0;
}



