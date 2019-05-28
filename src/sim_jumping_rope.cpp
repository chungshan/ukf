/**
 * @file offb_main.cpp
 * @author Julian Oes <julian@oes.ch>
 * @license BSD 3-clause
 *
 * @brief ROS node to do offboard control of PX4 through MAVROS.
 *
 * Initial code taken from http://dev.px4.io/ros-mavros-offboard.html
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <tf/tf.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/WrenchStamped.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Dense>
#include <thread>
#include <functional>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/LinkStates.h>
#define gravity 9.806
#define pi 3.1415926
#include <unsupported/Eigen/MatrixFunctions>


bool init = false;
bool initial = false;
bool jumping_rope_control = false;
bool initial_acc = false;
bool apply_wrench_flag = false;
bool balance_control = false;
bool balance_control_theta = false;

int flag=0;
geometry_msgs::Point balance_control_flag;
float KPx=5, KPy=5, KPz=0.5;
float KVx=1.3, KVy=1.3, KVz=0.2;
float KIx=0.33, KIy=0.33, KIz=0.05;
float KPyaw = 1;
double roll, pitch, yaw;
float r = 0.5;
float T = 2*pi;
using namespace std;
geometry_msgs::Point rope_control_input, rope_energy, trigger_control, sim_angle, sim_force, sim_pos, ref_point, sim_apply_force_, sim_acc, sim_tan_norm,sim_drone_att, sim_control_input, sim_drone_vel, thrust_test, pos_des, break_joint;
gazebo_msgs::ApplyBodyWrench drone_apply_force;
geometry_msgs::Wrench apply_wrench;

std::vector<double> quadratic_root(double a, double b, double c){
    std::vector<double> root;
    double x =0;
    if((b*b - 4*a*c)>0){

        double sqrt_ =  sqrt(b*b - 4.00000*a*c);
        root.push_back(  (    -1.0000000 * b+     sqrt_) /(2.0000000*a));
        root.push_back(  (    -1.0000000 * b-     sqrt_) /(2.0000000*a));
    }

    return  root;
}

typedef struct
{
    float roll;
    float x;
    float y;
    float z;
}vir;
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}
geometry_msgs::PoseStamped host_mocap;
geometry_msgs::PoseStamped initial_pose;
Eigen::Matrix3d R_IB;
Eigen::Vector4d mavatt_;
void host_pos(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
        host_mocap = *msg;
        if(init==false)
        {
        initial_pose = host_mocap;
        init = true;
        }
        tf::Quaternion Q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);

        tf::Matrix3x3(Q).getRPY(roll,pitch,yaw);


        //ROS_INFO("angle: %.2f,%.2f, %.2f",roll,pitch,yaw);
        sim_drone_att.x = roll;
        //sim_drone_att.y = pitch;
        sim_drone_att.z = yaw;

//        tf::quaternionTFToMsg(Q,desired.pose.orientation);
//        ROS_INFO("error: %.4f, %.4f, %.4f, %.4f ",desired.pose.orientation.x,desired.pose.orientation.y,desired.pose.orientation.z,desired.pose.orientation.w);

        double x = msg->pose.orientation.x;
        double y = msg->pose.orientation.y;
        double z = msg->pose.orientation.z;
        double w = msg->pose.orientation.w;
        mavatt_ << w,x,y,z;
        tf::Matrix3x3 R_IB_q;

        tf::Quaternion drone_q(x,y,z,w);
        R_IB_q.setRotation(drone_q);

        R_IB << w*w+x*x-y*y-z*z  , 2*x*y-2*w*z ,            2*x*z+2*w*y,
            2*x*y +2*w*z           , w*w-x*x+y*y-z*z    ,2*y*z-2*w*x,
            2*x*z -2*w*y          , 2*y*z+2*w*x        ,w*w-x*x-y*y+z*z;


}
geometry_msgs::TwistStamped host_mocapvel;
void host_vel(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
        host_mocapvel = *msg;

}
gazebo_msgs::LinkStates link_states;
geometry_msgs::PoseStamped iris_base, payload_pose;
geometry_msgs::Point payload_link_omega;
void gazebo_state_cb(const gazebo_msgs::LinkStates::ConstPtr& msg){
link_states = *msg;
if(link_states.name.size()>0){
    for(unsigned int i=0; i<link_states.name.size();i++){
        if(link_states.name[i].compare("iris_rplidar::iris::base_link")==0){
          iris_base.pose.position = link_states.pose[i].position;

        }
        if(link_states.name[i].compare("iris_rplidar::payload::payload_link")==0){
            payload_pose.pose.position = link_states.pose[i].position;
            payload_link_omega.y = link_states.twist[i].angular.y;

        }
    }
}
}

sensor_msgs::Imu imu_data;
void imu_cb(const sensor_msgs::Imu::ConstPtr& msg){
  imu_data = *msg;
  sim_drone_att.y = imu_data.angular_velocity.y;
}
geometry_msgs::Point rope_angle;
void rope_cb(const geometry_msgs::Point::ConstPtr& msg)
{
  rope_angle = *msg;
}
geometry_msgs::WrenchStamped wrench_;
void wrench_cb(const geometry_msgs::WrenchStamped::ConstPtr& msg){
  wrench_ = *msg;
}
geometry_msgs::Point force_est;
void force_est_cb(const geometry_msgs::Point::ConstPtr& msg){
  force_est = *msg;
}
sensor_msgs::Imu payload_imu;
void payload_imu_cb(const sensor_msgs::Imu::ConstPtr& msg){
  payload_imu = *msg;
}
geometry_msgs::Point force_on_drone;
void force_on_drone_cb(const geometry_msgs::Point::ConstPtr& msg){
  force_on_drone = *msg;
}

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
  rotmat = quat2RotMatrix(mavatt_);
  zb = rotmat.col(2);
  ratecmd(3) = std::max(0.0, std::min(1.0, norm_thrust_const_ * ref_acc.dot(zb))); //Calculate thrust
  return ratecmd;
}

float sumx,sumy,sumz;
geometry_msgs::Point last_pos;
geometry_msgs::Point velocity;
double last_omega;
double last_force_z;
void follow(vir& vir, geometry_msgs::PoseStamped& host_mocap, mavros_msgs::AttitudeTarget* pose,double theta,double theta_2,double omega,double omega_2)
{
float errx, erry, errz, err_roll;
float errvx , errvy ,errvz ;
float ux, uy, uz, uroll, upitch, uyaw, uz_test;
float ux_f, uy_f;
tf::Quaternion Q;
double E, E_des;
double J_rope, m_rope, r_g;
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
e3 << 0,0,1;
alpha = (payload_imu.angular_velocity.y - last_omega)*50;
last_omega = payload_imu.angular_velocity.y;
m_rope = 0.6;
r_g = 0.5;
J_rope = m_rope * r_g * r_g;
omega_des = g/r_g;
E = 0.5*J_rope*omega*omega + m_rope*g*r_g*(cos(theta) - 1);
//E_des = 0.5*J_rope*omega_des;
E_des = 0;
rope_energy.x = E;
rope_energy.y = E_des;
errx = vir.x - host_mocap.pose.position.x;
erry = vir.y - host_mocap.pose.position.y;
errz = vir.z - host_mocap.pose.position.z;
velocity.x = (host_mocap.pose.position.x-last_pos.x)/0.02;
velocity.y = (host_mocap.pose.position.y-last_pos.y)/0.02;
velocity.z = (host_mocap.pose.position.z-last_pos.z)/0.02;
errvx = 0 - velocity.x;
errvy = 0 - velocity.y;
errvz = 0 - velocity.z;
/*
if(jumping_rope_control==false){
sumx += KIx*errx*0.02;
sumy += KIy*erry*0.02;
sumz += KIz*errz*0.02;
}
*/

sumx += KIx*errx*0.02;
sumy += KIy*erry*0.02;
sumz += KIz*errz*0.02;

/*
sumx += KIx*errx*0.02;
sumy += KIy*erry*0.02;
sumz += KIz*errz*0.02;
*/
last_pos = host_mocap.pose.position;
if(sumz>0.2){
  sumz = 0.2;
}
else if(sumz<-0.2){
  sumz = -0.2;
}
if(sumx>0.6){
  sumx = 0.6;
}
else if(sumx<-0.6){
  sumx = -0.6;
}
if(sumy>0.6){
  sumy = 0.6;
}
else if(sumy<-0.6){
  sumy = -0.6;
}
err_roll = vir.roll - yaw;
if(err_roll>pi)
err_roll = err_roll - 2*pi;
else if(err_roll<-pi)
err_roll = err_roll + 2*pi;

ROS_INFO("err: x = %.3f, y = %.3f, z = %.3f, yaw = %.3f",errx,erry,errz,err_roll);
//ROS_INFO("err: %.3f,%.3f,%.3f",errvx,errvy,errvz);

if(jumping_rope_control == false){
ux = KPx*errx + KVx*errvx + sumx;
uy = KPy*erry + KVy*errvy + sumy;
uz = KPz*errz + KVz*errvz + sumz;
ux_f = cos(-yaw) * ux - sin(-yaw) * (uy);
uy_f = sin(-yaw) * ux + cos(-yaw) * (uy);
uz_f = gravity;

uroll = -atan2(uy_f,uz_f);
upitch = atan2(ux_f,uz_f);
pose->thrust = uz+0.4;

}
if(initial_acc == true){
  uy = KPy*erry + KVy*errvy + sumy;
  uz = KPz*errz + KVz*errvz + sumz;
  ux = 0.7*0.4*9.81;
  ux_f = cos(-yaw) * ux - sin(-yaw) * (uy);
  uy_f = sin(-yaw) * ux + cos(-yaw) * (uy);
  uz_f = gravity;
  uroll = -atan2(uy_f,uz_f);
  upitch = atan2(ux_f,uz_f);
  pose->thrust = uz+0.4;

}
/*
if(jumping_rope_control == true){
  //jumping rope controller
  ux = KPx*errx + KVx*errvx;
  uy = k_rope*(E-E_des)*omega*cos(theta);
  uz = KPz*errz + KVz*errvz;
  if(uy > ng){
    uy = ng;
  }
  if(uy < -ng){
    uy = -ng;
  }
  ux_f = cos(-yaw) * ux - sin(-yaw - 3.1415926) * (uy);
  uy_f = sin(-yaw) * ux + cos(-yaw - 3.1415926) * (uy);
  rope_control_input.y = uy_f;
}
*/

if(jumping_rope_control == true){
  sim_angle.x = 0;
  if(!balance_control_theta){
  ux = (E-E_des)*omega*cos(theta);
  ROS_INFO("ux = %f", ux);
  uz_test = KPz*errz + KVz*errvz + sumz;
  //uz = KPz*errz + KVz*errvz - 0.1*0.5*alpha*sin(theta) - 0.1*0.5*omega*omega*cos(theta);
  //uz = 2*KPz*errz + 2*KVz*errvz + 0.2 - 0.1*0.5*alpha*sin(theta) - 0.1*0.5*omega*omega*cos(theta);
  uz = 2*KPz*errz + 2*KVz*errvz + 0.2 - 0.1*mp*L*alpha*sin(theta) - 0.1*mp*L*omega*omega*cos(theta);
  sim_tan_norm.x =  - 0.1*0.5*alpha*sin(theta);
  sim_tan_norm.y =  - 0.1*0.5*omega*omega*cos(theta);
  //- 0.1*0.5*alpha*sin(theta) - 0.1*0.5*omega*omega*cos(theta)
  uy = KPy*erry + KVy*errvy;
  //- 0.1*0.5*omega*omega*cos(theta)
  if(ux > 0){
    ux =  0.3*ng;
  }
  if(ux < 0){
    ux =  - 0.3*ng;
  }
/*
  if(theta < -4.3 && theta > -1.7*3.1415926){
    ux = 1.2*ux;
  }
  if(theta <-0.4*3.1415926 && theta >-1.2*3.1415926 ){
    ux = 1.2*ux;
  }
*/
  last_force_z = wrench_.wrench.force.z;
  ux_f = cos(-yaw) * ux - sin(-yaw) * (uy);
  uy_f = sin(-yaw) * ux + cos(-yaw) * (uy);
  uz_f = gravity;
  uroll = -atan2(uy_f,uz_f);
  upitch = atan2(ux_f,uz_f);
  if(uz > 0.6){
    uz = 0.6;
  }
  if(uz < 0.0){
    uz = 0.0;
  }
  pose->thrust = uz+0.4;
  rope_control_input.y = ux_f;
}
  if (balance_control){

    if(!balance_control_theta){
      if(theta < -1.90*pi || theta > -0.1*pi){
        balance_control_theta = true;
      }
    }
    if(balance_control_theta){
      if(theta > -1.8*pi && theta < -0.2*pi){
        balance_control_theta = false;
        balance_control_flag.x = 0;
      }
    }
    if(balance_control_theta){
    ROS_INFO("""start balance!!!!!!!!!!!!!!!!!");
    balance_control_flag.x = 1;
    Eigen::VectorXd x_vector(6);
    Eigen::VectorXd y_vector(6);
    Eigen::VectorXd K_x(6);
    Eigen::VectorXd K_y(6);
    if(theta < -pi && theta > -2*pi){
      theta = (theta + 2*pi);
    }
    sim_angle.x = theta_2;
    x_vector << host_mocap.pose.position.x, host_mocap.pose.position.z, theta_2, host_mocapvel.twist.linear.x, host_mocapvel.twist.linear.z, omega;
    //y_vector << host_mocap.pose.position.y, host_mocap.pose.position.z, theta_2, host_mocapvel.twist.linear.y, host_mocapvel.twist.linear.z, omega_2;
    K_x << -0.0, 0, -146.8735, -6.8598, 0, -18.2266;
    ux = -K_x.transpose() * x_vector;
    //uy = -K_x.transpose() * y_vector;
    if(ux > 2*ng){
      ux = 2*ng;
    }
    if(ux < -2*ng){
      ux = -2*ng;
    }
    ROS_INFO("ux_balance = %f", ux);
    //rope_control_input.x = ux;
    //ux = KPx*errx + KVx*errvx + sumx;
    uy = KPy*erry + KVy*errvy + sumy;
    //uz = 2*KPz*errz + 2*KVz*errvz + 0.2  - 0.1*0.8*0.5*alpha*sin(theta) - 0.1*0.8*0.5*omega*omega*cos(theta);
    uz_f = 2*KPz*errz + 2*KVz*errvz + (mp+mq)*9.81;
    root_ = quadratic_root(20,15,-uz_f);
    if(root_.size()>0){
        for (unsigned int i=0;i<root_.size();i++) {
            std::cout << root_[i]   <<std::endl;
            if( root_[i] > 0){
                thrust_test.z = root_[i];
            }
        }

   }
    //sim_pos.z =  thrust_test.z;
    ux_f = cos(-yaw) * ux - sin(-yaw) * (uy);
    uy_f = sin(-yaw) * ux + cos(-yaw) * (uy);
    uroll = -atan2(uy_f,uz_f);
    upitch = atan2(ux_f,uz_f);
    rope_control_input.x = upitch;
    //uz_f = uz_f / 30;
    if(thrust_test.z > 0.9){
      thrust_test.z = 0.9;
    }
    if(thrust_test.z < 0.72){
      thrust_test.z = 0.72;
    }
    pose->thrust = thrust_test.z;
}
    balance_control = true;
  }
}

/*
if(jumping_rope_control == true){
  ux = 0.5*9.81*tan(theta) + k_rope*(E-E_des)*omega*cos(theta);
  uz = KPz*errz + KVz*errvz - 0.1*0.5*alpha*sin(theta) - 0.1*0.5*omega*omega*cos(theta);
  uy = KPy*erry + KVy*errvy;

  if(ux > ng){
    ux =  ng;
  }
  if(ux < - ng){
    ux =  - ng;
  }
  rope_control_input.x = ux;

  ux_f = cos(-yaw) * ux - sin(-yaw) * (uy);
  uy_f = sin(-yaw) * ux + cos(-yaw) * (uy);
  rope_control_input.y = uy_f;
}
*/


uyaw = KPyaw*err_roll;

Q.setRPY(uroll, upitch, uyaw);
tf::quaternionTFToMsg(Q,pose->orientation);

/*
Eigen::Vector3d b1,b2,b3;
total_thrust << 0.5*1.5*ux, 0.5*1.5*uy, 0.5*uz;
    b3 = total_thrust/total_thrust.norm();

    b1 << cos(0) , sin(0),0;
    b2 = b3.cross(b1) / (b3.cross(b1)).norm();
    Eigen::Matrix3d att;
    att<< b1 ,b2 ,b3;
    tf::Matrix3x3 q(
                    att(0,0),att(0,1),att(0,2),
                    att(1,0),att(1,1),att(1,2),
                    att(2,0),att(2,1),att(2,2)
                    );
*/


//pose->thrust = total_thrust.dot(R_IB*e3);

ROS_INFO("u: %.4f,%.4f,%.4f,%.4f",uroll,upitch,uyaw,pose->thrust);
sim_control_input.z = pose->thrust;
sim_control_input.y = uz_test + 0.4;
}

void follow_omega(vir& vir, geometry_msgs::PoseStamped& host_mocap, mavros_msgs::AttitudeTarget* pose,double theta,double theta_2,double omega,double omega_2)
{
float errx, erry, errz, err_roll;
float errvx , errvy ,errvz ;
float ux, uy, uz, uroll, upitch, uyaw, uz_test;
float ux_f, uy_f;
tf::Quaternion Q;
double E, E_des;
double J_rope, m_rope, r_g;
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
e3 << 0,0,1;
alpha = (payload_imu.angular_velocity.y - last_omega)*50;
last_omega = payload_imu.angular_velocity.y;
m_rope = 0.6;
r_g = 0.5;
J_rope = m_rope * r_g * r_g;
omega_des = g/r_g;
E = 0.5*J_rope*omega*omega + m_rope*g*r_g*(cos(theta) - 1);
//E_des = 0.5*J_rope*omega_des;
E_des = 0;
rope_energy.x = E;
rope_energy.y = E_des;
/*
errx = vir.x - host_mocap.pose.position.x;
erry = vir.y - host_mocap.pose.position.y;
errz = vir.z - host_mocap.pose.position.z;
velocity.x = (host_mocap.pose.position.x-last_pos.x)/0.02;
velocity.y = (host_mocap.pose.position.y-last_pos.y)/0.02;
velocity.z = (host_mocap.pose.position.z-last_pos.z)/0.02;
errvx = 0 - velocity.x;
errvy = 0 - velocity.y;
errvz = 0 - velocity.z;
*/
errx = -(host_mocap.pose.position.x - vir.x);
erry = -(host_mocap.pose.position.y - vir.y);
errz = -(host_mocap.pose.position.z - vir.z);

errvx = -(host_mocapvel.twist.linear.x - 0);
errvy = -(host_mocapvel.twist.linear.y - 0);
errvz = -(host_mocapvel.twist.linear.z - 0);

/*
if(jumping_rope_control==false){
sumx += KIx*errx*0.02;
sumy += KIy*erry*0.02;
sumz += KIz*errz*0.02;
}
*/

sumx += KIx*errx*0.02;
sumy += KIy*erry*0.02;
sumz += KIz*errz*0.02;

/*
sumx += KIx*errx*0.02;
sumy += KIy*erry*0.02;
sumz += KIz*errz*0.02;
*/
last_pos = host_mocap.pose.position;
if(sumz>0.2){
  sumz = 0.2;
}
else if(sumz<-0.2){
  sumz = -0.2;
}
if(sumx>0.6){
  sumx = 0.6;
}
else if(sumx<-0.6){
  sumx = -0.6;
}
if(sumy>0.6){
  sumy = 0.6;
}
else if(sumy<-0.6){
  sumy = -0.6;
}
err_roll = vir.roll - yaw;
if(err_roll>pi)
err_roll = err_roll - 2*pi;
else if(err_roll<-pi)
err_roll = err_roll + 2*pi;

ROS_INFO("err: x = %.3f, y = %.3f, z = %.3f, yaw = %.3f",errx,erry,errz,err_roll);
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
 a_fb <<  6*errx + 1.5*errvx + sumx, 6*erry + 1.5*errvy+ sumy, 10*errz + 3.33*errvz;
 a_ref << 0, 0, 0;
 a_des << a_ref + a_fb - g_;
 q_des = acc2quaternion(a_des,yaw);
 cmdbodyrate_ = attcontroller(q_des,a_des,mavatt_);
 pose->body_rate.x = cmdbodyrate_(0);
 pose->body_rate.y = cmdbodyrate_(1);
 pose->body_rate.z = cmdbodyrate_(2);
 pose->thrust = cmdbodyrate_(3);

 ROS_INFO("u: %.4f,%.4f,%.4f,%.4f",cmdbodyrate_(0),cmdbodyrate_(1),cmdbodyrate_(2),cmdbodyrate_(3));


}
if(initial_acc == true){

  a_fb <<  0, 6*erry + 1.5*errvy+ sumy, 10*errz + 3.33*errvz;
  a_ref << 0.4*9.81, 0, 0;
  a_des << a_ref + a_fb - g_; //control input f
  q_des = acc2quaternion(a_des,yaw);
  cmdbodyrate_ = attcontroller(q_des,a_des,mavatt_);
  pose->body_rate.x = cmdbodyrate_(0);
  pose->body_rate.y = cmdbodyrate_(1);
  pose->body_rate.z = cmdbodyrate_(2);
  pose->thrust = cmdbodyrate_(3);


}
/*
if(jumping_rope_control == true){
  //jumping rope controller
  ux = KPx*errx + KVx*errvx;
  uy = k_rope*(E-E_des)*omega*cos(theta);
  uz = KPz*errz + KVz*errvz;
  if(uy > ng){
    uy = ng;
  }
  if(uy < -ng){
    uy = -ng;
  }
  ux_f = cos(-yaw) * ux - sin(-yaw - 3.1415926) * (uy);
  uy_f = sin(-yaw) * ux + cos(-yaw - 3.1415926) * (uy);
  rope_control_input.y = uy_f;
}
*/

if(jumping_rope_control == true){
  sim_angle.x = 0;
  if(!balance_control_theta){
  ux = (E-E_des)*omega*cos(theta);

  if(ux > 1.25*ng){
    ux =  1.25*ng;
  }
  if(ux < - 1.25*ng){
    ux =  - 1.25*ng;
  }

  a_fb <<  0, 9*erry + 2.25*errvy+ sumy, 10*errz + 3.33*errvz;
  a_ref << ux, 0, 0;
  a_des << a_ref + a_fb - g_;
  q_des = acc2quaternion(a_des,yaw);
  cmdbodyrate_ = attcontroller(q_des,a_des,mavatt_);
  pose->body_rate.x = cmdbodyrate_(0);
  pose->body_rate.y = cmdbodyrate_(1);
  pose->body_rate.z = cmdbodyrate_(2);
  pose->thrust = cmdbodyrate_(3);


}
  if (balance_control){

    if(!balance_control_theta){
      if(theta < -1.95*pi || theta > -0.05*pi){
        balance_control_theta = true;
        vir.x = host_mocap.pose.position.x;
      }
    }
    if(balance_control_theta){
      if(theta > -1.70*pi && theta < -0.30*pi){
        balance_control_theta = false;
        balance_control_flag.x = 0;
      }
    }

    //for rope_theta_pose
    /*
    if(!balance_control_theta){
      if(theta_2 < 0.10*pi && theta_2 > -0.10*pi){
        balance_control_theta = true;
        vir.x = host_mocap.pose.position.x;
      }
    }
    if(balance_control_theta){
      if(theta_2 > 0.1*pi || theta_2 < -0.1*pi){
        balance_control_theta = false;
        balance_control_flag.x = 0;
      }
    }
*/
    if(balance_control_theta){
    ROS_INFO("""start balance!!!!!!!!!!!!!!!!!");
    balance_control_flag.x = 1;
    Eigen::VectorXd x_vector(6);
    Eigen::VectorXd y_vector(6);
    Eigen::VectorXd K_x(6);
    Eigen::VectorXd K_y(6);
    if(theta < -pi && theta > -2*pi){
      theta = (theta + 2*pi);
    }
    sim_angle.x = theta;
    x_vector << host_mocap.pose.position.x, host_mocap.pose.position.z, theta, host_mocapvel.twist.linear.x, host_mocapvel.twist.linear.z, omega;
    //y_vector << host_mocap.pose.position.y, host_mocap.pose.position.z, theta_2, host_mocapvel.twist.linear.y, host_mocapvel.twist.linear.z, omega_2;
    //K_x << -0.0, 0, -146.8735, -6.8598, 0, -18.2266;
    K_x << -0.0, 0, -29.51006, -0.70711, 0, -4.92088;
    ux = -K_x.transpose() * x_vector;
    //uy = -K_x.transpose() * y_vector;
    if(ux > 0.9*ng){
      ux = 0.9*ng;
    }
    if(ux < -0.9*ng){
      ux = -0.9*ng;
    }
    //pos_des.x = vir.x;
    //4.5*(vir.x - host_mocap.pose.position.x) + 1.1*(0-host_mocapvel.twist.linear.x)
    //3*erry + 0.75*errvy+ sumy
    a_fb <<  0,3*erry + 0.75*errvy+ sumy, 10*errz + 3.33*errvz;
    a_ref << ux, 0, 0;
    a_des << a_ref + a_fb - g_;
    q_des = acc2quaternion(a_des,yaw);
    cmdbodyrate_ = attcontroller(q_des,a_des,mavatt_);
    pose->body_rate.x = cmdbodyrate_(0);
    pose->body_rate.y = cmdbodyrate_(1);
    pose->body_rate.z = cmdbodyrate_(2);
    pose->thrust = cmdbodyrate_(3);
    rope_control_input.x = cmdbodyrate_(1);

}
    balance_control = true;
  }

}

/*
if(jumping_rope_control == true){
  ux = 0.5*9.81*tan(theta) + k_rope*(E-E_des)*omega*cos(theta);
  uz = KPz*errz + KVz*errvz - 0.1*0.5*alpha*sin(theta) - 0.1*0.5*omega*omega*cos(theta);
  uy = KPy*erry + KVy*errvy;

  if(ux > ng){
    ux =  ng;
  }
  if(ux < - ng){
    ux =  - ng;
  }
  rope_control_input.x = ux;

  ux_f = cos(-yaw) * ux - sin(-yaw) * (uy);
  uy_f = sin(-yaw) * ux + cos(-yaw) * (uy);
  rope_control_input.y = uy_f;
}
*/


uyaw = KPyaw*err_roll;



/*
Eigen::Vector3d b1,b2,b3;
total_thrust << 0.5*1.5*ux, 0.5*1.5*uy, 0.5*uz;
    b3 = total_thrust/total_thrust.norm();

    b1 << cos(0) , sin(0),0;
    b2 = b3.cross(b1) / (b3.cross(b1)).norm();
    Eigen::Matrix3d att;
    att<< b1 ,b2 ,b3;
    tf::Matrix3x3 q(
                    att(0,0),att(0,1),att(0,2),
                    att(1,0),att(1,1),att(1,2),
                    att(2,0),att(2,1),att(2,2)
                    );
*/


//pose->thrust = total_thrust.dot(R_IB*e3);
rope_control_input.x = cmdbodyrate_(1);

sim_control_input.z = pose->thrust;
sim_control_input.y = uz_test + 0.4;
}
int i_count;
std::vector<double> control_command;
void follow_omega_delay(vir& vir, geometry_msgs::PoseStamped& host_mocap, mavros_msgs::AttitudeTarget* pose,double theta,double theta_2,double omega,double omega_2)
{
float errx, erry, errz, err_roll;
float errvx , errvy ,errvz ;
float ux, uy, uz, uroll, upitch, uyaw, uz_test;
float ux_f, uy_f;
tf::Quaternion Q;
double E, E_des;
double J_rope, m_rope, r_g;
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
Eigen::Matrix4d A_;
Eigen::Vector4d B_;
Eigen::Vector4d control_gain_K;
double tau_;
double r_til;
double delta_t;
double control_input_delay;
double theta_des;
e3 << 0,0,1;
alpha = (payload_imu.angular_velocity.y - last_omega)*50;
last_omega = payload_imu.angular_velocity.y;
m_rope = 0.6;
r_g = 0.5;
J_rope = m_rope * r_g * r_g;
omega_des = g/r_g;
theta_des = 45/57.29577951;
E = 0.5*J_rope*omega*omega + m_rope*g*r_g*(cos(theta) - 1);
//E_des = 0.5*J_rope*omega_des;
E_des = m_rope*g*r_g*(cos(theta_des) - 1)+ 0.5*J_rope*(1.5*omega)*(1.5*omega);//for compress swinging motion
//E_des = 0;

rope_energy.x = E;
rope_energy.y = E_des;
//FSA
control_gain_K << 0, -29.51006*2, -0.70711*2, -4.92088*2;
tau_ = 0.10;//control input delay
delta_t = 0.033;//sampling time
r_til = 3;// delay count: tau_ / delta_t
control_input_delay = tau_ * 30;
A_ << 0 ,0 ,1 ,0,
      0 , 0, 0, 1,
      0, -(mp/mq)*g, 0, 0,
      0, (mp+mq)*g/(mq*L), 0, 0;
B_ << 0, 0, 1/mq, -1/(mq*L);
/*
errx = vir.x - host_mocap.pose.position.x;
erry = vir.y - host_mocap.pose.position.y;
errz = vir.z - host_mocap.pose.position.z;
velocity.x = (host_mocap.pose.position.x-last_pos.x)/0.02;
velocity.y = (host_mocap.pose.position.y-last_pos.y)/0.02;
velocity.z = (host_mocap.pose.position.z-last_pos.z)/0.02;
errvx = 0 - velocity.x;
errvy = 0 - velocity.y;
errvz = 0 - velocity.z;
*/
errx = -(host_mocap.pose.position.x - vir.x);
erry = -(host_mocap.pose.position.y - vir.y);
errz = -(host_mocap.pose.position.z - vir.z);

errvx = -(host_mocapvel.twist.linear.x - 0);
errvy = -(host_mocapvel.twist.linear.y - 0);
errvz = -(host_mocapvel.twist.linear.z - 0);

/*
if(jumping_rope_control==false){
sumx += KIx*errx*0.02;
sumy += KIy*erry*0.02;
sumz += KIz*errz*0.02;
}
*/

sumx += KIx*errx*0.02;
sumy += KIy*erry*0.02;
sumz += KIz*errz*0.02;

/*
sumx += KIx*errx*0.02;
sumy += KIy*erry*0.02;
sumz += KIz*errz*0.02;
*/
last_pos = host_mocap.pose.position;
if(sumz>0.2){
  sumz = 0.2;
}
else if(sumz<-0.2){
  sumz = -0.2;
}
if(sumx>0.6){
  sumx = 0.6;
}
else if(sumx<-0.6){
  sumx = -0.6;
}
if(sumy>0.6){
  sumy = 0.6;
}
else if(sumy<-0.6){
  sumy = -0.6;
}
err_roll = vir.roll - yaw;
if(err_roll>pi)
err_roll = err_roll - 2*pi;
else if(err_roll<-pi)
err_roll = err_roll + 2*pi;

ROS_INFO("err: x = %.3f, y = %.3f, z = %.3f, yaw = %.3f",errx,erry,errz,err_roll);
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
 a_fb <<  6*errx + 1.5*errvx + sumx, 6*erry + 1.5*errvy+ sumy, 10*errz + 3.33*errvz;
 a_ref << 0, 0, 0;
 a_des << a_ref + a_fb - g_;
 q_des = acc2quaternion(a_des,yaw);
 cmdbodyrate_ = attcontroller(q_des,a_des,mavatt_);
 pose->body_rate.x = cmdbodyrate_(0);
 pose->body_rate.y = cmdbodyrate_(1);
 pose->body_rate.z = cmdbodyrate_(2);
 pose->thrust = cmdbodyrate_(3);

 ROS_INFO("u: %.4f,%.4f,%.4f,%.4f",cmdbodyrate_(0),cmdbodyrate_(1),cmdbodyrate_(2),cmdbodyrate_(3));


}
if(initial_acc == true){

  a_fb <<  0, 6*erry + 1.5*errvy+ sumy, 10*errz + 3.33*errvz;
  a_ref << 0.4*9.81, 0, 0;
  a_des << a_ref + a_fb - g_; //control input f
  q_des = acc2quaternion(a_des,yaw);
  cmdbodyrate_ = attcontroller(q_des,a_des,mavatt_);
  pose->body_rate.x = cmdbodyrate_(0);
  pose->body_rate.y = cmdbodyrate_(1);
  pose->body_rate.z = cmdbodyrate_(2);
  pose->thrust = cmdbodyrate_(3);


}
/*
if(jumping_rope_control == true){
  //jumping rope controller
  ux = KPx*errx + KVx*errvx;
  uy = k_rope*(E-E_des)*omega*cos(theta);
  uz = KPz*errz + KVz*errvz;
  if(uy > ng){
    uy = ng;
  }
  if(uy < -ng){
    uy = -ng;
  }
  ux_f = cos(-yaw) * ux - sin(-yaw - 3.1415926) * (uy);
  uy_f = sin(-yaw) * ux + cos(-yaw - 3.1415926) * (uy);
  rope_control_input.y = uy_f;
}
*/

if(jumping_rope_control == true){
  if(E > E_des){
    break_joint.x = 1;
    break_joint.y = theta;

    ROS_INFO("-----------------Break joint---------------------");
    ROS_INFO("theta = %f", theta);
  }
  sim_angle.x = 0;
  if(!balance_control_theta){
  //limited path
  ux = 0.4*(E-E_des)*omega*cos(theta) + 0.1 * copysign(1,host_mocap.pose.position.x) * log(1-fabs(host_mocap.pose.position.x)/1);
  //general
  //ux = 0.4*(E-E_des)*omega*cos(theta);
  if(ux > 0.3*ng){
    ux =  0.3*ng;
  }
  if(ux < - 0.3*ng){
    ux =  - 0.3*ng;
  }

  a_fb <<  0, 9*erry + 2.25*errvy+ sumy, 10*errz + 3.33*errvz;
  a_ref << ux, 0, 0;
  a_des << a_ref + a_fb - g_;
  q_des = acc2quaternion(a_des,yaw);
  cmdbodyrate_ = attcontroller(q_des,a_des,mavatt_);
  pose->body_rate.x = cmdbodyrate_(0);
  pose->body_rate.y = cmdbodyrate_(1);
  pose->body_rate.z = cmdbodyrate_(2);
  pose->thrust = cmdbodyrate_(3);


}
  if (balance_control){

    if(!balance_control_theta){
      if(theta < -1.95*pi || theta > -0.05*pi){
        balance_control_theta = true;
        vir.x = host_mocap.pose.position.x;
      }
    }
    if(balance_control_theta){
      if(theta > -1.70*pi && theta < -0.30*pi){
        balance_control_theta = false;
        balance_control_flag.x = 0;
      }
    }

    //for rope_theta_pose
    /*
    if(!balance_control_theta){
      if(theta_2 < 0.10*pi && theta_2 > -0.10*pi){
        balance_control_theta = true;
        vir.x = host_mocap.pose.position.x;
      }
    }
    if(balance_control_theta){
      if(theta_2 > 0.1*pi || theta_2 < -0.1*pi){
        balance_control_theta = false;
        balance_control_flag.x = 0;
      }
    }
*/
    if(balance_control_theta){
    ROS_INFO("""start balance!!!!!!!!!!!!!!!!!");
    balance_control_flag.x = 1;
    Eigen::VectorXd x_vector(6);
    Eigen::VectorXd y_vector(6);
    Eigen::VectorXd K_x(6);
    Eigen::VectorXd K_y(6);

    double integral_term;
    Eigen::Matrix4d A_exp(4,4);
    Eigen::Vector4d state_x;
    if(theta < -pi && theta > -2*pi){
      theta = (theta + 2*pi);
    }
    sim_angle.x = theta;
    //x_vector << host_mocap.pose.position.x, host_mocap.pose.position.z, theta, host_mocapvel.twist.linear.x, host_mocapvel.twist.linear.z, omega;
    //y_vector << host_mocap.pose.position.y, host_mocap.pose.position.z, theta_2, host_mocapvel.twist.linear.y, host_mocapvel.twist.linear.z, omega_2;
    //K_x << -0.0, 0, -146.8735, -6.8598, 0, -18.2266;
    //K_x << -0.0, 0, -29.51006, -0.70711, 0, -4.92088;
    //ux = -K_x.transpose() * x_vector;
    //uy = -K_x.transpose() * y_vector;

    //FSA

    state_x << 0, theta, host_mocapvel.twist.linear.x, omega;
    double S_con;

    if(i_count > control_input_delay + 1){
          //std::cout << " start FSA ----" << std::endl;
          for(int j = 1; j < r_til +1 ; j++){
          //std::cout << "control command " << control_input_delay - j*delta_t*100  << ": " << control_command[control_input_delay - j*delta_t*100] << std::endl;
            A_exp = A_*j*delta_t;
            integral_term = integral_term + (control_gain_K.transpose()*A_exp.exp()*B_*control_command[control_input_delay - j] * delta_t).value();
            //S_con = S_con + (control_gain_K.transpose()*A_exp.exp()*B_*delta_t).value();

          //std::cout << "integral termss: " << A_exp.exp() << std::endl;
          }//
          ux = ((control_gain_K.transpose()*(A_*tau_).exp()*state_x).value()+integral_term);
          //std::cout << "S: " << S_con << std::endl;
    }else{
      if(i_count == 0){
        //out =  k1 * radian + k2 * speed + k3 * wheel_speed * 0.1;
        ux = ((control_gain_K.transpose()*(A_*tau_).exp()*state_x).value());
        //std::cout << "out : " << out << std::endl;
      }
      else{
          for (int k = 1; k < i_count + 1; k++)
          {
            double control_count;
            control_count = i_count - k;
            if( (control_count) < 0 ){
              control_count = 0;
            }
            A_exp = A_*k*delta_t;
            integral_term = integral_term + (control_gain_K.transpose()*A_exp.exp()*B_*control_command[control_count]* delta_t).value();
          }
          ux = ((control_gain_K.transpose()*(A_*tau_).exp()*state_x).value()+integral_term);
          //out =  k1 * radian + k2 * speed + k3 * wheel_speed * 0.1;

      }
    }
    ux = -ux;

    control_command.push_back(ux);
    i_count++;
    if(i_count > control_input_delay + 1){
      control_command.erase(control_command.begin());
      /*
      std::cout << "control command:" << std::endl;
      for(int p = 0; p < control_input_delay + 1; p++){
        std::cout << control_command[p] << "\t" ;
      }
      */
    }

    if(ux > 1.0*ng){
      ux = 1.0*ng;
    }
    if(ux < -1.0*ng){
      ux = -1.0*ng;
    }
    //pos_des.x = vir.x;
    //4.5*(vir.x - host_mocap.pose.position.x) + 1.1*(0-host_mocapvel.twist.linear.x)
    //3*erry + 0.75*errvy+ sumy
    a_fb <<  0,3*erry + 0.75*errvy+ sumy, 10*errz + 3.33*errvz;
    a_ref << ux, 0, 0;
    a_des << a_ref + a_fb - g_;
    q_des = acc2quaternion(a_des,yaw);
    cmdbodyrate_ = attcontroller(q_des,a_des,mavatt_);
    pose->body_rate.x = cmdbodyrate_(0);
    pose->body_rate.y = cmdbodyrate_(1);
    pose->body_rate.z = cmdbodyrate_(2);
    pose->thrust = cmdbodyrate_(3);
    rope_control_input.x = cmdbodyrate_(1);

}
    balance_control = true;
  }

}

/*
if(jumping_rope_control == true){
  ux = 0.5*9.81*tan(theta) + k_rope*(E-E_des)*omega*cos(theta);
  uz = KPz*errz + KVz*errvz - 0.1*0.5*alpha*sin(theta) - 0.1*0.5*omega*omega*cos(theta);
  uy = KPy*erry + KVy*errvy;

  if(ux > ng){
    ux =  ng;
  }
  if(ux < - ng){
    ux =  - ng;
  }
  rope_control_input.x = ux;

  ux_f = cos(-yaw) * ux - sin(-yaw) * (uy);
  uy_f = sin(-yaw) * ux + cos(-yaw) * (uy);
  rope_control_input.y = uy_f;
}
*/


uyaw = KPyaw*err_roll;



/*
Eigen::Vector3d b1,b2,b3;
total_thrust << 0.5*1.5*ux, 0.5*1.5*uy, 0.5*uz;
    b3 = total_thrust/total_thrust.norm();

    b1 << cos(0) , sin(0),0;
    b2 = b3.cross(b1) / (b3.cross(b1)).norm();
    Eigen::Matrix3d att;
    att<< b1 ,b2 ,b3;
    tf::Matrix3x3 q(
                    att(0,0),att(0,1),att(0,2),
                    att(1,0),att(1,1),att(1,2),
                    att(2,0),att(2,1),att(2,2)
                    );
*/


//pose->thrust = total_thrust.dot(R_IB*e3);
rope_control_input.x = cmdbodyrate_(1);

sim_control_input.z = pose->thrust;
sim_control_input.y = uz_test + 0.4;
}

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

/*
 * Call main using `rosrun offb offb_main`.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "simu_jumping_rope");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
                                ("/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
                                   ("/mavros/setpoint_position/local", 10);
    //ros::Publisher mocap_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
    //                               ("/mavros/mocap/pose", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                                       ("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                                         ("/mavros/set_mode");
    ros::Subscriber host_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, host_pos);
    ros::Publisher attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 2);
    ros::Subscriber host2_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 1, host_vel);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 1, imu_cb);
    ros::Subscriber ropea_sub = nh.subscribe<geometry_msgs::Point>("/leader_ukf/rope_theta", 2, rope_cb);
    ros::Subscriber wrench_sub = nh.subscribe<geometry_msgs::WrenchStamped>("/ft_sensor", 2, wrench_cb);
    ros::Subscriber force_sub = nh.subscribe<geometry_msgs::Point>("leader_ukf/ttt", 2, force_est_cb);
    ros::Subscriber imu_payload_sub = nh.subscribe<sensor_msgs::Imu>("/sim_payload_imu", 2, payload_imu_cb);
    ros::Publisher pos_pub = nh.advertise<geometry_msgs::Point>("/desired_position", 2);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Point>("/desired_velocity", 2);
    ros::Publisher mv_pub = nh.advertise<geometry_msgs::Point>("/motive_vel",2);
    ros::Publisher rope_control_input_pub = nh.advertise<geometry_msgs::Point>("/rope_control_input", 2);
    ros::Publisher rope_energy_pub = nh.advertise<geometry_msgs::Point>("/rope_energy", 2);
    ros::Publisher trigger_pub = nh.advertise<geometry_msgs::Point>("/trigger", 2);
    ros::Publisher sim_angle_pub = nh.advertise<geometry_msgs::Point>("/sim_angle", 2);
    ros::Publisher sim_force_pub = nh.advertise<geometry_msgs::Point>("/sim_force_sensor", 2);
    ros::Publisher sim_pos_pub = nh.advertise<geometry_msgs::Point>("/sim_pos", 2);
    ros::Publisher sim_apply_force_pub = nh.advertise<geometry_msgs::Point>("/sim_apply_force", 2);
    ros::Publisher sim_acc_pub = nh.advertise<geometry_msgs::Point>("/sim_acc", 2);
    ros::Publisher sim_tan_norm_pub = nh.advertise<geometry_msgs::Point>("/sim_tan_norm", 2);
    ros::Publisher sim_control_input_pub = nh.advertise<geometry_msgs::Point>("/sim_control_input", 2);
    ros::Publisher sim_drone_vel_pub = nh.advertise<geometry_msgs::Point>("/sim_drone_vel", 2);
    ros::Publisher sim_drone_att_pub = nh.advertise<geometry_msgs::Point>("/sim_drone_att", 2);
    ros::Publisher balance_control_flag_pub = nh.advertise<geometry_msgs::Point>("/balance_control_flag", 2);
    ros::Subscriber sim_force_on_drone_sub = nh.subscribe<geometry_msgs::Point>("/sim_force_on_drone", 2, force_on_drone_cb);
    ros::ServiceClient apply_force_client = nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
    ros::Subscriber gazebo_state_sub = nh.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 3, gazebo_state_cb);
    ros::Publisher break_joint_pub = nh.advertise<geometry_msgs::Point>("/break_joint", 2);
    // The setpoint publishing rate MUST be faster than 2Hz.
    ros::Rate rate(30);

    //nh.setParam("/mavros/vision_pose/tf/rate_limit", 200);
    double omega_n1, omega_n2;
    // Wait for FCU connection.
    while (ros::ok() && current_state.connected) {
        //mocap_pos_pub.publish(host_mocap);
        ros::spinOnce();
        rate.sleep();
    }
    vir vir1;

    vir1.x = 0.0;
    vir1.y = 0;
    vir1.z = 2.0;
    vir1.roll = 0;

    mavros_msgs::AttitudeTarget pose;
/*
    pose.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE|
                     mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE|
                     mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;
*/

    pose.thrust = 0;

    pose.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
    pose.body_rate.x = 0;
    pose.body_rate.y = 0;
    pose.body_rate.z = 0;

    //send a few setpoints before starting
   for(int i = 200; ros::ok() && i > 0; --i){
        attitude_pub.publish(pose);
        //mocap_pos_pub.publish(host_mocap);
        vir1.x = 0.0;
        vir1.y = 0.0;
        vir1.z = 2.0;
        vir1.roll = 0;

        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
        //ros::Time last_request(0);
    double last_angle;
    double last_vel;
    double time_now;
    while (ros::ok()) {
        //mocap_pos_pub.publish(host_mocap);
        if (current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {

            if (!current_state.armed &&
                    (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if( arming_client.call(arm_cmd) &&
                        arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }


        int c = getch();
        //ROS_INFO("C: %d",c);
        if (c != EOF) {
            switch (c) {
            case 65:    // key up
                vir1.z += 0.05;
                break;
            case 66:    // key down
                vir1.z += -0.05;
                break;
            case 67:    // key CW(->)
                vir1.roll -= 0.03;
                break;
            case 68:    // key CCW(<-)
                vir1.roll += 0.03;
                break;
                        case 119:    // key foward
                vir1.x += 0.05;
                break;
            case 120:    // key back
                vir1.x += -0.05;
                break;
            case 97:    // key left
                vir1.y += 0.05;
                break;
            case 100:    // key right
                vir1.y -= 0.05;
                break;
            case 49: // keyboard 1
                initial_acc = false;
                jumping_rope_control = true;
                trigger_control.x = 2;
                break;
            case 50: // keyboard 2
                initial_acc = true;
                trigger_control.x = 1;
              break;
            case 51: // keyboard 3
                apply_wrench_flag = true;
              break;
            case 52: // keyboard 4
                balance_control = true;
              break;
            case 115: // keyboard s
                jumping_rope_control = false;
                vir1.z = 0.2;
                break;
            case 108:    // close arming
                        {
                        offb_set_mode.request.custom_mode = "MANUAL";
                        set_mode_client.call(offb_set_mode);
                        arm_cmd.request.value = false;
                        arming_client.call(arm_cmd);
            break;
                        }
            case 63:
                return 0;
                break;
            }
        }
                if(vir1.roll>pi)
                vir1.roll = vir1.roll - 2*pi;
                else if(vir1.roll<-pi)
                vir1.roll = vir1.roll + 2*pi;
        double rope_theta, rope_omega, rope_omega_f, rope_theta1, rope_theta_2, rope_theta_pose;
//for no offset inverted pendulum

        if(wrench_.wrench.force.x < 0 && wrench_.wrench.force.z > 0){
          rope_theta = atan2(wrench_.wrench.force.z,wrench_.wrench.force.x);
          rope_theta = -2.5*3.1415926+rope_theta;
        }
        else{
        rope_theta = atan2(wrench_.wrench.force.z,wrench_.wrench.force.x) - 3.1415926/2;
}
break_joint.z = rope_theta;
//for offset inverted pendulum
        /*
        if(wrench_.wrench.force.x < 0 && -wrench_.wrench.force.z > 0){
          rope_theta = atan2(-wrench_.wrench.force.z,wrench_.wrench.force.x);
          rope_theta = -2.5*3.1415926+rope_theta;
        }
        else{
        rope_theta = atan2(-wrench_.wrench.force.z,wrench_.wrench.force.x) - 3.1415926/2;
}
*/
//calculated from pose (base link and payload)
        double ddx, ddz;
        ddx = payload_pose.pose.position.x - iris_base.pose.position.x;
        ddz = payload_pose.pose.position.z - iris_base.pose.position.z;
        pos_des.x = ddx;
        pos_des.z = ddz;
        rope_theta_pose = -(atan2(ddz,ddx) - 3.1415926/2);
        //sim_angle.y = rope_theta_pose;

        if(wrench_.wrench.force.y < 0 && wrench_.wrench.force.z > 0){
          rope_theta_2 = atan2(wrench_.wrench.force.z,wrench_.wrench.force.y);
          rope_theta_2 = -2.5*3.1415926+rope_theta_2;
        }
        else{
        rope_theta_2 = (atan2(wrench_.wrench.force.z,wrench_.wrench.force.y) - 3.1415926/2);
}

//calculated from estimated force
        if(force_est.z < 0){
          rope_theta1 = atan2(force_est.z,force_est.x) + 2*pi;
        }
        else{
          rope_theta1 = atan2(force_est.z,force_est.x);
        }
        if(force_est.x > 0 && force_est.z > 0){
          rope_theta1 = rope_theta1 + 1.5*pi;
        }
        else{
        rope_theta1 = rope_theta1 - pi/2;
        }

        rope_theta1 = -rope_theta1;

        /*
        if(force_on_drone.x < 0 && force_on_drone.z > 0){
          rope_theta = atan2(force_on_drone.z,force_on_drone.x);
          rope_theta = -2.5*3.1415926+rope_theta;
        }
        else{
        rope_theta = atan2(force_on_drone.z,force_on_drone.x) - 3.1415926/2;
}
*/
        //rope_theta = atan2(force_est.z,force_est.x) - 3.1415926/2;
        sim_angle.z = rope_theta;
        sim_angle.y = rope_theta1;
        /*
        rope_theta = rope_angle.x;
        rope_omega = rope_angle.y;
*/
        rope_omega = (rope_theta - last_angle)*50;
        //sim_angle.y = payload_imu.angular_velocity.y;
        //sim_angle.y = payload_link_omega.y;
        last_angle = rope_theta;
        rope_omega_f = (rope_omega + omega_n1 + omega_n2)/3;
        omega_n2 = omega_n1;
        omega_n1 = rope_omega;
//        ROS_INFO("setpoint: %.2f, %.2f, %.2f, %.2f", vir1.x, vir1.y, vir1.z, vir1.roll/pi*180);
        sim_force.x = -wrench_.wrench.force.x;
        sim_force.y = wrench_.wrench.force.y;
        sim_force.z = wrench_.wrench.force.z;
        sim_pos.x = host_mocap.pose.position.x;
        sim_pos.y = host_mocap.pose.position.y;
        sim_pos.z = host_mocap.pose.position.z;
        sim_pos_pub.publish(sim_pos);
        sim_force_pub.publish(sim_force);

        //follow(vir1,host_mocap,&pose,rope_theta, rope_theta_2, payload_imu.angular_velocity.y,  payload_imu.angular_velocity.x);
        //std::thread mThread{follow_omega,vir1,host_mocap,&pose,rope_theta, rope_theta_2, payload_imu.angular_velocity.y,  payload_imu.angular_velocity.x};
        //mThread.join();
        //follow_omega(vir1,host_mocap,&pose,rope_theta, rope_theta_pose, payload_imu.angular_velocity.y,  payload_imu.angular_velocity.x);
        follow_omega_delay(vir1,host_mocap,&pose,rope_theta1, rope_theta_pose, payload_imu.angular_velocity.y,  payload_imu.angular_velocity.x);
        //pos_des.x = vir1.x;
        //pos_des.y = vir1.y;
        //pos_des.z = vir1.z;
        if(apply_wrench_flag){
          ros::Duration duration_(0.8);
          //
          drone_apply_force.request.body_name="iris_rplidar::payload::payload";
          drone_apply_force.request.duration = duration_;
          drone_apply_force.request.reference_point = ref_point;
          apply_wrench.force.x = 1;
          apply_wrench.force.y = 0;
          apply_wrench.force.z = 0;
          drone_apply_force.request.wrench = apply_wrench;
          apply_force_client.call(drone_apply_force);
          sim_apply_force_.x = apply_wrench.force.x;
          sim_apply_force_.y = apply_wrench.force.y;
          sim_apply_force_.z = apply_wrench.force.z;
          sim_apply_force_pub.publish(sim_apply_force_);
          time_now = ros::Time::now().toSec();
          apply_wrench_flag = false;
        }
        double dtt;
        dtt = ros::Time::now().toSec() - time_now;
        if(dtt > 2.0){
        sim_apply_force_.x = 0;
        sim_apply_force_.y = 0;
        sim_apply_force_.z = 0;
        }


        double drone_a;

        sim_drone_vel.x = host_mocapvel.twist.linear.x;
        sim_drone_vel.z = host_mocapvel.twist.linear.z;

        last_vel = host_mocapvel.twist.linear.x;
        //sim_drone_vel_pub.publish(sim_drone_vel);
        //sim_apply_force_pub.publish(sim_apply_force_);
        //sim_tan_norm_pub.publish(sim_tan_norm);
        trigger_control.z = rope_omega_f;
        //mocap_pos_pub.publish(host_mocap);
        sim_acc.x = imu_data.linear_acceleration.x;
        sim_acc.z = imu_data.linear_acceleration.z;
        sim_acc_pub.publish(sim_acc);
        pose.header.stamp = ros::Time::now();
        attitude_pub.publish(pose);
        rope_control_input_pub.publish(rope_control_input);
        //pos_pub.publish(pos_des);
        rope_energy_pub.publish(rope_energy);
        //trigger_pub.publish(trigger_control);
        sim_angle_pub.publish(sim_angle);
        //sim_control_input_pub.publish(sim_control_input);
        balance_control_flag_pub.publish(balance_control_flag);
        sim_drone_att_pub.publish(sim_drone_att);
        break_joint_pub.publish(break_joint);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


