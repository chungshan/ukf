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
//for drone 1(.76)

float KPx=4, KPy=4, KPz=10;
float KVx=1.67, KVy=1.67, KVz=3.3;
float KIx=0.33, KIy=0.33, KIz=0.00;
float KPyaw = 1;

//for drone 3(.92)
/*
float KPx=3.5, KPy=4.5, KPz=10;
float KVx=1.8, KVy=1.5, KVz=4;
float KIx=0.33, KIy=0.33, KIz=0.05;
float KPyaw = 1;
*/
double roll, pitch, yaw;
float r = 0.5;
float T = 2*pi;
using namespace std;



typedef struct
{
    float yaw;
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
        //sim_drone_att.y = pitch;

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
const double norm_thrust_const_ = 0.045;//thrust factor may lead a stead-state error in Z-axis
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
double sumx, sumy, sumz;

void angular_ctrl(vir& vir, geometry_msgs::PoseStamped& host_mocap, mavros_msgs::AttitudeTarget* pose)
{
float errx, erry, errz, err_yaw;
float errvx , errvy ,errvz ;

const double g = 9.81;

Eigen::Vector3d e3;
e3 << 0,0,1;

errx = -(host_mocap.pose.position.x - vir.x);
erry = -(host_mocap.pose.position.y - vir.y);
errz = -(host_mocap.pose.position.z - vir.z);

errvx = -(host_mocapvel.twist.linear.x - 0);
errvy = -(host_mocapvel.twist.linear.y - 0);
errvz = -(host_mocapvel.twist.linear.z - 0);



sumx += KIx*errx*0.02;
sumy += KIy*erry*0.02;
sumz += KIz*errz*0.02;

/*
sumx += KIx*errx*0.02;
sumy += KIy*erry*0.02;
sumz += KIz*errz*0.02;
*/
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
err_yaw = vir.yaw - yaw;
if(err_yaw>pi)
err_yaw = err_yaw - 2*pi;
else if(err_yaw<-pi)
err_yaw = err_yaw + 2*pi;

ROS_INFO("err: x = %.3f, y = %.3f, z = %.3f, yaw = %.3f",errx,erry,errz,err_yaw);
//ROS_INFO("err: %.3f,%.3f,%.3f",errvx,errvy,errvz);

Eigen::Vector3d g_;
g_ << 0,0,-9.8;
Eigen::Vector3d a_des;
Eigen::Vector3d a_ref;
Eigen::Vector3d a_fb;
Eigen::Vector4d q_des;
Eigen::Vector4d cmdbodyrate_;

 a_fb <<  KPx*errx + KVx*errvx + sumx, KPy*erry + KVy*errvy+ sumy,  KPz*errz + KVz*errvz + KIz * sumz;
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
    ros::init(argc, argv, "angular_v_ctrl");
    ros::NodeHandle nh;

    std::string topic_state, topic_arming, topic_set_mode, topic_host_pos, topic_host_vel, topic_att, name_space;
    name_space = "drone1";//Set your namespace
    topic_state = name_space + "/mavros/state";
    topic_arming = name_space + "/mavros/cmd/arming";
    topic_set_mode = name_space + "/mavros/set_mode";
    topic_host_pos = name_space + "/mavros/local_position/pose";
    topic_host_vel = name_space + "/mavros/local_position/velocity_local";
    topic_att = name_space + "/mavros/setpoint_raw/attitude";
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
                                (topic_state, 2, state_cb);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                                       (topic_arming);
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                                         (topic_set_mode);
    ros::Subscriber host_sub = nh.subscribe<geometry_msgs::PoseStamped>(topic_host_pos, 2, host_pos);

    ros::Subscriber host_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>(topic_host_vel, 2, host_vel);
    ros::Publisher attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>(topic_att, 2);

    // The setpoint publishing rate MUST be faster than 2Hz.
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::Rate rate(50);

    //nh.setParam("/mavros/vision_pose/tf/rate_limit", 200);

    // Wait for FCU connection.
    while (ros::ok() && current_state.connected) {
        //mocap_pos_pub.publish(host_mocap);
        ros::spinOnce();
        rate.sleep();
    }
    vir vir1;

    vir1.x = 0.8;
    vir1.y = 0;
    vir1.z = 0.5;
    vir1.yaw = 0;

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
        vir1.x = 0.8;
        vir1.y = 0.0;
        vir1.z = 0.5;
        vir1.yaw = 0;

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
                vir1.yaw -= 0.03;
                break;
            case 68:    // key CCW(<-)
                vir1.yaw += 0.03;
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
            case 108:    // close arming
                        {
                        offb_set_mode.request.custom_mode = "MANUAL";
                        set_mode_client.call(offb_set_mode);
                        arm_cmd.request.value = false;
                        arming_client.call(arm_cmd);
            break;
            case 115: // keyboard s
                vir1.z = 0.2;
                break;
                        }
            case 63:
                return 0;
                break;
            }
        }
                if(vir1.yaw>pi)
                vir1.yaw = vir1.yaw - 2*pi;
                else if(vir1.yaw<-pi)
                vir1.yaw = vir1.yaw + 2*pi;
        angular_ctrl(vir1,host_mocap,&pose);
        attitude_pub.publish(pose);

        //ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


