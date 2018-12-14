#include <ros/ros.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "forceest.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_datatypes.h>
#include "geometry_msgs/Point.h"
#include "math.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Point.h"
#include "mavros_msgs/RCOut.h"
#include "sensor_msgs/MagneticField.h"
#include <string>
#define l 0.25
#define k 0.02
int drone_flag;
forceest forceest1(statesize,measurementsize);
geometry_msgs::Point euler, euler_ref, force, torque, bias, angular_v;
sensor_msgs::Imu drone2_imu;
void imu_cb(const sensor_msgs::Imu::ConstPtr &msg){
  drone2_imu = *msg;
}
sensor_msgs::Imu raw;
void raw_cb(const sensor_msgs::Imu::ConstPtr &msg){
  raw = *msg;
}
geometry_msgs::PoseStamped drone2_pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
  drone2_pose = *msg;
  forceest1.quat_m << drone2_pose.pose.orientation.x, drone2_pose.pose.orientation.y, drone2_pose.pose.orientation.z, drone2_pose.pose.orientation.w;

}
geometry_msgs::TwistStamped drone2_vel;
void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg){
  drone2_vel = *msg;
}

mavros_msgs::RCOut rc_out;
void rc_cb(const mavros_msgs::RCOut::ConstPtr &msg){
  rc_out = *msg;
}
sensor_msgs::MagneticField mag;
void mag_cb(const sensor_msgs::MagneticField::ConstPtr &msg){
  mag = *msg;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "force_estimate");
  ros::NodeHandle nh;

  std::string topic_imu, topic_mocap, topic_thrust, topic_vel, topic_mag, topic_raw;
  ros::param::get("~topic_imu", topic_imu);
  ros::param::get("~topic_mocap", topic_mocap);
  ros::param::get("~topic_thrust", topic_thrust);
  ros::param::get("~topic_vel", topic_vel);
  ros::param::get("~topic_vel", topic_vel);
  ros::param::get("~topic_drone", drone_flag);
  ros::param::get("~topic_mag", topic_mag);
  ros::param::get("~topic_raw", topic_raw);

  ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>(topic_imu, 2, imu_cb);
  ros::Subscriber raw_sub = nh.subscribe<sensor_msgs::Imu>(topic_raw, 2, raw_cb);
  ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PoseStamped>(topic_mocap, 2, pose_cb);
  ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>(topic_vel, 2, vel_cb);
  ros::Subscriber rc_sub = nh.subscribe<mavros_msgs::RCOut>(topic_thrust, 2, rc_cb);
  ros::Subscriber mag_sub = nh.subscribe<sensor_msgs::MagneticField>(topic_mag, 2, mag_cb);
  ros::Publisher euler_pub = nh.advertise<geometry_msgs::Point>("euler", 2);
  ros::Publisher euler_ref_pub = nh.advertise<geometry_msgs::Point>("euler_ref", 2);
  ros::Publisher force_pub = nh.advertise<geometry_msgs::Point>("force_estimate", 2);
  ros::Publisher torque_pub = nh.advertise<geometry_msgs::Point>("torque", 2);
  ros::Publisher bias_pub = nh.advertise<geometry_msgs::Point>("bias", 2);
  ros::Publisher angular_v_pub = nh.advertise<geometry_msgs::Point>("angular_v", 2);
  ros::Rate loop_rate(30);


  double measure_ex, measure_ey, measure_ez;
  Eigen::MatrixXd mnoise;
  mnoise.setZero(measurementsize,measurementsize);
  mnoise   = 3e-3*Eigen::MatrixXd::Identity(measurementsize , measurementsize);

  mnoise(mp_x,mp_x) = 1e-4;
  mnoise(mp_y,mp_y) = 1e-4;
  mnoise(mp_z,mp_z) = 1e-4;
  mnoise(mv_x,mv_x) = 1e-2;
  mnoise(mv_y,mv_y) = 1e-2;
  mnoise(mv_z,mv_z) = 1e-2;
/*
  mnoise(momega_x,momega_x) = 1e-2;
  mnoise(momega_y,momega_y) = 1e-2;
  mnoise(momega_z,momega_z) = 1e-2;
*/
  mnoise(me_x,me_x) = 1;//1
  mnoise(me_y,me_y) = 1;
  mnoise(me_z,me_z) = 1;
/*
  mnoise(mq_x,mq_x) = 1e-2;
  mnoise(mq_y,mq_y) = 1e-2;
  mnoise(mq_z,mq_z) = 1e-2;
  mnoise(mq_w,mq_w) = 1e-2;
*/
  forceest1.set_measurement_noise(mnoise);


  Eigen::MatrixXd pnoise;
  pnoise.setZero(statesize,statesize);
  pnoise(p_x,p_x) = 1e-2;
  pnoise(p_y,p_y) = 1e-2;
  pnoise(p_z,p_z) = 1e-2;
  pnoise(v_x,v_x) = 1e-2;
  pnoise(v_y,v_y) = 1e-2;
  pnoise(v_z,v_z) = 1e-2;
  pnoise(e_x,e_x) = 0.005;//0.5,調小beta收斂較快
  pnoise(e_y,e_y) = 0.005;
  pnoise(e_z,e_z) = 0.005;
/*
  pnoise(omega_x,omega_x) = 1e-2;
  pnoise(omega_y,omega_y) = 1e-2;
  pnoise(omega_z,omega_z) = 1e-2;
*/
  pnoise(F_x,F_x) = 0.05;
  pnoise(F_y,F_y) = 0.05;
  pnoise(F_z,F_z) = 0.05;
  pnoise(tau_z,tau_z) = 0.05;

  pnoise(beta_x,beta_x) = 0.05;//調大beta會無法收斂
  pnoise(beta_y,beta_y) = 0.05;
  pnoise(beta_z,beta_z) = 0.05;



  forceest1.set_process_noise(pnoise);



  Eigen::MatrixXd measurement_matrix;
  measurement_matrix.setZero(measurementsize,statesize);

  measurement_matrix(mp_x,p_x) = 1;
  measurement_matrix(mp_y,p_y) = 1;
  measurement_matrix(mp_z,p_z) = 1;


  measurement_matrix(mv_x,v_x) = 1;
  measurement_matrix(mv_y,v_y) = 1;
  measurement_matrix(mv_z,v_z) = 1;

/*
  measurement_matrix(momega_x,omega_x) = 1;
  measurement_matrix(momega_y,omega_y) = 1;
  measurement_matrix(momega_z,omega_z) = 1;
*/
  measurement_matrix(me_x,e_x) = 3.5;//1,調小，beta會劇烈震盪
  measurement_matrix(me_y,e_y) = 3.5;
  measurement_matrix(me_z,e_z) = 3.5;

/*
  measurement_matrix(mq_x,q_x) = 1;
  measurement_matrix(mq_y,q_y) = 1;
  measurement_matrix(mq_z,q_z) = 1;
  measurement_matrix(mq_w,q_w) = 1;
*/



  forceest1.set_measurement_matrix(measurement_matrix);

  while(ros::ok()){
    double F1, F2, F3, F4;
    double pwm1, pwm2, pwm3, pwm4;
    double U_x, U_y, U_z;
    Eigen::MatrixXd theta_q(4,3), phi_q(4,3);
    Eigen::Matrix3d A_q;
    Eigen::Vector3d y_k;
    Eigen::Vector3d mag_v;


  if(drone2_imu.angular_velocity.x!=0 && drone2_pose.pose.position.x !=0 && drone2_vel.twist.linear.x !=0){
    if(rc_out.channels.size()!=0 && rc_out.channels[0] != 0){

    pwm1 = rc_out.channels[0];
    pwm2 = rc_out.channels[1];
    pwm3 = rc_out.channels[2];
    pwm4 = rc_out.channels[3];

    }

if(drone_flag==3){
  double b;
    F1 = (5.6590*1e-4*(pwm3*pwm3) - 0.5995*pwm3 - 97.5178)*9.8/1000; // drone3
    F2 = (5.6590*1e-4*(pwm1*pwm1) - 0.5995*pwm1 - 97.5178)*9.8/1000; //left_right:127.5178
    F3 = (5.6590*1e-4*(pwm4*pwm4) - 0.5995*pwm4 - 97.5178)*9.8/1000; //up_down:97.5178
    F4 = (5.6590*1e-4*(pwm2*pwm2) - 0.5995*pwm2 - 97.5178)*9.8/1000;
  b = -(3.065625*1000/9.8-5.6590*1e-4*(pwm1*pwm1)+0.5995*pwm1);//no payload

  std::cout << "---b---" << std::endl;
  std::cout << b << std::endl;

}
if(drone_flag==2){
  double a;
    F1 = (8.1733*1e-4*(pwm3*pwm3) - 1.2950*pwm3 + 265.7775)*9.8/1000; //drone2
    F2 = (8.1733*1e-4*(pwm1*pwm1) - 1.2950*pwm1 + 265.7775)*9.8/1000; //left_right:265.7775
    F3 = (8.1733*1e-4*(pwm4*pwm4) - 1.2950*pwm4 + 265.7775)*9.8/1000; //up_down:265.7775
    F4 = (8.1733*1e-4*(pwm2*pwm2) - 1.2950*pwm2 + 265.7775)*9.8/1000;
    a = 3.065625*1000/9.8-8.1733*1e-4*(pwm1*pwm1)+1.2950*pwm1;//no payload

    std::cout << "---a---" << std::endl;
    std::cout << a << std::endl;

}
    forceest1.thrust = F1 + F2 + F3 + F4;

    std::cout << "----------thrust-------" << std::endl;
    std::cout << forceest1.thrust << std::endl;

    U_x = (sqrt(2)/2)*l*(F1 - F2 - F3 + F4);
    U_y = (sqrt(2)/2)*l*(-F1 - F2 + F3 + F4);
    U_z = k*F1 - k*F2 + k*F3 - k*F4;
    forceest1.U << U_x, U_y, U_z;
    double x = drone2_pose.pose.orientation.x;
    double y = drone2_pose.pose.orientation.y;
    double z = drone2_pose.pose.orientation.z;
    double w = drone2_pose.pose.orientation.w;

    forceest1.R_IB.setZero();
    forceest1.R_IB << w*w+x*x-y*y-z*z  , 2*x*y-2*w*z ,            2*x*z+2*w*y,
        2*x*y +2*w*z           , w*w-x*x+y*y-z*z    ,2*y*z-2*w*x,
        2*x*z -2*w*y          , 2*y*z+2*w*x        ,w*w-x*x-y*y+z*z;
    forceest1.angular_v_measure << drone2_imu.angular_velocity.x+0.5, drone2_imu.angular_velocity.y, drone2_imu.angular_velocity.z;

    forceest1.predict();

    Eigen::VectorXd measure;
    measure.setZero(measurementsize);

    measure << drone2_pose.pose.position.x, drone2_pose.pose.position.y, drone2_pose.pose.position.z,
               drone2_vel.twist.linear.x, drone2_vel.twist.linear.y, drone2_vel.twist.linear.z,
               measure_ex, measure_ey, measure_ez;
               //drone2_imu.angular_velocity.x, drone2_imu.angular_velocity.y, drone2_imu.angular_velocity.z;
    forceest1.quat_m << drone2_pose.pose.orientation.x, drone2_pose.pose.orientation.y, drone2_pose.pose.orientation.z, drone2_pose.pose.orientation.w;
    forceest1.qk11 = forceest1.qk1;
/*
    theta_q << forceest1.quaternion(3), -forceest1.quaternion(2), forceest1.quaternion(1),
               forceest1.quaternion(2), forceest1.quaternion(3), -forceest1.quaternion(0),
               -forceest1.quaternion(1), forceest1.quaternion(0), forceest1.quaternion(3),
               -forceest1.quaternion(0), -forceest1.quaternion(1), -forceest1.quaternion(2);
    phi_q << forceest1.quaternion(3), forceest1.quaternion(2), -forceest1.quaternion(1),
               -forceest1.quaternion(2), forceest1.quaternion(3), forceest1.quaternion(0),
               forceest1.quaternion(1), -forceest1.quaternion(0), forceest1.quaternion(3),
               -forceest1.quaternion(0), -forceest1.quaternion(1), -forceest1.quaternion(2);
    A_q = theta_q.transpose()*phi_q;
    mag_v << mag.magnetic_field.x,mag.magnetic_field.y,mag.magnetic_field.z;
    y_k = A_q*mag_v;
    std::cout << "---y_k---" << std::endl;
    std::cout << y_k << std::endl;
*/
    forceest1.correct(measure);
    forceest1.x[e_x] = 0;
    forceest1.x[e_y] = 0;
    forceest1.x[e_z] = 0;
/*
    std::cout << "--------" << std::endl;

    std::cout << "---position---" << std::endl;
    std::cout << forceest1.x[p_x] << std::endl;
    std::cout << forceest1.x[p_y] << std::endl;
    std::cout << forceest1.x[p_z] << std::endl;
    std::cout << "---velocity---" << std::endl;
    std::cout << forceest1.x[v_x] << std::endl;
    std::cout << forceest1.x[v_y] << std::endl;
    std::cout << forceest1.x[v_z] << std::endl;

    std::cout << "---error quaternion---" << std::endl;
    std::cout << forceest1.x[e_x] << std::endl;
    std::cout << forceest1.x[e_y] << std::endl;
    std::cout << forceest1.x[e_z] << std::endl;

    std::cout << "---omega---" << std::endl;
    std::cout << forceest1.x[omega_x] << std::endl;
    std::cout << forceest1.x[omega_y] << std::endl;
    std::cout << forceest1.x[omega_z] << std::endl;
*/

    std::cout << "---Force---" << std::endl;
    std::cout << forceest1.x[F_x] << std::endl;
    std::cout << forceest1.x[F_y] << std::endl;
    std::cout << forceest1.x[F_z] << std::endl;
    std::cout << forceest1.x[tau_z] << std::endl;
/*
    std::cout << "---angular velocity bias---" << std::endl;
    std::cout << forceest1.x[beta_x] << std::endl;
    std::cout << forceest1.x[beta_y] << std::endl;
    std::cout << forceest1.x[beta_z] << std::endl;
*/

    bias.x = forceest1.x[beta_x];
    bias.y = forceest1.x[beta_y];
    bias.z = forceest1.x[beta_z];
    bias_pub.publish(bias);

    euler.x = forceest1.euler_angle(0);//roll:forceest1.euler_angle(0)
    euler.y = forceest1.euler_angle(1);//pitch:forceest1.euler_angle(1)
    euler.z = forceest1.euler_angle(2);//yaw:forceest1.euler_angle(2)
    euler_pub.publish(euler);

    angular_v.x = drone2_imu.angular_velocity.x;
    angular_v.y = drone2_imu.angular_velocity.y;
    angular_v.z = drone2_imu.angular_velocity.z;

    angular_v_pub.publish(angular_v);
    tf::Quaternion quat_transform_ref(drone2_pose.pose.orientation.x,drone2_pose.pose.orientation.y,drone2_pose.pose.orientation.z,drone2_pose.pose.orientation.w);
    double roll_ref,pitch_ref,yaw_ref;
    tf::Matrix3x3(quat_transform_ref).getRPY(roll_ref,pitch_ref,yaw_ref);
    euler_ref.x = roll_ref*180/3.1415926;//roll_ref*180/3.1415926
    euler_ref.y = pitch_ref*180/3.1415926;//pitch_ref*180/3.1415926
    euler_ref.z = yaw_ref*180/3.1415926;//yaw_ref*180/3.1415926
    euler_ref_pub.publish(euler_ref);


    force.x = forceest1.x[F_x];
    force.y = forceest1.x[F_y];
    force.z = forceest1.x[F_z];

    torque.z = forceest1.x[tau_z];
    torque_pub.publish(torque);
    force_pub.publish(force);

    //ROS_INFO("q_x = %f", forceest1.x[v_x]);
      }
    loop_rate.sleep();
    ros::spinOnce();

   }
}
