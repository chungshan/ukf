#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <tf/transform_datatypes.h>
geometry_msgs::Point acc_bias, acc, omega, px4_acc, att_v;
void acc_bias_cb(const geometry_msgs::Point::ConstPtr &msg){
  acc_bias = *msg;
}
sensor_msgs::Imu imu;
void imu_cb(const sensor_msgs::Imu::ConstPtr &msg){
  imu = *msg;
}
sensor_msgs::Imu px4_imu;
void px4_imu_cb(const sensor_msgs::Imu::ConstPtr &msg){
  px4_imu = *msg;
}
mavros_msgs::AttitudeTarget att_;
void att_cb(const mavros_msgs::AttitudeTarget::ConstPtr &msg){
  att_ = *msg;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "plot");
  ros::NodeHandle nh;
  ros::Subscriber acc_bias_sub = nh.subscribe<geometry_msgs::Point>("/vins_estimator/acc_bias", 2, acc_bias_cb);
  ros::Subscriber acc_sub = nh.subscribe<sensor_msgs::Imu>("/dji_sdk/imu", 2, imu_cb);
  ros::Subscriber px4_acc_sub = nh.subscribe<sensor_msgs::Imu>("/drone3/mavros/imu/data", 2, px4_imu_cb);
  ros::Subscriber attitude_sub = nh.subscribe<mavros_msgs::AttitudeTarget>("/drone3/mavros/setpoint_raw/attitude", 2, att_cb);
  ros::Publisher acc_pub = nh.advertise<geometry_msgs::Point>("/acc", 2);
  ros::Publisher px4_acc_pub = nh.advertise<geometry_msgs::Point>("/acc_px4", 2);
  ros::Publisher omega_pub = nh.advertise<geometry_msgs::Point>("/omega", 2);
  ros::Publisher att_pub = nh.advertise<geometry_msgs::Point>("/att", 2);
  ros::Rate loop_rate(20);
  while(ros::ok()){
    acc.x = imu.linear_acceleration.x;
    acc.y = imu.linear_acceleration.y;
    acc.z = imu.linear_acceleration.z;
    omega.x = px4_imu.angular_velocity.x;
    omega.y = px4_imu.angular_velocity.y;
    omega.z = px4_imu.angular_velocity.z;
    px4_acc.x = px4_imu.linear_acceleration.x;
    px4_acc.y = px4_imu.linear_acceleration.y;
    px4_acc.z = px4_imu.linear_acceleration.z;
    tf::Quaternion quat_trans(att_.orientation.x,att_.orientation.y,att_.orientation.z,att_.orientation.w);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat_trans).getRPY(roll,pitch,yaw);
    att_v.x = roll;
    att_v.y = pitch;
    att_v.z = yaw;
    att_pub.publish(att_v);
    acc_pub.publish(acc);
    omega_pub.publish(omega);
    px4_acc_pub.publish(px4_acc);
    loop_rate.sleep();
    ros::spinOnce();
  }
}
