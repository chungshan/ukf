#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Imu.h>
geometry_msgs::Point acc_bias, acc, omega, px4_acc;
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
int main(int argc, char **argv)
{
  ros::init(argc, argv, "plot");
  ros::NodeHandle nh;
  ros::Subscriber acc_bias_sub = nh.subscribe<geometry_msgs::Point>("/vins_estimator/acc_bias", 2, acc_bias_cb);
  ros::Subscriber acc_sub = nh.subscribe<sensor_msgs::Imu>("/dji_sdk/imu", 2, imu_cb);
  ros::Subscriber px4_acc_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data_raw", 2, px4_imu_cb);
  ros::Publisher acc_pub = nh.advertise<geometry_msgs::Point>("/acc", 2);
  ros::Publisher px4_acc_pub = nh.advertise<geometry_msgs::Point>("/acc_px4", 2);
  ros::Publisher omega_pub = nh.advertise<geometry_msgs::Point>("/omega", 2);
  ros::Rate loop_rate(20);
  while(ros::ok()){
    acc.x = imu.linear_acceleration.x;
    acc.y = imu.linear_acceleration.y;
    acc.z = imu.linear_acceleration.z;
    omega.x = imu.angular_velocity.x;
    omega.y = imu.angular_velocity.y;
    omega.z = imu.angular_velocity.z;
    px4_acc.x = px4_imu.linear_acceleration.x;
    px4_acc.y = px4_imu.linear_acceleration.y;
    px4_acc.z = px4_imu.linear_acceleration.z;
    acc_pub.publish(acc);
    omega_pub.publish(omega);
    px4_acc_pub.publish(px4_acc);
    loop_rate.sleep();
    ros::spinOnce();
  }
}
