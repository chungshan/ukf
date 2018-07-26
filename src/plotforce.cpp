#include <ros/ros.h>
#include <UKF/output.h>
#include <geometry_msgs/Point.h>
#include "lpf2.h"
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
geometry_msgs::Point leader, follower, leader_force_estimate, vel, pose_error;
UKF::output leader_force,follower_force;
geometry_msgs::TwistStamped drone2_vel;
geometry_msgs::PoseStamped drone2_pos, drone3_pos;
sensor_msgs::Imu imu2;
void mocap_cb2(const geometry_msgs::PoseStamped::ConstPtr &msg){
  drone2_pos = *msg;
}

void mocap_cb3(const geometry_msgs::PoseStamped::ConstPtr &msg){
  drone3_pos = *msg;
}
void force_cb1(const geometry_msgs::Point::ConstPtr &msg){
  leader_force_estimate = *msg;
}
void vel_cb1(const geometry_msgs::TwistStamped::ConstPtr &msg){
  drone2_vel = *msg;
}
void force_cb(const UKF::output::ConstPtr &msg){
  leader_force = *msg;
}
void force_cb2(const UKF::output::ConstPtr &msg){
  follower_force = *msg;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "plotforce");
  ros::NodeHandle nh;
  ros::Subscriber force_sub = nh.subscribe<UKF::output>("/follower_ukf/output", 1, force_cb2);
  ros::Subscriber force_sub2 = nh.subscribe<geometry_msgs::Point>("/leader_force", 1, force_cb1);
  ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/drone2/mavros/setpoint_velocity/cmd_vel", 1, vel_cb1);
  ros::Subscriber mocap_sub2 = nh.subscribe<geometry_msgs::PoseStamped>("/drone2/mavros/local_position/pose", 1, mocap_cb2);
  ros::Subscriber mocap_sub3 = nh.subscribe<geometry_msgs::PoseStamped>("/drone3/mavros/local_position/pose", 1, mocap_cb3);
  ros::Publisher leader_force_pub = nh.advertise<geometry_msgs::Point>("/leader_filtered",1);
  ros::Publisher follower_vel_pub = nh.advertise<geometry_msgs::Point>("/follower_vel",1);
  ros::Publisher follower_force_pub = nh.advertise<geometry_msgs::Point>("/follower_force",1);
  ros::Publisher pose_error_pub = nh.advertise<geometry_msgs::Point>("/pose_error",1);

  //ros::Publisher follower_force_pub = nh.advertise<geometry_msgs::Point>("/leader_filtered",1);
  ros::Rate rate(50);
  double filter_;
  lpf2  lpf2(6,0.02);
  while (ros::ok()) {
    double pose_error_x;
    pose_error_x = drone3_pos.pose.position.x - drone2_pos.pose.position.x;
    pose_error.x = pose_error_x;
    pose_error_pub.publish(pose_error);
    //leader.x = leader_force.force.x;
    //leader.y = leader_force.force.y;
    //leader.z = leader_force.force.z;
    follower.x = -follower_force.force.x;
    //follower.y = follower_force.force.y;
    //follower.z = follower_force.force.z;
    vel.x = drone2_vel.twist.linear.x;
    filter_ = lpf2.filter(leader_force_estimate.x);
    //ROS_INFO("filter = %f", filter_);
    //leader.x = filter_;
    leader.x = leader_force_estimate.x;
    leader.z = leader_force_estimate.z;
    leader_force_pub.publish(leader);
    //follower_vel_pub.publish(vel);
    //follower_force_pub.publish(follower);
    ros::spinOnce();
    rate.sleep();
  }

}
