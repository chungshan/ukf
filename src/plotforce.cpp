#include <ros/ros.h>
#include <UKF/output.h>
#include <geometry_msgs/Point.h>
geometry_msgs::Point leader, follower;
UKF::output leader_force,follower_force;
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
  ros::Subscriber force_sub = nh.subscribe<UKF::output>("/leader_ukf/output", 1, force_cb);
  ros::Subscriber force_sub2 = nh.subscribe<UKF::output>("/follower_ukf/output", 1, force_cb2);
  ros::Publisher leader_force_pub = nh.advertise<geometry_msgs::Point>("/leader",1);
  ros::Publisher follower_force_pub = nh.advertise<geometry_msgs::Point>("/follower",1);
  ros::Rate rate(50);
  while (ros::ok()) {
    leader.x = leader_force.force.x;
    leader.y = leader_force.force.y;
    leader.z = leader_force.force.z;
    follower.x = follower_force.force.x;
    follower.y = follower_force.force.y;
    follower.z = follower_force.force.z;
    leader_force_pub.publish(leader);
    follower_force_pub.publish(follower);
    ros::spinOnce();
    rate.sleep();
  }

}
