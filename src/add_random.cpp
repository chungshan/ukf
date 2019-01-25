#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <iterator>
#include <random>
// add random noise to pose
geometry_msgs::PoseStamped pose,pose_random;
geometry_msgs::Point pose_v;
float RandomFloat(float a, float b) {
    float random = ((float) rand()) / (float) RAND_MAX;
    float diff = b - a;
    float r = random * diff;
    return a + r;
}
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
  pose = *msg;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_random");
  ros::NodeHandle nh;
  ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/RigidBody3/pose", 2, pose_cb);
  ros::Publisher drone_pub = nh.advertise<geometry_msgs::PoseStamped>("/drone3/mavros/vision_pose/pose",2);
  ros::Publisher poser_pub = nh.advertise<geometry_msgs::Point>("/pose_random",2);
  ros::Publisher pose_pub = nh.advertise<geometry_msgs::Point>("/pose",2);
  ros::Rate rate(30);
  const double mean = 0.0;
  const double stddev = 0.1;
  std::default_random_engine generatorx,generatory,generatorz;
  std::normal_distribution<double> distx(mean,stddev);
  std::normal_distribution<double> disty(mean,stddev);
  std::normal_distribution<double> distz(mean,stddev);
  while (ros::ok()) {
    if(pose.pose.position.x != 0){
    float rand_value_x, rand_value_y, rand_value_z;


    rand_value_x = RandomFloat(-0.1,0.1);
    rand_value_y = RandomFloat(-0.1,0.1);
    
    pose_random.header.stamp = pose.header.stamp;
    pose_random.pose.position.x = pose.pose.position.x + distx(generatorx);
    pose_random.pose.position.y = pose.pose.position.y + disty(generatory);
    pose_random.pose.position.z = pose.pose.position.z + distz(generatorz);

    pose_random.pose.orientation.x = pose.pose.orientation.x;
    pose_random.pose.orientation.y = pose.pose.orientation.y;
    pose_random.pose.orientation.z = pose.pose.orientation.z;
    pose_random.pose.orientation.w = pose.pose.orientation.w;

    pose_v.x = pose.pose.position.x;
    pose_v.y = pose.pose.position.y;
    pose_v.z = pose.pose.position.z;
    //poser_pub.publish(pose_random);
    pose_pub.publish(pose_v);
    drone_pub.publish(pose_random);
    }
    ros::spinOnce();
    rate.sleep();
  }


  return 0;

}
