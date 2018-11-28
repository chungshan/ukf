#include <ros/ros.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/RCOut.h>
mavros_msgs::ActuatorControl rc_channel;
mavros_msgs::OverrideRCIn rc_over;
mavros_msgs::State current_state;
mavros_msgs::RCOut rc_out;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
  current_state = *msg;
}
void rc_cb(const mavros_msgs::RCOut::ConstPtr& msg){
  rc_out = *msg;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "control_rc");
  ros::NodeHandle nh;
  ros::Publisher rc_pub = nh.advertise<mavros_msgs::ActuatorControl>("/drone3/mavros/actuator_control", 2);
  ros::Publisher rc_pub1 = nh.advertise<mavros_msgs::OverrideRCIn>("/drone3/mavros/rc/override", 2);
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/drone3/mavros/state", 2, state_cb);
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                                       ("drone3/mavros/set_mode");

  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                                     ("drone3/mavros/cmd/arming");

  ros::Subscriber rc_sub = nh.subscribe<mavros_msgs::RCOut>("/drone3/mavros/rc/out", 2, rc_cb);
  ros::Rate loop_rate(50);

  double sum;
  int count = 1;
  double average;
  while(ros::ok()){

    double rc_value;

    if(rc_out.channels.size()!=0 && rc_out.channels[0] != 0){
      rc_value =rc_out.channels[0];
      sum = sum + rc_value;

      average = sum / count;
      count += 1;
    }



    ROS_INFO("channel 1 = %f", rc_value);
    ROS_INFO("average = %f", average);
    ros::spinOnce();
    loop_rate.sleep();


  }

  return 0;
}
