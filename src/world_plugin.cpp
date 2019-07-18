#include <gazebo-7/gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <gazebo-7/gazebo/physics/Model.hh>
#include <gazebo-7/gazebo/gazebo.hh>
#include <gazebo-7/gazebo/physics/physics.hh>
#include <gazebo-7/gazebo/common/Events.hh>
#include <string>
#include <iostream>
#include <geometry_msgs/Point.h>

geometry_msgs::Point break_joint, uav2_psi_groundtruth;
void break_joint_cb(const geometry_msgs::Point::ConstPtr& msg){
  break_joint = *msg;
}

namespace gazebo
{
class WorldPluginTutorial : public WorldPlugin
{
public:
  WorldPluginTutorial() : WorldPlugin()
  {
  }

  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
  {
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    this->world = _world;
    this->updateRate = common::Time(0,common::Time::SecToNano(3));
    this->prevUpdateTime = common::Time::GetWallTime();
    this->rosSub = this->rosNode.subscribe<geometry_msgs::Point>("/break_joint", 2, break_joint_cb);
    this->rosPub = this->rosNode.advertise<geometry_msgs::Point>("/theta_groundtruth", 2);
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
               boost::bind(&WorldPluginTutorial::OnUpdate, this, _1));
    //this->iris_model = this->world->GetModel("iris1");
    //this->iris_base_link = this->iris_model->GetLink("iris1::base_link");
  }
  public: void OnUpdate(const common::UpdateInfo & /*_info*/)
  {

    //Ground truth calculation
    if(common::Time::GetWallTime() - this->prevUpdateTime > 15){
      this->iris_model = this->world->GetModel("iris1");
      this->iris_model2 = this->world->GetModel("iris2");
      this->payload_model = this->world->GetModel("payload");
      this->iris_base_link = this->iris_model->GetLink("iris1::base_link");
      this->iris_base_link2 = this->iris_model2->GetLink("iris2::base_link");
      //this->joint_5 = this->payload_model->GetJoint("payload::payload_link_joint3");
      this->payload_g = this->payload_model->GetLink("payload::payload_rec");//_g_box
      this->payload_g1 = this->payload_model->GetLink("payload::payload_rec_g1_box");
      this->payload_g2 = this->payload_model->GetLink("payload::payload_rec_g2_box");
      //For uav1 theta
      double uav1_x_pose = this->iris_base_link->GetWorldPose().pos.x;
      double uav1_z_pose = this->iris_base_link->GetWorldPose().pos.z;
      double g_pos_x = this->payload_g->GetWorldPose().pos.x;
      double g_pos_y = this->payload_g->GetWorldPose().pos.y;
      double g_pos_z = this->payload_g->GetWorldPose().pos.z;
      double uav1_dx = g_pos_x  - uav1_x_pose;
      double uav1_dz = g_pos_z  - uav1_z_pose;
      double uav1_theta_theta = atan2(uav1_dx,-uav1_dz);

      //For uav2 theta
      double uav2_x_pose = this->iris_base_link2->GetWorldPose().pos.x;
      double uav2_z_pose = this->iris_base_link2->GetWorldPose().pos.z;
      double g1_pos_x = this->payload_g1->GetWorldPose().pos.x;
      double g1_pos_y = this->payload_g1->GetWorldPose().pos.y;
      double g1_pos_z = this->payload_g1->GetWorldPose().pos.z;
      double uav2_dx = g1_pos_x  - uav2_x_pose;
      double uav2_dz = g1_pos_z  - uav2_z_pose;
      double uav2_theta = atan2(uav2_dx,-uav2_dz);
      //For uav2 theta_theta

      double uav2_dxx = g_pos_x  - uav2_x_pose;
      double uav2_dzz = g_pos_z  - uav2_z_pose;
      double uav2_theta_theta = atan2(uav2_dxx,-uav2_dzz);
      //For uav2 theta_psi

      double uav2_dxxx = g1_pos_x  - g_pos_x;
      double uav2_dyyy = g1_pos_y  - g_pos_y;
      //double uav2_theta_psi = uav2_theta - uav2_theta_theta;
      double uav2_theta_psi = atan2(uav2_dxxx, uav2_dyyy);

      //For theta
      double ang_x_pose = (uav1_x_pose + uav2_x_pose)/2;
      double ang_z_pose = (uav1_z_pose + uav2_z_pose)/2;
      double payload_dx = g_pos_x - ang_x_pose;
      double payload_dz = g_pos_z - ang_z_pose;
      double payload_theta = atan2(payload_dx, -payload_dz);

      uav2_psi_groundtruth.x = uav2_theta_psi;
      uav2_psi_groundtruth.y = uav2_theta_theta;
      uav2_psi_groundtruth.z = payload_theta;
      this->rosPub.publish(uav2_psi_groundtruth);
      //ROS_INFO("l = %f, l* = %f", y_pos,joint_pos_y);
}

  //Two drones add joints between payload's links and drones
double x;
    if(common::Time::GetWallTime() - this->prevUpdateTime > 10 && add_inv_2){
    this->iris_model = this->world->GetModel("iris1");
    this->iris_base_link = this->iris_model->GetLink("iris1::base_link");
    this->payload_model = this->world->GetModel("payload");
    this->payload_link = this->payload_model->GetLink("payload::payload_link1_box");
    this->joint_ = this->world->GetPhysicsEngine()->CreateJoint("revolute2", this->iris_model);
    this->joint_->Load(this->iris_base_link, this->payload_link, math::Pose(0,0,0.0,0,0,0));
    this->joint_->Attach(this->iris_base_link,this->payload_link);
    ignition::math::Vector3d joint_axis(0,1,0), joint_axis2(1,0,0);
    this->joint_->SetAxis(0, joint_axis);
    this->joint_->SetAxis(1, joint_axis2);
    this->joint_->SetName("payload_drone_joint");
    ROS_INFO("add joint1");

    this->iris_model2 = this->world->GetModel("iris2");
    this->iris_base_link2 = this->iris_model2->GetLink("iris2::base_link");
    this->payload_model2 = this->world->GetModel("payload");
    this->payload_link2 = this->payload_model2->GetLink("payload::payload_link2_box");
    this->joint_2 = this->world->GetPhysicsEngine()->CreateJoint("revolute2", this->iris_model2);
    this->joint_2->Load(this->iris_base_link2, this->payload_link2, math::Pose(0,0.0,0,0,0,0));
    this->joint_2->Attach(this->iris_base_link2,this->payload_link2);
    this->joint_2->SetAxis(0, joint_axis);
    this->joint_2->SetAxis(1, joint_axis2);
    this->joint_2->SetName("payload_drone_joint2");
    ROS_INFO("add joint2");
     add_inv_2 = false;

    //For single drones, break joint
/*
if(break_joint.x == 1){
    this->iris_model3 = this->world->GetModel("iris_rplidar");

    this->joint_3 = this->iris_model3->GetJoint("iris_rplidar::payload::payload_link_joint");
    this->joint_3->Reset();
    this->joint_3->Detach();
    this->joint_3->Fini();
    ROS_INFO("theta = %f", break_joint.y);
    ROS_INFO("theta measurement = %f", break_joint.z);
    ROS_INFO("remove joint");
   // this->world->SetPaused(true);

    add_inv_2 = false;

}
*/
ros::spinOnce();
}

  }
    private: physics::WorldPtr world;

    private: event::ConnectionPtr updateConnection;
    common::Time updateRate;
    common::Time prevUpdateTime;
    //For double drones
    private: physics::JointPtr joint_, joint_2;
    private: physics::ModelPtr iris_model, iris_model2;
    private: physics::ModelPtr payload_model, payload_model2;
    private: physics::LinkPtr iris_base_link, iris_base_link2;
    private: physics::LinkPtr payload_link, payload_link2, payload, payload_box2, payload_g, payload_g1, payload_g2;
    bool add_inv = true;
    bool add_inv_2 = true;
    //For single drones
    private: physics::JointPtr joint_3, joint_4, joint_5;
    private: physics::ModelPtr iris_model3;
    private: ros::NodeHandle rosNode;
    private: ros::Subscriber rosSub;
    private: ros::Publisher rosPub;



};
GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}
