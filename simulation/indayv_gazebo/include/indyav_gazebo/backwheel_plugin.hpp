#pragma once

#include <rclcpp/rclcpp.hpp>
#include "indyav_interfaces/msg/revs_stamped.hpp"
#include <gazebo/common/Event.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <sdf/sdf.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo/physics/physics.hh>
//#include <indyav_gazebo/wheel_plugin.hpp>

namespace gazebo
{
class BackWheelPlugin : public ModelPlugin
{
public:
  BackWheelPlugin();
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void Init();
  void Callback(const indyav_interfaces::msg::RevsStamped::SharedPtr _msg);
  void OnUpdate();
  gazebo_ros::Node::SharedPtr rosnode;

protected:
  std::vector<std::string> wheel_names_;
  physics::ModelPtr model_;
  rclcpp::Subscription<indyav_interfaces::msg::RevsStamped>::SharedPtr sub_;
  event::ConnectionPtr updateConnection_;

  std::string back_axle_joint_name_;
  physics::JointPtr back_axle_joint_;

  double wheel_rotational_vel_ = 0.0;
  double max_wheel_rotational_vel_ = 0.0;
  double max_velocity_ = 0.0;
};

GZ_REGISTER_MODEL_PLUGIN(BackWheelPlugin)
}
