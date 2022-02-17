#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2/LinearMath/Quaternion.h>

// TODO: remove this code when p3d in gazebo is replaced with the sylphase xacro

class DumbTruthOdom : public rclcpp::Node
{

public:
  DumbTruthOdom() : Node("dumb_truth_odom_tf")
  {
     this->declare_parameter<std::string>("topic_name", "/ins_odom"); //default topic
     this->get_parameter<std::string>("topic_name", topic_name);
     sub_ = this->create_subscription<nav_msgs::msg::Odometry>(topic_name, 10, std::bind(&DumbTruthOdom::callback, this, std::placeholders::_1));
     tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }

private:
  void callback(const nav_msgs::msg::Odometry::SharedPtr _msg)
  {  
     geometry_msgs::msg::TransformStamped transformStamped; 
     transformStamped.header.stamp = rclcpp::Time();
     transformStamped.header.frame_id = _msg->header.frame_id;
     transformStamped.child_frame_id = _msg->child_frame_id;
     transformStamped.transform.translation.x = _msg->pose.pose.position.x;
     transformStamped.transform.translation.y = _msg->pose.pose.position.y;
     transformStamped.transform.translation.z = _msg->pose.pose.position.z;
     transformStamped.transform.rotation.x = _msg->pose.pose.orientation.x;
     transformStamped.transform.rotation.y = _msg->pose.pose.orientation.y;
     transformStamped.transform.rotation.z = _msg->pose.pose.orientation.z;
     transformStamped.transform.rotation.w = _msg->pose.pose.orientation.w;
     tf_broadcaster_->sendTransform(transformStamped);
  }
  
  std::string topic_name;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DumbTruthOdom>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
