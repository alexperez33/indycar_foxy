
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <indyav_interfaces/msg/revs_stamped.hpp>
#include <indyav_interfaces/msg/steering_stamped.hpp>

#include <sstream>
#include <string>

class JoyTeleop : public rclcpp::Node
{

public:
  JoyTeleop() : Node("joy_teleop"){
    
    this->declare_parameter<int>("axis_steer", 0);
    this->declare_parameter<int>("axis_throttle", 0);
    this->declare_parameter<double>("scale_steer", 0.0);
    this->declare_parameter<double>("scale_trigger", 0.0);
    this->declare_parameter<std::string>("throttle_topic", "");
    this->declare_parameter<std::string>("steering_topic", "");

    this->get_parameter<int>("axis_steer", steer_axis_);
    this->get_parameter<int>("axis_throttle", throttle_axis_);
    this->get_parameter<double>("scale_steer", s_scale_);
    this->get_parameter<double>("scale_trigger", r_scale_);
    this->get_parameter<std::string>("throttle_topic", t_advertise_);
    this->get_parameter<std::string>("steering_topic", s_advertise_);

    std::cout << t_advertise_ << std::endl;


    //create publishers and subscribers
    throttle_pub_ = this->create_publisher<indyav_interfaces::msg::RevsStamped>(t_advertise_, 1);
    steering_pub_ = this->create_publisher<indyav_interfaces::msg::SteeringStamped>(s_advertise_, 1);
    joy_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 100, 
          std::bind(&JoyTeleop::callback, this, std::placeholders::_1));
  }

private:
  void callback(const sensor_msgs::msg::Joy::SharedPtr _joy){
    static indyav_interfaces::msg::RevsStamped radians_per_second;
    static indyav_interfaces::msg::SteeringStamped steering_angle;

    steering_angle.steering_angle = s_scale_ * _joy->axes.at(steer_axis_);

    // Gamepad trigger incorrectly defaults to 0 when resting position is 1. Moving the trigger updates the value,
    // correcting the error. To seamlessly work around the problem, the angular velocity is not assigned until this change
    // is detected.
    static bool gamepad_init = false;
    if (_joy->axes.at(throttle_axis_) != 0)
      gamepad_init = true;
    if (gamepad_init)
      radians_per_second.radians_per_second = r_scale_ * (abs(_joy->axes.at(throttle_axis_) - 1)) / 2;

    steering_pub_->publish(steering_angle);
    throttle_pub_->publish(radians_per_second);
  }


  int steer_axis_, throttle_axis_;
  double s_scale_, r_scale_;
  std::string t_advertise_, s_advertise_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_;
  rclcpp::Publisher<indyav_interfaces::msg::SteeringStamped>::SharedPtr steering_pub_;
  rclcpp::Publisher<indyav_interfaces::msg::RevsStamped>::SharedPtr throttle_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JoyTeleop>();
  rclcpp::spin(node);
  rclcpp::shutdown();
}