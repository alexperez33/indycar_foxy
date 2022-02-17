#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include "indyav_interfaces/msg/steering_stamped.hpp"

class SimulatedSteeringDriver : public rclcpp::Node
{

public:
  SimulatedSteeringDriver() : Node("simulated_steering_driver"){
    //get parameters
    //this->declare_parameter<std::string>("input_topic", ""); //default topic
    //this->get_parameter<std::string>("input_topic", input_topic);
    input_topic = "/steering";

    sub_ = this->create_subscription<indyav_interfaces::msg::SteeringStamped>(input_topic, 10, 
          std::bind(&SimulatedSteeringDriver::callback, this, std::placeholders::_1));

    // get all the controllers from the ros params
    std::string name = "/car/simulated_hardware_controllers/steering";

    auto left_name = name + "/left";
    pubs_[left_name] = this->create_publisher<std_msgs::msg::Float64>(left_name + "/command", 5);

    auto right_name = name + "/right";
    pubs_[right_name] = this->create_publisher<std_msgs::msg::Float64>(right_name + "/command", 5);
  }

private:
  void callback(const indyav_interfaces::msg::SteeringStamped::SharedPtr _msg)
  {
    double cmd_angle = _msg->steering_angle;
    for (auto i = pubs_.begin(); i != pubs_.end(); ++i)
    {
      // TODO: implement ackerman steering angles
      std_msgs::msg::Float64 msg;
      msg.data = cmd_angle;
      i->second->publish(msg);
    }
  }

  std::string input_topic;
  rclcpp::Subscription<indyav_interfaces::msg::SteeringStamped>::SharedPtr sub_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> pubs_;

};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimulatedSteeringDriver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
}
