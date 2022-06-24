#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class EchoServer : public rclcpp::Node
{
public:
  EchoServer()
    : Node("echo_ros2")
  {
    RCLCPP_INFO(this->get_logger(), "eval on echo start");
    publisher_ = this->create_publisher<std_msgs::msg::String>("to_linux", 10);
    subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "to_stm", rclcpp::QoS(10).best_effort(),
        std::bind(&EchoServer::topic_callback, this, _1));
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr message)
  {
    publisher_->publish(*message);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EchoServer>());
  rclcpp::shutdown();
  return 0;
}
