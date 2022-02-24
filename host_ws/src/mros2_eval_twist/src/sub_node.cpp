#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class Subscriber : public rclcpp::Node
{
public:
  uint32_t count_ = 0;
  Subscriber() : Node("mros2_sub")
  {
    subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("to_linux", rclcpp::QoS(10).best_effort(), std::bind(&Subscriber::topic_callback, this, _1));
  }

private:
  std::array<rclcpp::Time, 200> sublogs;
  void topic_callback(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    if (count_ < 200){
      auto subscribed_message = geometry_msgs::msg::Twist();
      subscribed_message = *message;
      sublogs[count_] = this->get_clock()->now();
      count_++;
    } else {
      std::ofstream writing_file;
      std::string filename = "twist_sublog.txt";
      writing_file.open(filename, std::ios::out);
      for (int i=0; i<200; i++){
        const std::string writing_text = std::to_string(sublogs[i].nanoseconds());
        writing_file << writing_text << std::endl;
      }
    }
  }
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Subscriber>());
  rclcpp::shutdown();
  return 0;
}