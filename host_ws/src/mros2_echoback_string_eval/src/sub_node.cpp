#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class Subscriber : public rclcpp::Node
{
public:
  uint32_t count_ = 0;
  Subscriber() : Node("mros2_sub")
  {
    subscriber_ = this->create_subscription<std_msgs::msg::String>("to_linux", rclcpp::QoS(10).best_effort(), std::bind(&Subscriber::topic_callback, this, _1));
  }

private:
  std::array<rclcpp::Time, 200> sublogs;
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    rclcpp::Time now = this->get_clock()->now();
    if (count_ < 200){
      sublogs[count_] = now;
      auto subscribed_message = std_msgs::msg::String();
      subscribed_message = *msg;
      count_++;
    } else {
      std::ofstream writing_file;
      std::string filename = "string_sublog.txt";
      writing_file.open(filename, std::ios::out);
      for (int i=0; i<200; i++){
        const std::string writing_text = std::to_string(sublogs[i].nanoseconds());
        writing_file << writing_text << std::endl;
      }
    }
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Subscriber>());
  rclcpp::shutdown();
  return 0;
}