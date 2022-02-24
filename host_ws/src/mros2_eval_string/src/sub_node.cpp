#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

#define NUM_EVAL 10
#define LEN_STR   1

class Subscriber : public rclcpp::Node
{
public:
  Subscriber()
    : Node("mros2_sub"), count_(0)
  {
    subscriber_ = this->create_subscription<std_msgs::msg::String>("to_linux", rclcpp::QoS(10).best_effort(), std::bind(&Subscriber::topic_callback, this, _1));
  }

private:
  std::array<std::string, NUM_EVAL> sublogs;
  std::array<rclcpp::Time, NUM_EVAL> timelogs;
  void topic_callback(const std_msgs::msg::String::SharedPtr message)
  {
    /* eval point end */
    timelogs[count_] = this->get_clock()->now();

    sublogs[count_] = message->data;
    count_++;

    if (count_ >= NUM_EVAL)
    {
      std::ofstream writing_file;
      std::string filename = "../results/string" + std::to_string(LEN_STR) + "_sublog.txt";
      writing_file.open(filename, std::ios::out);
      for (int i = 0; i < NUM_EVAL; i++)
      {
        const std::string writing_text = sublogs[i] + ":" + std::to_string(timelogs[i].nanoseconds());
        writing_file << writing_text << std::endl;
      }
      RCLCPP_INFO(this->get_logger(), "eval on sub completed");
      rclcpp::shutdown();
    }
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
  uint16_t count_;
};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Subscriber>());
  rclcpp::shutdown();
  return 0;
}