#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

#define NUM_EVAL 1000

std::string platform;

class Subscriber : public rclcpp::Node
{
public:
  Subscriber()
    : Node("mros2_sub"), count_(0)
  {
    RCLCPP_INFO(this->get_logger(), "eval on sub start");
    subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("to_linux", rclcpp::QoS(10).best_effort(), std::bind(&Subscriber::topic_callback, this, _1));
  }

private:
  std::array<float, NUM_EVAL> sublogs;
  std::array<rclcpp::Time, NUM_EVAL> timelogs;
  void topic_callback(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    /* eval point end */
    timelogs[count_] = this->get_clock()->now();

    sublogs[count_] = message->linear.x;
    count_++;

    if (count_ >= NUM_EVAL)
    {
      std::ofstream writing_file;
      std::string filename = "twist_sub.csv";
      std::string filepath = "../results/" + platform + "/" + filename;
      writing_file.open(filepath, std::ios::out);
      if (!writing_file)
      {
        RCLCPP_ERROR(this->get_logger(), "writing file not found!!");
        rclcpp::shutdown();
      }
      for (int i = 0; i < NUM_EVAL; i++)
      {
        const std::string writing_text = std::to_string(sublogs[i]) + "," + std::to_string(timelogs[i].nanoseconds());
        writing_file << writing_text << std::endl;
      }
      RCLCPP_INFO(this->get_logger(), "eval on sub completed");
      rclcpp::shutdown();
    }
  }
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
  uint16_t count_;
};


int main(int argc, char *argv[])
{
  if (argc != 2)
  {
    std::cout << "arguments error: " << argc << std::endl;
    return 1;
  }
  platform = argv[1];

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Subscriber>());
  rclcpp::shutdown();
  return 0;
}