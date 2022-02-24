#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

#define NUM_EVAL 10+1
#define EVAL_INTERVAL 1000ms
#define LEN_STR   1

class Random_Msg
{
public:
  static std::string get_random(const int len)
  {
    static const char alphanum[] =
        "0123456789"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz";
    srand(time(NULL));
    std::string random_string;
    for (int i = 0; i < len; ++i)
    {
      random_string += alphanum[rand() % (sizeof(alphanum) - 1)];
    }
    return random_string;
  }
};

class Publisher : public rclcpp::Node
{
public:
  Publisher()
    : Node("pub_mros2"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("to_stm", 10);
    timer_ = this->create_wall_timer(EVAL_INTERVAL, std::bind(&Publisher::timer_callback, this));
  }

private:
  std::array<std::string, NUM_EVAL> publogs;
  std::array<rclcpp::Time, NUM_EVAL> timelogs;
  void timer_callback()
  {
    if (count_ < NUM_EVAL)
    {
      auto message = std_msgs::msg::String();
      message.data = Random_Msg::get_random(LEN_STR);
      publogs[count_] = message.data;

      /* eval point start */
      timelogs[count_] = this->get_clock()->now();
      publisher_->publish(message);

      count_++;
    }
    else
    {
      std::ofstream writing_file;
      std::string filename = "../results/string" + std::to_string(LEN_STR) + "_publog.csv";
      writing_file.open(filename, std::ios::out);
      for (int i = 0; i < NUM_EVAL; i++)
      {
        const std::string writing_text = publogs[i] + "," + std::to_string(timelogs[i].nanoseconds());
        writing_file << writing_text << std::endl;
      }
      RCLCPP_INFO(this->get_logger(), "eval on pub completed");
      rclcpp::shutdown();
    }
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  uint16_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Publisher>());
  rclcpp::shutdown();
  return 0;
}
