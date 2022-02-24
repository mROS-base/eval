#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

#define NUM_EVAL 10+1
#define EVAL_INTERVAL 1000ms

class Publisher : public rclcpp::Node
{
public:
  Publisher()
    : Node("pub_mros2"), count_(0)
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("to_stm", 10);
    timer_ = this->create_wall_timer(EVAL_INTERVAL, std::bind(&Publisher::timer_callback, this));
  }

private:
  std::array<float, NUM_EVAL> publogs;
  std::array<rclcpp::Time, NUM_EVAL> timelogs;
  void timer_callback()
  {
    if (count_ < NUM_EVAL)
    {
      auto message = geometry_msgs::msg::Twist();
      std::srand(std::time(nullptr));
      message.linear.x = (float)(rand()) / ((float)(RAND_MAX / 128));
      message.linear.y = (float)(rand()) / ((float)(RAND_MAX / 128));
      message.linear.z = (float)(rand()) / ((float)(RAND_MAX / 128));
      message.angular.x = (float)(rand()) / ((float)(RAND_MAX / 128));
      message.angular.y = (float)(rand()) / ((float)(RAND_MAX / 128));
      message.angular.z = (float)(rand()) / ((float)(RAND_MAX / 128));
      publogs[count_] = message.linear.x;

      /* eval point start */
      timelogs[count_] = this->get_clock()->now();
      publisher_->publish(message);

      count_++;
    }
    else
    {
      std::ofstream writing_file;
      std::string filename = "../results/twist_publog.csv";
      writing_file.open(filename, std::ios::out);
      for (int i = 0; i < NUM_EVAL; i++)
      {
        const std::string writing_text = std::to_string(publogs[i]) + "," + std::to_string(timelogs[i].nanoseconds());
        writing_file << writing_text << std::endl;
      }
      RCLCPP_INFO(this->get_logger(), "eval on pub completed");
      rclcpp::shutdown();
    }
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  uint16_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Publisher>());
  rclcpp::shutdown();
  return 0;
}
