#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16.hpp"

using namespace std::chrono_literals;

#define NUM_EVAL 1000+1
#define EVAL_INTERVAL 100ms

std::string platform;

class Publisher : public rclcpp::Node
{
public:
  Publisher()
    : Node("pub_mros2"), count_(0)
  {
    RCLCPP_INFO(this->get_logger(), "eval on pub start");
    publisher_ = this->create_publisher<std_msgs::msg::UInt16>("to_stm", 10);
    timer_ = this->create_wall_timer(EVAL_INTERVAL, std::bind(&Publisher::timer_callback, this));
  }

private:
  std::array<std::uint16_t, NUM_EVAL> publogs;
  std::array<rclcpp::Time, NUM_EVAL> timelogs;
  void timer_callback()
  {
    if (count_ < NUM_EVAL)
    {
      auto message = std_msgs::msg::UInt16();
      message.data = rand() % 128;
      publogs[count_] = message.data;

      /* eval point start */
      timelogs[count_] = this->get_clock()->now();
      publisher_->publish(message);

      count_++;
    }
    else
    {
      std::ofstream writing_file;
      std::string filename = "uint16_pub.csv";
      std::string filepath = "../results/" + platform + "/" + filename;
      writing_file.open(filepath, std::ios::out);
      if (!writing_file)
      {
        RCLCPP_ERROR(this->get_logger(), "writing file not found!!");
        rclcpp::shutdown();
      }
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
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr publisher_;
  uint16_t count_;
};

int main(int argc, char * argv[])
{
  if (argc != 2)
  {
    std::cout << "arguments error: " << argc << std::endl;
    return 1;
  }
  platform = argv[1];
  srand(time(NULL));

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Publisher>());
  rclcpp::shutdown();
  return 0;
}
