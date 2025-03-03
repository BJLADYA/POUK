#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

using std::placeholders::_1;

class Talker : public rclcpp::Node
{
public:
  Talker() : Node("talker"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("chat", 10);
    sub_ = this->create_subscription<std_msgs::msg::String>("reply", 10, std::bind(&Talker::reply_cb, this, _1));
    timer_ = this->create_wall_timer(500ms, std::bind(&Talker::timer_callback, this));
  }

private:
  void reply_cb(const std_msgs::msg::String & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Reply from listener: %s", msg.data.c_str());
  }

  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Talker>());
  rclcpp::shutdown();
  return 0;
}
