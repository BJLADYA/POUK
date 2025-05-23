#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class Listener : public rclcpp::Node
{
public:
  Listener() : Node("listener")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
		"chat", 
		10, 
		[this](const std_msgs::msg::String & msg)
		{
			topic_callback(msg);
		});
		
    pub_ = this->create_publisher<std_msgs::msg::String>("reply", 10);
  }

private:
  void topic_callback(const std_msgs::msg::String & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());

    auto reply = std_msgs::msg::String();
    reply.data = "Recieved by listener: " + msg.data;
    pub_->publish(reply);
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Listener>());
  rclcpp::shutdown();
  return 0;
}
