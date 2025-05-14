#include <rclcpp/rclcpp.hpp>
#include <lab6/patrol_bot.h>

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<PatrolBot>());
	rclcpp::shutdown();
	return 0;
}

