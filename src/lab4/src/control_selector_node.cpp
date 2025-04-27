#include <rclcpp/rclcpp.hpp>
#include <lab4/control_selector.hpp>

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ControlSelector>());
	rclcpp::shutdown();

	return 0;
}