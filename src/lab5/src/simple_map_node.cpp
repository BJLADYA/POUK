#include <rclcpp/rclcpp.hpp>
#include <lab5/mapper.h>

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Mapper>());
	rclcpp::shutdown();

	return 0;
}
