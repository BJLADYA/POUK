#include <rclcpp/rclcpp.hpp>
#include <lab3/line_control.h>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineControl>());
    rclcpp::shutdown();
    return 0;
}