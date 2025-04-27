#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <cstdlib>
#include <iostream>

class Selector final : public rclcpp::Node
{
public:
	Selector(): 
		Node("selector_client") 
	{
		selector_pub = this->create_publisher<std_msgs::msg::UInt16>("/selector", 10);

		controlSelect();
	}

private:
	void controlSelect() {
		int control = -1;
		std_msgs::msg::UInt16 msg;

		std::cout << "\033[2J\033[1;1H";

		while (rclcpp::ok()) {
			std::cout << "Select control mode:\n";

			if (control == 1) {
				std::cout << "1. Dummy\t<\n";
			} else {
				std::cout << "1. Dummy\n";
			}

			if (control == 2) {
				std::cout << "2. Voyager\t<\n";
			} else {
				std::cout << "2. Voyager\n";
			}

			if (control == 3) {
				std::cout << "3. WallFollower\t<\n";
			} else {
				std::cout << "3. WallFollower\n";
			}
			
			std::cout << "> ";

			std::cin >> control;

			msg.data = control - 1;
			selector_pub->publish(msg);

			std::cout << "\033[2J\033[1;1H";
		}
	}

	rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr selector_pub;
};

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Selector>());
	rclcpp::shutdown();

	return 0;
}