#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <lab4/control.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

class ControlSelector final : public rclcpp::Node
{
public:
	ControlSelector();
	~ControlSelector();

	enum ControlEnum {
		DUMMY,
		VOYAGER,
		WALLFOLLOWER,
		nControls
	};

private:
	void selectCallback(const std_msgs::msg::UInt16& msg);
	void laserCallback(const sensor_msgs::msg::LaserScan& msg);
	void poseCallback(const nav_msgs::msg::Odometry& msg);
	void timerCallback();

	Control* controlPtr;
	Control* controls[nControls];

	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;

	rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr selector_sub;
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
	
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Logger logger_;
};