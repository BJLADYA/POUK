#include <lab4/control_selector.hpp>
#include <lab4/dummy_control.hpp>
#include <lab4/voyager_control.hpp>
#include <lab4/wallfollower_control.hpp>

ControlSelector::ControlSelector()
:	Node("control_selector"),
	controlPtr(nullptr),
	logger_(this->get_logger())
{
	cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

	selector_sub = this->create_subscription<std_msgs::msg::UInt16>(
		"/selector",
		10,
		[this] (const std_msgs::msg::UInt16 msg) {
			this->selectCallback(msg);
		}
	);

	laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
		"/scan",
		10,
		[this](const sensor_msgs::msg::LaserScan msg) {
			this->laserCallback(msg);
		}
	);

	odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
		"/odom",
		10,
		[this](const nav_msgs::msg::Odometry msg) {
			this->poseCallback(msg);
		}
	);

	timer_ = this->create_wall_timer(
		std::chrono::milliseconds(100),
		[this]() {
			this->timerCallback();
		}
	);

	double min_range = this->declare_parameter("min_range", 1.0);
	double max_vel = this->declare_parameter("max_vel", 0.5);
	double max_omega = this->declare_parameter("max_omega", 0.5);

	double wall_offset = this->declare_parameter("wall_offset", 0.7);

	controls[DUMMY] = new DummyControl(logger_);
	controls[VOYAGER] = new VoyagerControl(
		logger_,
		min_range,
		max_vel,
		max_omega
	);
	controls[WALLFOLLOWER] = new WallfollowerControl(
		logger_,
		wall_offset
	);

	controlPtr = controls[VOYAGER];
}

ControlSelector::~ControlSelector()
{
	for (int i = 0; i < nControls; i++) {
		delete controls[i];
	}
}

void ControlSelector::selectCallback(const std_msgs::msg::UInt16 &msg)
{
	if (msg.data >= nControls) {
		RCLCPP_ERROR_STREAM(logger_, "Wrong algorithm number" << msg.data);
		controlPtr = nullptr;
	}
	else {
		controlPtr = controls[msg.data];
		RCLCPP_INFO_STREAM(logger_, "Select " << controlPtr->getName() << " control");
	}
}

void ControlSelector::laserCallback(const sensor_msgs::msg::LaserScan &msg)
{
	if (controlPtr) {
		controlPtr->setLaserData(msg.ranges);
	}
}

void ControlSelector::poseCallback(const nav_msgs::msg::Odometry &msg)
{
	double x = msg.pose.pose.position.x;
	double y = msg.pose.pose.position.y;
	double theta = 2*atan2(
		msg.pose.pose.orientation.z,
		msg.pose.pose.orientation.w
	);

	if (controlPtr) {
		controlPtr->setRobotPose(x, y, theta);
	}
}

void ControlSelector::timerCallback()
{
	geometry_msgs::msg::Twist cmd;
	
	if (controlPtr) {
		controlPtr->getControl(cmd.linear.x, cmd.angular.z);
	}
	else {
		RCLCPP_WARN_STREAM(logger_, "NO CONTROL");
	}

	cmd_pub->publish(cmd);
}
