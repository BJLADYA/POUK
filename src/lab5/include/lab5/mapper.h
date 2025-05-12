#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class Mapper final : public rclcpp::Node
{
public:
	Mapper();

private:
	void laserCallback(const sensor_msgs::msg::LaserScan& scan);

	void prepareMapMessage(nav_msgs::msg::OccupancyGrid& map_msg, const rclcpp::Time& stamp);
	void DrawLine(
		nav_msgs::msg::OccupancyGrid& map,
		float x0, float y0, float x1, float y1,
		int8_t value);
	bool determineScanTransform(geometry_msgs::msg::TransformStamped& scanTransform,
		const rclcpp::Time& stamp,
		const std::string& laser_frame);

	rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

	std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
	
	std::string map_frame_;
	double map_resolution_;
	int map_width_;
	int map_height_;
};