#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

double right_distance = 0.0;
bool obstacle = false;
const double WALL_DISTANCE = 0.7;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;
rclcpp::Node::SharedPtr node;

void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  RCLCPP_DEBUG(node->get_logger(), "Laser msg: %f", msg->scan_time);
  right_distance = msg->ranges[msg->ranges.size() / 4];
  obstacle = false;
  const int fov = 90;
  const double kMinRange = 0.5;

  int dAngle = (180 - fov) / 2;
  if (std::isinf(right_distance) || std::isnan(right_distance))
  {
    right_distance = msg->range_max;
  }
  for (size_t i = dAngle; i < msg->ranges.size() - dAngle; i++)
  {
    if (msg->ranges[i] < kMinRange)
    {
      obstacle = true;
      RCLCPP_WARN(node->get_logger(), "OBSTACLE!!!");
      break;
    }
  }
}


/**
 * Функция, которая будет вызвана при
 * получении сообщения с текущем положением робота
 * параметр функции msg - ссылка на полученное сообщение
 */
void poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  RCLCPP_DEBUG(node->get_logger(),
    "Pose msg: x = %f y = %f theta = %f",
    msg->pose.pose.position.x,
    msg->pose.pose.position.y,
    2 * atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w)
  );
}

void timerCallback()
{
  static int counter = 0;
  counter++;
  RCLCPP_DEBUG(node->get_logger(), "on timer %d", counter);
  geometry_msgs::msg::Twist cmd;

  if (!obstacle) {
    if (right_distance == WALL_DISTANCE){
      RCLCPP_INFO(node->get_logger(), "go forward");
      cmd.linear.x = 1;
      cmd.angular.z = 0;
    }
    else{
      RCLCPP_INFO(node->get_logger(), "turning");
      cmd.linear.x = 1;
      cmd.angular.z = WALL_DISTANCE - right_distance;
    }
  }
  else {
    RCLCPP_INFO(node->get_logger(), "go left");
    cmd.linear.x = 0;
    cmd.angular.z = 0.5;
  }

  pub->publish(cmd);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  node = std::make_shared<rclcpp::Node>("control_node");

  auto laser_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
    "base_scan", 10,
    [](const sensor_msgs::msg::LaserScan::SharedPtr msg) { laserCallback(msg); });
  auto pose_sub = node->create_subscription<nav_msgs::msg::Odometry>(
    "base_pose_ground_truth", 10,
    [](const nav_msgs::msg::Odometry::SharedPtr msg) { poseCallback(msg); });


  auto timer = node->create_wall_timer(100ms, timerCallback);
  pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}