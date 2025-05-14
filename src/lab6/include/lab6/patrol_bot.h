#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class PatrolBot : public rclcpp::Node
{
public:
	PatrolBot();

private:
	void clickPointCallback(const geometry_msgs::msg::PointStamped &msg);
	void controlLoop();
	void SendGoal();

	rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
	rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
	rclcpp::TimerBase::SharedPtr timer_;

	std::vector<NavigateToPose::Goal> goals_;
	int trace_queue_;
	int count_;
	bool done_;
};