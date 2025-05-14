#include <lab6/patrol_bot.h>

PatrolBot::PatrolBot() : Node("patrol_bot_node"), trace_queue_(-1), count_(0), done_(false)
{
	action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

	point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
		"/clicked_point",
		10,
		[this](const geometry_msgs::msg::PointStamped &msg) {
			clickPointCallback(msg);
		});

	timer_ = this->create_wall_timer(
		std::chrono::milliseconds(100),
		[this](){
			controlLoop();
		});

	goals_.resize(5);

	RCLCPP_INFO(this->get_logger(), "Waiting for action server...");

	if (!action_client_->wait_for_action_server(std::chrono::seconds(10)))
	{
		RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
		rclcpp::shutdown();
	}

	RCLCPP_INFO(this->get_logger(), "Connected to action server");
}

void PatrolBot::clickPointCallback(const geometry_msgs::msg::PointStamped &msg)
{
	if (count_ >= 5) return;
	count_++;
	RCLCPP_INFO(this->get_logger(), "Received clicked point: x=%.2f, y=%.2f", msg.point.x, msg.point.y);

	auto &goal = goals_[count_ - 1];
	goal.pose.header.frame_id = "map";
	goal.pose.header.stamp = this->now();
	goal.pose.pose.position = msg.point;

	double target_angle = M_PI / 2;
	goal.pose.pose.orientation.z = sin(target_angle / 2);
	goal.pose.pose.orientation.w = cos(target_angle / 2);

	if (count_ == 1)
		done_ = true;
}

void PatrolBot::controlLoop()
{
	if (done_)
	{
		SendGoal();
	}
}

void PatrolBot::SendGoal()
{
	trace_queue_++;
	if (trace_queue_ == 5 || trace_queue_ == count_)
	{
		trace_queue_ = 0;
	}

	done_ = false;

	auto goal_msg = NavigateToPose::Goal();
	goal_msg.pose = goals_[trace_queue_].pose;

	RCLCPP_INFO(this->get_logger(), "Sending goal %d", trace_queue_);

	auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
	send_goal_options.goal_response_callback =
		[this](const rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr & goal_handle) {
			if (!goal_handle) {
				RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
			} else {
				RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
			}
		};

	send_goal_options.feedback_callback =
		[](GoalHandleNavigateToPose::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback)
		{
			// RCLCPP_INFO(rclcpp::get_logger("control_node"), "Current position x = %.2f", feedback->current_pose.pose.position.x);
		};

	send_goal_options.result_callback =
		[this](const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & result) {
			switch (result.code) {
				case rclcpp_action::ResultCode::SUCCEEDED:
					RCLCPP_INFO(this->get_logger(), "Goal reached!");
					break;
				case rclcpp_action::ResultCode::ABORTED:
					RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
					break;
				case rclcpp_action::ResultCode::CANCELED:
					RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
					break;
				default:
					RCLCPP_ERROR(this->get_logger(), "Unknown result code");
					break;
			}
			done_ = true;
		};

	action_client_->async_send_goal(goal_msg, send_goal_options);
}