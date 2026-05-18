#include <kordon_navigation/geo_point_controller.hpp>

#include <cmath>
#include <functional>

namespace kordon_navigation
{

GeoPointController::GeoPointController(const rclcpp::NodeOptions& options)
	: rclcpp_lifecycle::LifecycleNode("geo_point_controller", options)
{
	declare_parameter("robot_id", "kordon001");
	declare_parameter("gps_topic", "gps/fix");
	declare_parameter("odom_topic", "diff_drive_controller/odom");
	declare_parameter("cmd_vel_topic", "diff_drive_controller/cmd_vel");
	declare_parameter("goal_topic", "navigation/go_to_geo_point");
	declare_parameter("nav2_action_name", "navigate_to_pose");
	declare_parameter("nav2_goal_frame", "odom");
}

CallbackReturn GeoPointController::on_configure(const rclcpp_lifecycle::State&)
{
	robot_id_ = get_parameter("robot_id").as_string();
	gps_topic_ = get_parameter("gps_topic").as_string();
	odom_topic_ = get_parameter("odom_topic").as_string();
	cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
	goal_topic_ = get_parameter("goal_topic").as_string();
	nav2_action_name_ = get_parameter("nav2_action_name").as_string();
	nav2_goal_frame_ = get_parameter("nav2_goal_frame").as_string();
	
	nav2_client_ = rclcpp_action::create_client<NavigateToPose>(
		get_node_base_interface(),
		get_node_graph_interface(),
		get_node_logging_interface(),
		get_node_waitables_interface(),
		nav2_action_name_
	);

	// Subscriptions
	gps_sub_ = create_subscription<NavSatFix>(
		gps_topic_,
		rclcpp::SensorDataQoS(),
		std::bind(&GeoPointController::gps_callback, this, std::placeholders::_1)
	);

	odom_sub_ = create_subscription<Odometry>(
		odom_topic_,
		10,
		std::bind(&GeoPointController::odom_callback, this, std::placeholders::_1)
	);

	goal_sub_ = create_subscription<GeoPointGoal>(
		goal_topic_,
		10,
		std::bind(&GeoPointController::goal_callback, this, std::placeholders::_1)
	);

	// Publishers
	cmd_vel_pub_ = create_publisher<TwistStamped>(cmd_vel_topic_, 10);

	RCLCPP_INFO(
		get_logger(),
		"Configured: robot_id=%s gps_topic=%s odom_topic=%s cmd_vel_topic=%s",
		robot_id_.c_str(),
		gps_topic_.c_str(),
		odom_topic_.c_str(),
		cmd_vel_topic_.c_str()
	);

	return CallbackReturn::SUCCESS;
}

CallbackReturn GeoPointController::on_activate(const rclcpp_lifecycle::State&)
{
	active_ = true;
	RCLCPP_INFO(get_logger(), "Activated");
	return CallbackReturn::SUCCESS;
}

CallbackReturn GeoPointController::on_deactivate(const rclcpp_lifecycle::State&)
{
	active_ = false;
	RCLCPP_INFO(get_logger(), "Deactivated");
	return CallbackReturn::SUCCESS;
}

CallbackReturn GeoPointController::on_cleanup(const rclcpp_lifecycle::State&)
{
	active_ = false;
	gps_sub_.reset();
	odom_sub_.reset();
	cmd_vel_pub_.reset();
	goal_sub_.reset();

	RCLCPP_INFO(get_logger(), "Cleaned up");
	return CallbackReturn::SUCCESS;
}

CallbackReturn GeoPointController::on_shutdown(const rclcpp_lifecycle::State&)
{
	active_ = false;
	gps_sub_.reset();
	odom_sub_.reset();
	cmd_vel_pub_.reset();
	goal_sub_.reset();

	RCLCPP_INFO(get_logger(), "Shutdown");
	return CallbackReturn::SUCCESS;
}

void GeoPointController::gps_callback(const NavSatFix::SharedPtr msg)
{
	if (!active_)
	{
		return;
	}

	std::lock_guard<std::mutex> lock(state_mutex_);

	latitude_ = msg->latitude;
	longitude_ = msg->longitude;
	altitude_ = msg->altitude;
	has_gps_ = msg->status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX;
}

void GeoPointController::odom_callback(const Odometry::SharedPtr msg)
{
	if (!active_)
	{
		return;
	}

	const auto& position = msg->pose.pose.position;
	const auto& orientation = msg->pose.pose.orientation;

	const double siny_cosp =
		2.0 * (orientation.w * orientation.z + orientation.x * orientation.y);

	const double cosy_cosp =
		1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z);

	std::lock_guard<std::mutex> lock(state_mutex_);

	x_ = position.x;
	y_ = position.y;
	yaw_ = std::atan2(siny_cosp, cosy_cosp);
	has_odom_ = true;
}

void GeoPointController::goal_callback(const GeoPointGoal::SharedPtr msg)
{
	if (!active_)
	{
		return;
	}

	std::lock_guard<std::mutex> lock(state_mutex_);

	active_command_id_ = msg->command_id;
	target_latitude_ = msg->latitude;
	target_longitude_ = msg->longitude;
	target_altitude_ = msg->altitude;
	has_goal_ = true;

	send_nav2_goal_locked();

	RCLCPP_INFO(
		get_logger(),
		"GeoPointGoal accepted: command_id=%s lat=%.8f lon=%.8f alt=%.2f",
		active_command_id_.c_str(),
		target_latitude_,
		target_longitude_,
		target_altitude_
	);
}

void GeoPointController::send_nav2_goal_locked()
{
	if (!has_goal_ || !nav2_client_)
	{
		return;
	}

	if (!has_gps_)
	{
		RCLCPP_WARN(get_logger(), "Cannot send Nav2 goal: no GPS fix");
		return;
	}

	if (!has_odom_)
	{
		RCLCPP_WARN(get_logger(), "Cannot send Nav2 goal: no odom");
		return;
	}

	if (!nav2_client_->wait_for_action_server(std::chrono::seconds(1)))
	{
		RCLCPP_WARN(get_logger(), "Nav2 action server is not available");
		return;
	}

	constexpr double earth_radius_m = 6371000.0;

	const double lat1 = latitude_ * M_PI / 180.0;
	const double lon1 = longitude_ * M_PI / 180.0;
	const double lat2 = target_latitude_ * M_PI / 180.0;
	const double lon2 = target_longitude_ * M_PI / 180.0;

	const double dlat = lat2 - lat1;
	const double dlon = lon2 - lon1;

	const double dx = dlon * std::cos((lat1 + lat2) * 0.5) * earth_radius_m;
	const double dy = dlat * earth_radius_m;

	NavigateToPose::Goal goal;
	goal.pose.header.stamp = now();
	goal.pose.header.frame_id = nav2_goal_frame_;
	goal.pose.pose.position.x = x_ + dx;
	goal.pose.pose.position.y = y_ + dy;
	goal.pose.pose.orientation.z = std::sin(yaw_ * 0.5);
	goal.pose.pose.orientation.w = std::cos(yaw_ * 0.5);

	auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions{};

	options.goal_response_callback =
		[this](const GoalHandleNavigateToPose::SharedPtr& goal_handle)
		{
			if (!goal_handle)
			{
				RCLCPP_WARN(get_logger(), "Nav2 rejected goal");
				return;
			}

			RCLCPP_INFO(get_logger(), "Nav2 accepted goal");
		};

	options.result_callback =
		[this](const GoalHandleNavigateToPose::WrappedResult& result)
		{
			RCLCPP_INFO(
				get_logger(),
				"Nav2 result received: code=%d",
				static_cast<int>(result.code)
			);
		};

	nav2_client_->async_send_goal(goal, options);

	RCLCPP_INFO(
		get_logger(),
		"Sent Nav2 goal: command_id=%s dx=%.2f dy=%.2f odom_x=%.2f odom_y=%.2f",
		active_command_id_.c_str(),
		dx,
		dy,
		x_ + dx,
		y_ + dy
	);
}

} // namespace kordon_navigation
