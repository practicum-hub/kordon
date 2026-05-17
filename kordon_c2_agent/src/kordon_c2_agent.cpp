#include <kordon_c2_agent/kordon_c2_agent.hpp>

namespace kordon_c2_agent
{

// Constructor
KordonC2Agent::KordonC2Agent(const rclcpp::NodeOptions& options)
	: rclcpp_lifecycle::LifecycleNode("kordon_2c_agent", options)
{
	declare_parameter("robot_id", "kordon-001");
	declare_parameter("odom_topic", "diff_drive_controller/odom");
	declare_parameter("grpc_address", "localhost:50051");
	declare_parameter("telemetry_rate_hz", 5.0);
}

// Lifecycles
CallbackReturn KordonC2Agent::on_configure(const rclcpp_lifecycle::State&)
{
	robot_id_ = get_parameter("robot_id").as_string();
	odom_topic_ = get_parameter("odom_topic").as_string();
	grpc_address_ = get_parameter("grpc_address").as_string();

	grpc_channel_ = grpc::CreateChannel(
		grpc_address_,
		grpc::InsecureChannelCredentials()
	);

	robot_service_stub_ = c2_highground::v1::RobotService::NewStub(grpc_channel_);

	c2_highground::v1::RegisterRobotRequest request;
	request.set_id(robot_id_);
	
	grpc::ClientContext context;
	c2_highground::v1::RegisterRobotResponse response;

	const grpc::Status status = robot_service_stub_->RegisterRobot(
		&context,
		request,
		&response
	);

	if (!status.ok())
	{
		RCLCPP_ERROR(
			get_logger(),
			"RegisterRobot failed: code=%d message=%s",
			static_cast<int>(status.error_code()),
			status.error_message().c_str()
		);

		return CallbackReturn::FAILURE;
	}

	RCLCPP_INFO(
		get_logger(),
		"Robot registered: status=%d message=%s",
		static_cast<int>(response.status()),
		response.message().c_str()
	);

	odom_sub_ = create_subscription<Odometry>(
		odom_topic_,
		10,
		[this](const Odometry::SharedPtr odometry_msg)
		{ odometry_callback(odometry_msg); });

	// Telemetry
	telemetry_rate_hz_ = get_parameter("telemetry_rate_hz").as_double();

	const auto telemetry_period = std::chrono::duration<double>(
		1.0 / telemetry_rate_hz_
	);

	telemetry_timer_ = create_wall_timer(
		std::chrono::duration_cast<std::chrono::milliseconds>(telemetry_period),
		std::bind(&KordonC2Agent::send_telemetry, this)
	);

	telemetry_timer_->cancel();

	RCLCPP_INFO(
		get_logger(),
		"Configured: robot_id=%s odom_topic=%s",
		robot_id_.c_str(),
		odom_topic_.c_str()
	);
	return CallbackReturn::SUCCESS;
}

CallbackReturn KordonC2Agent::on_activate(const rclcpp_lifecycle::State&)
{
	active_ = true;

	if (telemetry_timer_)
	{
		telemetry_timer_->reset();
	}
	
	RCLCPP_INFO(
		get_logger(),
		"Activated"
	);
	return CallbackReturn::SUCCESS;
}

CallbackReturn KordonC2Agent::on_deactivate(const rclcpp_lifecycle::State&)
{
	active_ = false;

	if (telemetry_timer_) {
		telemetry_timer_->cancel();
	}
	
	RCLCPP_INFO(
		get_logger(),
		"Deactivated"
	);
	return CallbackReturn::SUCCESS;
}

CallbackReturn KordonC2Agent::on_cleanup(const rclcpp_lifecycle::State&)
{
	active_ = false;
	odom_sub_.reset();
	
	RCLCPP_INFO(
		get_logger(),
		"Deactivated"
	);
	return CallbackReturn::SUCCESS;
}

CallbackReturn KordonC2Agent::on_shutdown(const rclcpp_lifecycle::State&)
{
	active_ = false;
	odom_sub_.reset();
	
	RCLCPP_INFO(
		get_logger(),
		"Deactivated"
	);
	return CallbackReturn::SUCCESS;
}

// Callbacks
void KordonC2Agent::odometry_callback(const Odometry::SharedPtr odom_msg) {
	if (!active_) 
	{
		return;
	}

	const auto & position = odom_msg->pose.pose.position;
	const auto & orientation = odom_msg->pose.pose.orientation;

	const double yaw = quaternion_to_yaw(
		orientation.x,
		orientation.y,
		orientation.z,
		orientation.w
	);

	{
		std::lock_guard<std::mutex> lock(odom_mutex_);
		last_x_ = position.x;
		last_y_ = position.y;
		last_yaw_ = yaw;
		has_odom_ = true;
	}

	RCLCPP_INFO(
		get_logger(),
		"robot_id=%s frame_id=%s x=%.3f y=%.3f yaw=%.3f",
		robot_id_.c_str(),
		odom_msg->header.frame_id.c_str(),
		position.x,
		position.y,
		yaw
	);
}

double KordonC2Agent::quaternion_to_yaw(
	double x,
	double y,
	double z,
	double w
)
{
	const double siny_cosp = 2.0 * (w * z + x * y);
	const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);

	return std::atan2(siny_cosp, cosy_cosp);
}

void KordonC2Agent::send_telemetry()
{
	double x;
	double y;
	double yaw;

	{
		std::lock_guard<std::mutex> lock(odom_mutex_);

		if (!has_odom_)
		{
			return;
		}

		x = last_x_;
		y = last_y_;
		yaw = last_yaw_;
	}

	if (!robot_service_stub_)
	{
		RCLCPP_WARN_THROTTLE(
			get_logger(),
			*get_clock(),
			5000,
			"RobotService stub is not initialized"
		);
		return;
	}

	c2_highground::v1::SendTelemetryRequest request;
	request.set_robot_id(robot_id_);

	auto* pose = request.mutable_pose();
	pose->set_x(x);
	pose->set_y(y);
	pose->set_yaw(yaw);

	grpc::ClientContext context;
	c2_highground::v1::SendTelemetryResponse response;

	const grpc::Status status = robot_service_stub_->SendTelemetry(
		&context,
		request,
		&response
	);

	if (!status.ok())
	{
		RCLCPP_WARN_THROTTLE(
			get_logger(),
			*get_clock(),
			5000,
			"SendTelemetry failed: code=%d message=%s",
			static_cast<int>(status.error_code()),
			status.error_message().c_str()
		);
		return;
	}

	RCLCPP_DEBUG(
		get_logger(),
		"Telemetry sent: x=%.3f y=%.3f yaw=%.3f",
		x,
		y,
		yaw
	);
}

} // namespace kordon_c2_agent

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(kordon_c2_agent::KordonC2Agent)
