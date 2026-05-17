#pragma once

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <v1/robot.grpc.pb.h>
#include <grpcpp/grpcpp.h>

namespace kordon_c2_agent
{

using CallbackReturn =
	rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using Odometry = nav_msgs::msg::Odometry;

class KordonC2Agent final : public rclcpp_lifecycle::LifecycleNode
{
public:
	explicit KordonC2Agent(const rclcpp::NodeOptions& options);

protected:
	CallbackReturn
	on_configure(const rclcpp_lifecycle::State& prev_state) override;
	CallbackReturn
	on_activate(const rclcpp_lifecycle::State& prev_state) override;
	CallbackReturn
	on_deactivate(const rclcpp_lifecycle::State& prev_state) override;
	CallbackReturn
	on_cleanup(const rclcpp_lifecycle::State& prev_state) override;
	CallbackReturn
	on_shutdown(const rclcpp_lifecycle::State& prev_state) override;

private:
	void odometry_callback(const Odometry::SharedPtr odometry_msg);

	static double quaternion_to_yaw(double x, double y, double z, double w);

	std::string robot_id_;
	std::string odom_topic_;
	bool active_{false};

	rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;

	// gRPC
	std::string grpc_address_;

	std::shared_ptr<grpc::Channel> grpc_channel_;
	std::shared_ptr<c2_highground::v1::RobotService::Stub> robot_service_stub_;

	// 
	double telemetry_rate_hz_;
	bool has_odom_{false};

	double last_x_{0.0};
	double last_y_{0.0};
	double last_yaw_{0.0};

	std::mutex odom_mutex_;

	rclcpp::TimerBase::SharedPtr telemetry_timer_;

	void send_telemetry();
};

} // namespace kordon_c2_agent