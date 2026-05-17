#pragma once

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <thread>
#include <atomic>

#include <grpcpp/grpcpp.h>
#include <v1/robot.grpc.pb.h>

namespace kordon_c2_agent
{

using CallbackReturn =
	rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using Odometry = nav_msgs::msg::Odometry;
using NavSatFix = sensor_msgs::msg::NavSatFix;
using TwistStamped = geometry_msgs::msg::TwistStamped;

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

	// Local telemetry
	double telemetry_rate_hz_;
	bool has_odom_{false};

	double last_x_{0.0};
	double last_y_{0.0};
	double last_yaw_{0.0};

	std::mutex odom_mutex_;

	rclcpp::TimerBase::SharedPtr telemetry_timer_;

	void send_telemetry();

	// GPS Telemetry
	rclcpp::Subscription<NavSatFix>::SharedPtr gps_sub_;

	std::string gps_topic_;

	struct GeoPositionState
	{
		double latitude{0.0};
		double longitude{0.0};
		double altitude{0.0};
		bool valid{false};
	};

	GeoPositionState last_geo_position_;

	void gps_callback(const NavSatFix::SharedPtr msg);

	// Nav
	std::string cmd_vel_topic_;

	rclcpp::Publisher<TwistStamped>::SharedPtr cmd_vel_pub_;

	std::thread command_stream_thread_;
	std::atomic_bool command_stream_running_{false};

	void listen_command_stream();
	void handle_robot_command(const c2_highground::v1::RobotCommand& command);

private:
	bool has_geo_target_{false};
	double target_latitude_{0.0};
	double target_longitude_{0.0};
	double target_altitude_{0.0};

	void start_go_to_geo_point(const c2_highground::v1::GoToGeoPoint& target);
	void update_go_to_geo_point();
	static double normalize_angle(double angle);
};

} // namespace kordon_c2_agent