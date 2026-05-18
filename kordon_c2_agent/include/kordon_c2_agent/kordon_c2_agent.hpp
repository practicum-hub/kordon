#pragma once

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <kordon_interfaces/msg/geo_point_goal.hpp>
#include <thread>
#include <atomic>
#include <cstdint>
#include <vector>

#include <grpcpp/grpcpp.h>
#include <v1/robot.grpc.pb.h>


namespace kordon_c2_agent
{

using CallbackReturn =
	rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using Odometry = nav_msgs::msg::Odometry;
using NavSatFix = sensor_msgs::msg::NavSatFix;
using LaserScan = sensor_msgs::msg::LaserScan;
using TwistStamped = geometry_msgs::msg::TwistStamped;
using GeoPointGoal = kordon_interfaces::msg::GeoPointGoal;

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
	rclcpp::Subscription<LaserScan>::SharedPtr scan_sub_;

	std::string gps_topic_;
	std::string scan_topic_;

	struct GeoPositionState
	{
		double latitude{0.0};
		double longitude{0.0};
		double altitude{0.0};
		bool valid{false};
	};

	GeoPositionState last_geo_position_;

	void gps_callback(const NavSatFix::SharedPtr msg);

	struct LidarState
	{
		std::string frame_id;
		double angle_min{0.0};
		double angle_increment{0.0};
		double range_min{0.0};
		double range_max{0.0};
		std::vector<float> ranges;
		int64_t stamp_unix_ms{0};
		bool valid{false};
	};

	LidarState last_lidar_;
	std::mutex lidar_mutex_;

	void scan_callback(const LaserScan::SharedPtr msg);

	// Nav
	std::string cmd_vel_topic_;

	rclcpp::Publisher<TwistStamped>::SharedPtr cmd_vel_pub_;

	std::thread command_stream_thread_;
	std::atomic_bool command_stream_running_{false};

	void listen_command_stream();
	void handle_robot_command(const c2_highground::v1::RobotCommand& command);

	std::string geo_goal_topic_;
	rclcpp::Publisher<GeoPointGoal>::SharedPtr geo_goal_pub_;
};

} // namespace kordon_c2_agent
