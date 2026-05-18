#pragma once

#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <kordon_interfaces/msg/geo_point_goal.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>


namespace kordon_navigation
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

using NavSatFix = sensor_msgs::msg::NavSatFix;
using Odometry = nav_msgs::msg::Odometry;
using TwistStamped = geometry_msgs::msg::TwistStamped;
using GeoPointGoal = kordon_interfaces::msg::GeoPointGoal;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class GeoPointController final : public rclcpp_lifecycle::LifecycleNode
{
public:
	explicit GeoPointController(const rclcpp::NodeOptions& options);

protected:
	CallbackReturn on_configure(const rclcpp_lifecycle::State& prev_state) override;
	CallbackReturn on_activate(const rclcpp_lifecycle::State& prev_state) override;
	CallbackReturn on_deactivate(const rclcpp_lifecycle::State& prev_state) override;
	CallbackReturn on_cleanup(const rclcpp_lifecycle::State& prev_state) override;
	CallbackReturn on_shutdown(const rclcpp_lifecycle::State& prev_state) override;

private:
	void gps_callback(const NavSatFix::SharedPtr msg);
	void odom_callback(const Odometry::SharedPtr msg);
	void goal_callback(const GeoPointGoal::SharedPtr msg);
	void send_nav2_goal_locked();

	std::string robot_id_;
	std::string gps_topic_;
	std::string odom_topic_;
	std::string cmd_vel_topic_;

	bool active_{false};

	rclcpp::Subscription<NavSatFix>::SharedPtr gps_sub_;
	rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
	rclcpp::Subscription<GeoPointGoal>::SharedPtr goal_sub_;

	rclcpp::Publisher<TwistStamped>::SharedPtr cmd_vel_pub_;

	std::mutex state_mutex_;

	double latitude_{0.0};
	double longitude_{0.0};
	double altitude_{0.0};
	bool has_gps_{false};

	double x_{0.0};
	double y_{0.0};
	double yaw_{0.0};
	double has_odom_{false};

	std::string goal_topic_;
	std::string active_command_id_;
	double target_latitude_{0.0};
	double target_longitude_{0.0};
	double target_altitude_{0.0};
	bool has_goal_{false};

	rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_;
	std::string nav2_action_name_;
	std::string nav2_goal_frame_;

};

}
