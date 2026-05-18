#include <kordon_navigation/geo_point_controller.hpp>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);

	auto node = std::make_shared<kordon_navigation::GeoPointController>(
		rclcpp::NodeOptions{}
	);

	rclcpp::spin(node->get_node_base_interface());

	rclcpp::shutdown();
	return 0;
}