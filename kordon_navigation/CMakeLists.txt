cmake_minimum_required(VERSION 3.8)
project(kordon_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(kordon_interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(geo_point_controller
  src/geo_point_controller.cpp
  src/geo_point_controller_node.cpp
)

ament_target_dependencies(geo_point_controller
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  sensor_msgs
  nav_msgs
  geometry_msgs
  kordon_interfaces
  nav2_msgs
  rclcpp_action
  tf2_ros
  tf2_geometry_msgs
)

include_directories(include)

install(
  TARGETS
    geo_point_controller
  DESTINATION
    lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
