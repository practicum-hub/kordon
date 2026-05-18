cmake_minimum_required(VERSION 3.8)
project(kordon_c2_agent)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(kordon_interfaces REQUIRED)

find_package(Protobuf REQUIRED)
find_package(gRPC REQUIRED)

# Start proto configuration
set(C2_PROTO_ROOT "/home/antondeulia/Projects/ROS2/C2_HighGround/proto")

set(ROBOT_PROTO "${C2_PROTO_ROOT}/v1/robot.proto")

if (NOT EXISTS "${ROBOT_PROTO}")
  message(FATAL_ERROR "robot.proto not found: ${ROBOT_PROTO}")
endif()

set(GENERATED_PROTO_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")

file(MAKE_DIRECTORY "${GENERATED_PROTO_DIR}")

set(GENERATED_PROTO_SRCS
  "${GENERATED_PROTO_DIR}/v1/robot.pb.cc"
  "${GENERATED_PROTO_DIR}/v1/robot.grpc.pb.cc"
)

set(GENERATED_PROTO_HDRS
  "${GENERATED_PROTO_DIR}/v1/robot.pb.h"
  "${GENERATED_PROTO_DIR}/v1/robot.grpc.pb.h"
)

add_custom_command(
  OUTPUT
    ${GENERATED_PROTO_SRCS}
    ${GENERATED_PROTO_HDRS}
  COMMAND ${Protobuf_PROTOC_EXECUTABLE}
  ARGS
    --proto_path="${C2_PROTO_ROOT}"
    --cpp_out="${GENERATED_PROTO_DIR}"
    --grpc_out="${GENERATED_PROTO_DIR}"
    --plugin=protoc-gen-grpc=$<TARGET_FILE:gRPC::grpc_cpp_plugin>
    "${ROBOT_PROTO}"
  DEPENDS "${ROBOT_PROTO}"
  COMMENT "Generating C++ protobuf/gRPC files from robot.proto"
)

add_library(kordon_c2_proto
  ${GENERATED_PROTO_SRCS}
)

set_target_properties(kordon_c2_proto PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)

target_include_directories(kordon_c2_proto
  PRIVATE
    "${GENERATED_PROTO_DIR}"
)

target_link_libraries(kordon_c2_proto
  PUBLIC
    protobuf::libprotobuf
    gRPC::grpc++
)
# End proto configuration

add_library(kordon_c2_agent_component SHARED
  src/kordon_c2_agent.cpp
)

target_include_directories(kordon_c2_agent_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    "${GENERATED_PROTO_DIR}"
)

ament_target_dependencies(kordon_c2_agent_component
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  nav_msgs
  sensor_msgs
  kordon_interfaces
)

target_link_libraries(kordon_c2_agent_component
  kordon_c2_proto
)

rclcpp_components_register_node(kordon_c2_agent_component
  PLUGIN "kordon_c2_agent::KordonC2Agent"
  EXECUTABLE kordon_c2_agent
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    kordon_c2_agent_component
    kordon_c2_proto
  EXPORT export_kordon_c2_agent
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_kordon_c2_agent HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  nav_msgs
  kordon_interfaces
)

ament_package()
