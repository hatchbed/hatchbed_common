cmake_minimum_required(VERSION 3.5)
project(hatchbed_common)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

install(DIRECTORY include/
  DESTINATION include/
)

include_directories(
  include
)

add_executable(example_node example/example_node.cpp)
set_property(TARGET example_node PROPERTY CXX_STANDARD 17)
ament_target_dependencies(example_node rclcpp)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_package()
