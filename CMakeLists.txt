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
target_link_libraries(example_node PUBLIC rclcpp::rclcpp)

install(TARGETS example_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_package()
