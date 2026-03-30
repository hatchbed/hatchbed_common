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
target_link_libraries(example_node
  rclcpp::rclcpp
)

install(TARGETS example_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_cpplint ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_cpplint REQUIRED)
  ament_cpplint(
    FILTERS "-build/include_order"
    MAX_LINE_LENGTH 100)
endif()

ament_package()
