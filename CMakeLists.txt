cmake_minimum_required(VERSION 3.5)
project(hatchbed_common)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Header-only interface library; provides the hatchbed_common::hatchbed_common target
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} INTERFACE
  rclcpp::rclcpp
)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)

add_executable(example_node example/example_node.cpp)
set_property(TARGET example_node PROPERTY CXX_STANDARD 17)
target_link_libraries(example_node PRIVATE ${PROJECT_NAME})

install(TARGETS example_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_cpplint ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_cpplint REQUIRED)
  ament_cpplint(
    FILTERS "-build/include_order"
    MAX_LINE_LENGTH 100)

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ros_names test/test_ros_names.cpp)
  target_link_libraries(test_ros_names ${PROJECT_NAME})
endif()

ament_package()
