cmake_minimum_required(VERSION 3.8)
project(hatchbed_common)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

# -----------------------------------------------------------------------------
# Core header-only library
# -----------------------------------------------------------------------------
add_library(hatchbed_common INTERFACE)
set_target_properties(hatchbed_common PROPERTIES EXPORT_NAME common)
target_include_directories(hatchbed_common INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(hatchbed_common INTERFACE
  fmt::fmt
  rclcpp::rclcpp)
target_compile_features(hatchbed_common INTERFACE cxx_std_17)
add_library(hatchbed_common::common ALIAS hatchbed_common)

# -----------------------------------------------------------------------------
# Logging header-only library
# -----------------------------------------------------------------------------
add_library(hatchbed_common_logging INTERFACE)
set_target_properties(hatchbed_common_logging PROPERTIES EXPORT_NAME logging)
target_include_directories(hatchbed_common_logging INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(hatchbed_common_logging INTERFACE
  Eigen3::Eigen
  fmt::fmt
  rclcpp::rclcpp
  stdc++fs
  tf2::tf2
  ${std_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${tf2_msgs_TARGETS}
  ${visualization_msgs_TARGETS})
add_library(hatchbed_common::logging ALIAS hatchbed_common_logging)

# -----------------------------------------------------------------------------
# Localization library
# -----------------------------------------------------------------------------
add_library(hatchbed_common_localization SHARED
  src/localization/components/imu_to_twist.cpp
  src/localization/components/odom_tf_broadcast.cpp
  src/localization/components/odom_to_twist.cpp
  src/localization/components/reframe_odom.cpp
  src/localization/components/tf_reroot.cpp
  src/localization/components/twist_mux.cpp
  src/localization/covariance_util.cpp
  src/localization/pose_buffer.cpp
)
set_target_properties(hatchbed_common_localization PROPERTIES EXPORT_NAME localization)
target_link_libraries(hatchbed_common_localization PUBLIC
  hatchbed_common
  Eigen3::Eigen
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${rclcpp_components_TARGETS}
  ${sensor_msgs_TARGETS}
  tf2::tf2
  ${tf2_msgs_TARGETS}
  tf2_ros::tf2_ros)
target_include_directories(hatchbed_common_localization PUBLIC ${tf2_INCLUDE_DIRS})
add_library(hatchbed_common::localization ALIAS hatchbed_common_localization)

rclcpp_components_register_node(hatchbed_common_localization
  PLUGIN "hatchbed_common::localization::ImuToTwist"
  EXECUTABLE imu_to_twist)
rclcpp_components_register_node(hatchbed_common_localization
  PLUGIN "hatchbed_common::localization::OdomTfBroadcast"
  EXECUTABLE odom_tf_broadcast)
rclcpp_components_register_node(hatchbed_common_localization
  PLUGIN "hatchbed_common::localization::OdomToTwist"
  EXECUTABLE odom_to_twist)
rclcpp_components_register_node(hatchbed_common_localization
  PLUGIN "hatchbed_common::localization::ReframeOdom"
  EXECUTABLE reframe_odom)
rclcpp_components_register_node(hatchbed_common_localization
  PLUGIN "hatchbed_common::localization::TfReroot"
  EXECUTABLE tf_reroot)
rclcpp_components_register_node(hatchbed_common_localization
  PLUGIN "hatchbed_common::localization::TwistMux"
  EXECUTABLE twist_mux)

# -----------------------------------------------------------------------------
# Pointcloud library
# -----------------------------------------------------------------------------
add_library(hatchbed_common_pointcloud SHARED
  src/pointcloud/components/crop_box.cpp
  src/pointcloud/components/crop_radius.cpp
  src/pointcloud/components/merge_point_clouds.cpp
  src/pointcloud/point_cloud2_util.cpp)
set_target_properties(hatchbed_common_pointcloud PROPERTIES EXPORT_NAME pointcloud)
target_link_libraries(hatchbed_common_pointcloud PUBLIC
  hatchbed_common
  Eigen3::Eigen
  ${geometry_msgs_TARGETS}
  ${rclcpp_components_TARGETS}
  ${sensor_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  ${visualization_msgs_TARGETS})
add_library(hatchbed_common::pointcloud ALIAS hatchbed_common_pointcloud)

rclcpp_components_register_node(hatchbed_common_pointcloud
  PLUGIN "hatchbed_common::pointcloud::CropBox"
  EXECUTABLE crop_box_node)

rclcpp_components_register_node(hatchbed_common_pointcloud
  PLUGIN "hatchbed_common::pointcloud::CropRadius"
  EXECUTABLE crop_radius_node)

rclcpp_components_register_node(hatchbed_common_pointcloud
  PLUGIN "hatchbed_common::pointcloud::MergePointClouds"
  EXECUTABLE merge_point_clouds)

# -----------------------------------------------------------------------------
# Example
# -----------------------------------------------------------------------------
add_executable(example_node example/example_node.cpp)
target_link_libraries(example_node PRIVATE hatchbed_common)

# -----------------------------------------------------------------------------
# Install
# -----------------------------------------------------------------------------
install(TARGETS
  hatchbed_common
  hatchbed_common_logging
  hatchbed_common_localization
  hatchbed_common_pointcloud
  EXPORT hatchbed_commonTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  imu_to_twist
  odom_tf_broadcast
  odom_to_twist
  reframe_odom
  tf_reroot
  twist_mux
  crop_box_node
  crop_radius_node
  merge_point_clouds
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(TARGETS example_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/)

ament_export_targets(hatchbed_commonTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Eigen3
  fmt
  geometry_msgs
  nav_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  tf2
  tf2_msgs
  tf2_ros
  visualization_msgs)

# -----------------------------------------------------------------------------
# Tests
# -----------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_cpplint ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_cpplint REQUIRED)
  ament_cpplint(FILTERS "-build/include_order" MAX_LINE_LENGTH 100)

  find_package(ament_cmake_gtest REQUIRED)
  find_package(rclcpp_lifecycle REQUIRED)

  ament_add_gtest(test_param_handler test/test_param_handler.cpp)
  target_link_libraries(test_param_handler hatchbed_common rclcpp_lifecycle::rclcpp_lifecycle)

  ament_add_gtest(test_ros_names test/test_ros_names.cpp)
  target_link_libraries(test_ros_names hatchbed_common)

  ament_add_gtest(test_covariance_util test/localization/test_covariance_util.cpp)
  target_link_libraries(test_covariance_util hatchbed_common_localization)

  ament_add_gtest(test_pose_buffer test/localization/test_pose_buffer.cpp)
  target_link_libraries(test_pose_buffer hatchbed_common_localization)

  ament_add_gtest(test_point_cloud2_util test/pointcloud/test_point_cloud2_util.cpp)
  target_link_libraries(test_point_cloud2_util hatchbed_common_pointcloud)

  ament_add_gtest(test_logging test/logging/test_logging.cpp)
  target_link_libraries(test_logging hatchbed_common_logging)

  ament_add_gtest(test_formatting test/logging/test_formatting.cpp)
  target_link_libraries(test_formatting hatchbed_common_logging)

  ament_add_gtest(test_message_formatting test/logging/test_message_formatting.cpp)
  target_link_libraries(test_message_formatting hatchbed_common_logging)
endif()

ament_package()
