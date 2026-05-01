^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hatchbed_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.6 (2026-05-01)
------------------
* ParamHandler now accepts rclcpp_lifecycle::LifecycleNode via a template constructor that
  stores ROS node interfaces internally, removing the hard dependency on rclcpp::Node::SharedPtr.
* Add unit tests for ParamHandler covering regular nodes, lifecycle nodes, dynamic parameter
  updates, static parameter rejection, and numeric range constraints.
* Add localization utilities: covariance rotation helpers and PoseBuffer with interpolation.
* Add point cloud utilities: field helpers, transform/deskew functions, and utility nodes.
* Add logging utilities with fmt support.
* Add helper for validating ROS parameter names.
* Add fmt support for SimpleProfiler.
* Contributors: Marc Alban

0.1.5 (2026-03-29)
------------------
* Enable static analysis and linting tests.
* Contributors: Marc Alban

0.1.4 (2026-02-18)
------------------
* Replace ament_target_dependencies with target_link_libraries.
* Contributors: Marc Alban

0.1.2 (2025-04-06)
------------------
* Add simple timing profiler utility.
* Add global callback for any changes to registered dynamic parameter.
* Contributors: Marc Alban

0.1.1 (2024-11-24)
------------------
* Contributors: Marc Alban
