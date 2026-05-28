# hatchbed_common

Common C++ utility libraries for ROS 2 nodes.

## Libraries

| Library | CMake target | Description |
|---|---|---|
| common | `hatchbed_common::common` | Parameter handler, simple profiler |
| logging | `hatchbed_common::logging` | fmt-based logging macros and message formatters |
| transforms | `hatchbed_common::transforms` | Pose buffer with interpolation, covariance rotation, SE(3) conversion; ReframeOdom and TfReroot composable nodes |
| odometry | `hatchbed_common::odometry` | ImuToTwist, OdomTfBroadcast, OdomToTwist, VelocitySensorMux composable nodes |
| pointcloud | `hatchbed_common::pointcloud` | PointCloud2 iterators, transforms, composable nodes |

## Documentation

- [Parameter Utilities](docs/param_utils.md) — Declarative ROS 2 parameter management with dynamic reconfigure support
- [Logging Utilities](docs/logging_utils.md) — fmt-style logging macros, default logger, and message formatters for geometry, Eigen, TF2, and sensor types
- [Transform Utilities](docs/transforms_utils.md) — Time-indexed pose buffer with LERP/SLERP interpolation, covariance rotation, SE(3) conversion, ReframeOdom and TfReroot nodes
- [Odometry Utilities](docs/odometry_utils.md) — ImuToTwist, OdomTfBroadcast, OdomToTwist, and VelocitySensorMux composable nodes
- [Point Cloud Utilities](docs/pointcloud_utils.md) — Type-safe field/point iterators, cloud creation, rigid transform and motion deskew, CropBox/CropRadius/MergePointClouds nodes
- [Profiling Utilities](docs/profiling_utils.md) — Hierarchical RAII timing profiler with rolling averages and thread-local storage
