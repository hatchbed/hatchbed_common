# hatchbed_common

Common C++ utility libraries for ROS 2 nodes.

## Libraries

| Library | CMake target | Description |
|---|---|---|
| common | `hatchbed_common::common` | Parameter handler, simple profiler |
| logging | `hatchbed_common::logging` | fmt-based logging macros and message formatters |
| localization | `hatchbed_common::localization` | Pose buffer with interpolation, covariance rotation; ImuToTwist, OdomTfBroadcast, OdomToTwist, ReframeOdom, TfReroot, TwistMux composable nodes |
| pointcloud | `hatchbed_common::pointcloud` | PointCloud2 iterators, transforms, composable nodes |

## Documentation

- [Parameter Utilities](docs/param_utils.md) — Declarative ROS 2 parameter management with dynamic reconfigure support
- [Logging Utilities](docs/logging_utils.md) — fmt-style logging macros, default logger, and message formatters for geometry, Eigen, TF2, and sensor types
- [Localization Utilities](docs/localization_utils.md) — Time-indexed pose buffer with LERP/SLERP interpolation and covariance rotation helpers
- [Point Cloud Utilities](docs/pointcloud_utils.md) — Type-safe field/point iterators, cloud creation, rigid transform and motion deskew, CropBox/CropRadius/MergePointClouds nodes
- [Profiling Utilities](docs/profiling_utils.md) — Hierarchical RAII timing profiler with rolling averages and thread-local storage
