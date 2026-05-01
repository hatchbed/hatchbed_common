# Logging Utilities

The logging utilities live in `logging.h`, `logging/macros.h`, `logging/formatting.h`,
and `logging/message_formatting.h`.  They extend the standard ROS 2 logging macros
with `fmt`-style format strings, a configurable default logger, and ready-made
`fmt` formatters for common ROS 2 message types and Eigen types.

---

## CMake dependency

```cmake
find_package(hatchbed_common REQUIRED)
target_link_libraries(my_target hatchbed_common::logging)
```

---

## Logging macros

Include `<hatchbed_common/logging.h>` to get the macros and format support together.

### Severity levels

| Macro prefix | ROS 2 severity |
|---|---|
| `HB_DEBUG` | DEBUG |
| `HB_INFO` | INFO |
| `HB_WARN` | WARN |
| `HB_ERROR` | ERROR |
| `HB_FATAL` | FATAL |

### Macro variants

Each severity has six variants:

| Variant | Behaviour |
|---|---|
| `HB_<SEV>(logger, ...)` | Always logs |
| `HB_<SEV>_ONCE(logger, ...)` | Logs only the first time |
| `HB_<SEV>_SKIPFIRST(logger, ...)` | Skips the first occurrence, logs all subsequent |
| `HB_<SEV>_EXPRESSION(logger, expr, ...)` | Logs only when `expr` is true |
| `HB_<SEV>_FUNCTION(logger, func, ...)` | Logs only when `func()` returns true |
| `HB_<SEV>_THROTTLE(logger, clock, duration, ...)` | Throttles by time |

### Logger argument

The first argument (`logger`) accepts any of:

- `rclcpp::Logger` instance
- `rclcpp::Node*` or `rclcpp::Node::SharedPtr` — the node's logger is used
- A `std::string` — a child logger with that name under the default logger
- `nullptr` — the package-level default logger (set via `set_default_logger()`)

### Message argument

After the logger, pass either a plain string or a `fmt`-style format string followed by
arguments.  Both compile-time (string literal) and runtime format strings are supported.

### Examples

```cpp
#include <hatchbed_common/logging.h>

// Plain strings
HB_INFO(logger, "Node initialized");
HB_WARN(node.get(), "Retrying connection");

// fmt format strings
HB_INFO(logger, "Processing {} points in frame '{}'", cloud.size(), cloud.header.frame_id);
HB_ERROR(logger, "Transform failed: {}", ex.what());
HB_DEBUG(nullptr, "Pose: {}", pose);   // uses default logger; requires message_formatting.h

// Conditional / filtered
HB_INFO_ONCE(logger, "First message received");
HB_WARN_EXPRESSION(logger, count > 100, "High count: {}", count);
HB_DEBUG_THROTTLE(logger, clock, std::chrono::seconds(1), "Rate: {:.1f} Hz", hz);
HB_DEBUG_THROTTLE(logger, nullptr, 500, "Throttled (500 ms)");  // duration in ms
```

---

## Default logger

```cpp
#include <hatchbed_common/logging.h>

// Set the package-wide default logger (call once at node startup)
hatchbed_common::set_default_logger(get_logger());

// Retrieve it
auto logger = hatchbed_common::get_default_logger();
```

Passing `nullptr` as the logger argument to any macro resolves to the default logger.

---

## Message formatters

Include `<hatchbed_common/logging/message_formatting.h>` to enable `fmt` formatting of
ROS 2 message types and Eigen types.  This is a superset of `formatting.h`.

Once included, any supported type can appear directly in a format string:

```cpp
#include <hatchbed_common/logging/message_formatting.h>

geometry_msgs::msg::Pose pose = ...;
HB_INFO(logger, "Goal pose: {}", pose);
// -> "Goal pose: Pose(p: [1.0, 0.0, 0.5], q: [0.0, 0.0, 0.0, 1.0])"

Eigen::Isometry3d T = ...;
HB_DEBUG(logger, "Transform: {}", T);

nav_msgs::msg::Odometry odom = ...;
HB_INFO(logger, "Odometry:\n{}", odom);
```

### Supported types

**ROS 2 / builtin:**

| Type | Example output |
|---|---|
| `rclcpp::Time` | `"1234567890.123456789"` |
| `rclcpp::Duration` | `"1.500000000"` |
| `builtin_interfaces::msg::Time` | same format |
| `std_msgs::msg::Header` | `"[frame: 'map', stamp: 1234...]"` |
| `std_msgs::msg::ColorRGBA` | `"[r: 1.0, g: 0.0, b: 0.0, a: 1.0]"` |

**Geometry:**

| Type | Example output |
|---|---|
| `geometry_msgs::msg::Point` | `"[1.0, 2.0, 3.0]"` |
| `geometry_msgs::msg::Vector3` | `"[1.0, 2.0, 3.0]"` |
| `geometry_msgs::msg::Quaternion` | `"[x, y, z, w]"` |
| `geometry_msgs::msg::Pose` | `"Pose(p: [...], q: [...])"` |
| `geometry_msgs::msg::PoseStamped` | includes header |
| `geometry_msgs::msg::PoseWithCovariance` | includes `"cov: [6x6]"` |
| `geometry_msgs::msg::Transform` | `"Transform(t: [...], r: [...])"` |
| `geometry_msgs::msg::Twist` | `"Twist(v: [...], w: [...])"` |
| `geometry_msgs::msg::TwistStamped` | includes header |
| `geometry_msgs::msg::TwistWithCovariance` | includes covariance |

**Eigen:**

| Type | Example output |
|---|---|
| `Eigen::Vector3d` | `"[1.0, 2.0, 3.0]"` |
| `Eigen::Matrix3d` | newline-separated rows |
| `Eigen::Isometry3d` | `"t: [...], r: [x, y, z, w]"` |
| `Eigen::Quaterniond` | `"[x, y, z, w]"` |
| `Eigen::AngleAxisd` | `"AngleAxis(angle: X, axis: [...])"` |

**TF2:**

| Type | Example output |
|---|---|
| `tf2::Vector3` | `"[x, y, z]"` |
| `tf2::Quaternion` | `"[x, y, z, w]"` |
| `tf2::Transform` | `"(t: [...], r: [...])"` |

**Sensor / nav summary formatters:**

| Type | Example output |
|---|---|
| `sensor_msgs::msg::Image` | `"Image(header, 640x480, rgb8)"` |
| `sensor_msgs::msg::PointCloud2` | `"PointCloud2(header, 1024x1, is_dense)"` |
| `sensor_msgs::msg::LaserScan` | `"LaserScan(header, 360 pts, 0.0–10.0 m)"` |
| `sensor_msgs::msg::Imu` | `"Imu(header, accel: [...], gyro: [...])"` |
| `nav_msgs::msg::Odometry` | multiline with pose and twist |
| `nav_msgs::msg::Path` | `"Path(header, 42 poses)"` |
| `visualization_msgs::msg::Marker` | multiline with type, action, and pose |
| `visualization_msgs::msg::MarkerArray` | `"MarkerArray(3 markers)"` |

**Standard library (for fmt < v10):**

| Type | Output |
|---|---|
| `std::exception` | exception message |
| `std::optional<T>` | `"none"` or `"optional(value)"` |
| `std::variant<...>` | `"variant(current_value)"` |
| `std::filesystem::path` | quoted string |
| `std::shared_ptr<T>` / `std::unique_ptr<T>` | formatted `*T` or `"[null]"` |

---

[Back to README](../README.md)
