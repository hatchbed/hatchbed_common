# Transform Utilities

The transform utilities provide building blocks for pose estimation pipelines:
a time-indexed pose buffer with interpolation (`pose_buffer.h`), covariance
rotation helpers (`covariance_util.h`), and an SE(3) conversion helper
(`transform_util.h`). Two composable nodes are also provided for re-framing
odometry and bridging disconnected TF trees.

---

## CMake dependency

```cmake
find_package(hatchbed_common REQUIRED)
target_link_libraries(my_target hatchbed_common::transforms)
```

---

## PoseBuffer

`hatchbed_common::transforms::PoseBuffer` (`transforms/pose_buffer.h`) maintains a
time-ordered history of `Eigen::Isometry3d` poses and supports interpolated
lookup at arbitrary timestamps.

### Construction

```cpp
#include <hatchbed_common/transforms/pose_buffer.h>

// Default: keep up to 10 seconds of history
hatchbed_common::transforms::PoseBuffer buffer;

// Custom maximum age
hatchbed_common::transforms::PoseBuffer buffer(5.0);  // 5 seconds
```

### Adding poses

```cpp
buffer.push(stamp, pose);
```

Timestamps must be monotonically non-decreasing.  Entries older than `max_age`
seconds relative to the newest stamp are automatically discarded.  Quaternions
are normalized on insertion.

### Querying poses

```cpp
auto result = buffer.lookup(query_stamp);
if (result) {
    Eigen::Isometry3d pose = *result;
    // use pose
}
```

`lookup()` returns `std::nullopt` when:
- The buffer has fewer than two entries
- `query_stamp` is outside the buffered time range (extrapolation is not performed)

Interpolation uses LERP for translation and SLERP for rotation.

### Transforming the history

Apply a left-multiplied rigid transform to every pose in the buffer:

```cpp
Eigen::Isometry3d T_correction = ...;
buffer.applyTransform(T_correction);  // pose_i = T_correction * pose_i
```

This is useful when an upstream pose-graph correction shifts the global reference
frame for all historical poses simultaneously.

### Inspection and management

```cpp
buffer.size()             // number of entries
buffer.empty()            // true if no entries
buffer.earliestStamp()    // std::optional<rclcpp::Time>
buffer.latestStamp()      // std::optional<rclcpp::Time>
buffer.expireBefore(t)    // discard entries older than t
buffer.clear()            // remove all entries
```

### Example: deskewing point cloud scans

```cpp
#include <hatchbed_common/transforms/pose_buffer.h>
#include <hatchbed_common/transforms/transform_util.h>

namespace hc = hatchbed_common::transforms;

hc::PoseBuffer odom_buffer;

// In TF callback:
odom_buffer.push(tf_msg.header.stamp, hc::toIsometry(tf_msg));

// When processing a scan:
auto start_pose = odom_buffer.lookup(scan_start_stamp);
auto end_pose   = odom_buffer.lookup(scan_end_stamp);
if (!start_pose || !end_pose) {
    RCLCPP_WARN(get_logger(), "Insufficient pose history for deskew.");
    return;
}
// interpolate per-point using fraction t in [0, 1]
```

---

## Covariance utilities

`hatchbed_common::transforms::rotateCovariance` (`transforms/covariance_util.h`) rotates a
6x6 pose covariance matrix by a given orientation.

### Type alias

```cpp
using Covariance = std::array<double, 36>;  // row-major 6x6
```

This matches the layout used by `geometry_msgs::msg::PoseWithCovariance` and
`nav_msgs::msg::Odometry`.

### rotateCovariance

```cpp
#include <hatchbed_common/transforms/covariance_util.h>

Covariance rotateCovariance(const Covariance& cov_in,
                            const tf2::Quaternion& q);
```

Applies the rotation `q` to both the translational and rotational blocks of the
covariance matrix using the Jacobian formula:

```
C_out = J * C_in * J^T,   where J = diag(R, R)
```

`R` is the 3x3 rotation matrix corresponding to `q`.  The four 3x3 blocks of the
6x6 matrix are each transformed independently.

### Example

```cpp
// Express a body-frame covariance in the map frame
geometry_msgs::msg::PoseWithCovariance pwc = ...;
tf2::Quaternion q;
tf2::fromMsg(pwc.pose.orientation, q);

auto rotated = hatchbed_common::transforms::rotateCovariance(pwc.covariance, q);
```

---

## toIsometry

`hatchbed_common::transforms::toIsometry` (`transforms/transform_util.h`) converts a
`geometry_msgs::msg::TransformStamped` to an `Eigen::Isometry3d`.

```cpp
#include <hatchbed_common/transforms/transform_util.h>

geometry_msgs::msg::TransformStamped tf_msg = ...;
Eigen::Isometry3d T = hatchbed_common::transforms::toIsometry(tf_msg);
```

This is the canonical way to convert a ROS TF transform into an Eigen SE(3) type
for use with `PoseBuffer` and other Eigen-based geometry code.

---

## Composable nodes

The `hatchbed_common_transforms` library registers two composable nodes for use in
launch files.  Each can also be run as a standalone executable.

### ReframeOdom

Re-expresses a `nav_msgs/Odometry` in a different fixed TF frame by capturing the
transform once on the first message and holding it constant thereafter.

- **Executable:** `reframe_odom`
- **Topics:** `odom_in` (input) -> `odom_out` (output)
- **Key parameters:**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `target_frame` | string | `""` | Target TF frame for the output odometry (required) |

On the first message the transform from the odometry's source frame to `target_frame` is
looked up and frozen.  Pose, twist (linear and angular), and all covariance blocks are
rotated using `rotateCovariance()`.  Avoids drift that would accumulate from a continuously-
updated TF lookup.

---

### TfReroot

Bridges two disconnected TF trees by auto-detecting a shared pivot frame and broadcasting
a `parent_frame -> child_frame` edge on the main `/tf` topic.

- **Executable:** `tf_reroot`
- **Topics:** `tf_source` (tf2_msgs/TFMessage, input); broadcasts to `/tf`
- **Key parameters:**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `parent_frame` | string | `""` | Frame in the main TF tree to use as parent (required) |
| `child_frame` | string | `""` | Frame reachable via `tf_source` to use as child (required) |

Maintains two independent TF buffers -- one fed from `tf_source`, one from `/tf` and
`/tf_static`.  Automatically finds a pivot frame reachable from both `child_frame` and
`parent_frame`, then computes and broadcasts
`T_parent_child = T_parent_pivot * inv(T_child_pivot)`.  Re-detects the pivot if a
lookup fails.

---

[Back to README](../README.md)
