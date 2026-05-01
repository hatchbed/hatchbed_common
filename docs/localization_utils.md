# Localization Utilities

The localization utilities provide two building blocks for pose estimation pipelines:
a time-indexed pose buffer with interpolation (`pose_buffer.h`) and covariance
rotation helpers (`covariance_util.h`).

---

## CMake dependency

```cmake
find_package(hatchbed_common REQUIRED)
target_link_libraries(my_target hatchbed_common::localization)
```

---

## PoseBuffer

`hatchbed_common::PoseBuffer` (`localization/pose_buffer.h`) maintains a
time-ordered history of `Eigen::Isometry3d` poses and supports interpolated
lookup at arbitrary timestamps.

### Construction

```cpp
#include <hatchbed_common/localization/pose_buffer.h>

// Default: keep up to 10 seconds of history
hatchbed_common::PoseBuffer buffer;

// Custom maximum age
hatchbed_common::PoseBuffer buffer(5.0);  // 5 seconds
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
hatchbed_common::PoseBuffer odom_buffer;

// In odometry callback:
odom_buffer.push(msg->header.stamp, toIsometry(msg->pose.pose));

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

`hatchbed_common::rotateCovariance` (`localization/covariance_util.h`) rotates a
6x6 pose covariance matrix by a given orientation.

### Type alias

```cpp
using Covariance = std::array<double, 36>;  // row-major 6x6
```

This matches the layout used by `geometry_msgs::msg::PoseWithCovariance` and
`nav_msgs::msg::Odometry`.

### rotateCovariance

```cpp
#include <hatchbed_common/localization/covariance_util.h>

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

auto rotated = hatchbed_common::rotateCovariance(pwc.covariance, q);
```

---

## Composable nodes

The `hatchbed_common_localization` library registers six composable nodes for use in
launch files.  Each can also be run as a standalone executable.

### ImuToTwist

Converts `sensor_msgs/Imu` to `geometry_msgs/TwistWithCovarianceStamped`.

- **Executable:** `imu_to_twist`
- **Topics:** `imu` (input) → `twist` (output)
- **Key parameters:**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `differential` | bool | `false` | Derive angular velocity by differentiating consecutive orientations instead of copying the `angular_velocity` field |

In direct mode the `angular_velocity` field and its diagonal covariance are copied verbatim.
In differential mode the angular velocity is computed from consecutive quaternion deltas and
covariance is propagated as `C_w = 2 * C_theta / dt^2`.  Requires the IMU's
`orientation_covariance[0] >= 0`.

---

### OdomTfBroadcast

Derives a TF broadcast from a `nav_msgs/Odometry` topic, inserting a
`parent_frame -> child_frame` edge into the main TF tree.

- **Executable:** `odom_tf_broadcast`
- **Topics:** `odom` (input); broadcasts to `/tf`
- **Key parameters:**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `parent_frame` | string | `""` | Parent frame for the broadcast (required) |
| `child_frame` | string | `""` | Child frame; must match either `header.frame_id` or `child_frame_id` of the odometry (required) |
| `timestamp_offset` | double | `0.0` | Seconds added to the odometry stamp; positive values future-date the transform |

`T_parent_child` is computed by looking up the known odometry frame in the main TF tree
and composing with the odometry transform.  Useful when an odometry source already provides
the relative transform between two frames but must be re-parented in the global tree.

---

### OdomToTwist

Extracts the twist from a `nav_msgs/Odometry` and republishes it as
`geometry_msgs/TwistWithCovarianceStamped`.

- **Executable:** `odom_to_twist`
- **Topics:** `odom` (input) → `twist` (output)
- **Key parameters:**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `frame_id` | string | `""` | Override output header frame; empty = use odometry `header.frame_id` |
| `differential` | bool | `false` | Derive twist from pose finite-differences instead of copying the twist field |

In direct mode the twist field is copied verbatim.  In differential mode the body-frame
twist is derived via `dT = T_prev_inv * T_curr`; covariance is propagated as
`C_twist = 2 * C_pose / dt^2`.

---

### ReframeOdom

Re-expresses a `nav_msgs/Odometry` in a different fixed TF frame by capturing the
transform once on the first message and holding it constant thereafter.

- **Executable:** `reframe_odom`
- **Topics:** `odom_in` (input) → `odom_out` (output)
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

Maintains two independent TF buffers — one fed from `tf_source`, one from `/tf` and
`/tf_static`.  Automatically finds a pivot frame reachable from both `child_frame` and
`parent_frame`, then computes and broadcasts
`T_parent_child = T_parent_pivot * inv(T_child_pivot)`.  Re-detects the pivot if a
lookup fails.

---

### TwistMux

Fuses one or more twist-like sensor topics into a single
`geometry_msgs/TwistWithCovarianceStamped` by selecting, per output axis, the
non-timed-out input with the lowest diagonal covariance.

- **Executable:** `twist_mux`
- **Topics:** per-input topics (configured via parameters) → `twist` (output)
- **Key parameters:**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `output_frame` | string | `"base_link"` | Frame ID for the output header |
| `output_rate` | double | `50.0` | Publish rate in Hz |
| `inputs` | string[] | `[]` | Ordered list of input names |
| `<name>.topic` | string | — | Topic for input `<name>` (required) |
| `<name>.type` | string | — | `imu`, `twist`, `twist_with_cov`, `vector3`, or `odometry` |
| `<name>.expected_rate` | double | `10.0` | Expected rate (Hz); input times out after `2 / expected_rate` s |
| `<name>.axes` | string[] | `[]` | Output axes this input provides: any of `lx ly lz ax ay az` |
| `<name>.cov_override` | double[] | `[]` | Per-axis covariance overrides; `0` = use the value from the message |

Output value and covariance default to `0` / `1e9` for any axis not covered by an active input.

Example YAML:
```yaml
output_frame: "base_link"
output_rate: 50.0
inputs: ["wheel_odom", "imu"]
wheel_odom.topic: /odom/twist
wheel_odom.type: twist_with_cov
wheel_odom.axes: ["lx", "ly"]
wheel_odom.expected_rate: 50.0
imu.topic: /imu/data
imu.type: imu
imu.axes: ["ax", "ay", "az"]
imu.expected_rate: 100.0
```

---

[Back to README](../README.md)
