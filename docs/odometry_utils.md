# Odometry Utilities

The odometry utilities provide composable nodes for converting and fusing odometry-related
sensor data.  All nodes live in the `hatchbed_common::odometry` namespace.

---

## CMake dependency

```cmake
find_package(hatchbed_common REQUIRED)
target_link_libraries(my_target hatchbed_common::odometry)
```

---

## Composable nodes

The `hatchbed_common_odometry` library registers four composable nodes for use in
launch files.  Each can also be run as a standalone executable.

### ImuToTwist

Converts `sensor_msgs/Imu` to `geometry_msgs/TwistWithCovarianceStamped`.

- **Executable:** `imu_to_twist`
- **Topics:** `imu` (input) -> `twist` (output)
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
- **Topics:** `odom` (input) -> `twist` (output)
- **Key parameters:**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `frame_id` | string | `""` | Override output header frame; empty = use odometry `header.frame_id` |
| `differential` | bool | `false` | Derive twist from pose finite-differences instead of copying the twist field |

In direct mode the twist field is copied verbatim.  In differential mode the body-frame
twist is derived via `dT = T_prev_inv * T_curr`; covariance is propagated as
`C_twist = 2 * C_pose / dt^2`.

---

### VelocitySensorMux

Fuses one or more twist-like sensor topics into a single
`geometry_msgs/TwistWithCovarianceStamped` by selecting, per output axis, the
non-timed-out input with the lowest diagonal covariance.

- **Executable:** `velocity_sensor_mux`
- **Topics:** per-input topics (configured via parameters) -> `twist` (output)
- **Key parameters:**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `output_frame` | string | `"base_link"` | Frame ID for the output header |
| `output_rate` | double | `50.0` | Publish rate in Hz |
| `inputs` | string[] | `[]` | Ordered list of input names |
| `<name>.topic` | string | -- | Topic for input `<name>` (required) |
| `<name>.type` | string | -- | `imu`, `twist`, `twist_with_cov`, `vector3`, or `odometry` |
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
