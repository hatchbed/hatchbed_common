# Point Cloud Utilities

`hatchbed_common/pointcloud/point_cloud2_util.hpp` provides type-safe field iterators,
point iterators, cloud creation helpers, and rigid transform / motion-deskew functions
for `sensor_msgs::msg::PointCloud2`.

---

## CMake dependency

```cmake
find_package(hatchbed_common REQUIRED)
target_link_libraries(my_target hatchbed_common::pointcloud)
```

---

## Field iterators

Iterate over a single named field across all points without needing to know the byte
layout of the cloud.

### Checking for a field

```cpp
if (hatchbed_common::hasField<float>(cloud, "intensity")) {
    // cloud has a float "intensity" field
}
```

### Read-only iterator

```cpp
auto x_it = hatchbed_common::getFieldIterator<float>(cloud, "x");
if (!x_it) {
    // field missing or wrong type
    return;
}
const size_t n = cloud.width * cloud.height;
for (size_t i = 0; i < n; ++i, ++x_it) {
    float x = *x_it;
}
```

### Read-write iterator

```cpp
auto intensity_it = hatchbed_common::getFieldIterator<float>(mutable_cloud, "intensity");
for (size_t i = 0; i < n; ++i, ++intensity_it) {
    *intensity_it *= 0.5f;
}
```

Supported field types: `int8_t`, `uint8_t`, `int16_t`, `uint16_t`, `int32_t`,
`uint32_t`, `float`, `double`.

---

## Point iterators

Iterate over all xyz points as `Eigen::Vector3f` without manually handling byte offsets.
The cloud must have `x`, `y`, and `z` fields of type `float32`.

### Range-based loop

```cpp
for (const auto & pt : hatchbed_common::ConstPointRange(cloud)) {
    if (!pt.allFinite()) continue;
    process(pt);
}
```

### Index-based

```cpp
hatchbed_common::ConstPointIterator it(cloud);
hatchbed_common::ConstPointIterator end(cloud, cloud.width * cloud.height);
for (; it != end; ++it) {
    Eigen::Vector3f p = *it;
}
```

---

## Cloud creation

Build an xyz-only `PointCloud2` from a collection of Eigen points:

```cpp
std::vector<Eigen::Vector3f> pts = { ... };
auto msg = hatchbed_common::createPointCloud2(pts);
msg->header.frame_id = "map";
msg->header.stamp = now();
pub->publish(*msg);
```

The output cloud has `float32` x, y, z fields and `point_step = 12`.

Convert a cloud to a vector of Eigen points:

```cpp
std::vector<Eigen::Vector3f> pts = hatchbed_common::toEigen<float>(cloud);
```

---

## Transforming clouds

### Static rigid transform

```cpp
sensor_msgs::msg::PointCloud2 out;
bool ok = hatchbed_common::transformPointCloud(cloud_in, T_out_sensor, out);
```

- All `x`/`y`/`z` points are transformed; non-finite points are dropped.
- All other fields are copied verbatim.
- Returns `false` if the cloud is missing `x`, `y`, or `z` float fields.

### Motion deskew

Correct for sensor motion during a scan using per-point timestamps.

The cloud must have `x`/`y`/`z` (`float32`) and `t` (`uint32_t`, nanosecond offset
from the start of the scan) fields.

```cpp
Eigen::Vector3d linear_vel  = {0.5, 0.0, 0.0};   // m/s in sensor frame
Eigen::Vector3d angular_vel = {0.0, 0.0, 0.1};   // rad/s in sensor frame
Eigen::Isometry3d T_out_sensor = ...;             // static sensor-to-output transform

sensor_msgs::msg::PointCloud2 out;
bool ok = hatchbed_common::transformAndDeskewPointCloud(
    cloud_in, linear_vel, angular_vel, T_out_sensor, out);
```

For each point with timestamp offset `dt_i` (seconds), the deskew transform is:

```
p_out = T_deskew(dt_i) * T_out_sensor * p_sensor
```

where `T_deskew(dt_i)` integrates the linear and angular velocities over `dt_i`.
Non-finite points are dropped; the order of valid points is preserved.
Returns `false` if required fields are missing.

---

## Composable nodes

The `hatchbed_common_pointcloud` library also registers several composable nodes
for use in launch files:

### CropBox

Removes or keeps points within a configurable 3D box in a reference frame.

- **Executable:** `crop_box_node`
- **Topics:** `points_in` (input) → `points_out` (output)
- **Parameters:** configured via YAML (see `config/crop_box.yaml` for an example)

### CropRadius

Removes or keeps points within a 2D XY radius of a configurable source frame.

- **Executable:** `crop_radius_node`
- **Topics:** `points_in` → `points_out`
- **Published:** `crop_radius_markers` (`visualization_msgs/MarkerArray`) showing the crop cylinder
- **Key parameters:**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `radius` | double | 1.0 | Crop radius in metres (dynamic) |
| `source_frame` | string | `""` | TF frame to crop around; empty = cloud origin |
| `invert` | bool | `true` | `true` = remove inside radius; `false` = keep inside radius (dynamic) |
| `transient_local_input` | bool | `false` | Subscribe with transient_local QoS |

### MergePointClouds

Merges one or more PointCloud2 inputs into a single xyz-only output published at a
fixed rate.  Inputs are cached on arrival; the timer concatenates the latest snapshot
from every input.

- **Executable:** `merge_point_clouds`
- **Topics:** per-input topics (configured via parameters) → `points_out`
- **Key parameters:**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `output_rate` | double | 1.0 | Publish rate in Hz |
| `output_frame` | string | `""` | Transform all inputs to this frame before merging; empty = no transform |
| `inputs` | string[] | `[]` | Ordered list of input names |
| `<name>.topic` | string | — | Absolute topic for input `<name>` (required) |
| `<name>.transient_local` | bool | `false` | Subscribe with transient_local QoS |

Example YAML:
```yaml
inputs: ["slam_map", "lidar"]
slam_map.topic: /map_cloud
slam_map.transient_local: true
lidar.topic: /lidar/points
output_rate: 5.0
output_frame: "map"
```

---

[Back to README](../README.md)
