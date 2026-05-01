// Copyright 2026 Hatchbed L.L.C.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace hatchbed_common {
namespace pointcloud {

// Traits mapping C++ types to sensor_msgs PointField datatype constants.
template <typename T> struct point_field_type;
template <> struct point_field_type<int8_t> {
  static constexpr uint8_t value = sensor_msgs::msg::PointField::INT8;
};
template <> struct point_field_type<uint8_t> {
  static constexpr uint8_t value = sensor_msgs::msg::PointField::UINT8;
};
template <> struct point_field_type<int16_t> {
  static constexpr uint8_t value = sensor_msgs::msg::PointField::INT16;
};
template <> struct point_field_type<uint16_t> {
  static constexpr uint8_t value = sensor_msgs::msg::PointField::UINT16;
};
template <> struct point_field_type<int32_t> {
  static constexpr uint8_t value = sensor_msgs::msg::PointField::INT32;
};
template <> struct point_field_type<uint32_t> {
  static constexpr uint8_t value = sensor_msgs::msg::PointField::UINT32;
};
template <> struct point_field_type<float> {
  static constexpr uint8_t value = sensor_msgs::msg::PointField::FLOAT32;
};
template <> struct point_field_type<double> {
  static constexpr uint8_t value = sensor_msgs::msg::PointField::FLOAT64;
};

// Iterator wrapper returned by getFieldIterator(). Evaluates to false if the
// requested field was absent or had the wrong type; otherwise dereferences to
// the field value. Iter may be PointCloud2ConstIterator<T> (read-only) or
// PointCloud2Iterator<T> (read-write).
template <typename Iter>
class FieldIteratorBase
{
public:
  FieldIteratorBase() = default;

  explicit FieldIteratorBase(Iter iter) : iter_(std::move(iter)) {}

  explicit operator bool() const { return iter_.has_value(); }

  auto & operator*() { return **iter_; }
  const auto & operator*() const { return **iter_; }
  FieldIteratorBase & operator++() { ++(*iter_); return *this; }

private:
  std::optional<Iter> iter_;
};

template <typename T>
using FieldIterator = FieldIteratorBase<sensor_msgs::PointCloud2ConstIterator<T>>;

template <typename T>
using MutableFieldIterator = FieldIteratorBase<sensor_msgs::PointCloud2Iterator<T>>;

// Returns true if cloud contains a field with the given name whose datatype
// matches T.
template <typename T>
bool hasField(const sensor_msgs::msg::PointCloud2 & cloud, const std::string & field_name)
{
  auto it = std::find_if(
    cloud.fields.begin(), cloud.fields.end(),
    [&field_name](const sensor_msgs::msg::PointField & f) { return f.name == field_name; });
  return it != cloud.fields.end() && it->datatype == point_field_type<T>::value;
}

// Returns a FieldIterator<T> (read-only) for the named field if it exists and
// its datatype matches T, or an invalid iterator (evaluates to false) otherwise.
template <typename T>
FieldIterator<T> getFieldIterator(
  const sensor_msgs::msg::PointCloud2 & cloud, const std::string & field_name)
{
  auto it = std::find_if(
    cloud.fields.begin(), cloud.fields.end(),
    [&field_name](const sensor_msgs::msg::PointField & f) { return f.name == field_name; });

  if (it == cloud.fields.end() || it->datatype != point_field_type<T>::value) {
    return FieldIterator<T>{};
  }

  return FieldIterator<T>{sensor_msgs::PointCloud2ConstIterator<T>(cloud, field_name)};
}

// Returns a MutableFieldIterator<T> (read-write) for the named field if it
// exists and its datatype matches T, or an invalid iterator otherwise.
template <typename T>
MutableFieldIterator<T> getFieldIterator(
  sensor_msgs::msg::PointCloud2 & cloud, const std::string & field_name)
{
  auto it = std::find_if(
    cloud.fields.begin(), cloud.fields.end(),
    [&field_name](const sensor_msgs::msg::PointField & f) { return f.name == field_name; });

  if (it == cloud.fields.end() || it->datatype != point_field_type<T>::value) {
    return MutableFieldIterator<T>{};
  }

  return MutableFieldIterator<T>{sensor_msgs::PointCloud2Iterator<T>(cloud, field_name)};
}

class ConstPointIterator {
public:
  explicit ConstPointIterator(const sensor_msgs::msg::PointCloud2 & points, size_t index = 0);

  Eigen::Vector3f operator*() const;
  ConstPointIterator & operator++();
  bool operator!=(const ConstPointIterator & other) const;

private:
  const sensor_msgs::msg::PointCloud2 & points_;
  size_t index_;
  size_t num_points_;
  sensor_msgs::PointCloud2ConstIterator<float> iter_x_;
  sensor_msgs::PointCloud2ConstIterator<float> iter_y_;
  sensor_msgs::PointCloud2ConstIterator<float> iter_z_;
};

class ConstPointRange {
public:
  explicit ConstPointRange(const sensor_msgs::msg::PointCloud2 & cloud)
  : cloud_(cloud) {}

  ConstPointIterator begin() const {return ConstPointIterator(cloud_, 0);}
  ConstPointIterator end() const {return ConstPointIterator(cloud_, cloud_.width * cloud_.height);}

private:
  const sensor_msgs::msg::PointCloud2 & cloud_;
};

template<class T>
std::vector<Eigen::Matrix<T, 3, 1>> toEigen(const sensor_msgs::msg::PointCloud2 & msg)
{
  size_t num_points = msg.width * msg.height;

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");

  std::vector<Eigen::Matrix<T, 3, 1>> points;
  points.resize(num_points);

  for (size_t i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z) {
    points[i] = {*iter_x, *iter_y, *iter_z};
  }

  return points;
}

template<class T>
sensor_msgs::msg::PointCloud2::SharedPtr createPointCloud2(const T & points)
{
  auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  msg->width = points.size();
  msg->height = 1;
  msg->is_dense = true;
  msg->is_bigendian = false;

  sensor_msgs::msg::PointField x_field, y_field, z_field;

  x_field.name = "x";
  x_field.offset = 0;
  x_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  x_field.count = 1;

  y_field.name = "y";
  y_field.offset = 4;
  y_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  y_field.count = 1;

  z_field.name = "z";
  z_field.offset = 8;
  z_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  z_field.count = 1;

  msg->fields = {x_field, y_field, z_field};
  msg->point_step = 12;  // 3 fields * 4 bytes each
  msg->row_step = msg->point_step * msg->width;

  msg->data.resize(msg->row_step * msg->height);

  sensor_msgs::PointCloud2Iterator<float> iter_x2(*msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y2(*msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z2(*msg, "z");
  for (const auto & point : points) {
    *iter_x2 = point.x();
    *iter_y2 = point.y();
    *iter_z2 = point.z();
    ++iter_x2;
    ++iter_y2;
    ++iter_z2;
  }

  return msg;
}

// Transform a PointCloud2 from sensor to output frame and deskew it using a
// constant body-frame twist.
//
// T_out_sensor is the static transform from the sensor frame to the output
// frame (T_out_sensor * p_sensor = p_output).
//
// linear_vel and angular_vel are the body-frame velocity of the output frame,
// expressed in the output frame, assumed constant over the scan duration.
//
// The per-point "t" field (uint32, nanosecond offset from scan start) is used
// to compute the motion correction applied to each point:
//   dt_i = t_field_ns * 1e-9
//   T_deskew(dt_i) = rigid body motion from linear_vel and angular_vel over dt_i
//   p_out = T_deskew(dt_i) * T_out_sensor * p_sensor
//
// The output PointCloud2 has the same fields and point_step as the input.
// NaN/Inf points are dropped; the remaining valid points appear in the same
// relative order as in the input. The xyz fields are overwritten with the
// transformed and deskewed coordinates; all other fields are copied verbatim
// via memcpy.
//
// Returns false if any required field (x, y, z float32 or t uint32) is missing,
// true on success.
bool transformAndDeskewPointCloud(
    const sensor_msgs::msg::PointCloud2& msg,
    const Eigen::Vector3d& linear_vel,
    const Eigen::Vector3d& angular_vel,
    const Eigen::Isometry3d& T_out_sensor,
    sensor_msgs::msg::PointCloud2& out);

// Apply a static rigid transform T to every finite point in msg, writing the
// result to out. All fields other than x, y, z are copied verbatim.
// Returns false if x, y, z float32 fields are missing; true on success.
bool transformPointCloud(
    const sensor_msgs::msg::PointCloud2& msg,
    const Eigen::Isometry3d& T,
    sensor_msgs::msg::PointCloud2& out);

}  // namespace pointcloud
}  // namespace hatchbed_common
