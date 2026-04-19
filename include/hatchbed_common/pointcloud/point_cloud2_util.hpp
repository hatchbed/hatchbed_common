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
// requested field was absent or had the wrong type; otherwise behaves like a
// PointCloud2ConstIterator<T> with single dereference.
template <typename T>
class FieldIterator
{
public:
  FieldIterator() = default;

  explicit FieldIterator(sensor_msgs::PointCloud2ConstIterator<T> iter)
  : iter_(std::move(iter)) {}

  explicit operator bool() const { return iter_.has_value(); }

  const T & operator*() const { return **iter_; }
  FieldIterator & operator++() { ++(*iter_); return *this; }

private:
  std::optional<sensor_msgs::PointCloud2ConstIterator<T>> iter_;
};

// Returns a FieldIterator<T> for the named field if it exists and its datatype
// matches T, or an invalid FieldIterator (evaluates to false) otherwise.
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

}  // namespace pointcloud
}  // namespace hatchbed_common
