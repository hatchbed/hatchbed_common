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

#include <hatchbed_common/pointcloud/point_cloud2_util.hpp>

#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <string>

#include <Eigen/Core>

namespace hatchbed_common {
namespace pointcloud {

ConstPointIterator::ConstPointIterator(
    const sensor_msgs::msg::PointCloud2 & points, size_t index)
: points_(points), index_(index), num_points_(points.width * points.height),
  iter_x_(points, "x"), iter_y_(points, "y"), iter_z_(points, "z")
{
    if (index_ > 0) {
        iter_x_ += index_;
        iter_y_ += index_;
        iter_z_ += index_;
    }
}

Eigen::Vector3f ConstPointIterator::operator*() const
{
    if (index_ >= num_points_) {
        std::cerr << "ConstPointIterator: Dereferencing end or out-of-bounds iterator"
                  << std::endl;
        return Eigen::Vector3f::Zero();
    }
    return {*iter_x_, *iter_y_, *iter_z_};
}

ConstPointIterator & ConstPointIterator::operator++()
{
    if (index_ < num_points_) {
        ++index_;
        ++iter_x_;
        ++iter_y_;
        ++iter_z_;
    }
    return *this;
}

bool ConstPointIterator::operator!=(const ConstPointIterator & other) const
{
    return index_ != other.index_;
}

bool transformAndDeskewPointCloud(
    const sensor_msgs::msg::PointCloud2& msg,
    const Eigen::Vector3d& linear_vel,
    const Eigen::Vector3d& angular_vel,
    const Eigen::Isometry3d& T_out_sensor,
    sensor_msgs::msg::PointCloud2& out)
{
    if (!hasField<float>(msg, "x") || !hasField<float>(msg, "y") ||
        !hasField<float>(msg, "z") || !hasField<uint32_t>(msg, "t")) {
        return false;
    }

    out = msg;

    auto out_x = getFieldIterator<float>(out, "x");
    auto out_y = getFieldIterator<float>(out, "y");
    auto out_z = getFieldIterator<float>(out, "z");
    auto out_t = getFieldIterator<uint32_t>(out, "t");

    const double omega_norm = angular_vel.norm();
    const bool has_rotation = omega_norm > 1e-10;
    const Eigen::Vector3d omega_axis =
        has_rotation ? Eigen::Vector3d(angular_vel / omega_norm) : Eigen::Vector3d::UnitZ();

    uint32_t last_t = std::numeric_limits<uint32_t>::max();
    Eigen::Matrix4d M_combined = T_out_sensor.matrix();

    const size_t num_points = static_cast<size_t>(out.width) * out.height;
    for (size_t i = 0; i < num_points; ++i, ++out_x, ++out_y, ++out_z, ++out_t) {
        if (!std::isfinite(*out_x) || !std::isfinite(*out_y) || !std::isfinite(*out_z)) {
            continue;
        }

        if (*out_t != last_t) {
            last_t = *out_t;
            const double dt = static_cast<double>(last_t) * 1e-9;

            Eigen::Isometry3d T_deskew = Eigen::Isometry3d::Identity();
            if (has_rotation) {
                T_deskew.linear() =
                    Eigen::AngleAxisd(omega_norm * dt, omega_axis).toRotationMatrix();
            }
            T_deskew.translation() = linear_vel * dt;
            M_combined = T_deskew.matrix() * T_out_sensor.matrix();
        }

        const Eigen::Vector4d p =
            M_combined * Eigen::Vector4d(*out_x, *out_y, *out_z, 1.0);
        *out_x = static_cast<float>(p.x());
        *out_y = static_cast<float>(p.y());
        *out_z = static_cast<float>(p.z());
    }

    return true;
}

}  // namespace pointcloud
}  // namespace hatchbed_common
