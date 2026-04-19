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

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

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

}  // namespace pointcloud
}  // namespace hatchbed_common
