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

#include <hatchbed_common/pointcloud/convex_prism.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <hatchbed_common/pointcloud/point_cloud2_util.hpp>

namespace hatchbed_common {
namespace pointcloud {

ConvexPrism makeConvexPrism(const sensor_msgs::msg::PointCloud2& cloud) {
    if (!hasXYZFloat(cloud)) {
        return {};
    }

    sensor_msgs::PointCloud2ConstIterator<float> ix(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iy(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iz(cloud, "z");

    const uint32_t n = cloud.width * cloud.height;
    std::vector<Eigen::Vector2f> xy_pts;
    xy_pts.reserve(n);
    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();

    for (uint32_t i = 0; i < n; ++i, ++ix, ++iy, ++iz) {
        const float x = *ix, y = *iy, z = *iz;
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
        xy_pts.emplace_back(x, y);
        min_z = std::min(min_z, z);
        max_z = std::max(max_z, z);
    }

    if (xy_pts.empty()) {
        return {};
    }
    return {convexHull2D(std::move(xy_pts)), min_z, max_z};
}

std::shared_ptr<sensor_msgs::msg::PointCloud2> cropConvexPrism(
    const sensor_msgs::msg::PointCloud2& cloud,
    const ConvexPrism& prism,
    float tolerance)
{
    auto output = std::make_shared<sensor_msgs::msg::PointCloud2>();
    output->header = cloud.header;
    output->fields = cloud.fields;
    output->point_step = cloud.point_step;
    output->is_bigendian = cloud.is_bigendian;
    output->is_dense = cloud.is_dense;
    output->height = 1;

    if (prism.hull.empty() || !hasXYZFloat(cloud)) {
        output->width = 0;
        output->row_step = 0;
        return output;
    }

    sensor_msgs::PointCloud2ConstIterator<float> ix(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iy(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iz(cloud, "z");

    const float z_min = prism.min_z - tolerance;
    const float z_max = prism.max_z + tolerance;
    const uint32_t n = cloud.width * cloud.height;

    output->data.reserve(static_cast<size_t>(n) * cloud.point_step);

    for (uint32_t i = 0; i < n; ++i, ++ix, ++iy, ++iz) {
        const float x = *ix, y = *iy, z = *iz;
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
        if (z < z_min || z > z_max) continue;
        if (distToConvexPolygon2D(Eigen::Vector2f(x, y), prism.hull) > tolerance) continue;
        const uint8_t* src = cloud.data.data() + static_cast<size_t>(i) * cloud.point_step;
        output->data.insert(output->data.end(), src, src + cloud.point_step);
    }

    output->width = static_cast<uint32_t>(output->data.size() / cloud.point_step);
    output->row_step = static_cast<uint32_t>(output->data.size());
    return output;
}

ConvexPrism padConvexPrism(const ConvexPrism& prism, float padding) {
    ConvexPrism result;
    result.min_z = prism.min_z - padding;
    result.max_z = prism.max_z + padding;

    const int n = static_cast<int>(prism.hull.size());
    if (n == 0) {
        return result;
    }

    // For each edge of the CCW hull, translate it outward by padding along its
    // outward normal: for edge direction d, outward normal = (d.y(), -d.x()).
    std::vector<Eigen::Vector2f> origins(n), dirs(n);
    for (int i = 0; i < n; ++i) {
        const Eigen::Vector2f d = (prism.hull[(i + 1) % n] - prism.hull[i]).normalized();
        dirs[i] = d;
        origins[i] = prism.hull[i] + padding * Eigen::Vector2f(d.y(), -d.x());
    }

    // Each output vertex is the intersection of the offset edges meeting at the
    // corresponding input vertex.
    result.hull.reserve(n);
    for (int j = 0; j < n; ++j) {
        const int prev = (j - 1 + n) % n;
        const Eigen::Vector2f& p1 = origins[prev];
        const Eigen::Vector2f& d1 = dirs[prev];
        const Eigen::Vector2f& p2 = origins[j];
        const Eigen::Vector2f& d2 = dirs[j];
        const float cross = d1.x() * d2.y() - d1.y() * d2.x();
        if (std::abs(cross) < 1e-9f) {
            result.hull.push_back(p2);
        } else {
            const Eigen::Vector2f dp = p2 - p1;
            const float t = (dp.x() * d2.y() - dp.y() * d2.x()) / cross;
            result.hull.push_back(p1 + t * d1);
        }
    }

    return result;
}

ConvexPrism transformConvexPrism(const ConvexPrism& prism, const Eigen::Isometry3f& pose) {
    ConvexPrism result;
    result.hull.reserve(prism.hull.size());
    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();
    for (const auto& v : prism.hull) {
        const Eigen::Vector3f bot = pose * Eigen::Vector3f(v.x(), v.y(), prism.min_z);
        const Eigen::Vector3f top = pose * Eigen::Vector3f(v.x(), v.y(), prism.max_z);
        result.hull.emplace_back(bot.x(), bot.y());
        min_z = std::min({min_z, bot.z(), top.z()});
        max_z = std::max({max_z, bot.z(), top.z()});
    }
    result.min_z = min_z;
    result.max_z = max_z;
    return result;
}

}  // namespace pointcloud
}  // namespace hatchbed_common
