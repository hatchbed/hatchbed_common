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
#include <cstring>
#include <limits>
#include <vector>

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

    // Shorter than this an edge has no direction worth trusting: normalizing it amplifies
    // whatever noise put its endpoints apart, and a zero-length one normalizes to NaN. A hull
    // over a mesh's vertices routinely has a few.
    constexpr float kMinEdge = 1e-4f;
    // How far past a vertex a mitred corner may reach, as a multiple of the padding. Two
    // nearly-parallel offset edges meet a long way from the corner they belong to -- in the
    // limit, nowhere -- so past this the corner is cut instead. Without it a shallow corner
    // flings a vertex far enough to leave the ring self-intersecting, and a self-intersecting
    // ring is not a polygon any containment test can be asked about.
    constexpr float kMitreLimit = 2.0f;

    // Each edge, as an outward-offset line: origin and direction. Edges too short to have one
    // are dropped, which loses nothing -- their endpoints are covered by their neighbours.
    std::vector<Eigen::Vector2f> origins;  // the offset line's start, i.e. offset from()
    std::vector<Eigen::Vector2f> dirs;
    std::vector<Eigen::Vector2f> from;
    std::vector<Eigen::Vector2f> to;
    for (int i = 0; i < n; ++i) {
        const Eigen::Vector2f& a = prism.hull[i];
        const Eigen::Vector2f& b = prism.hull[(i + 1) % n];
        const Eigen::Vector2f edge = b - a;
        if (edge.norm() < kMinEdge) {
            continue;
        }
        const Eigen::Vector2f d = edge.normalized();
        // For a CCW hull the outward normal of direction d is (d.y, -d.x).
        const Eigen::Vector2f normal(d.y(), -d.x());
        dirs.push_back(d);
        origins.push_back(a + padding * normal);
        from.push_back(a);
        to.push_back(b + padding * normal);
    }

    const int edges = static_cast<int>(dirs.size());
    if (edges == 0) {
        // A hull that is one point, or several indistinguishable ones: grow it into a square
        // rather than return nothing, so a degenerate shape still covers its own padding.
        const Eigen::Vector2f& c = prism.hull.front();
        result.hull = {{c.x() - padding, c.y() - padding},
                       {c.x() + padding, c.y() - padding},
                       {c.x() + padding, c.y() + padding},
                       {c.x() - padding, c.y() + padding}};
        return result;
    }

    // Every edge's own offset endpoints go in, which is what guarantees the result covers the
    // original grown by padding along each edge. Corners then add their mitre where it is
    // well conditioned, extending the corner rather than cutting it.
    std::vector<Eigen::Vector2f> offset;
    offset.reserve(static_cast<size_t>(edges) * 3);
    for (int j = 0; j < edges; ++j) {
        const int next = (j + 1) % edges;
        offset.push_back(origins[j]);
        offset.push_back(to[j]);

        const Eigen::Vector2f& p1 = origins[j];
        const Eigen::Vector2f& d1 = dirs[j];
        const Eigen::Vector2f& p2 = origins[next];
        const Eigen::Vector2f& d2 = dirs[next];
        const float cross = d1.x() * d2.y() - d1.y() * d2.x();
        if (std::abs(cross) < 1e-6f) {
            continue;  // as good as parallel: the two offsets already meet where they are
        }
        const Eigen::Vector2f dp = p2 - p1;
        const float t = (dp.x() * d2.y() - dp.y() * d2.x()) / cross;
        const Eigen::Vector2f mitre = p1 + t * d1;
        if ((mitre - from[next]).norm() <= kMitreLimit * padding) {
            offset.push_back(mitre);
        }
    }

    // Re-hulled rather than trusted: the offsets are computed per corner and a corner knows
    // nothing about the rest of the ring, so only this makes the result convex, wound the way
    // it came in, and free of the duplicate and out-of-order vertices that make a containment
    // test answer nonsense. It can only drop vertices that were inside the others.
    result.hull = convexHull2D(std::move(offset));
    return result;
}

ConvexPrism makeConvexPrism(const std::vector<Eigen::Vector3f>& points) {
    std::vector<Eigen::Vector2f> xy_pts;
    xy_pts.reserve(points.size());
    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();
    for (const auto& p : points) {
        if (!p.allFinite()) continue;
        xy_pts.emplace_back(p.x(), p.y());
        min_z = std::min(min_z, p.z());
        max_z = std::max(max_z, p.z());
    }
    if (xy_pts.empty()) {
        return {};
    }
    return {convexHull2D(std::move(xy_pts)), min_z, max_z};
}

ConvexPrism transformConvexPrism(const ConvexPrism& prism, const Eigen::Isometry3f& pose) {
    if (prism.hull.empty()) {
        return {};
    }

    // Whether the pose leaves the Z axis where it found it, which is what decides whether the
    // result is a prism at all. Rotation about Z and any translation do; anything else tilts
    // the volume. The tolerance is a numerical one, not a modelling one -- a caller deciding
    // how much tilt it can live with belongs on the other side of this call.
    const Eigen::Vector3f up = pose.linear() * Eigen::Vector3f::UnitZ();
    if ((up - Eigen::Vector3f::UnitZ()).squaredNorm() < 1e-12f) {
        ConvexPrism result;
        result.hull.reserve(prism.hull.size());
        for (const auto& v : prism.hull) {
            const Eigen::Vector3f moved = pose * Eigen::Vector3f(v.x(), v.y(), prism.min_z);
            result.hull.emplace_back(moved.x(), moved.y());
        }
        // Rotating about Z carries the footprint round without reordering or unwinding it, so
        // the hull stays a hull and does not have to be rebuilt.
        const float bottom = (pose * Eigen::Vector3f(0.0f, 0.0f, prism.min_z)).z();
        const float top = (pose * Eigen::Vector3f(0.0f, 0.0f, prism.max_z)).z();
        result.min_z = std::min(bottom, top);
        result.max_z = std::max(bottom, top);
        return result;
    }

    // Tilted: the smallest prism containing the tilted volume, from every corner rather than
    // from the bottom ring alone. Taking only the bottom ring would drop wherever the top
    // ring swung out to, which is not an approximation but a hole. See the header on why a
    // caller holding the original vertices should use those instead.
    std::vector<Eigen::Vector3f> corners;
    corners.reserve(prism.hull.size() * 2);
    for (const auto& v : prism.hull) {
        corners.push_back(pose * Eigen::Vector3f(v.x(), v.y(), prism.min_z));
        corners.push_back(pose * Eigen::Vector3f(v.x(), v.y(), prism.max_z));
    }
    return makeConvexPrism(corners);
}

PrismBounds boundsOfConvexPrism(const ConvexPrism& prism) {
    PrismBounds bounds;
    if (prism.hull.empty()) {
        return bounds;  // leaves max_z < min_z, which contains nothing
    }
    Eigen::Vector2f min_xy(std::numeric_limits<float>::max(),
                           std::numeric_limits<float>::max());
    Eigen::Vector2f max_xy(std::numeric_limits<float>::lowest(),
                           std::numeric_limits<float>::lowest());
    for (const auto& v : prism.hull) {
        min_xy = min_xy.cwiseMin(v);
        max_xy = max_xy.cwiseMax(v);
    }
    bounds.min_xy = min_xy;
    bounds.max_xy = max_xy;
    bounds.min_z = prism.min_z;
    bounds.max_z = prism.max_z;
    return bounds;
}

bool insideConvexPrism(const Eigen::Vector3f& point, const ConvexPrism& prism,
                        float tolerance) {
    if (point.z() < prism.min_z - tolerance || point.z() > prism.max_z + tolerance) {
        return false;
    }
    const Eigen::Vector2f xy(point.x(), point.y());
    if (tolerance <= 0.0f) {
        return insideConvexPolygon2D(xy, prism.hull);
    }
    return distToConvexPolygon2D(xy, prism.hull) <= tolerance;
}

size_t removeConvexPrisms(const sensor_msgs::msg::PointCloud2& cloud,
                           const std::vector<ConvexPrism>& prisms,
                           const std::vector<PrismBounds>& bounds,
                           const Eigen::Isometry3f& cloud_to_prism_frame,
                           sensor_msgs::msg::PointCloud2& output,
                           sensor_msgs::msg::PointCloud2* removed) {
    auto x_iter = getFieldIterator<float>(cloud, "x");
    auto y_iter = getFieldIterator<float>(cloud, "y");
    auto z_iter = getFieldIterator<float>(cloud, "z");
    if (!x_iter || !y_iter || !z_iter) {
        return 0;
    }

    output = cloud;
    if (removed != nullptr) {
        *removed = cloud;
    }
    const size_t size = static_cast<size_t>(cloud.width) * cloud.height;
    size_t kept = 0;
    size_t cut = 0;
    for (size_t i = 0; i < size; ++i, ++x_iter, ++y_iter, ++z_iter) {
        const Eigen::Vector3f point(*x_iter, *y_iter, *z_iter);
        bool drop = !point.allFinite();
        if (!drop) {
            const Eigen::Vector3f in_prism_frame = cloud_to_prism_frame * point;
            for (size_t p = 0; p < prisms.size(); ++p) {
                // The footprint box first: it is four comparisons against a point most
                // prisms are nowhere near, where the polygon test is a loop over edges.
                if (p < bounds.size() && !bounds[p].contains(in_prism_frame)) {
                    continue;
                }
                if (insideConvexPrism(in_prism_frame, prisms[p])) {
                    drop = true;
                    break;
                }
            }
        }
        if (drop) {
            if (removed != nullptr) {
                std::memcpy(&removed->data[cut * removed->point_step],
                            &cloud.data[i * cloud.point_step], removed->point_step);
            }
            ++cut;
            continue;
        }
        std::memcpy(&output.data[kept * output.point_step],
                    &cloud.data[i * cloud.point_step], output.point_step);
        ++kept;
    }

    output.width = static_cast<uint32_t>(kept);
    output.height = 1;
    output.row_step = output.point_step * output.width;
    output.is_dense = true;  // every point left is finite, and none of them is a gap
    output.data.resize(output.row_step);

    if (removed != nullptr) {
        removed->width = static_cast<uint32_t>(cut);
        removed->height = 1;
        removed->row_step = removed->point_step * removed->width;
        removed->is_dense = true;
        removed->data.resize(removed->row_step);
    }
    return cut;
}

}  // namespace pointcloud
}  // namespace hatchbed_common
