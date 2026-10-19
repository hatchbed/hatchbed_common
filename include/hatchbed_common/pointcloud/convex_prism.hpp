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

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <hatchbed_common/pointcloud/convex_hull_2d.hpp>

namespace hatchbed_common {
namespace pointcloud {

// A prism defined by a convex polygon cross-section in XY extruded along the
// Z axis between min_z and max_z. All coordinates are in a single consistent
// frame -- the caller is responsible for any required frame transforms.
struct ConvexPrism {
    std::vector<Eigen::Vector2f> hull;  // CCW 2D convex hull of XY cross-section
    float min_z = 0.0f;
    float max_z = 0.0f;
};

// Build a ConvexPrism from a collection of 3D points by projecting onto the XY
// plane, computing their 2D convex hull, and recording the full Z extent.
//
// ContainerT must support range-for. Each element must expose .x, .y, .z
// numeric members (e.g. pcl::PointXYZ or Eigen::Vector3f). Returns an empty
// prism if the input is empty.
template <typename ContainerT>
ConvexPrism makeConvexPrism(const ContainerT& points) {
    std::vector<Eigen::Vector2f> xy_pts;
    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();
    for (const auto& p : points) {
        xy_pts.emplace_back(static_cast<float>(p.x), static_cast<float>(p.y));
        min_z = std::min(min_z, static_cast<float>(p.z));
        max_z = std::max(max_z, static_cast<float>(p.z));
    }
    if (xy_pts.empty()) {
        return {};
    }
    return {convexHull2D(std::move(xy_pts)), min_z, max_z};
}

// Build a ConvexPrism from Eigen vertices. The template above wants .x/.y/.z as members,
// which Eigen exposes as methods, so vertices carried as Eigen::Vector3f need this.
ConvexPrism makeConvexPrism(const std::vector<Eigen::Vector3f>& points);

// Build a ConvexPrism from a PointCloud2 by projecting XYZ onto the XY plane,
// computing the 2D convex hull, and recording the full Z extent. The cloud must
// have x, y, z fields of type FLOAT32. NaN/Inf points are ignored. Returns an
// empty prism if the cloud has no valid points or lacks required fields.
ConvexPrism makeConvexPrism(const sensor_msgs::msg::PointCloud2& cloud);

// Return a new ConvexPrism expanded outward by padding on all sides: Z bounds
// grow by padding in each direction, and each hull edge is translated outward
// by padding along its outward normal. New hull vertices are the intersections
// of consecutive offset edges. The result is a slight over-approximation of
// the Minkowski sum at sharp corners (corners are extended rather than rounded).
ConvexPrism padConvexPrism(const ConvexPrism& prism, float padding);

// Return a new ConvexPrism holding the volume of prism transformed by pose.
//
// A prism is a footprint extruded along Z, so a pose that leaves the Z axis alone -- any
// translation, any rotation about Z -- maps it to another prism exactly, and that is the
// path taken when pose does so.
//
// Any other pose tilts the volume, and a tilted prism is not a prism: no footprint extruded
// along Z is equal to it. What comes back is then the smallest prism that CONTAINS the
// tilted volume, built by transforming all of the original's corners and re-hulling them.
// That never loses part of the volume, which matters when a prism is being used to decide
// what to remove, but it can be a good deal larger than the volume it stands for -- the
// worse the more of a prism's height is empty. A caller that still has the vertices the
// prism was built from should rebuild from those instead: the prism already over-covers
// whatever it was built from, and tilting an over-approximation compounds it. A diagonal
// strut through a 1 x 1 x 3 m envelope prisms to that whole block; tilt the block 45 degrees
// and it becomes 2.8 m across, where re-hulling the strut's own vertices gives a sliver.
ConvexPrism transformConvexPrism(const ConvexPrism& prism, const Eigen::Isometry3f& pose);

// The prism's footprint bounds, for a cheap reject before the polygon test. Empty prisms
// give an inverted (max < min) box, which no point is inside.
struct PrismBounds {
    Eigen::Vector2f min_xy = Eigen::Vector2f::Zero();
    Eigen::Vector2f max_xy = Eigen::Vector2f::Zero();
    float min_z = 1.0f;
    float max_z = 0.0f;

    bool contains(const Eigen::Vector3f& point) const {
        return point.z() >= min_z && point.z() <= max_z &&
               point.x() >= min_xy.x() && point.x() <= max_xy.x() &&
               point.y() >= min_xy.y() && point.y() <= max_xy.y();
    }
};

PrismBounds boundsOfConvexPrism(const ConvexPrism& prism);

// Whether a point lies within prism, expanded by tolerance. Both must be in the same frame.
bool insideConvexPrism(const Eigen::Vector3f& point, const ConvexPrism& prism,
                        float tolerance = 0.0f);

// Copy points from cloud to output, dropping every point inside any of prisms and keeping
// the rest, compacted. All original fields are preserved and the header is untouched, so
// the output stays in the input's frame. The cloud and the prisms must be expressed in the
// same frame, EXCEPT that cloud_to_prism_frame is applied to each point first -- which is
// what lets a cloud be tested against prisms held in a body frame without transforming and
// transforming back, since the point copied out is the original and not the transformed one.
//
// The cloud must have x, y, z fields of type FLOAT32. Points that are not finite are
// dropped: they are not evidence of anything, and no consumer of a compacted cloud wants
// them.
//
// removed, when given, receives exactly the points output did not, in the same form. It
// costs a second buffer and nothing else -- the decision has already been made either way --
// so it is worth asking for whenever somebody is looking at what the filter is eating.
//
// Returns the number of points removed, so a caller can report it.
size_t removeConvexPrisms(const sensor_msgs::msg::PointCloud2& cloud,
                           const std::vector<ConvexPrism>& prisms,
                           const std::vector<PrismBounds>& bounds,
                           const Eigen::Isometry3f& cloud_to_prism_frame,
                           sensor_msgs::msg::PointCloud2& output,
                           sensor_msgs::msg::PointCloud2* removed = nullptr);

// Copy points from cloud to output, keeping only those whose distance to the
// surface of prism is at most tolerance. All original fields are preserved.
// The cloud and prism must be in the same frame. NaN/Inf points are dropped.
// The cloud must have x, y, z fields of type FLOAT32.
std::shared_ptr<sensor_msgs::msg::PointCloud2> cropConvexPrism(
    const sensor_msgs::msg::PointCloud2& cloud,
    const ConvexPrism& prism,
    float tolerance);

// Return a copy of input containing only those points whose distance to the
// surface of prism is at most tolerance. The input cloud and prism must
// already be in the same frame.
//
// ContainerT must support range-for and push_back. Each element must expose
// .x, .y, .z numeric members (e.g. pcl::PointXYZ or Eigen::Vector3f).
template <typename ContainerT>
ContainerT cropToConvexPrism(
    const ContainerT& input,
    const ConvexPrism& prism,
    float tolerance)
{
    ContainerT output;
    if (prism.hull.empty()) {
        return output;
    }
    const float z_min = prism.min_z - tolerance;
    const float z_max = prism.max_z + tolerance;
    for (const auto& pt : input) {
        if (static_cast<float>(pt.z) < z_min || static_cast<float>(pt.z) > z_max) continue;
        if (distToConvexPolygon2D(
                Eigen::Vector2f(static_cast<float>(pt.x), static_cast<float>(pt.y)),
                prism.hull) > tolerance) continue;
        output.push_back(pt);
    }
    return output;
}

}  // namespace pointcloud
}  // namespace hatchbed_common
