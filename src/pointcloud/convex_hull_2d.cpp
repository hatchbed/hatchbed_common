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

#include <hatchbed_common/pointcloud/convex_hull_2d.hpp>

#include <algorithm>
#include <limits>

namespace hatchbed_common {
namespace pointcloud {

std::vector<Eigen::Vector2f> convexHull2D(std::vector<Eigen::Vector2f> pts) {
    const int n = static_cast<int>(pts.size());
    if (n < 2) return pts;

    std::sort(pts.begin(), pts.end(), [](const Eigen::Vector2f& a, const Eigen::Vector2f& b) {
        return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
    });

    auto cross = [](const Eigen::Vector2f& o,
                    const Eigen::Vector2f& a,
                    const Eigen::Vector2f& b) -> float {
        return (a.x() - o.x()) * (b.y() - o.y()) - (a.y() - o.y()) * (b.x() - o.x());
    };

    std::vector<Eigen::Vector2f> hull;
    hull.reserve(2 * n);

    for (int i = 0; i < n; i++) {
        while (hull.size() >= 2 && cross(hull[hull.size()-2], hull.back(), pts[i]) <= 0.0f)
            hull.pop_back();
        hull.push_back(pts[i]);
    }
    const int lower = static_cast<int>(hull.size());
    for (int i = n - 2; i >= 0; i--) {
        while (static_cast<int>(hull.size()) > lower &&
               cross(hull[hull.size()-2], hull.back(), pts[i]) <= 0.0f)
            hull.pop_back();
        hull.push_back(pts[i]);
    }
    hull.pop_back();
    return hull;
}

float distToConvexPolygon2D(
    const Eigen::Vector2f& p,
    const std::vector<Eigen::Vector2f>& hull)
{
    const int n = static_cast<int>(hull.size());
    if (n == 0) return 0.0f;
    if (n == 1) return (p - hull[0]).norm();

    bool inside = true;
    float min_dist = std::numeric_limits<float>::max();
    for (int i = 0; i < n; i++) {
        const Eigen::Vector2f& a = hull[i];
        const Eigen::Vector2f& b = hull[(i + 1) % n];
        const Eigen::Vector2f ab = b - a;
        const Eigen::Vector2f ap = p - a;
        if (ab.x() * ap.y() - ab.y() * ap.x() < 0.0f) inside = false;
        const float t = std::max(0.0f, std::min(1.0f, ap.dot(ab) / ab.squaredNorm()));
        min_dist = std::min(min_dist, (p - (a + t * ab)).norm());
    }
    return inside ? 0.0f : min_dist;
}

}  // namespace pointcloud
}  // namespace hatchbed_common
