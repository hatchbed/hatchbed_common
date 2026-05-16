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

#include <gtest/gtest.h>
#include <hatchbed_common/pointcloud/convex_hull_2d.hpp>

#include <cmath>
#include <vector>

#include <Eigen/Core>

using hatchbed_common::pointcloud::convexHull2D;
using hatchbed_common::pointcloud::distToConvexPolygon2D;

// CCW unit square hull used across multiple tests.
static std::vector<Eigen::Vector2f> squareHull() {
    return {
        {-0.5f, -0.5f},
        { 0.5f, -0.5f},
        { 0.5f,  0.5f},
        {-0.5f,  0.5f},
    };
}

// =============================================================================
// convexHull2D
// =============================================================================

TEST(ConvexHull2D, EmptyInput) {
    std::vector<Eigen::Vector2f> pts;
    auto hull = convexHull2D(pts);
    EXPECT_TRUE(hull.empty());
}

TEST(ConvexHull2D, SinglePoint) {
    std::vector<Eigen::Vector2f> pts = {{1.0f, 2.0f}};
    auto hull = convexHull2D(pts);
    ASSERT_EQ(hull.size(), 1u);
    EXPECT_NEAR(hull[0].x(), 1.0f, 1e-5f);
    EXPECT_NEAR(hull[0].y(), 2.0f, 1e-5f);
}

TEST(ConvexHull2D, TwoPoints) {
    std::vector<Eigen::Vector2f> pts = {{0.0f, 0.0f}, {1.0f, 1.0f}};
    auto hull = convexHull2D(pts);
    ASSERT_EQ(hull.size(), 2u);
}

TEST(ConvexHull2D, SquareCorners) {
    std::vector<Eigen::Vector2f> pts = {
        {-0.5f, -0.5f},
        { 0.5f, -0.5f},
        { 0.5f,  0.5f},
        {-0.5f,  0.5f},
        { 0.0f,  0.0f},  // interior point that must not appear in hull
    };
    auto hull = convexHull2D(pts);
    EXPECT_EQ(hull.size(), 4u);
}

TEST(ConvexHull2D, CollinearPoints) {
    std::vector<Eigen::Vector2f> pts = {
        {0.0f, 0.0f},
        {1.0f, 0.0f},
        {2.0f, 0.0f},
        {3.0f, 0.0f},
    };
    EXPECT_NO_THROW({ auto hull = convexHull2D(pts); });
    auto hull = convexHull2D(pts);
    EXPECT_FALSE(hull.empty());
}

TEST(ConvexHull2D, HullIsCounterClockwise) {
    std::vector<Eigen::Vector2f> pts = {
        {-0.5f, -0.5f},
        { 0.5f, -0.5f},
        { 0.5f,  0.5f},
        {-0.5f,  0.5f},
        { 0.0f,  0.0f},
    };
    auto hull = convexHull2D(pts);
    ASSERT_GE(hull.size(), 3u);

    // Cross product of consecutive edges must be positive (CCW) for all vertices.
    const int n = static_cast<int>(hull.size());
    for (int i = 0; i < n; ++i) {
        const Eigen::Vector2f& o = hull[i];
        const Eigen::Vector2f& a = hull[(i + 1) % n];
        const Eigen::Vector2f& b = hull[(i + 2) % n];
        const float cross = (a.x() - o.x()) * (b.y() - o.y()) -
                            (a.y() - o.y()) * (b.x() - o.x());
        EXPECT_GT(cross, 0.0f) << "Failed at vertex " << i;
    }
}

// =============================================================================
// distToConvexPolygon2D
// =============================================================================

TEST(DistToConvexPolygon2D, EmptyHull) {
    std::vector<Eigen::Vector2f> hull;
    EXPECT_NEAR(distToConvexPolygon2D({1.0f, 1.0f}, hull), 0.0f, 1e-5f);
}

TEST(DistToConvexPolygon2D, SingleVertexHull) {
    std::vector<Eigen::Vector2f> hull = {{1.0f, 0.0f}};
    EXPECT_NEAR(distToConvexPolygon2D({4.0f, 0.0f}, hull), 3.0f, 1e-5f);
}

TEST(DistToConvexPolygon2D, PointInsideSquare) {
    auto hull = squareHull();
    EXPECT_NEAR(distToConvexPolygon2D({0.0f, 0.0f}, hull), 0.0f, 1e-5f);
}

TEST(DistToConvexPolygon2D, PointOnEdge) {
    auto hull = squareHull();
    // (0.5, 0) lies on the right edge of the unit square.
    EXPECT_NEAR(distToConvexPolygon2D({0.5f, 0.0f}, hull), 0.0f, 1e-5f);
}

TEST(DistToConvexPolygon2D, PointOutsideEdge) {
    auto hull = squareHull();
    // (0.8, 0) is 0.3 from the right edge at x=0.5.
    EXPECT_NEAR(distToConvexPolygon2D({0.8f, 0.0f}, hull), 0.3f, 1e-5f);
}

TEST(DistToConvexPolygon2D, PointOutsideCorner) {
    auto hull = squareHull();
    // (0.7, 0.7) is closest to corner (0.5, 0.5): dist = sqrt(0.04+0.04).
    const float expected = std::sqrt(0.04f + 0.04f);
    EXPECT_NEAR(distToConvexPolygon2D({0.7f, 0.7f}, hull), expected, 1e-5f);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
