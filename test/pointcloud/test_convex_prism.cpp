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
#include <hatchbed_common/pointcloud/convex_prism.hpp>

#include <cmath>
#include <limits>
#include <tuple>
#include <vector>

#include <Eigen/Geometry>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

using hatchbed_common::pointcloud::ConvexPrism;
using hatchbed_common::pointcloud::makeConvexPrism;
using hatchbed_common::pointcloud::padConvexPrism;
using hatchbed_common::pointcloud::transformConvexPrism;
using hatchbed_common::pointcloud::cropToConvexPrism;
using hatchbed_common::pointcloud::cropConvexPrism;

// POD type satisfying the .x, .y, .z requirement for template overloads.
struct Pt { float x, y, z; };

// CCW unit square hull, z in [0, 1].
static ConvexPrism makeSquarePrism() {
    ConvexPrism p;
    p.hull = {
        {-0.5f, -0.5f},
        { 0.5f, -0.5f},
        { 0.5f,  0.5f},
        {-0.5f,  0.5f},
    };
    p.min_z = 0.0f;
    p.max_z = 1.0f;
    return p;
}

// Build a PointCloud2 with float32 x, y, z, intensity fields (point_step=16).
static sensor_msgs::msg::PointCloud2 makeXYZCloud(
    const std::vector<std::tuple<float, float, float, float>>& pts)
{
    sensor_msgs::msg::PointCloud2 msg;
    msg.height = 1;
    msg.width  = static_cast<uint32_t>(pts.size());
    msg.is_bigendian = false;
    msg.is_dense     = true;

    auto addField = [&](const std::string& name, uint32_t offset) {
        sensor_msgs::msg::PointField f;
        f.name     = name;
        f.offset   = offset;
        f.datatype = sensor_msgs::msg::PointField::FLOAT32;
        f.count    = 1;
        msg.fields.push_back(f);
    };

    addField("x",         0);
    addField("y",         4);
    addField("z",         8);
    addField("intensity", 12);

    msg.point_step = 16;
    msg.row_step   = msg.point_step * msg.width;
    msg.data.resize(msg.row_step, 0);

    sensor_msgs::PointCloud2Iterator<float> ix(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iy(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iz(msg, "z");
    sensor_msgs::PointCloud2Iterator<float> ii(msg, "intensity");

    for (const auto& [x, y, z, intensity] : pts) {
        *ix = x; *iy = y; *iz = z; *ii = intensity;
        ++ix; ++iy; ++iz; ++ii;
    }

    return msg;
}

// =============================================================================
// makeConvexPrism (template)
// =============================================================================

TEST(MakeConvexPrismTemplate, EmptyContainer) {
    std::vector<Pt> pts;
    auto prism = makeConvexPrism(pts);
    EXPECT_TRUE(prism.hull.empty());
}

TEST(MakeConvexPrismTemplate, FlatPoints) {
    std::vector<Pt> pts = {
        {-0.5f, -0.5f, 2.0f},
        { 0.5f, -0.5f, 2.0f},
        { 0.5f,  0.5f, 5.0f},
        {-0.5f,  0.5f, 5.0f},
    };
    auto prism = makeConvexPrism(pts);
    EXPECT_EQ(prism.hull.size(), 4u);
    EXPECT_NEAR(prism.min_z, 2.0f, 1e-5f);
    EXPECT_NEAR(prism.max_z, 5.0f, 1e-5f);
}

TEST(MakeConvexPrismTemplate, InteriorPointsExcluded) {
    std::vector<Pt> pts = {
        {-0.5f, -0.5f, 0.0f},
        { 0.5f, -0.5f, 0.0f},
        { 0.5f,  0.5f, 0.0f},
        {-0.5f,  0.5f, 0.0f},
        { 0.0f,  0.0f, 0.0f},  // interior
    };
    auto prism = makeConvexPrism(pts);
    EXPECT_EQ(prism.hull.size(), 4u);
}

// =============================================================================
// makeConvexPrism (PointCloud2)
// =============================================================================

TEST(MakeConvexPrismCloud, MissingXYZFields) {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.height = 1;
    cloud.width  = 0;
    cloud.point_step = 0;
    cloud.row_step   = 0;
    auto prism = makeConvexPrism(cloud);
    EXPECT_TRUE(prism.hull.empty());
}

TEST(MakeConvexPrismCloud, EmptyCloud) {
    auto cloud = makeXYZCloud({});
    auto prism = makeConvexPrism(cloud);
    EXPECT_TRUE(prism.hull.empty());
}

TEST(MakeConvexPrismCloud, ValidCloud) {
    auto cloud = makeXYZCloud({
        {-0.5f, -0.5f, 1.0f, 0.0f},
        { 0.5f, -0.5f, 2.0f, 0.0f},
        { 0.5f,  0.5f, 3.0f, 0.0f},
        {-0.5f,  0.5f, 4.0f, 0.0f},
    });
    auto prism = makeConvexPrism(cloud);
    EXPECT_EQ(prism.hull.size(), 4u);
    EXPECT_NEAR(prism.min_z, 1.0f, 1e-5f);
    EXPECT_NEAR(prism.max_z, 4.0f, 1e-5f);
}

TEST(MakeConvexPrismCloud, NaNPointsSkipped) {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    auto cloud = makeXYZCloud({
        {nan, nan, nan, 0.0f},
        {-0.5f, -0.5f, 2.0f, 0.0f},
        { 0.5f, -0.5f, 2.0f, 0.0f},
        { 0.5f,  0.5f, 5.0f, 0.0f},
        {-0.5f,  0.5f, 5.0f, 0.0f},
    });
    auto prism = makeConvexPrism(cloud);
    EXPECT_EQ(prism.hull.size(), 4u);
    EXPECT_NEAR(prism.min_z, 2.0f, 1e-5f);
    EXPECT_NEAR(prism.max_z, 5.0f, 1e-5f);
}

// =============================================================================
// padConvexPrism
// =============================================================================

TEST(PadConvexPrism, ZeroPadding) {
    auto prism  = makeSquarePrism();
    auto padded = padConvexPrism(prism, 0.0f);
    ASSERT_EQ(padded.hull.size(), prism.hull.size());
    for (size_t i = 0; i < prism.hull.size(); ++i) {
        EXPECT_NEAR(padded.hull[i].x(), prism.hull[i].x(), 1e-5f);
        EXPECT_NEAR(padded.hull[i].y(), prism.hull[i].y(), 1e-5f);
    }
    EXPECT_NEAR(padded.min_z, prism.min_z, 1e-5f);
    EXPECT_NEAR(padded.max_z, prism.max_z, 1e-5f);
}

TEST(PadConvexPrism, SquarePaddedByTenth) {
    auto prism  = makeSquarePrism();
    auto padded = padConvexPrism(prism, 0.1f);
    ASSERT_EQ(padded.hull.size(), 4u);
    for (const auto& v : padded.hull) {
        EXPECT_NEAR(std::abs(v.x()), 0.6f, 1e-5f);
        EXPECT_NEAR(std::abs(v.y()), 0.6f, 1e-5f);
    }
    EXPECT_NEAR(padded.min_z, -0.1f, 1e-5f);
    EXPECT_NEAR(padded.max_z,  1.1f, 1e-5f);
}

TEST(PadConvexPrism, EmptyHull) {
    ConvexPrism prism;
    prism.min_z = 0.0f;
    prism.max_z = 1.0f;
    EXPECT_NO_THROW({
        auto padded = padConvexPrism(prism, 0.2f);
        EXPECT_TRUE(padded.hull.empty());
        EXPECT_NEAR(padded.min_z, -0.2f, 1e-5f);
        EXPECT_NEAR(padded.max_z,  1.2f, 1e-5f);
    });
}

// =============================================================================
// transformConvexPrism
// =============================================================================

TEST(TransformConvexPrism, IdentityPreservesAll) {
    auto prism      = makeSquarePrism();
    auto transformed = transformConvexPrism(prism, Eigen::Isometry3f::Identity());
    ASSERT_EQ(transformed.hull.size(), prism.hull.size());
    for (size_t i = 0; i < prism.hull.size(); ++i) {
        EXPECT_NEAR(transformed.hull[i].x(), prism.hull[i].x(), 1e-5f);
        EXPECT_NEAR(transformed.hull[i].y(), prism.hull[i].y(), 1e-5f);
    }
    EXPECT_NEAR(transformed.min_z, prism.min_z, 1e-5f);
    EXPECT_NEAR(transformed.max_z, prism.max_z, 1e-5f);
}

TEST(TransformConvexPrism, PureTranslationShiftsHullAndZ) {
    auto prism = makeSquarePrism();
    Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
    T.translation() = Eigen::Vector3f(1.0f, 2.0f, 3.0f);
    auto transformed = transformConvexPrism(prism, T);

    ASSERT_EQ(transformed.hull.size(), prism.hull.size());
    for (size_t i = 0; i < prism.hull.size(); ++i) {
        EXPECT_NEAR(transformed.hull[i].x(), prism.hull[i].x() + 1.0f, 1e-5f);
        EXPECT_NEAR(transformed.hull[i].y(), prism.hull[i].y() + 2.0f, 1e-5f);
    }
    EXPECT_NEAR(transformed.min_z, prism.min_z + 3.0f, 1e-5f);
    EXPECT_NEAR(transformed.max_z, prism.max_z + 3.0f, 1e-5f);
}

TEST(TransformConvexPrism, RotationAboutZ90) {
    auto prism = makeSquarePrism();
    Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
    T.linear() = Eigen::AngleAxisf(static_cast<float>(M_PI / 2.0), Eigen::Vector3f::UnitZ())
                     .toRotationMatrix();
    auto transformed = transformConvexPrism(prism, T);

    ASSERT_EQ(transformed.hull.size(), prism.hull.size());
    for (size_t i = 0; i < prism.hull.size(); ++i) {
        // 90-deg rotation about Z: (x, y) -> (-y, x)
        EXPECT_NEAR(transformed.hull[i].x(), -prism.hull[i].y(), 1e-5f);
        EXPECT_NEAR(transformed.hull[i].y(),  prism.hull[i].x(), 1e-5f);
    }
    EXPECT_NEAR(transformed.min_z, prism.min_z, 1e-5f);
    EXPECT_NEAR(transformed.max_z, prism.max_z, 1e-5f);
}

// =============================================================================
// cropToConvexPrism (template)
// =============================================================================

TEST(CropToConvexPrism, EmptyPrism) {
    ConvexPrism prism;
    std::vector<Pt> pts = {{0.0f, 0.0f, 0.5f}};
    auto result = cropToConvexPrism(pts, prism, 0.0f);
    EXPECT_TRUE(result.empty());
}

TEST(CropToConvexPrism, PointInsidePrism) {
    auto prism = makeSquarePrism();
    std::vector<Pt> pts = {{0.0f, 0.0f, 0.5f}};
    auto result = cropToConvexPrism(pts, prism, 0.0f);
    EXPECT_EQ(result.size(), 1u);
}

TEST(CropToConvexPrism, PointOutsideXY) {
    auto prism = makeSquarePrism();
    std::vector<Pt> pts = {{2.0f, 0.0f, 0.5f}};
    auto result = cropToConvexPrism(pts, prism, 0.0f);
    EXPECT_TRUE(result.empty());
}

TEST(CropToConvexPrism, PointBelowZ) {
    auto prism = makeSquarePrism();
    std::vector<Pt> pts = {{0.0f, 0.0f, -1.0f}};
    auto result = cropToConvexPrism(pts, prism, 0.0f);
    EXPECT_TRUE(result.empty());
}

TEST(CropToConvexPrism, PointAboveZ) {
    auto prism = makeSquarePrism();
    std::vector<Pt> pts = {{0.0f, 0.0f, 2.0f}};
    auto result = cropToConvexPrism(pts, prism, 0.0f);
    EXPECT_TRUE(result.empty());
}

TEST(CropToConvexPrism, WithinTolerance) {
    auto prism = makeSquarePrism();
    // Point is 0.2m outside the right edge (at x=0.5).
    std::vector<Pt> pts = {{0.7f, 0.0f, 0.5f}};

    auto kept = cropToConvexPrism(pts, prism, 0.3f);
    EXPECT_EQ(kept.size(), 1u);

    auto dropped = cropToConvexPrism(pts, prism, 0.1f);
    EXPECT_TRUE(dropped.empty());
}

TEST(CropToConvexPrism, ReturnedContainerHasCorrectSize) {
    auto prism = makeSquarePrism();
    std::vector<Pt> pts = {
        { 0.0f,  0.0f, 0.5f},  // inside
        { 2.0f,  0.0f, 0.5f},  // outside XY
        { 0.0f,  0.0f, 5.0f},  // above Z
    };
    auto result = cropToConvexPrism(pts, prism, 0.0f);
    EXPECT_EQ(result.size(), 1u);
}

// =============================================================================
// cropConvexPrism (PointCloud2)
// =============================================================================

TEST(CropConvexPrismCloud, EmptyPrism) {
    ConvexPrism prism;
    auto cloud = makeXYZCloud({{0.0f, 0.0f, 0.5f, 1.0f}});
    auto out   = cropConvexPrism(cloud, prism, 0.0f);
    ASSERT_NE(out, nullptr);
    EXPECT_EQ(out->width, 0u);
    EXPECT_EQ(out->fields.size(), cloud.fields.size());
}

TEST(CropConvexPrismCloud, MissingXYZFields) {
    ConvexPrism prism = makeSquarePrism();
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.height = 1;
    cloud.width  = 0;
    cloud.point_step = 0;
    cloud.row_step   = 0;
    auto out = cropConvexPrism(cloud, prism, 0.0f);
    ASSERT_NE(out, nullptr);
    EXPECT_EQ(out->width, 0u);
}

TEST(CropConvexPrismCloud, NaNPointsDropped) {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    auto prism = makeSquarePrism();
    auto cloud = makeXYZCloud({
        {nan, nan, nan,  0.0f},
        {0.0f, 0.0f, 0.5f, 1.0f},
    });
    auto out = cropConvexPrism(cloud, prism, 0.0f);
    ASSERT_NE(out, nullptr);
    EXPECT_EQ(out->width, 1u);
}

TEST(CropConvexPrismCloud, PointsInsideKept) {
    auto prism = makeSquarePrism();
    auto cloud = makeXYZCloud({{0.0f, 0.0f, 0.5f, 0.0f}});
    auto out   = cropConvexPrism(cloud, prism, 0.0f);
    ASSERT_NE(out, nullptr);
    EXPECT_EQ(out->width, 1u);
}

TEST(CropConvexPrismCloud, PointsOutsideDropped) {
    auto prism = makeSquarePrism();
    auto cloud = makeXYZCloud({{2.0f, 0.0f, 0.5f, 0.0f}});
    auto out   = cropConvexPrism(cloud, prism, 0.0f);
    ASSERT_NE(out, nullptr);
    EXPECT_EQ(out->width, 0u);
}

TEST(CropConvexPrismCloud, AllFieldsPreserved) {
    auto prism = makeSquarePrism();
    auto cloud = makeXYZCloud({{0.0f, 0.0f, 0.5f, 42.0f}});
    auto out   = cropConvexPrism(cloud, prism, 0.0f);
    ASSERT_NE(out, nullptr);
    ASSERT_EQ(out->width, 1u);

    sensor_msgs::PointCloud2ConstIterator<float> ii(*out, "intensity");
    EXPECT_NEAR(*ii, 42.0f, 1e-5f);
}

TEST(CropConvexPrismCloud, ToleranceApplied) {
    auto prism = makeSquarePrism();
    // Point is 0.2m outside the right edge.
    auto cloud = makeXYZCloud({{0.7f, 0.0f, 0.5f, 0.0f}});

    auto kept    = cropConvexPrism(cloud, prism, 0.3f);
    auto dropped = cropConvexPrism(cloud, prism, 0.1f);

    ASSERT_NE(kept,    nullptr);
    ASSERT_NE(dropped, nullptr);
    EXPECT_EQ(kept->width,    1u);
    EXPECT_EQ(dropped->width, 0u);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
