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
#include <hatchbed_common/pointcloud/point_cloud2_util.hpp>

#include <cmath>
#include <cstring>
#include <limits>

#include <sensor_msgs/point_cloud2_iterator.hpp>

using hatchbed_common::pointcloud::hasField;
using hatchbed_common::pointcloud::getFieldIterator;
using hatchbed_common::pointcloud::transformAndDeskewPointCloud;
using hatchbed_common::pointcloud::transformPointCloud;

// Build a PointCloud2 with float32 x, y, z and uint32 t fields.
// Points are given as (x, y, z, t_ns).
static sensor_msgs::msg::PointCloud2 makeCloud(
    const std::vector<std::tuple<float, float, float, uint32_t>>& pts)
{
    sensor_msgs::msg::PointCloud2 msg;
    msg.height = 1;
    msg.width  = static_cast<uint32_t>(pts.size());
    msg.is_bigendian = false;
    msg.is_dense     = true;

    auto addField = [&](const std::string& name, uint32_t offset, uint8_t dtype) {
        sensor_msgs::msg::PointField f;
        f.name     = name;
        f.offset   = offset;
        f.datatype = dtype;
        f.count    = 1;
        msg.fields.push_back(f);
    };

    addField("x", 0,  sensor_msgs::msg::PointField::FLOAT32);
    addField("y", 4,  sensor_msgs::msg::PointField::FLOAT32);
    addField("z", 8,  sensor_msgs::msg::PointField::FLOAT32);
    addField("t", 12, sensor_msgs::msg::PointField::UINT32);

    msg.point_step = 16;
    msg.row_step   = msg.point_step * msg.width;
    msg.data.resize(msg.row_step, 0);

    sensor_msgs::PointCloud2Iterator<float>    iter_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float>    iter_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float>    iter_z(msg, "z");
    sensor_msgs::PointCloud2Iterator<uint32_t> iter_t(msg, "t");

    for (const auto& [x, y, z, t] : pts) {
        *iter_x = x; *iter_y = y; *iter_z = z; *iter_t = t;
        ++iter_x; ++iter_y; ++iter_z; ++iter_t;
    }

    return msg;
}

// Read point i from cloud into (x, y, z).
static std::tuple<float, float, float> getPoint(
    const sensor_msgs::msg::PointCloud2& cloud, size_t i)
{
    float x, y, z;
    std::memcpy(&x, cloud.data.data() + i * cloud.point_step + 0,  sizeof(float));
    std::memcpy(&y, cloud.data.data() + i * cloud.point_step + 4,  sizeof(float));
    std::memcpy(&z, cloud.data.data() + i * cloud.point_step + 8,  sizeof(float));
    return {x, y, z};
}

// =============================================================================
// hasField
// =============================================================================

TEST(HasField, ReturnsTrueForCorrectTypeAndName) {
    auto cloud = makeCloud({{1.f, 2.f, 3.f, 0u}});
    EXPECT_TRUE(hasField<float>(cloud, "x"));
    EXPECT_TRUE(hasField<float>(cloud, "y"));
    EXPECT_TRUE(hasField<float>(cloud, "z"));
    EXPECT_TRUE(hasField<uint32_t>(cloud, "t"));
}

TEST(HasField, ReturnsFalseForMissingField) {
    auto cloud = makeCloud({{1.f, 2.f, 3.f, 0u}});
    EXPECT_FALSE(hasField<float>(cloud, "intensity"));
}

TEST(HasField, ReturnsFalseForWrongType) {
    auto cloud = makeCloud({{1.f, 2.f, 3.f, 0u}});
    // "x" exists but is FLOAT32, not FLOAT64
    EXPECT_FALSE(hasField<double>(cloud, "x"));
    // "t" exists but is UINT32, not FLOAT32
    EXPECT_FALSE(hasField<float>(cloud, "t"));
}

// =============================================================================
// getFieldIterator (const / mutable)
// =============================================================================

TEST(GetFieldIterator, ConstIteratorValidForPresentField) {
    auto cloud = makeCloud({{1.f, 2.f, 3.f, 42u}});
    auto iter = getFieldIterator<float>(cloud, "x");
    ASSERT_TRUE(static_cast<bool>(iter));
    EXPECT_FLOAT_EQ(*iter, 1.f);
}

TEST(GetFieldIterator, ConstIteratorInvalidForMissingField) {
    auto cloud = makeCloud({{1.f, 2.f, 3.f, 0u}});
    auto iter = getFieldIterator<float>(cloud, "intensity");
    EXPECT_FALSE(static_cast<bool>(iter));
}

TEST(GetFieldIterator, ConstIteratorInvalidForWrongType) {
    auto cloud = makeCloud({{1.f, 2.f, 3.f, 0u}});
    auto iter = getFieldIterator<double>(cloud, "x");
    EXPECT_FALSE(static_cast<bool>(iter));
}

TEST(GetFieldIterator, MutableIteratorCanWrite) {
    auto cloud = makeCloud({{1.f, 2.f, 3.f, 0u}});
    auto iter = getFieldIterator<float>(cloud, "x");
    ASSERT_TRUE(static_cast<bool>(iter));
    *iter = 99.f;
    auto check = getFieldIterator<float>(cloud, "x");
    EXPECT_FLOAT_EQ(*check, 99.f);
}

TEST(GetFieldIterator, IteratorAdvancesCorrectly) {
    auto cloud = makeCloud({{1.f, 0.f, 0.f, 0u}, {2.f, 0.f, 0.f, 0u}, {3.f, 0.f, 0.f, 0u}});
    auto iter = getFieldIterator<float>(cloud, "x");
    ASSERT_TRUE(static_cast<bool>(iter));
    EXPECT_FLOAT_EQ(*iter, 1.f); ++iter;
    EXPECT_FLOAT_EQ(*iter, 2.f); ++iter;
    EXPECT_FLOAT_EQ(*iter, 3.f);
}

// =============================================================================
// transformAndDeskewPointCloud
// =============================================================================

TEST(TransformAndDeskew, ReturnsFalseIfTFieldMissing) {
    // Cloud without "t" field
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.width = 1; cloud.height = 1;
    sensor_msgs::msg::PointField f;
    f.name = "x"; f.offset = 0; f.datatype = sensor_msgs::msg::PointField::FLOAT32; f.count = 1;
    cloud.fields.push_back(f);
    f.name = "y"; f.offset = 4; cloud.fields.push_back(f);
    f.name = "z"; f.offset = 8; cloud.fields.push_back(f);
    cloud.point_step = 12; cloud.row_step = 12;
    cloud.data.resize(12, 0);

    sensor_msgs::msg::PointCloud2 out;
    bool ok = transformAndDeskewPointCloud(
        cloud, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        Eigen::Isometry3d::Identity(), out);
    EXPECT_FALSE(ok);
}

TEST(TransformAndDeskew, ReturnsTrueWithValidCloud) {
    auto cloud = makeCloud({{1.f, 0.f, 0.f, 0u}});
    sensor_msgs::msg::PointCloud2 out;
    bool ok = transformAndDeskewPointCloud(
        cloud, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        Eigen::Isometry3d::Identity(), out);
    EXPECT_TRUE(ok);
}

TEST(TransformAndDeskew, IdentityTransformZeroTwistPreservesPoints) {
    auto cloud = makeCloud({{1.f, 2.f, 3.f, 0u}, {-1.f, 0.f, 4.f, 0u}});
    sensor_msgs::msg::PointCloud2 out;
    transformAndDeskewPointCloud(
        cloud, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        Eigen::Isometry3d::Identity(), out);

    auto [x0, y0, z0] = getPoint(out, 0);
    EXPECT_NEAR(x0, 1.f, 1e-5f);
    EXPECT_NEAR(y0, 2.f, 1e-5f);
    EXPECT_NEAR(z0, 3.f, 1e-5f);

    auto [x1, y1, z1] = getPoint(out, 1);
    EXPECT_NEAR(x1, -1.f, 1e-5f);
    EXPECT_NEAR(y1,  0.f, 1e-5f);
    EXPECT_NEAR(z1,  4.f, 1e-5f);
}

TEST(TransformAndDeskew, StaticTransformAppliedWithZeroTwist) {
    // T_out_sensor: translate by (5, 0, 0)
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() = Eigen::Vector3d(5.0, 0.0, 0.0);

    auto cloud = makeCloud({{1.f, 2.f, 3.f, 0u}});
    sensor_msgs::msg::PointCloud2 out;
    transformAndDeskewPointCloud(
        cloud, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), T, out);

    auto [x, y, z] = getPoint(out, 0);
    EXPECT_NEAR(x, 6.f, 1e-5f);
    EXPECT_NEAR(y, 2.f, 1e-5f);
    EXPECT_NEAR(z, 3.f, 1e-5f);
}

TEST(TransformAndDeskew, LinearTwistDeskewsPointsInTime) {
    // Moving at 1 m/s in x (base frame). Identity sensor transform.
    // t=0 ns -> dt=0s -> no correction
    // t=500000000 ns -> dt=0.5s -> +0.5 in x
    // t=1000000000 ns -> dt=1.0s -> +1.0 in x
    const uint32_t t0 = 0u;
    const uint32_t t1 = 500000000u;
    const uint32_t t2 = 1000000000u;

    auto cloud = makeCloud({
        {0.f, 0.f, 0.f, t0},
        {0.f, 0.f, 0.f, t1},
        {0.f, 0.f, 0.f, t2},
    });

    sensor_msgs::msg::PointCloud2 out;
    transformAndDeskewPointCloud(
        cloud,
        Eigen::Vector3d(1.0, 0.0, 0.0),
        Eigen::Vector3d::Zero(),
        Eigen::Isometry3d::Identity(),
        out);

    auto [x0, y0, z0] = getPoint(out, 0);
    EXPECT_NEAR(x0, 0.0f, 1e-5f);

    auto [x1, y1, z1] = getPoint(out, 1);
    EXPECT_NEAR(x1, 0.5f, 1e-5f);

    auto [x2, y2, z2] = getPoint(out, 2);
    EXPECT_NEAR(x2, 1.0f, 1e-5f);
}

TEST(TransformAndDeskew, AngularTwistDeskewsRotation) {
    // Rotating at pi/2 rad/s about Z. Identity sensor transform.
    // t=0 ns -> no rotation
    // t=1000000000 ns -> 90 deg rotation about Z: (1,0,0) -> (0,1,0)
    const uint32_t t0 = 0u;
    const uint32_t t1 = 1000000000u;

    auto cloud = makeCloud({
        {1.f, 0.f, 0.f, t0},
        {1.f, 0.f, 0.f, t1},
    });

    sensor_msgs::msg::PointCloud2 out;
    transformAndDeskewPointCloud(
        cloud,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d(0.0, 0.0, M_PI / 2.0),
        Eigen::Isometry3d::Identity(),
        out);

    auto [x0, y0, z0] = getPoint(out, 0);
    EXPECT_NEAR(x0, 1.0f, 1e-5f);
    EXPECT_NEAR(y0, 0.0f, 1e-5f);

    auto [x1, y1, z1] = getPoint(out, 1);
    EXPECT_NEAR(x1, 0.0f, 1e-5f);
    EXPECT_NEAR(y1, 1.0f, 1e-5f);
}

TEST(TransformAndDeskew, NanPointsPreservedAsNan) {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    auto cloud = makeCloud({{nan, nan, nan, 0u}, {1.f, 2.f, 3.f, 0u}});

    sensor_msgs::msg::PointCloud2 out;
    transformAndDeskewPointCloud(
        cloud, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        Eigen::Isometry3d::Identity(), out);

    auto [x0, y0, z0] = getPoint(out, 0);
    EXPECT_TRUE(std::isnan(x0));

    auto [x1, y1, z1] = getPoint(out, 1);
    EXPECT_NEAR(x1, 1.f, 1e-5f);
}

TEST(TransformAndDeskew, OutputHasSameFieldsAndPointStep) {
    auto cloud = makeCloud({{1.f, 2.f, 3.f, 100u}});
    sensor_msgs::msg::PointCloud2 out;
    transformAndDeskewPointCloud(
        cloud, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        Eigen::Isometry3d::Identity(), out);

    EXPECT_EQ(out.point_step, cloud.point_step);
    EXPECT_EQ(out.fields.size(), cloud.fields.size());
    for (size_t i = 0; i < cloud.fields.size(); ++i) {
        EXPECT_EQ(out.fields[i].name,     cloud.fields[i].name);
        EXPECT_EQ(out.fields[i].offset,   cloud.fields[i].offset);
        EXPECT_EQ(out.fields[i].datatype, cloud.fields[i].datatype);
    }
}

// =============================================================================
// transformPointCloud
// =============================================================================

TEST(TransformPointCloud, ReturnsFalseIfXyzMissing) {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.width = 1; cloud.height = 1;
    cloud.point_step = 4; cloud.row_step = 4;
    cloud.data.resize(4, 0);

    sensor_msgs::msg::PointCloud2 out;
    EXPECT_FALSE(transformPointCloud(cloud, Eigen::Isometry3d::Identity(), out));
}

TEST(TransformPointCloud, IdentityPreservesPoints) {
    auto cloud = makeCloud({{1.f, 2.f, 3.f, 0u}, {-1.f, 0.f, 4.f, 0u}});
    sensor_msgs::msg::PointCloud2 out;
    EXPECT_TRUE(transformPointCloud(cloud, Eigen::Isometry3d::Identity(), out));

    auto [x0, y0, z0] = getPoint(out, 0);
    EXPECT_NEAR(x0, 1.f, 1e-5f);
    EXPECT_NEAR(y0, 2.f, 1e-5f);
    EXPECT_NEAR(z0, 3.f, 1e-5f);

    auto [x1, y1, z1] = getPoint(out, 1);
    EXPECT_NEAR(x1, -1.f, 1e-5f);
    EXPECT_NEAR(y1,  0.f, 1e-5f);
    EXPECT_NEAR(z1,  4.f, 1e-5f);
}

TEST(TransformPointCloud, TranslationApplied) {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() = Eigen::Vector3d(5.0, 0.0, 0.0);

    auto cloud = makeCloud({{1.f, 2.f, 3.f, 0u}});
    sensor_msgs::msg::PointCloud2 out;
    transformPointCloud(cloud, T, out);

    auto [x, y, z] = getPoint(out, 0);
    EXPECT_NEAR(x, 6.f, 1e-5f);
    EXPECT_NEAR(y, 2.f, 1e-5f);
    EXPECT_NEAR(z, 3.f, 1e-5f);
}

TEST(TransformPointCloud, NanPointsPreservedAsNan) {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    auto cloud = makeCloud({{nan, nan, nan, 0u}, {1.f, 2.f, 3.f, 0u}});

    sensor_msgs::msg::PointCloud2 out;
    transformPointCloud(cloud, Eigen::Isometry3d::Identity(), out);

    auto [x0, y0, z0] = getPoint(out, 0);
    EXPECT_TRUE(std::isnan(x0));

    auto [x1, y1, z1] = getPoint(out, 1);
    EXPECT_NEAR(x1, 1.f, 1e-5f);
}

TEST(TransformPointCloud, OutputHasSameFieldsAndPointStep) {
    auto cloud = makeCloud({{1.f, 2.f, 3.f, 100u}});
    sensor_msgs::msg::PointCloud2 out;
    transformPointCloud(cloud, Eigen::Isometry3d::Identity(), out);

    EXPECT_EQ(out.point_step, cloud.point_step);
    EXPECT_EQ(out.fields.size(), cloud.fields.size());
    for (size_t i = 0; i < cloud.fields.size(); ++i) {
        EXPECT_EQ(out.fields[i].name,     cloud.fields[i].name);
        EXPECT_EQ(out.fields[i].offset,   cloud.fields[i].offset);
        EXPECT_EQ(out.fields[i].datatype, cloud.fields[i].datatype);
    }
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
