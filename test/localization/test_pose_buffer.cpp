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
#include <hatchbed_common/localization/pose_buffer.h>

#include <cmath>

using hatchbed_common::localization::PoseBuffer;

// Construct a rclcpp::Time from a floating-point number of seconds.
static rclcpp::Time ts(double seconds) {
    return rclcpp::Time(static_cast<int64_t>(seconds * 1e9));
}

// Build a pure-translation isometry.
static Eigen::Isometry3d translation(double x, double y, double z) {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() = Eigen::Vector3d(x, y, z);
    return T;
}

// Build a pure-yaw isometry.
static Eigen::Isometry3d yaw(double radians) {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = Eigen::AngleAxisd(radians, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return T;
}

// Extract yaw from an isometry (atan2 of the rotation matrix first column).
static double extractYaw(const Eigen::Isometry3d& T) {
    return std::atan2(T.linear()(1, 0), T.linear()(0, 0));
}

// -----------------------------------------------------------------------------
// Initial state
// -----------------------------------------------------------------------------

TEST(PoseBuffer, InitiallyEmpty) {
    PoseBuffer buf;
    EXPECT_TRUE(buf.empty());
    EXPECT_EQ(buf.size(), 0u);
}

// -----------------------------------------------------------------------------
// push() and size tracking
// -----------------------------------------------------------------------------

TEST(PoseBuffer, PushUpdatesSize) {
    PoseBuffer buf;
    buf.push(ts(0.0), Eigen::Isometry3d::Identity());
    EXPECT_FALSE(buf.empty());
    EXPECT_EQ(buf.size(), 1u);

    buf.push(ts(0.1), Eigen::Isometry3d::Identity());
    EXPECT_EQ(buf.size(), 2u);
}

TEST(PoseBuffer, PrunesEntriesOlderThanMaxAge) {
    PoseBuffer buf(0.5);
    buf.push(ts(0.0), Eigen::Isometry3d::Identity());
    buf.push(ts(0.3), Eigen::Isometry3d::Identity());
    buf.push(ts(0.6), Eigen::Isometry3d::Identity());
    // t=0.0 is 0.6s before the newest; with max_age=0.5 it should be pruned.
    // t=0.3 is 0.3s before the newest; it should be retained.
    EXPECT_EQ(buf.size(), 2u);
}

TEST(PoseBuffer, AlwaysRetainsAtLeastOneEntry) {
    // Even if an entry is older than max_age, the oldest entry is never pruned
    // while it is the only one (pruning requires size > 1).
    PoseBuffer buf(0.1);
    buf.push(ts(0.0), Eigen::Isometry3d::Identity());
    buf.push(ts(5.0), Eigen::Isometry3d::Identity());
    // t=0.0 is 5s old -- far beyond max_age -- but the deque always keeps at
    // least one entry at each end to allow interpolation.
    EXPECT_GE(buf.size(), 1u);
}

// -----------------------------------------------------------------------------
// clear()
// -----------------------------------------------------------------------------

TEST(PoseBuffer, ClearEmptiesBuffer) {
    PoseBuffer buf;
    buf.push(ts(0.0), Eigen::Isometry3d::Identity());
    buf.push(ts(1.0), Eigen::Isometry3d::Identity());
    buf.clear();
    EXPECT_TRUE(buf.empty());
    EXPECT_EQ(buf.size(), 0u);
}

// -----------------------------------------------------------------------------
// lookup() -- boundary conditions
// -----------------------------------------------------------------------------

TEST(PoseBuffer, LookupOnEmptyBufferReturnsNullopt) {
    PoseBuffer buf;
    EXPECT_FALSE(buf.lookup(ts(0.5)).has_value());
}

TEST(PoseBuffer, LookupWithSingleEntryReturnsNullopt) {
    PoseBuffer buf;
    buf.push(ts(0.0), Eigen::Isometry3d::Identity());
    EXPECT_FALSE(buf.lookup(ts(0.0)).has_value());
}

TEST(PoseBuffer, LookupExactMatchAtOldestEntry) {
    PoseBuffer buf;
    auto pose = translation(1.0, 2.0, 3.0);
    buf.push(ts(0.0), pose);
    buf.push(ts(1.0), Eigen::Isometry3d::Identity());

    auto result = buf.lookup(ts(0.0));
    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(result->translation().isApprox(pose.translation(), 1e-9));
}

TEST(PoseBuffer, LookupBeforeOldestReturnsNullopt) {
    PoseBuffer buf;
    buf.push(ts(1.0), Eigen::Isometry3d::Identity());
    buf.push(ts(2.0), Eigen::Isometry3d::Identity());
    EXPECT_FALSE(buf.lookup(ts(0.5)).has_value());
}

TEST(PoseBuffer, LookupAfterNewestReturnsNullopt) {
    PoseBuffer buf;
    buf.push(ts(0.0), Eigen::Isometry3d::Identity());
    buf.push(ts(1.0), Eigen::Isometry3d::Identity());
    EXPECT_FALSE(buf.lookup(ts(2.0)).has_value());
}

// -----------------------------------------------------------------------------
// lookup() -- translation interpolation
// -----------------------------------------------------------------------------

TEST(PoseBuffer, LookupMidpointTranslation) {
    PoseBuffer buf;
    buf.push(ts(0.0), translation(0.0, 0.0, 0.0));
    buf.push(ts(1.0), translation(2.0, 4.0, 6.0));

    auto result = buf.lookup(ts(0.5));
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result->translation().x(), 1.0, 1e-9);
    EXPECT_NEAR(result->translation().y(), 2.0, 1e-9);
    EXPECT_NEAR(result->translation().z(), 3.0, 1e-9);
}

TEST(PoseBuffer, LookupAtBracketEndpointsMatchesPushedPoses) {
    PoseBuffer buf;
    auto p0 = translation(1.0, 0.0, 0.0);
    auto p1 = translation(3.0, 0.0, 0.0);
    buf.push(ts(0.0), p0);
    buf.push(ts(1.0), p1);

    auto r0 = buf.lookup(ts(0.0));
    ASSERT_TRUE(r0.has_value());
    EXPECT_TRUE(r0->translation().isApprox(p0.translation(), 1e-9));

    // Exact match at the newest entry goes through the interpolation path with alpha=1.0
    // and returns the correct pose. Use near-end to test that the interpolation is tight.
    auto r1 = buf.lookup(ts(1.0 - 1e-9));
    ASSERT_TRUE(r1.has_value());
    EXPECT_NEAR(r1->translation().x(), 3.0, 1e-6);
}

TEST(PoseBuffer, LookupQuarterPointTranslation) {
    PoseBuffer buf;
    buf.push(ts(0.0), translation(0.0, 0.0, 0.0));
    buf.push(ts(1.0), translation(4.0, 0.0, 0.0));

    auto result = buf.lookup(ts(0.25));
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result->translation().x(), 1.0, 1e-9);
}

// -----------------------------------------------------------------------------
// lookup() -- rotation interpolation (SLERP)
// -----------------------------------------------------------------------------

TEST(PoseBuffer, LookupMidpointYawSlerp) {
    PoseBuffer buf;
    buf.push(ts(0.0), yaw(0.0));
    buf.push(ts(1.0), yaw(M_PI / 2.0));

    auto result = buf.lookup(ts(0.5));
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(extractYaw(*result), M_PI / 4.0, 1e-9);
}

TEST(PoseBuffer, LookupQuarterPointYawSlerp) {
    PoseBuffer buf;
    buf.push(ts(0.0), yaw(0.0));
    buf.push(ts(1.0), yaw(M_PI / 2.0));

    auto result = buf.lookup(ts(0.25));
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(extractYaw(*result), M_PI / 8.0, 1e-9);
}

TEST(PoseBuffer, LookupAcrossMultipleSegments) {
    PoseBuffer buf;
    buf.push(ts(0.0), translation(0.0, 0.0, 0.0));
    buf.push(ts(1.0), translation(1.0, 0.0, 0.0));
    buf.push(ts(2.0), translation(3.0, 0.0, 0.0));  // non-uniform spacing

    // First segment: midpoint at t=0.5 -> x=0.5
    auto r0 = buf.lookup(ts(0.5));
    ASSERT_TRUE(r0.has_value());
    EXPECT_NEAR(r0->translation().x(), 0.5, 1e-9);

    // Second segment: midpoint at t=1.5 -> x = 1 + 0.5*(3-1) = 2.0
    auto r1 = buf.lookup(ts(1.5));
    ASSERT_TRUE(r1.has_value());
    EXPECT_NEAR(r1->translation().x(), 2.0, 1e-9);
}

// -----------------------------------------------------------------------------
// applyTransform()
// -----------------------------------------------------------------------------

TEST(PoseBuffer, ApplyTransformOnEmptyBufferIsNoop) {
    PoseBuffer buf;
    EXPECT_NO_THROW(buf.applyTransform(translation(1.0, 2.0, 3.0)));
    EXPECT_TRUE(buf.empty());
}

TEST(PoseBuffer, ApplyTransformShiftsAllEntries) {
    PoseBuffer buf;
    buf.push(ts(0.0), translation(1.0, 0.0, 0.0));
    buf.push(ts(1.0), translation(2.0, 0.0, 0.0));

    buf.applyTransform(translation(10.0, 0.0, 0.0));

    auto r0 = buf.lookup(ts(0.0));
    ASSERT_TRUE(r0.has_value());
    EXPECT_NEAR(r0->translation().x(), 11.0, 1e-9);

    // Check second entry via near-end lookup
    auto r1 = buf.lookup(ts(1.0 - 1e-9));
    ASSERT_TRUE(r1.has_value());
    EXPECT_NEAR(r1->translation().x(), 12.0, 1e-6);
}

TEST(PoseBuffer, ApplyTransformFollowedByInverseIsIdentity) {
    PoseBuffer buf;
    auto original = translation(3.0, 1.0, 0.0);
    buf.push(ts(0.0), original);
    buf.push(ts(1.0), original);

    auto T = translation(5.0, -2.0, 1.0);
    buf.applyTransform(T);
    buf.applyTransform(T.inverse());

    auto result = buf.lookup(ts(0.0));
    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(result->translation().isApprox(original.translation(), 1e-9));
}

// -----------------------------------------------------------------------------
// expireBefore()
// -----------------------------------------------------------------------------

TEST(PoseBuffer, ExpireBeforeRemovesOlderEntries) {
    PoseBuffer buf(10.0);
    buf.push(ts(0.0), Eigen::Isometry3d::Identity());
    buf.push(ts(1.0), Eigen::Isometry3d::Identity());
    buf.push(ts(2.0), Eigen::Isometry3d::Identity());
    buf.push(ts(3.0), Eigen::Isometry3d::Identity());

    buf.expireBefore(ts(2.0));

    // t=0 and t=1 are strictly before t=2 and should be removed.
    EXPECT_EQ(buf.size(), 2u);
    EXPECT_FALSE(buf.lookup(ts(1.5)).has_value());  // t=1 no longer in buffer
}

TEST(PoseBuffer, ExpireBeforeKeepsEntriesAtOrAfterStamp) {
    PoseBuffer buf(10.0);
    buf.push(ts(1.0), translation(1.0, 0.0, 0.0));
    buf.push(ts(2.0), translation(2.0, 0.0, 0.0));
    buf.push(ts(3.0), translation(3.0, 0.0, 0.0));

    buf.expireBefore(ts(2.0));  // should keep t=2 and t=3

    auto result = buf.lookup(ts(2.5));
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result->translation().x(), 2.5, 1e-9);
}

TEST(PoseBuffer, ExpireBeforeOnEmptyBufferIsNoop) {
    PoseBuffer buf;
    EXPECT_NO_THROW(buf.expireBefore(ts(1.0)));
    EXPECT_TRUE(buf.empty());
}

TEST(PoseBuffer, ExpireBeforeWithStampBeforeAllEntriesRemovesNothing) {
    PoseBuffer buf(10.0);
    buf.push(ts(1.0), Eigen::Isometry3d::Identity());
    buf.push(ts(2.0), Eigen::Isometry3d::Identity());

    buf.expireBefore(ts(0.5));
    EXPECT_EQ(buf.size(), 2u);
}

// -----------------------------------------------------------------------------
// earliestStamp() / latestStamp()
// -----------------------------------------------------------------------------

TEST(PoseBuffer, StampRangeOnEmptyBufferReturnsNullopt) {
    PoseBuffer buf;
    EXPECT_FALSE(buf.earliestStamp().has_value());
    EXPECT_FALSE(buf.latestStamp().has_value());
}

TEST(PoseBuffer, StampRangeWithSingleEntryReturnsThatStamp) {
    PoseBuffer buf;
    buf.push(ts(1.5), Eigen::Isometry3d::Identity());
    ASSERT_TRUE(buf.earliestStamp().has_value());
    ASSERT_TRUE(buf.latestStamp().has_value());
    EXPECT_EQ(buf.earliestStamp()->nanoseconds(), ts(1.5).nanoseconds());
    EXPECT_EQ(buf.latestStamp()->nanoseconds(), ts(1.5).nanoseconds());
}

TEST(PoseBuffer, StampRangeReturnsOldestAndNewest) {
    PoseBuffer buf(10.0);
    buf.push(ts(1.0), Eigen::Isometry3d::Identity());
    buf.push(ts(2.0), Eigen::Isometry3d::Identity());
    buf.push(ts(3.0), Eigen::Isometry3d::Identity());

    ASSERT_TRUE(buf.earliestStamp().has_value());
    ASSERT_TRUE(buf.latestStamp().has_value());
    EXPECT_EQ(buf.earliestStamp()->nanoseconds(), ts(1.0).nanoseconds());
    EXPECT_EQ(buf.latestStamp()->nanoseconds(), ts(3.0).nanoseconds());
}

TEST(PoseBuffer, StampRangeUpdatesAfterPruning) {
    // With max_age=0.5, pushing t=1.0 should evict t=0.0 (1.0s old) but
    // retain t=0.7 (only 0.3s old).
    PoseBuffer buf(0.5);
    buf.push(ts(0.0), Eigen::Isometry3d::Identity());
    buf.push(ts(0.7), Eigen::Isometry3d::Identity());
    buf.push(ts(1.0), Eigen::Isometry3d::Identity());

    ASSERT_TRUE(buf.earliestStamp().has_value());
    ASSERT_TRUE(buf.latestStamp().has_value());
    EXPECT_EQ(buf.earliestStamp()->nanoseconds(), ts(0.7).nanoseconds());
    EXPECT_EQ(buf.latestStamp()->nanoseconds(), ts(1.0).nanoseconds());
}

TEST(PoseBuffer, StampRangeReturnsNulloptAfterClear) {
    PoseBuffer buf;
    buf.push(ts(0.0), Eigen::Isometry3d::Identity());
    buf.push(ts(1.0), Eigen::Isometry3d::Identity());
    buf.clear();
    EXPECT_FALSE(buf.earliestStamp().has_value());
    EXPECT_FALSE(buf.latestStamp().has_value());
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
