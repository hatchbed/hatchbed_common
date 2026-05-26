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
#include <hatchbed_common/transforms/transform_util.h>

#include <cmath>

using hatchbed_common::transforms::toIsometry;

// Build a TransformStamped from raw components.
static geometry_msgs::msg::TransformStamped makeTransform(
    double tx, double ty, double tz,
    double qw, double qx, double qy, double qz)
{
    geometry_msgs::msg::TransformStamped tf;
    tf.transform.translation.x = tx;
    tf.transform.translation.y = ty;
    tf.transform.translation.z = tz;
    tf.transform.rotation.w = qw;
    tf.transform.rotation.x = qx;
    tf.transform.rotation.y = qy;
    tf.transform.rotation.z = qz;
    return tf;
}

// Build a TransformStamped from a yaw angle (rotation around Z).
static geometry_msgs::msg::TransformStamped makeYaw(double radians) {
    const double half = radians * 0.5;
    return makeTransform(0.0, 0.0, 0.0, std::cos(half), 0.0, 0.0, std::sin(half));
}

// -----------------------------------------------------------------------------
// Identity transform
// -----------------------------------------------------------------------------

TEST(TransformUtil, IdentityTransformGivesIdentityIsometry) {
    auto T = toIsometry(makeTransform(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0));
    EXPECT_TRUE(T.isApprox(Eigen::Isometry3d::Identity(), 1e-9));
}

// -----------------------------------------------------------------------------
// Pure translation
// -----------------------------------------------------------------------------

TEST(TransformUtil, PureTranslationPreservesTranslation) {
    auto T = toIsometry(makeTransform(1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0));
    EXPECT_NEAR(T.translation().x(), 1.0, 1e-9);
    EXPECT_NEAR(T.translation().y(), 2.0, 1e-9);
    EXPECT_NEAR(T.translation().z(), 3.0, 1e-9);
    EXPECT_TRUE(T.linear().isApprox(Eigen::Matrix3d::Identity(), 1e-9));
}

TEST(TransformUtil, PureTranslationNegativeValues) {
    auto T = toIsometry(makeTransform(-5.0, 0.0, 7.5, 1.0, 0.0, 0.0, 0.0));
    EXPECT_NEAR(T.translation().x(), -5.0, 1e-9);
    EXPECT_NEAR(T.translation().y(),  0.0, 1e-9);
    EXPECT_NEAR(T.translation().z(),  7.5, 1e-9);
}

// -----------------------------------------------------------------------------
// Pure rotation
// -----------------------------------------------------------------------------

TEST(TransformUtil, Yaw90RotatesXIntoY) {
    // 90 deg yaw: x-axis maps to y-axis
    auto T = toIsometry(makeYaw(M_PI / 2.0));
    Eigen::Vector3d x_axis(1.0, 0.0, 0.0);
    Eigen::Vector3d rotated = T.linear() * x_axis;
    EXPECT_NEAR(rotated.x(),  0.0, 1e-9);
    EXPECT_NEAR(rotated.y(),  1.0, 1e-9);
    EXPECT_NEAR(rotated.z(),  0.0, 1e-9);
}

TEST(TransformUtil, Yaw180NegatesXAndY) {
    auto T = toIsometry(makeYaw(M_PI));
    Eigen::Vector3d x_axis(1.0, 0.0, 0.0);
    Eigen::Vector3d y_axis(0.0, 1.0, 0.0);
    EXPECT_NEAR((T.linear() * x_axis).x(), -1.0, 1e-9);
    EXPECT_NEAR((T.linear() * y_axis).y(), -1.0, 1e-9);
}

TEST(TransformUtil, PureRotationHasZeroTranslation) {
    auto T = toIsometry(makeYaw(1.23));
    EXPECT_NEAR(T.translation().x(), 0.0, 1e-9);
    EXPECT_NEAR(T.translation().y(), 0.0, 1e-9);
    EXPECT_NEAR(T.translation().z(), 0.0, 1e-9);
}

// -----------------------------------------------------------------------------
// Combined translation and rotation
// -----------------------------------------------------------------------------

TEST(TransformUtil, CombinedTranslationAndRotationRoundTrip) {
    // Build a known transform: translate (1,2,3), rotate 90 deg around Z.
    const double half_pi = M_PI / 4.0;
    auto tf = makeTransform(1.0, 2.0, 3.0, std::cos(half_pi), 0.0, 0.0, std::sin(half_pi));
    auto T = toIsometry(tf);

    EXPECT_NEAR(T.translation().x(), 1.0, 1e-9);
    EXPECT_NEAR(T.translation().y(), 2.0, 1e-9);
    EXPECT_NEAR(T.translation().z(), 3.0, 1e-9);

    Eigen::Vector3d rotated = T.linear() * Eigen::Vector3d(1.0, 0.0, 0.0);
    EXPECT_NEAR(rotated.x(), 0.0, 1e-9);
    EXPECT_NEAR(rotated.y(), 1.0, 1e-9);
    EXPECT_NEAR(rotated.z(), 0.0, 1e-9);
}

// -----------------------------------------------------------------------------
// Quaternion convention: w,x,y,z order from message
// -----------------------------------------------------------------------------

TEST(TransformUtil, QuaternionComponentOrderIsCorrect) {
    // 180 deg rotation around X: q = (0, 1, 0, 0) in (w,x,y,z)
    auto tf = makeTransform(0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
    auto T = toIsometry(tf);

    // Under 180 deg around X: y->-y, z->-z, x unchanged
    Eigen::Vector3d y_axis(0.0, 1.0, 0.0);
    Eigen::Vector3d z_axis(0.0, 0.0, 1.0);
    EXPECT_NEAR((T.linear() * y_axis).y(), -1.0, 1e-9);
    EXPECT_NEAR((T.linear() * z_axis).z(), -1.0, 1e-9);
}

// -----------------------------------------------------------------------------
// Output is a valid rotation matrix
// -----------------------------------------------------------------------------

TEST(TransformUtil, LinearPartIsOrthogonal) {
    // R * R^T should equal Identity for a valid rotation matrix
    auto T = toIsometry(makeTransform(1.0, 2.0, 3.0, 0.5, 0.5, 0.5, 0.5));
    Eigen::Matrix3d RRt = T.linear() * T.linear().transpose();
    EXPECT_TRUE(RRt.isApprox(Eigen::Matrix3d::Identity(), 1e-9));
}

TEST(TransformUtil, LinearPartHasUnitDeterminant) {
    auto T = toIsometry(makeTransform(0.0, 0.0, 0.0, 0.5, 0.5, 0.5, 0.5));
    EXPECT_NEAR(T.linear().determinant(), 1.0, 1e-9);
}

// -----------------------------------------------------------------------------
// Composition: toIsometry(A) * toIsometry(B) is consistent
// -----------------------------------------------------------------------------

TEST(TransformUtil, CompositionMatchesExpected) {
    // Two 90-deg yaw rotations should compose to a 180-deg yaw.
    auto T90  = toIsometry(makeYaw(M_PI / 2.0));
    auto T180 = toIsometry(makeYaw(M_PI));
    auto composed = T90 * T90;

    EXPECT_TRUE(composed.linear().isApprox(T180.linear(), 1e-9));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
