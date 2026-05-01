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
#include <hatchbed_common/localization/covariance_util.h>
#include <Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.hpp>
#include <array>
#include <cmath>

using hatchbed_common::localization::Covariance;
using hatchbed_common::localization::rotateCovariance;

// Helper to compare two covariance arrays element-wise
void expectCovarianceNear(const Covariance& a, const Covariance& b, double tol = 1e-9)
{
    for (int i = 0; i < 36; ++i) {
        EXPECT_NEAR(a[i], b[i], tol) << "  at index " << i
            << " (row " << i/6 << ", col " << i%6 << ")";
    }
}

// Helper to build a covariance from an Eigen matrix
Covariance fromEigen(const Eigen::Matrix<double, 6, 6>& M)
{
    Covariance c;
    Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(c.data()) = M;
    return c;
}

// Identity quaternion
tf2::Quaternion identityQuat()
{
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    return q;
}

// -----------------------------------------------------------------------------
// Identity rotation leaves covariance unchanged
// -----------------------------------------------------------------------------
TEST(CovarianceUtil, IdentityRotationPreservesCovariance)
{
    Covariance cov_in;
    for (int i = 0; i < 36; ++i) cov_in[i] = static_cast<double>(i);

    auto cov_out = rotateCovariance(cov_in, identityQuat());
    expectCovarianceNear(cov_in, cov_out);
}

// -----------------------------------------------------------------------------
// Zero covariance stays zero under any rotation
// -----------------------------------------------------------------------------
TEST(CovarianceUtil, ZeroCovarianceStaysZero)
{
    Covariance cov_in = {};
    tf2::Quaternion q;
    q.setRPY(0.3, 0.5, 1.2);

    auto cov_out = rotateCovariance(cov_in, q);
    expectCovarianceNear(cov_in, cov_out);
}

// -----------------------------------------------------------------------------
// Diagonal isotropic covariance (s^2*I) is invariant under any rotation
// -----------------------------------------------------------------------------
TEST(CovarianceUtil, IsotropicCovarianceInvariant)
{
    // R * (s^2*I) * R^T = s^2*I for any R
    Covariance cov_in = {};
    cov_in[0]  = 2.0;   // x
    cov_in[7]  = 2.0;   // y
    cov_in[14] = 2.0;   // z
    cov_in[21] = 0.5;   // roll
    cov_in[28] = 0.5;   // pitch
    cov_in[35] = 0.5;   // yaw

    tf2::Quaternion q;
    q.setRPY(0.3, 0.5, 1.2);

    auto cov_out = rotateCovariance(cov_in, q);
    expectCovarianceNear(cov_in, cov_out);
}

// -----------------------------------------------------------------------------
// 180 degree yaw rotation: x<->-x, y<->-y, z unchanged
// A covariance aligned with X should rotate to align with -X (same numerically
// since covariance is quadratic -- signs cancel)
// -----------------------------------------------------------------------------
TEST(CovarianceUtil, Yaw180PreservesAxisAlignedCovariance)
{
    Covariance cov_in = {};
    cov_in[0]  = 1.0;  // x variance
    cov_in[7]  = 2.0;  // y variance
    cov_in[14] = 3.0;  // z variance
    cov_in[21] = 4.0;  // roll variance
    cov_in[28] = 5.0;  // pitch variance
    cov_in[35] = 6.0;  // yaw variance

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, M_PI);  // 180 deg yaw

    // Under 180 degree yaw: x->-x, y->-y, z->z
    // Diagonal variances are unchanged (squared), off-diagonals may flip sign
    // but for a diagonal input matrix the result should be the same diagonal
    auto cov_out = rotateCovariance(cov_in, q);
    expectCovarianceNear(cov_in, cov_out);
}

// -----------------------------------------------------------------------------
// 90 degree yaw rotation: X variance should move to Y, Y variance to X
// -----------------------------------------------------------------------------
TEST(CovarianceUtil, Yaw90SwapsXYVariance)
{
    Covariance cov_in = {};
    cov_in[0] = 1.0;   // x variance only
    cov_in[7] = 4.0;   // y variance only

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, M_PI / 2.0);  // 90 deg yaw

    // After 90 degree yaw: old X axis becomes new Y axis, old Y becomes new -X
    // So x variance and y variance swap
    Covariance expected = {};
    expected[0] = 4.0;  // new x variance (was y)
    expected[7] = 1.0;  // new y variance (was x)

    auto cov_out = rotateCovariance(cov_in, q);
    expectCovarianceNear(expected, cov_out, 1e-9);
}

// -----------------------------------------------------------------------------
// Symmetry is preserved: if C is symmetric, R*C*R^T is symmetric
// -----------------------------------------------------------------------------
TEST(CovarianceUtil, SymmetryIsPreserved)
{
    // Build a random symmetric positive-definite 6x6 covariance via A*A^T
    Eigen::Matrix<double, 6, 6> A;
    A << 1, 2, 3, 4, 5, 6,
         0, 2, 1, 3, 2, 1,
         0, 0, 3, 2, 1, 4,
         0, 0, 0, 4, 3, 2,
         0, 0, 0, 0, 5, 1,
         0, 0, 0, 0, 0, 6;
    Eigen::Matrix<double, 6, 6> sym = A * A.transpose();

    Covariance cov_in = fromEigen(sym);

    tf2::Quaternion q;
    q.setRPY(0.3, 0.5, 1.2);

    auto cov_out = rotateCovariance(cov_in, q);

    Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> C_out(cov_out.data());
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j)
            EXPECT_NEAR(C_out(i, j), C_out(j, i), 1e-9)
                << "  asymmetry at (" << i << "," << j << ")";
}

// -----------------------------------------------------------------------------
// Double rotation: rotating by q then by q^-1 should recover the original
// -----------------------------------------------------------------------------
TEST(CovarianceUtil, DoubleRotationIsIdentity)
{
    Eigen::Matrix<double, 6, 6> A;
    A << 1, 2, 3, 4, 5, 6,
         0, 2, 1, 3, 2, 1,
         0, 0, 3, 2, 1, 4,
         0, 0, 0, 4, 3, 2,
         0, 0, 0, 0, 5, 1,
         0, 0, 0, 0, 0, 6;
    Covariance cov_in = fromEigen(A * A.transpose());

    tf2::Quaternion q;
    q.setRPY(0.3, 0.5, 1.2);

    auto cov_rotated = rotateCovariance(cov_in, q);
    auto cov_recovered = rotateCovariance(cov_rotated, q.inverse());

    expectCovarianceNear(cov_in, cov_recovered, 1e-9);
}

// -----------------------------------------------------------------------------
// Linear and angular blocks are independent -- a rotation-only covariance
// in the angular block should not bleed into the linear block
// -----------------------------------------------------------------------------
TEST(CovarianceUtil, LinearAndAngularBlocksAreIndependent)
{
    Covariance cov_in = {};
    // Only set angular variance (bottom-right block)
    cov_in[21] = 1.0;  // roll
    cov_in[28] = 2.0;  // pitch
    cov_in[35] = 3.0;  // yaw

    tf2::Quaternion q;
    q.setRPY(0.3, 0.5, 1.2);

    auto cov_out = rotateCovariance(cov_in, q);

    // Top-left (linear) block should remain zero
    Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> C_out(cov_out.data());
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            EXPECT_NEAR(C_out(i, j), 0.0, 1e-9)
                << "  linear block contaminated at (" << i << "," << j << ")";
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
