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

#include <hatchbed_common/transforms/covariance_util.h>

#include <Eigen/Geometry>

namespace hatchbed_common {
namespace transforms {

using Covariance = std::array<double, 36>;

Covariance rotateCovariance(
    const Covariance& cov_in,
    const tf2::Quaternion& q)
{
    Eigen::Matrix3d R = Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z()).toRotationMatrix();
    Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> C(cov_in.data());

    Eigen::Matrix<double, 6, 6, Eigen::RowMajor> result;
    result.block<3, 3>(0, 0) = R * C.block<3, 3>(0, 0) * R.transpose();  // pos-pos
    result.block<3, 3>(0, 3) = R * C.block<3, 3>(0, 3) * R.transpose();  // pos-rot
    result.block<3, 3>(3, 0) = R * C.block<3, 3>(3, 0) * R.transpose();  // rot-pos
    result.block<3, 3>(3, 3) = R * C.block<3, 3>(3, 3) * R.transpose();  // rot-rot

    Covariance cov_out;
    Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(cov_out.data()) = result;
    return cov_out;
}

}  // namespace transforms
}  // namespace hatchbed_common
