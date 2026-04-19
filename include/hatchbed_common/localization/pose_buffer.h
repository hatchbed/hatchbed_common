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

#include <deque>
#include <optional>
#include <utility>

#include <Eigen/Geometry>
#include <rclcpp/time.hpp>

namespace hatchbed_common {
namespace localization {

// Bounded, time-ordered history of poses with SLERP/LERP interpolation.
//
// Poses are appended with push() and queried at arbitrary times with lookup().
// Entries older than max_age are dropped automatically on each push().
// The entire buffer can be left-multiplied by a transform with applyTransform(),
// which re-expresses all stored poses in a different frame without discarding data.
class PoseBuffer {
public:
    using Entry = std::pair<rclcpp::Time, Eigen::Isometry3d>;

    // max_age: how many seconds of history to retain.
    explicit PoseBuffer(double max_age = 10.0);

    // Append a pose and drop entries older than max_age relative to stamp.
    // Stamps must be monotonically non-decreasing.
    void push(const rclcpp::Time& stamp, const Eigen::Isometry3d& pose);

    // Return the interpolated pose at stamp (LERP translation, SLERP rotation).
    // Returns nullopt if stamp falls outside the buffered time range.
    std::optional<Eigen::Isometry3d> lookup(const rclcpp::Time& stamp) const;

    // Left-multiply every buffered pose by T: pose = T * pose.
    // Use this to re-express the entire history in a different frame.
    void applyTransform(const Eigen::Isometry3d& T);

    // Remove all entries with a stamp strictly older than the given stamp.
    void expireBefore(const rclcpp::Time& stamp);

    void   clear();
    bool   empty() const;
    size_t size()  const;

private:
    double            max_age_;
    std::deque<Entry> buffer_;
};

}  // namespace localization
}  // namespace hatchbed_common
