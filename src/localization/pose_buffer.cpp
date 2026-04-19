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

#include <hatchbed_common/localization/pose_buffer.h>

#include <algorithm>

namespace hatchbed_common {
namespace localization {

PoseBuffer::PoseBuffer(double max_age) : max_age_(max_age) {}

void PoseBuffer::push(const rclcpp::Time& stamp, const Eigen::Isometry3d& pose) {
    buffer_.push_back({stamp, pose});
    while (buffer_.size() > 1 &&
           (stamp - buffer_.front().first).seconds() > max_age_) {
        buffer_.pop_front();
    }
}

std::optional<Eigen::Isometry3d> PoseBuffer::lookup(const rclcpp::Time& stamp) const {
    if (buffer_.size() < 2) {
        return std::nullopt;
    }

    // Find the first entry whose stamp >= query stamp.
    auto it = std::lower_bound(
        buffer_.begin(), buffer_.end(), stamp,
        [](const Entry& e, const rclcpp::Time& t) { return e.first < t; });

    if (it == buffer_.end()) {
        return std::nullopt;  // stamp is after the newest entry
    }

    if (it == buffer_.begin()) {
        // Exact match at the oldest entry is valid; anything earlier is not.
        if (it->first == stamp) {
            return it->second;
        }
        return std::nullopt;
    }

    const auto& next = *it;
    const auto& prev = *std::prev(it);

    const double dt = (next.first - prev.first).seconds();
    if (dt <= 0.0) {
        return prev.second;
    }

    const double alpha = (stamp - prev.first).seconds() / dt;

    Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
    result.translation() =
        (1.0 - alpha) * prev.second.translation() +
        alpha * next.second.translation();
    result.linear() =
        Eigen::Quaterniond(prev.second.linear())
            .slerp(alpha, Eigen::Quaterniond(next.second.linear()))
            .toRotationMatrix();
    return result;
}

void PoseBuffer::expireBefore(const rclcpp::Time& stamp) {
    while (!buffer_.empty() && buffer_.front().first < stamp) {
        buffer_.pop_front();
    }
}

void PoseBuffer::applyTransform(const Eigen::Isometry3d& T) {
    for (auto& [stamp, pose] : buffer_) {
        pose = T * pose;
    }
}

void   PoseBuffer::clear() { buffer_.clear(); }
bool   PoseBuffer::empty() const { return buffer_.empty(); }
size_t PoseBuffer::size()  const { return buffer_.size(); }

}  // namespace localization
}  // namespace hatchbed_common
