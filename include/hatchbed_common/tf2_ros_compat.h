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

#include <memory>
#include <type_traits>

#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

namespace hatchbed_common {
namespace detail {

// Detect the tf2_ros API introduced in ros2/geometry2#940.  Newer tf2_ros
// classes expose a nested RequiredInterfaces alias and their node constructors
// take the node by reference (bundled as a NodeInterfaces) rather than by
// pointer.  This trait is true only when that alias is present.
template <typename T, typename = void>
struct tf_uses_node_interfaces : std::false_type {};

template <typename T>
struct tf_uses_node_interfaces<T, std::void_t<typename T::RequiredInterfaces>>
    : std::true_type {};

}  // namespace detail

// Construct a tf2_ros::TransformListener bound to the given node, compatible
// with both the old (node pointer) and new (node reference) tf2_ros APIs.
//
// The branch that does not match the installed tf2_ros is discarded by
// 'if constexpr' before instantiation, so only the valid constructor call is
// compiled.  This avoids any build-system feature detection.
template <typename NodeT>
std::shared_ptr<tf2_ros::TransformListener> makeTransformListener(
    tf2_ros::Buffer& buffer, NodeT* node, bool spin_thread = true) {
  if constexpr (detail::tf_uses_node_interfaces<tf2_ros::TransformListener>::value) {
    return std::make_shared<tf2_ros::TransformListener>(buffer, *node, spin_thread);
  } else {
    return std::make_shared<tf2_ros::TransformListener>(buffer, node, spin_thread);
  }
}

}  // namespace hatchbed_common
