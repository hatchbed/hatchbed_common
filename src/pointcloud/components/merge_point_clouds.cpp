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

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <hatchbed_common/param_handler.h>
#include <hatchbed_common/pointcloud/point_cloud2_util.hpp>
#include <hatchbed_common/ros_names.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace hatchbed_common {
namespace pointcloud {

/**
 * Merges one or more PointCloud2 inputs into a single xyz-only output cloud,
 * published at a fixed rate.  Each input is cached on arrival; the timer
 * concatenates the latest snapshot from every input and publishes.  When
 * output_frame is non-empty every input is transformed into that frame before
 * concatenation.
 *
 * Per-input configuration parameters (for each name N in 'inputs'):
 *   N.topic           (string)  -- absolute topic to subscribe to (required)
 *   N.transient_local (bool)    -- use transient_local QoS; default false
 */
class MergePointClouds : public rclcpp::Node {
public:
  explicit MergePointClouds(const rclcpp::NodeOptions & options)
  : Node("merge_point_clouds", options)
  {
    init_timer_ = create_wall_timer(std::chrono::milliseconds(100),
                                    std::bind(&MergePointClouds::onInit, this));
  }

private:
  struct InputEntry {
    std::string name;
    sensor_msgs::msg::PointCloud2::ConstSharedPtr latest;
    rclcpp::SubscriptionBase::SharedPtr sub;
  };

  void onInit()
  {
    init_timer_->cancel();

    params_ = std::make_shared<hatchbed_common::ParamHandler>(shared_from_this());
    params_->register_verbose_logging_param();

    params_->param(&output_rate_, "output_rate", 1.0,
                   "Publish rate (Hz).").min(0.01).max(100.0).declare();
    params_->param(&output_frame_, "output_frame", std::string(""),
                   "Transform all inputs into this frame before merging; "
                   "empty = no transform (inputs assumed same frame).").declare();

    std::vector<std::string> input_names;
    params_->param(&input_names, "inputs", std::vector<std::string>{},
                   "Ordered list of input names; for each N declare N.topic "
                   "and optionally N.transient_local.").declare();

    if (!output_frame_.empty()) {
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    std::unordered_set<std::string> seen;
    for (const auto & name : input_names) {
      if (name.empty()) {
        RCLCPP_WARN(get_logger(), "inputs: empty name; skipping.");
        continue;
      }
      if (!hatchbed_common::isValidParamName(name)) {
        RCLCPP_WARN(get_logger(),
          "inputs: '%s' is not a valid parameter name; skipping.", name.c_str());
        continue;
      }
      if (!seen.insert(name).second) {
        RCLCPP_WARN(get_logger(), "inputs: duplicate name '%s'; skipping.", name.c_str());
        continue;
      }
      parseInput(name);
    }

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("points_out", 10);

    const auto period = std::chrono::duration<double>(1.0 / output_rate_);
    output_timer_ = create_timer(period, std::bind(&MergePointClouds::onTimer, this));

    RCLCPP_INFO(get_logger(), "[merge_point_clouds] Initialized: %zu inputs at %.1f Hz.",
                inputs_.size(), output_rate_);
  }

  void parseInput(const std::string & name)
  {
    const std::string pfx = name + ".";

    const std::string topic = params_->param(
      pfx + "topic", std::string(""),
      "Absolute topic to subscribe to.").declare().value();

    const bool transient_local = params_->param(
      pfx + "transient_local", false,
      "Subscribe with transient_local QoS.").declare().value();

    if (topic.empty()) {
      RCLCPP_WARN(get_logger(), "input '%s' missing topic; skipping.", name.c_str());
      return;
    }

    InputEntry entry;
    entry.name = name;

    const size_t idx = inputs_.size();
    auto qos = transient_local ? rclcpp::QoS(1).transient_local() : rclcpp::QoS(10);
    entry.sub = create_subscription<sensor_msgs::msg::PointCloud2>(
      topic, qos,
      [this, idx](const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg) {
        inputs_[idx].latest = msg;
      });

    RCLCPP_INFO(get_logger(), "Registered input '%s' on '%s'%s.",
                name.c_str(), topic.c_str(),
                transient_local ? " (transient_local)" : "");
    inputs_.push_back(std::move(entry));
  }

  static Eigen::Isometry3d toIsometry(const geometry_msgs::msg::TransformStamped & tf)
  {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    const auto & t = tf.transform.translation;
    const auto & r = tf.transform.rotation;
    T.translation() = Eigen::Vector3d(t.x, t.y, t.z);
    T.linear() = Eigen::Quaterniond(r.w, r.x, r.y, r.z).toRotationMatrix();
    return T;
  }

  void onTimer()
  {
    std::vector<Eigen::Vector3f> merged;
    std::string out_frame = output_frame_;

    for (const auto & input : inputs_) {
      if (!input.latest) {
        continue;
      }

      const auto & cloud = *input.latest;

      auto x_iter = getFieldIterator<float>(cloud, "x");
      auto y_iter = getFieldIterator<float>(cloud, "y");
      auto z_iter = getFieldIterator<float>(cloud, "z");
      if (!x_iter || !y_iter || !z_iter) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
          "[merge_point_clouds] input '%s' missing x/y/z float fields; skipping.",
          input.name.c_str());
        continue;
      }

      bool need_transform = !output_frame_.empty() &&
                            cloud.header.frame_id != output_frame_;
      Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
      if (need_transform) {
        try {
          auto tf = tf_buffer_->lookupTransform(
            output_frame_, cloud.header.frame_id, tf2::TimePointZero);
          T = toIsometry(tf);
        } catch (const tf2::TransformException & ex) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "[merge_point_clouds] Could not transform input '%s': %s",
            input.name.c_str(), ex.what());
          continue;
        }
      } else if (out_frame.empty() && !cloud.header.frame_id.empty()) {
        out_frame = cloud.header.frame_id;
      }

      const size_t n = cloud.width * cloud.height;
      merged.reserve(merged.size() + n);

      for (size_t i = 0; i < n; ++i, ++x_iter, ++y_iter, ++z_iter) {
        Eigen::Vector3f p(*x_iter, *y_iter, *z_iter);
        if (!p.allFinite()) {
          continue;
        }
        if (need_transform) {
          p = (T * p.cast<double>()).cast<float>();
        }
        merged.push_back(p);
      }
    }

    auto out = createPointCloud2(merged);
    out->header.stamp = now();
    out->header.frame_id = out_frame;
    pub_->publish(*out);
  }

  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr output_timer_;
  std::shared_ptr<hatchbed_common::ParamHandler> params_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

  std::vector<InputEntry> inputs_;
  std::string output_frame_;
  double output_rate_ = 1.0;
};

}  // namespace pointcloud
}  // namespace hatchbed_common

RCLCPP_COMPONENTS_REGISTER_NODE(hatchbed_common::pointcloud::MergePointClouds)
