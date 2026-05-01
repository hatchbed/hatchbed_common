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
#include <cstring>
#include <functional>
#include <memory>
#include <string>

#include <hatchbed_common/param_handler.h>
#include <hatchbed_common/pointcloud/point_cloud2_util.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

namespace hatchbed_common {
namespace pointcloud {

class CropRadius : public rclcpp::Node {
public:
  explicit CropRadius(const rclcpp::NodeOptions & options)
  : Node("crop_radius", options)
  {
    init_timer_ = create_wall_timer(std::chrono::milliseconds(100),
                                    std::bind(&CropRadius::onInit, this));
  }

private:
  void onInit()
  {
    init_timer_->cancel();

    params_ = std::make_shared<hatchbed_common::ParamHandler>(shared_from_this());
    params_->register_verbose_logging_param();

    params_->param(&radius_, "radius", 1.0,
                   "2D XY radius to crop (m).").min(0.0).dynamic().declare();
    params_->param(&source_frame_, "source_frame", std::string(""),
                   "Frame to crop around; empty = cloud's own origin.").declare();
    params_->param(&invert_, "invert", true,
                   "Remove points inside the radius instead of outside.").dynamic().declare();

    bool transient_local = false;
    params_->param(&transient_local, "transient_local_input", false,
                   "Subscribe with transient_local QoS.").declare();

    if (!source_frame_.empty()) {
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    auto qos = transient_local ? rclcpp::QoS(1).transient_local() : rclcpp::QoS(10);
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "points_in", qos, std::bind(&CropRadius::onPoints, this, std::placeholders::_1));

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("points_out", 10);
    pub_marker_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "crop_radius_markers", 10);

    RCLCPP_INFO(get_logger(), "[crop_radius] Initialized.");
  }

  void onPoints(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & points_in)
  {
    auto x_iter = getFieldIterator<float>(*points_in, "x");
    auto y_iter = getFieldIterator<float>(*points_in, "y");
    auto z_iter = getFieldIterator<float>(*points_in, "z");

    if (!x_iter || !y_iter || !z_iter) {
      RCLCPP_WARN(get_logger(), "[crop_radius] Input points missing x, y, z float fields.");
      return;
    }

    float cx = 0.0f, cy = 0.0f;
    if (!source_frame_.empty()) {
      try {
        auto tf = tf_buffer_->lookupTransform(
          points_in->header.frame_id, source_frame_, tf2::TimePointZero);
        cx = static_cast<float>(tf.transform.translation.x);
        cy = static_cast<float>(tf.transform.translation.y);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "[crop_radius] Could not get transform: %s", ex.what());
        return;
      }
    }

    const float r2 = static_cast<float>(radius_ * radius_);

    auto points_out = std::make_shared<sensor_msgs::msg::PointCloud2>(*points_in);

    size_t size = points_in->width * points_in->height;
    size_t idx = 0;
    for (size_t i = 0; i < size; ++i, ++x_iter, ++y_iter, ++z_iter) {
      float x = *x_iter;
      float y = *y_iter;
      float dx = x - cx;
      float dy = y - cy;
      bool inside = dx * dx + dy * dy < r2;

      if (inside != invert_) {
        std::memcpy(
          &points_out->data[idx * points_out->point_step],
          &points_in->data[i * points_in->point_step],
          points_out->point_step);
        idx++;
      }
    }

    points_out->width = idx;
    points_out->height = 1;
    points_out->row_step = points_out->point_step * points_out->width;
    points_out->is_dense = false;
    points_out->data.resize(points_out->row_step);
    pub_->publish(*points_out);

    if (pub_marker_->get_subscription_count() > 0) {
      const std::string & marker_frame =
        source_frame_.empty() ? points_in->header.frame_id : source_frame_;

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = marker_frame;
      marker.header.stamp = points_in->header.stamp;
      marker.ns = "crop_radius";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 2.0 * radius_;
      marker.scale.y = 2.0 * radius_;
      marker.scale.z = 10.0;
      marker.color.r = invert_ ? 1.0f : 0.0f;
      marker.color.g = invert_ ? 0.0f : 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 0.3f;
      marker.frame_locked = true;

      visualization_msgs::msg::MarkerArray marker_array;
      marker_array.markers.push_back(marker);
      pub_marker_->publish(marker_array);
    }
  }

  rclcpp::TimerBase::SharedPtr init_timer_;
  std::shared_ptr<hatchbed_common::ParamHandler> params_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_;

  double radius_ = 1.0;
  std::string source_frame_;
  bool invert_ = true;
};

}  // namespace pointcloud
}  // namespace hatchbed_common

RCLCPP_COMPONENTS_REGISTER_NODE(hatchbed_common::pointcloud::CropRadius)
