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

#include <hatchbed_common/param_handler.h>
#include <hatchbed_common/pointcloud/point_cloud2_util.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace hatchbed_common {
namespace pointcloud {

class CropBox : public rclcpp::Node {
public:
  explicit CropBox(const rclcpp::NodeOptions & options)
  : Node("crop_box", options)
  {
    init_timer_ = create_wall_timer(std::chrono::milliseconds(100),
                                    std::bind(&CropBox::onInit, this));
  }

private:
  void onInit()
  {
    init_timer_->cancel();

    params_ = std::make_shared<hatchbed_common::ParamHandler>(shared_from_this());
    params_->register_verbose_logging_param();

    params_->param(&min_x_, "min_x", 0.0, "Min x bound of box (m).").dynamic().declare();
    params_->param(&min_y_, "min_y", 0.0, "Min y bound of box (m).").dynamic().declare();
    params_->param(&min_z_, "min_z", 0.0, "Min z bound of box (m).").dynamic().declare();
    params_->param(&max_x_, "max_x", 0.0, "Max x bound of box (m).").dynamic().declare();
    params_->param(&max_y_, "max_y", 0.0, "Max y bound of box (m).").dynamic().declare();
    params_->param(&max_z_, "max_z", 0.0, "Max z bound of box (m).").dynamic().declare();
    params_->param(&invert_, "invert", true,
                   "Remove points inside the box instead of outside.").dynamic().declare();

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "points_in", 10, std::bind(&CropBox::onPoints, this, std::placeholders::_1));

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("points_out", 10);
    pub_marker_ = create_publisher<visualization_msgs::msg::MarkerArray>("crop_box_markers", 10);

    RCLCPP_INFO(get_logger(), "[crop_box] Initialized.");
  }

  void onPoints(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & points_in)
  {
    auto x_iter = getFieldIterator<float>(*points_in, "x");
    auto y_iter = getFieldIterator<float>(*points_in, "y");
    auto z_iter = getFieldIterator<float>(*points_in, "z");

    if (!x_iter || !y_iter || !z_iter) {
      RCLCPP_WARN(get_logger(), "[crop_box] Input points missing x, y, z float fields.");
      return;
    }

    auto points_out = std::make_shared<sensor_msgs::msg::PointCloud2>(*points_in);

    size_t size = points_in->width * points_in->height;
    size_t idx = 0;
    for (size_t i = 0; i < size; ++i, ++x_iter, ++y_iter, ++z_iter) {
      float x = *x_iter;
      float y = *y_iter;
      float z = *z_iter;

      bool inside = x > min_x_ && x < max_x_ &&
                    y > min_y_ && y < max_y_ &&
                    z > min_z_ && z < max_z_;

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
      visualization_msgs::msg::Marker marker;
      marker.header = points_in->header;
      marker.ns = "crop_box";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = (min_x_ + max_x_) / 2.0;
      marker.pose.position.y = (min_y_ + max_y_) / 2.0;
      marker.pose.position.z = (min_z_ + max_z_) / 2.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = max_x_ - min_x_;
      marker.scale.y = max_y_ - min_y_;
      marker.scale.z = max_z_ - min_z_;
      if (invert_) {
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
      } else {
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
      }
      marker.color.b = 0.0f;
      marker.color.a = 0.5f;
      marker.frame_locked = true;

      visualization_msgs::msg::MarkerArray marker_array;
      marker_array.markers.push_back(marker);
      pub_marker_->publish(marker_array);
    }
  }

  rclcpp::TimerBase::SharedPtr init_timer_;
  std::shared_ptr<hatchbed_common::ParamHandler> params_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_;

  double min_x_ = 0.0;
  double min_y_ = 0.0;
  double min_z_ = 0.0;
  double max_x_ = 0.0;
  double max_y_ = 0.0;
  double max_z_ = 0.0;
  bool invert_ = true;
};

}  // namespace pointcloud
}  // namespace hatchbed_common

RCLCPP_COMPONENTS_REGISTER_NODE(hatchbed_common::pointcloud::CropBox)
