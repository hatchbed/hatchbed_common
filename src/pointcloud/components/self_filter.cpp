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
#include <exception>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <hatchbed_common/param_handler.h>
#include <hatchbed_common/pointcloud/convex_prism.hpp>
#include <hatchbed_common/pointcloud/self_shapes.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/time.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

namespace hatchbed_common {
namespace pointcloud {

// How often the prism markers are published, in seconds. The shapes move at the speed the
// machine articulates, so this is about keeping a viewer current rather than about keeping up.
constexpr double kMarkerPeriod = 1.0;

// Removes a robot's own structure from its sensors' returns.
//
// A hemispherical lidar mounted on a machine sees the machine. On a lift platform that can
// be most of what it sees -- a chassis dome under a deck returns the deck, the guardrails
// and its own bracket long before it returns anything worth registering or avoiding. Left
// in, those returns are indistinguishable from the world: they drag a scan's centroid, they
// dilute every ratio computed over the scan, and they fill a costmap with the vehicle.
//
// The shapes come from the robot description (see buildSelfShapes) and follow TF, so a lift
// that raises, an extension that slides and a guardrail set that a launch argument left off
// are all handled by not being assumed. See self_shapes.hpp for what is built and how
// finely; this node is the plumbing around it.
class SelfFilter : public rclcpp::Node
{
public:
  explicit SelfFilter(const rclcpp::NodeOptions & options)
  : Node("self_filter", options)
  {
    init_timer_ = create_wall_timer(std::chrono::milliseconds(100),
                                    std::bind(&SelfFilter::onInit, this));
  }

private:
  void onInit()
  {
    init_timer_->cancel();

    params_ = std::make_shared<hatchbed_common::ParamHandler>(shared_from_this());
    params_->register_verbose_logging_param();

    params_->param(&filter_frame_, "filter_frame", std::string("base_link"),
                   "Frame the shapes are held and the test is made in. Wants to be a frame "
                   "the robot's own links do not tilt in -- a body frame, not a level world "
                   "one -- since a prism that tilts has to be rebuilt rather than moved.")
      .declare();
    params_->param(&padding_, "padding", 0.05,
                   "How far every shape is grown (m), covering the difference between a "
                   "model and a machine.").min(0.0).max(1.0).declare();
    params_->param(&geometry_, "geometry", std::string("both"),
                   "Which of a link's geometry to build shapes from: 'both', 'collision' or "
                   "'visual'. Both, because the two are approximations of the same structure "
                   "and collision geometry is often the cruder one -- a chassis in two boxes "
                   "leaves out its battery box and bumper. The union can only over-cover.")
      .declare();
    params_->param(&combine_elements_, "combine_link_elements", false,
                   "One prism per link rather than one per element, where a link does not "
                   "say otherwise. Cheaper and blinder: combining fills whatever the "
                   "elements did not, so it suits a link whose elements already touch.")
      .declare();
    params_->param(&tf_timeout_, "tf_timeout", 0.1,
                   "How long to wait for a transform at a cloud's own stamp (s).")
      .min(0.0).max(5.0).dynamic().declare();
    params_->param(&drop_on_missing_transform_, "drop_on_missing_transform", false,
                   "Drop a cloud that cannot be placed, rather than passing it through "
                   "unfiltered. Passing through is the more debuggable failure; both end "
                   "with a vehicle that stops.").dynamic().declare();
    params_->param(&report_period_, "report_period", 10.0,
                   "How often to log the fraction being removed (s). 0 never logs.")
      .min(0.0).max(600.0).dynamic().declare();

    readStructuralConfig();

    // Transient local, because the description is latched once at startup and this node may
    // well come up after it.
    description_sub_ = create_subscription<std_msgs::msg::String>(
      "robot_description", rclcpp::QoS(1).transient_local(),
      std::bind(&SelfFilter::onDescription, this, std::placeholders::_1));

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "points_in", rclcpp::SensorDataQoS(),
      std::bind(&SelfFilter::onPoints, this, std::placeholders::_1));
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("points_out",
                                                           rclcpp::SensorDataQoS());
    // Relative, like everything else here: one of these runs per sensor, and keeping them
    // apart is the launch's business (a namespace per sensor) rather than something to bake
    // into the names.
    removed_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("points_removed",
                                                                   rclcpp::SensorDataQoS());
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("filter_markers",
                                                                          rclcpp::QoS(1));

    // The only thing that publishes the markers. They are a picture of where the shapes are,
    // not a per-cloud output, and at cloud rate they would cost more than they show.
    marker_timer_ = create_wall_timer(
      std::chrono::duration<double>(kMarkerPeriod),
      std::bind(&SelfFilter::publishMarkers, this));
  }

  // The structural half of the configuration -- which links combine, which are left out,
  // which are grouped -- read from the parameter overrides rather than declared.
  //
  // Declaring would mean knowing the link names before reading them, which is the thing
  // being read. These describe the robot rather than tuning it, so they are startup-only and
  // do not appear in `ros2 param list`; the yaml that sets them is the record of what they
  // are.
  void readStructuralConfig()
  {
    const auto & overrides = get_node_parameters_interface()->get_parameter_overrides();
    for (const auto & [key, value] : overrides) {
      const auto parts = split(key);
      if (parts.size() == 3 && parts[0] == "link") {
        const std::string & link = parts[1];
        try {
          if (parts[2] == "geometry") {
            if (const auto geometry = geometryFromString(value.get<std::string>())) {
              shape_config_.link_geometry[link] = *geometry;
            } else {
              RCLCPP_WARN(get_logger(), "ignoring '%s': '%s' is not both, collision or visual",
                          key.c_str(), value.get<std::string>().c_str());
            }
          } else if (parts[2] == "combine") {
            shape_config_.link_combine[link] = value.get<bool>();
          } else if (parts[2] == "enabled" && !value.get<bool>()) {
            shape_config_.disabled_links.insert(link);
          } else if (parts[2] == "padding") {
            shape_config_.link_padding[link] = static_cast<float>(value.get<double>());
          } else {
            RCLCPP_WARN(get_logger(), "ignoring unknown setting '%s'", key.c_str());
          }
        } catch (const std::exception & error) {
          RCLCPP_WARN(get_logger(), "ignoring '%s': %s", key.c_str(), error.what());
        }
      } else if (parts.size() == 3 && parts[0] == "group" && parts[2] == "links") {
        try {
          shape_config_.groups.push_back({parts[1], value.get<std::vector<std::string>>()});
        } catch (const std::exception & error) {
          RCLCPP_WARN(get_logger(), "ignoring group '%s': %s", parts[1].c_str(), error.what());
        }
      }
    }
  }

  static std::optional<SelfShapeConfig::Geometry> geometryFromString(const std::string & name)
  {
    if (name == "both") {
      return SelfShapeConfig::Geometry::Both;
    }
    if (name == "collision") {
      return SelfShapeConfig::Geometry::Collision;
    }
    if (name == "visual") {
      return SelfShapeConfig::Geometry::Visual;
    }
    return std::nullopt;
  }

  static std::vector<std::string> split(const std::string & key)
  {
    std::vector<std::string> parts;
    size_t start = 0;
    while (start <= key.size()) {
      const size_t dot = key.find('.', start);
      if (dot == std::string::npos) {
        parts.push_back(key.substr(start));
        break;
      }
      parts.push_back(key.substr(start, dot - start));
      start = dot + 1;
    }
    return parts;
  }

  void onDescription(const std_msgs::msg::String::ConstSharedPtr & description)
  {
    if (const auto geometry = geometryFromString(geometry_)) {
      shape_config_.geometry = *geometry;
    } else {
      RCLCPP_WARN(get_logger(), "'%s' is not both, collision or visual; using both",
                  geometry_.c_str());
      shape_config_.geometry = SelfShapeConfig::Geometry::Both;
    }
    shape_config_.combine_elements = combine_elements_;
    shape_config_.padding = static_cast<float>(padding_);

    shapes_ = buildSelfShapes(description->data, shape_config_,
                              [this](const std::string & warning) {
                                RCLCPP_WARN(get_logger(), "%s", warning.c_str());
                              });
    if (shapes_.empty()) {
      RCLCPP_ERROR(get_logger(), "the description yielded no shapes; nothing will be removed");
      return;
    }
    marker_set_changed_ = true;
    RCLCPP_INFO(get_logger(), "filtering against %zu prisms from %zu group(s)", shapes_.size(),
                shape_config_.groups.size());
  }

  void onPoints(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud)
  {
    if (shapes_.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "no shapes yet, so nothing is being removed; is "
                           "robot_description published?");
      pub_->publish(*cloud);
      return;
    }

    const auto cloud_to_filter = lookup(filter_frame_, cloud->header.frame_id,
                                        cloud->header.stamp);
    if (!cloud_to_filter) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "cannot place '%s' in '%s', so this cloud is %s",
                            cloud->header.frame_id.c_str(), filter_frame_.c_str(),
                            drop_on_missing_transform_ ? "dropped" : "passed through unfiltered");
      if (!drop_on_missing_transform_) {
        pub_->publish(*cloud);
      }
      return;
    }

    if (!updateShapes(cloud->header.stamp)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "none of the robot's own links could be placed, so this cloud is "
                            "%s", drop_on_missing_transform_ ? "dropped" : "passed through");
      if (!drop_on_missing_transform_) {
        pub_->publish(*cloud);
      }
      return;
    }

    sensor_msgs::msg::PointCloud2 kept;
    sensor_msgs::msg::PointCloud2 cut;
    const bool want_removed = removed_pub_->get_subscription_count() > 0;
    const size_t removed = removeConvexPrisms(*cloud, prisms_, bounds_, *cloud_to_filter, kept,
                                              want_removed ? &cut : nullptr);
    pub_->publish(kept);
    if (want_removed) {
      removed_pub_->publish(cut);
    }

    last_stamp_ = cloud->header.stamp;
    report(*cloud, removed);
  }

  // Brings every shape up to where its link is now and mirrors them into the flat arrays the
  // point loop reads.
  //
  // A shape that cannot be placed for this cloud keeps the prism it last had rather than
  // dropping out of the set. A momentary lookup failure -- a cloud stamped a little ahead of
  // where TF has got to, an FK publisher that skipped a beat -- is not evidence that the
  // structure moved, and treating it as though the shape were gone leaks self returns
  // through for that cloud and makes the markers blink. The pose it is standing in for
  // changes at the speed a lift moves, so a cloud or two of staleness stays inside the
  // padding; a shape that has never been placed at all has no prism to stand in and is the
  // only case left out.
  bool updateShapes(const rclcpp::Time & stamp)
  {
    // At the cloud's stamp for every source, or at the latest TF has for every source --
    // never a mix. A shape built from several links is a hull over all of them, so placing
    // one member at one time and another at another describes a configuration the machine was
    // never in: the hull comes out somewhere between the two, and it changes shape every time
    // the mix does. The lift is the case that matters, being the one whose members move
    // relative to each other.
    const auto at_stamp = [&](const std::string & frame) {
      return lookup(filter_frame_, frame, stamp);
    };
    const auto at_latest = [&](const std::string & frame) {
      return lookupLatest(frame);
    };

    prisms_.clear();
    bounds_.clear();
    size_t missing = 0;
    size_t stale = 0;
    for (auto & shape : shapes_) {
      if (!shape.update(at_stamp) && !shape.update(at_latest)) {
        if (shape.prism().hull.empty()) {
          ++missing;
          continue;
        }
        ++stale;
      }
      prisms_.push_back(shape.prism());
      bounds_.push_back(shape.bounds());
    }

    if (missing > 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "%zu of %zu shapes have never been placed and are not being "
                           "removed; is anything publishing their transforms?",
                           missing, shapes_.size());
    }
    if (stale > 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "%zu of %zu shapes could not be placed for this cloud and are "
                           "being applied where they last were", stale, shapes_.size());
    }
    return !prisms_.empty();
  }

  // Where a link is, at the latest TF has it.
  //
  // Falling back to this is deliberate and is not the same concession as the one for the
  // cloud's own frame. That one carries vehicle motion, which at a metre a second is
  // centimetres over a sweep and has to be the cloud's own stamp. A link's pose in the body
  // frame carries only articulation -- a lift at centimetres a second -- so the difference
  // between the cloud's stamp and a few milliseconds ago is a fraction of a millimetre, well
  // under the padding, and far cheaper than not filtering that shape at all.
  std::optional<Eigen::Isometry3f> lookupLatest(const std::string & frame)
  {
    try {
      const auto transform = tf_buffer_->lookupTransform(filter_frame_, frame,
                                                         tf2::TimePointZero);
      return tf2::transformToEigen(transform).cast<float>();
    } catch (const tf2::TransformException &) {
      return std::nullopt;
    }
  }

  std::optional<Eigen::Isometry3f> lookup(const std::string & target, const std::string & source,
                                          const rclcpp::Time & stamp)
  {
    try {
      const auto transform = tf_buffer_->lookupTransform(
        target, source, stamp, rclcpp::Duration::from_seconds(tf_timeout_));
      return tf2::transformToEigen(transform).cast<float>();
    } catch (const tf2::TransformException &) {
      return std::nullopt;
    }
  }

  void report(const sensor_msgs::msg::PointCloud2 & cloud, size_t removed)
  {
    if (report_period_ <= 0.0) {
      return;
    }
    const size_t size = static_cast<size_t>(cloud.width) * cloud.height;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(),
                         static_cast<uint64_t>(report_period_ * 1000.0),
                         "removed %.1f%% of %zu returns (%zu prisms)",
                         size == 0 ? 0.0 : 100.0 * static_cast<double>(removed) / size, size,
                         prisms_.size());
  }

  // The prisms as they are being applied: side walls and both caps, one namespace each so a
  // viewer can take them apart. Whether a filter is eating the right volume is a question
  // about a shape, and no number answers it.
  void publishMarkers()
  {
    if (marker_pub_->get_subscription_count() == 0 || shapes_.empty() || !last_stamp_) {
      return;
    }

    visualization_msgs::msg::MarkerArray markers;
    std_msgs::msg::Header header;
    header.frame_id = filter_frame_;
    // The stamp of the cloud these prisms were placed for, not the time they are being
    // published at. It is when they were true, and -- since a transform was resolved at it to
    // place them -- it is a time whoever draws them can resolve too. Stamping with now()
    // asks a viewer for a transform into its own fixed frame at a time that has not been
    // published yet, which under sim time it intermittently cannot do.
    header.stamp = *last_stamp_;

    // Only when the set itself changed, which is when a new description arrives. Clearing on
    // every publish is what makes an array of markers blink: every shape is deleted and
    // re-added rather than moved, and a viewer draws the gap. Each shape keeps its own
    // namespace, so publishing it again simply moves it.
    if (marker_set_changed_) {
      visualization_msgs::msg::Marker clear;
      clear.header = header;
      clear.action = visualization_msgs::msg::Marker::DELETEALL;
      markers.markers.push_back(clear);
      marker_set_changed_ = false;
    }

    for (const auto & shape : shapes_) {
      const ConvexPrism & prism = shape.prism();
      if (prism.hull.size() < 3) {
        continue;
      }

      visualization_msgs::msg::Marker marker;
      marker.header = header;
      marker.ns = shape.name();
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
      marker.action = visualization_msgs::msg::Marker::ADD;
      // These describe where the robot's own structure is, so they belong to the body frame
      // and should follow it rather than be pinned to where it was at one instant.
      marker.frame_locked = true;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 1.0;
      marker.scale.y = 1.0;
      marker.scale.z = 1.0;
      marker.color.r = 0.9f;
      marker.color.g = 0.3f;
      marker.color.b = 0.1f;
      marker.color.a = 0.25f;  // translucent: what is behind a shape is the point
      // Long enough to outlive the publish period several times over, so a shape that stops
      // being published -- because the description changed -- goes away on its own rather
      // than sitting there.
      marker.lifetime = rclcpp::Duration::from_seconds(kMarkerPeriod * 3.0);

      const size_t n = prism.hull.size();
      for (size_t v = 0; v < n; ++v) {
        const auto & a = prism.hull[v];
        const auto & b = prism.hull[(v + 1) % n];
        addTriangle(marker, {a.x(), a.y(), prism.min_z}, {b.x(), b.y(), prism.min_z},
                    {b.x(), b.y(), prism.max_z});
        addTriangle(marker, {a.x(), a.y(), prism.min_z}, {b.x(), b.y(), prism.max_z},
                    {a.x(), a.y(), prism.max_z});
        if (v > 0 && v + 1 < n) {
          addTriangle(marker, {prism.hull[0].x(), prism.hull[0].y(), prism.min_z},
                      {a.x(), a.y(), prism.min_z}, {b.x(), b.y(), prism.min_z});
          addTriangle(marker, {prism.hull[0].x(), prism.hull[0].y(), prism.max_z},
                      {a.x(), a.y(), prism.max_z}, {b.x(), b.y(), prism.max_z});
        }
      }
      markers.markers.push_back(std::move(marker));
    }
    marker_pub_->publish(markers);
  }

  static void addTriangle(visualization_msgs::msg::Marker & marker, const Eigen::Vector3f & a,
                          const Eigen::Vector3f & b, const Eigen::Vector3f & c)
  {
    for (const auto & corner : {a, b, c}) {
      geometry_msgs::msg::Point point;
      point.x = corner.x();
      point.y = corner.y();
      point.z = corner.z();
      marker.points.push_back(point);
    }
  }

  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr marker_timer_;
  std::shared_ptr<hatchbed_common::ParamHandler> params_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr description_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr removed_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  SelfShapeConfig shape_config_;
  std::vector<SelfShape> shapes_;
  // The shapes being applied to the cloud in hand, flattened for the point loop.
  std::vector<ConvexPrism> prisms_;
  std::vector<PrismBounds> bounds_;
  // The cloud the prisms in hand were placed for, which is what the markers are stamped with.
  std::optional<builtin_interfaces::msg::Time> last_stamp_;
  bool marker_set_changed_ = false;

  std::string filter_frame_;
  std::string geometry_ = "both";
  double padding_ = 0.05;
  bool combine_elements_ = false;
  double tf_timeout_ = 0.1;
  bool drop_on_missing_transform_ = false;
  double report_period_ = 10.0;
};

}  // namespace pointcloud
}  // namespace hatchbed_common

RCLCPP_COMPONENTS_REGISTER_NODE(hatchbed_common::pointcloud::SelfFilter)
