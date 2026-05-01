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

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <hatchbed_common/param_handler.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace hatchbed_common {
namespace localization {

/**
 * Broadcasts a TF transform derived from an odometry message to establish
 * parent_frame -> child_frame in the main TF tree.
 *
 * An odometry message expresses the pose of child_frame_id in header.frame_id
 * (T_header_child).  child_frame must match either header.frame_id or
 * child_frame_id.  The remaining frame is looked up in the main TF tree to
 * compute T_parent_child:
 *
 *   child_frame == header.frame_id:
 *     known = child_frame_id
 *     T_parent_child = T_parent_known * inv(T_header_child)
 *
 *   child_frame == child_frame_id:
 *     known = header.frame_id
 *     T_parent_child = T_parent_known * T_header_child
 *
 * The lookup of T_parent_known handles the identity case (parent_frame ==
 * known) so no TF lookup is needed when parent_frame directly appears in the
 * odometry message.  The only hard requirement is that child_frame must not
 * already have a parent in the main TF tree.
 */
class OdomTfBroadcast : public rclcpp::Node {
public:
    explicit OdomTfBroadcast(const rclcpp::NodeOptions& options)
    : Node("odom_tf_broadcast", options)
    {
        init_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&OdomTfBroadcast::onInit, this));
    }

private:
    void onInit() {
        init_timer_->cancel();

        params_ = std::make_shared<hatchbed_common::ParamHandler>(shared_from_this());
        params_->register_verbose_logging_param();

        params_->param(&child_frame_, "child_frame", std::string(""),
            "Frame to broadcast as a child of parent_frame.  Must match either "
            "header.frame_id or child_frame_id of the odometry message.").declare();

        params_->param(&parent_frame_, "parent_frame", std::string(""),
            "Frame that will become the parent of child_frame in the main TF "
            "tree.").declare();

        params_->param(&timestamp_offset_, "timestamp_offset", 0.0,
            "Seconds added to the odometry message timestamp before broadcasting. "
            "A positive value future-dates the transform so that it remains valid "
            "between low-frequency updates (e.g. set to 1.5x the odometry period)."
            ).min(-1.0).max(10.0).dynamic().declare();

        if (child_frame_.empty() || parent_frame_.empty()) {
            RCLCPP_ERROR(get_logger(), "child_frame and parent_frame must both be set.");
            return;
        }

        tf_buffer_      = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_    = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
            "odom", rclcpp::QoS(10),
            std::bind(&OdomTfBroadcast::onOdom, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
            "Broadcasting '%s' -> '%s' from odometry.",
            parent_frame_.c_str(), child_frame_.c_str());
    }

    void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        const std::string& frame_header = msg->header.frame_id;
        const std::string& frame_child  = msg->child_frame_id;

        // T_header_child: pose of child_frame_id expressed in header.frame_id.
        tf2::Transform T_header_child;
        T_header_child.setOrigin(tf2::Vector3(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z));
        T_header_child.setRotation(tf2::Quaternion(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w));

        tf2::Transform T_parent_child;

        if (child_frame_ == frame_header) {
            // child_frame is the "world-like" frame in the message; frame_child
            // is the known frame in the main tree.
            // T_parent_child = T_parent_known * inv(T_header_child)
            tf2::Transform T_parent_known;
            if (!lookupTransform(parent_frame_, frame_child, T_parent_known)) {
                return;
            }
            T_parent_child = T_parent_known * T_header_child.inverse();

        } else if (child_frame_ == frame_child) {
            // child_frame is the "robot-like" frame in the message; frame_header
            // is the known frame in the main tree.
            // T_parent_child = T_parent_known * T_header_child
            tf2::Transform T_parent_known;
            if (!lookupTransform(parent_frame_, frame_header, T_parent_known)) {
                return;
            }
            T_parent_child = T_parent_known * T_header_child;

        } else {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "child_frame '%s' is neither header.frame_id ('%s') nor "
                "child_frame_id ('%s') of the odometry message.",
                child_frame_.c_str(), frame_header.c_str(), frame_child.c_str());
            return;
        }

        geometry_msgs::msg::TransformStamped out;
        out.header.stamp    = rclcpp::Time(msg->header.stamp) +
                              rclcpp::Duration::from_seconds(timestamp_offset_);
        out.header.frame_id = parent_frame_;
        out.child_frame_id  = child_frame_;
        out.transform       = toMsg(T_parent_child);
        tf_broadcaster_->sendTransform(out);
    }

    bool lookupTransform(
        const std::string& target, const std::string& source,
        tf2::Transform& result)
    {
        if (target == source) {
            result.setIdentity();
            return true;
        }
        try {
            result = fromMsg(
                tf_buffer_->lookupTransform(target, source, tf2::TimePointZero).transform);
            return true;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_DEBUG(get_logger(),
                "Waiting for '%s' -> '%s': %s", target.c_str(), source.c_str(), ex.what());
            return false;
        }
    }

    static tf2::Transform fromMsg(const geometry_msgs::msg::Transform& t) {
        return tf2::Transform(
            tf2::Quaternion(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w),
            tf2::Vector3(t.translation.x, t.translation.y, t.translation.z));
    }

    static geometry_msgs::msg::Transform toMsg(const tf2::Transform& t) {
        geometry_msgs::msg::Transform msg;
        msg.translation.x = t.getOrigin().x();
        msg.translation.y = t.getOrigin().y();
        msg.translation.z = t.getOrigin().z();
        msg.rotation.x    = t.getRotation().x();
        msg.rotation.y    = t.getRotation().y();
        msg.rotation.z    = t.getRotation().z();
        msg.rotation.w    = t.getRotation().w();
        return msg;
    }

    rclcpp::TimerBase::SharedPtr                   init_timer_;
    std::shared_ptr<hatchbed_common::ParamHandler> params_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    std::shared_ptr<tf2_ros::Buffer>                         tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>              tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>           tf_broadcaster_;

    std::string child_frame_;
    std::string parent_frame_;
    double      timestamp_offset_ = 0.0;
};

}  // namespace localization
}  // namespace hatchbed_common

RCLCPP_COMPONENTS_REGISTER_NODE(hatchbed_common::localization::OdomTfBroadcast)
