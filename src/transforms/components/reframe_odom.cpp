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

#include <hatchbed_common/param_handler.h>
#include <hatchbed_common/transforms/covariance_util.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <hatchbed_common/tf2_ros_compat.h>

namespace hatchbed_common {
namespace transforms {

/**
 * Reframes an odometry topic to another fixed frame.  The transform is only
 * captured once and kept fixed for the purpose of this process.
 *
 * This, in effect, aligns the odometry topic to the selected fixed frame at an
 * anchor point, but the poses in the reframed odometry topic remain rigid with
 * respect to each other even if the source and target frame drift over
 * time.
 */
class ReframeOdom : public rclcpp::Node {
public:
    explicit ReframeOdom(const rclcpp::NodeOptions& options)
    : Node("reframe_odom", options)
    {
        init_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ReframeOdom::onInit, this));
    }

private:
    void onInit() {
        init_timer_->cancel();

        params_ = std::make_shared<hatchbed_common::ParamHandler>(shared_from_this());
        params_->register_verbose_logging_param();

        params_->param(&target_frame_, "target_frame", std::string(""),
            "Target fixed frame for the output odometry.").declare();

        tf_buffer_      = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_    = makeTransformListener(*tf_buffer_, this);

        pub_odom_ = create_publisher<nav_msgs::msg::Odometry>(
            "odom_out", rclcpp::QoS(10));
        sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
            "odom_in", rclcpp::QoS(10),
            std::bind(&ReframeOdom::onOdom, this, std::placeholders::_1));
    }

    void onOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        const std::string& frame_header = msg->header.frame_id;
        if (target_frame_ == frame_header) {
            // The odometry message is already in the target frame, so just
            // republish.
            pub_odom_->publish(*msg);
            return;
        }

        if (!initialized_ && !lookupTransform(target_frame_, frame_header, transform_)) {
            return;
        }
        initialized_ = true;

        tf2::Transform pose;
        pose.setOrigin(tf2::Vector3(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z));
        pose.setRotation(tf2::Quaternion(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w));

        // Transform the pose.
        pose = transform_ * pose;

        // Transform the twist.
        tf2::Vector3 linear(msg->twist.twist.linear.x, msg->twist.twist.linear.y,
                            msg->twist.twist.linear.z);
        tf2::Vector3 angular(msg->twist.twist.angular.x, msg->twist.twist.angular.y,
                             msg->twist.twist.angular.z);
        linear  = transform_.getBasis() * linear;
        angular = transform_.getBasis() * angular;

        auto position    = pose.getOrigin();
        auto orientation = pose.getRotation();

        nav_msgs::msg::Odometry out;
        out.header = msg->header;
        out.header.frame_id = target_frame_;
        out.child_frame_id = msg->child_frame_id;
        out.pose.pose.position.x    = position.x();
        out.pose.pose.position.y    = position.y();
        out.pose.pose.position.z    = position.z();
        out.pose.pose.orientation.x = orientation.x();
        out.pose.pose.orientation.y = orientation.y();
        out.pose.pose.orientation.z = orientation.z();
        out.pose.pose.orientation.w = orientation.w();
        out.twist.twist.linear.x    = linear.x();
        out.twist.twist.linear.y    = linear.y();
        out.twist.twist.linear.z    = linear.z();
        out.twist.twist.angular.x   = angular.x();
        out.twist.twist.angular.y   = angular.y();
        out.twist.twist.angular.z   = angular.z();
        out.pose.covariance  = rotateCovariance(msg->pose.covariance, transform_.getRotation());
        out.twist.covariance = rotateCovariance(msg->twist.covariance, transform_.getRotation());
        pub_odom_->publish(out);
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

    rclcpp::TimerBase::SharedPtr                   init_timer_;
    std::shared_ptr<hatchbed_common::ParamHandler> params_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr    pub_odom_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    std::shared_ptr<tf2_ros::Buffer>                         tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>              tf_listener_;

    std::string target_frame_;

    bool initialized_ = false;
    tf2::Transform transform_;
};

}  // namespace transforms
}  // namespace hatchbed_common

RCLCPP_COMPONENTS_REGISTER_NODE(hatchbed_common::transforms::ReframeOdom)
