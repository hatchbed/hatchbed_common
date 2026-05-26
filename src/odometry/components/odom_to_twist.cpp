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
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <hatchbed_common/param_handler.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>

namespace hatchbed_common {
namespace odometry {

/**
 * Extracts the twist from a nav_msgs/Odometry message and republishes it as
 * geometry_msgs/TwistWithCovarianceStamped.
 *
 * Direct mode (differential: false):
 *   Copies the twist field verbatim.  An optional frame_id override replaces
 *   the header frame if non-empty.
 *
 * Differential mode (differential: true):
 *   Derives the body-frame twist by finite-differencing consecutive poses:
 *     dT = T_prev_inv * T_curr  (relative transform in the child frame)
 *     linear  = dT.translation / dt
 *     angular = (angle / dt) * axis  from dT's rotation quaternion
 *   Covariance is propagated from pose.covariance: C_twist = 2 * C_pose / dt^2.
 */
class OdomToTwist : public rclcpp::Node {
public:
    explicit OdomToTwist(const rclcpp::NodeOptions& options)
    : Node("odom_to_twist", options)
    {
        init_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&OdomToTwist::onInit, this));
    }

private:
    void onInit() {
        init_timer_->cancel();

        params_ = std::make_shared<hatchbed_common::ParamHandler>(shared_from_this());
        params_->register_verbose_logging_param();

        params_->param(&frame_id_, "frame_id", std::string(""),
            "Override the output header frame_id.  If empty, the odometry header "
            "frame_id is used unchanged.").declare();

        params_->param(&differential_, "differential", false,
            "Derive twist from the differential of the pose rather than reading the "
            "twist field directly.").declare();

        pub_twist_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
            "twist", rclcpp::QoS(10));

        sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
            "odom", rclcpp::QoS(10),
            std::bind(&OdomToTwist::onOdom, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Converting odometry to twist (%s mode).",
            differential_ ? "differential" : "direct");
    }

    void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        geometry_msgs::msg::TwistWithCovarianceStamped twist;
        twist.header = msg->header;
        if (!frame_id_.empty()) {
            twist.header.frame_id = frame_id_;
        }

        if (differential_) {
            const tf2::Transform T_curr = poseToTransform(msg->pose.pose);

            if (!has_prev_) {
                prev_T_     = T_curr;
                prev_stamp_ = msg->header.stamp;
                has_prev_   = true;
                return;
            }

            const double dt =
                (rclcpp::Time(msg->header.stamp) - rclcpp::Time(prev_stamp_)).seconds();

            // Advance state before any early returns so we never stall.
            const tf2::Transform T_prev = prev_T_;
            prev_T_     = T_curr;
            prev_stamp_ = msg->header.stamp;

            if (dt <= 0.0) {
                return;
            }

            // dT = T_prev_inv * T_curr: relative motion in the child (body) frame.
            const tf2::Transform dT = T_prev.inverse() * T_curr;

            const tf2::Vector3 lin = dT.getOrigin() / dt;
            twist.twist.twist.linear.x = lin.x();
            twist.twist.twist.linear.y = lin.y();
            twist.twist.twist.linear.z = lin.z();

            // dq = (cos(a/2), sin(a/2)*axis)  =>  w = (a/dt) * axis
            tf2::Quaternion dq = dT.getRotation();
            dq.normalize();
            if (dq.w() < 0.0) {
                dq = tf2::Quaternion(-dq.x(), -dq.y(), -dq.z(), -dq.w());
            }

            const double vec_norm = std::sqrt(
                dq.x() * dq.x() + dq.y() * dq.y() + dq.z() * dq.z());
            const double angle = 2.0 * std::atan2(vec_norm, dq.w());

            if (vec_norm > 1e-10) {
                const double scale = angle / (vec_norm * dt);
                twist.twist.twist.angular.x = scale * dq.x();
                twist.twist.twist.angular.y = scale * dq.y();
                twist.twist.twist.angular.z = scale * dq.z();
            }

            // Independent-sample finite-difference covariance: C_twist = 2 * C_pose / dt^2
            const double inv_dt2 = 1.0 / (dt * dt);
            for (int i = 0; i < 6; ++i) {
                for (int j = 0; j < 6; ++j) {
                    twist.twist.covariance[i * 6 + j] =
                        2.0 * msg->pose.covariance[i * 6 + j] * inv_dt2;
                }
            }

        } else {
            twist.twist = msg->twist;
        }

        pub_twist_->publish(twist);
    }

    static tf2::Transform poseToTransform(const geometry_msgs::msg::Pose& pose) {
        return tf2::Transform(
            tf2::Quaternion(
                pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w),
            tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
    }

    rclcpp::TimerBase::SharedPtr                   init_timer_;
    std::shared_ptr<hatchbed_common::ParamHandler> params_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                     sub_odom_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_twist_;

    std::string    frame_id_;
    bool           differential_ = false;

    // Differential mode state.
    tf2::Transform prev_T_;
    rclcpp::Time   prev_stamp_;
    bool           has_prev_ = false;
};

}  // namespace odometry
}  // namespace hatchbed_common

RCLCPP_COMPONENTS_REGISTER_NODE(hatchbed_common::odometry::OdomToTwist)
