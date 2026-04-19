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
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

namespace hatchbed_common {
namespace localization {

/**
 * Converts a sensor_msgs/Imu message to geometry_msgs/TwistWithCovarianceStamped.
 *
 * Direct mode (differential: false):
 *   Copies angular_velocity and its diagonal covariance from the IMU message.
 *   Linear velocity is zero (IMUs do not provide it).
 *
 * Differential mode (differential: true):
 *   Derives angular velocity by differentiating consecutive orientation quaternions.
 *   Given dq = q_prev_inv * q_curr = (cos(a/2), sin(a/2)*axis), the body-frame
 *   angular velocity is w = (a / dt) * axis.
 *   Covariance is propagated from orientation_covariance: C_w = 2 * C_theta / dt^2.
 *   Requires the orientation field to be populated (orientation_covariance[0] >= 0).
 */
class ImuToTwist : public rclcpp::Node {
public:
    explicit ImuToTwist(const rclcpp::NodeOptions& options)
    : Node("imu_to_twist", options)
    {
        init_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ImuToTwist::onInit, this));
    }

private:
    void onInit() {
        init_timer_->cancel();

        params_ = std::make_shared<hatchbed_common::ParamHandler>(shared_from_this());
        params_->register_verbose_logging_param();

        params_->param(&differential_, "differential", false,
            "Derive angular velocity from the differential of the orientation rather "
            "than reading angular_velocity directly.  Requires the orientation field "
            "to be populated.").declare();

        pub_twist_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
            "twist", rclcpp::QoS(10));

        sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
            "imu", rclcpp::QoS(10),
            std::bind(&ImuToTwist::onImu, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Converting IMU to twist (%s mode).",
            differential_ ? "differential" : "direct");
    }

    void onImu(const sensor_msgs::msg::Imu::SharedPtr msg) {
        geometry_msgs::msg::TwistWithCovarianceStamped twist;
        twist.header = msg->header;

        if (differential_) {
            if (msg->orientation_covariance[0] < 0.0) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                    "differential mode requires orientation to be populated "
                    "(orientation_covariance[0] < 0 indicates it is not); "
                    "dropping message.");
                return;
            }

            tf2::Quaternion q_curr(
                msg->orientation.x, msg->orientation.y,
                msg->orientation.z, msg->orientation.w);

            if (!has_prev_) {
                prev_q_     = q_curr;
                prev_stamp_ = msg->header.stamp;
                has_prev_   = true;
                return;
            }

            const double dt =
                (rclcpp::Time(msg->header.stamp) - rclcpp::Time(prev_stamp_)).seconds();

            // Advance state before any early returns so we never stall.
            const tf2::Quaternion q_prev = prev_q_;
            prev_q_     = q_curr;
            prev_stamp_ = msg->header.stamp;

            if (dt <= 0.0) {
                return;
            }

            // dq = q_prev_inv * q_curr: body-frame rotation over dt.
            // Ensure shortest-path (w >= 0) before extracting angle.
            tf2::Quaternion dq = q_prev.inverse() * q_curr;
            dq.normalize();
            if (dq.w() < 0.0) {
                dq = tf2::Quaternion(-dq.x(), -dq.y(), -dq.z(), -dq.w());
            }

            // dq = (cos(a/2), sin(a/2)*axis)  =>  w = (a/dt) * axis
            const double vec_norm = std::sqrt(
                dq.x() * dq.x() + dq.y() * dq.y() + dq.z() * dq.z());
            const double angle = 2.0 * std::atan2(vec_norm, dq.w());

            if (vec_norm > 1e-10) {
                const double scale = angle / (vec_norm * dt);
                twist.twist.twist.angular.x = scale * dq.x();
                twist.twist.twist.angular.y = scale * dq.y();
                twist.twist.twist.angular.z = scale * dq.z();
            }

            // Independent-sample finite-difference covariance: C_w = 2 * C_theta / dt^2
            const double inv_dt2 = 1.0 / (dt * dt);
            twist.twist.covariance[21] = 2.0 * msg->orientation_covariance[0] * inv_dt2;
            twist.twist.covariance[28] = 2.0 * msg->orientation_covariance[4] * inv_dt2;
            twist.twist.covariance[35] = 2.0 * msg->orientation_covariance[8] * inv_dt2;

        } else {
            twist.twist.twist.angular    = msg->angular_velocity;
            twist.twist.covariance[21]   = msg->angular_velocity_covariance[0];
            twist.twist.covariance[28]   = msg->angular_velocity_covariance[4];
            twist.twist.covariance[35]   = msg->angular_velocity_covariance[8];
        }

        pub_twist_->publish(twist);
    }

    rclcpp::TimerBase::SharedPtr                   init_timer_;
    std::shared_ptr<hatchbed_common::ParamHandler> params_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr                       sub_imu_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_twist_;

    bool            differential_ = false;

    // Differential mode state.
    tf2::Quaternion prev_q_;
    rclcpp::Time    prev_stamp_;
    bool            has_prev_ = false;
};

}  // namespace localization
}  // namespace hatchbed_common

RCLCPP_COMPONENTS_REGISTER_NODE(hatchbed_common::localization::ImuToTwist)
