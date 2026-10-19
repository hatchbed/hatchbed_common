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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <hatchbed_common/param_handler.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace hatchbed_common {
namespace odometry {

// Diagonal indices into the 6x6 row-major twist covariance.
constexpr int kCovLx = 0;   // linear x  (0, 0)
constexpr int kCovLy = 7;   // linear y  (1, 1)
constexpr int kCovLz = 14;  // linear z  (2, 2)
constexpr int kCovAx = 21;  // angular x (3, 3)
constexpr int kCovAy = 28;  // angular y (4, 4)
constexpr int kCovAz = 35;  // angular z (5, 5)

/**
 * Converts wheel (and steering) joint states from a sensor_msgs/JointState into a
 * body-frame geometry_msgs/TwistWithCovarianceStamped.  Only forward velocity (linear x)
 * and yaw rate (angular z) are observed; all other axes are marked unmeasured with a large
 * variance so a downstream fuser (e.g. hatchbed_common::odometry::VelocitySensorMux) ignores
 * them.
 *
 * Two kinematic models are supported via the "model" parameter:
 *
 *   differential:
 *     v     = wheel_radius * (w_left + w_right) / 2
 *     yaw   = wheel_radius * (w_right - w_left) / track
 *
 *   ackermann:
 *     v     = wheel_radius * (w_left + w_right) / 2   (the two driven wheels)
 *     yaw   = v * tan(steering_angle) / wheelbase     (bicycle model)
 *
 * The wheel joints are assumed to follow the ROS convention that a positive joint velocity
 * corresponds to forward motion, and that the left/right wheels share a common spin axis.
 * The twist is reported at the wheel-odometry reference point (the axle midpoint for
 * differential, the rear axle for ackermann); mount the output frame there or account for
 * the lever arm downstream.
 */
class JointStatesToTwist : public rclcpp::Node {
public:
    explicit JointStatesToTwist(const rclcpp::NodeOptions& options)
    : Node("joint_states_to_twist", options)
    {
        init_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&JointStatesToTwist::onInit, this));
    }

private:
    void onInit() {
        init_timer_->cancel();

        params_ = std::make_shared<hatchbed_common::ParamHandler>(shared_from_this());
        params_->register_verbose_logging_param();

        params_->param(&frame_id_, "frame_id", std::string("base_link"),
            "Frame ID for the output twist header.").declare();

        params_->param(&model_, "model", std::string("differential"),
            "Kinematic model used to derive the twist: 'differential' (yaw from the "
            "left/right wheel speed difference) or 'ackermann' (yaw from the steering "
            "angle via the bicycle model).").declare();

        params_->param(&left_wheel_joint_, "left_wheel_joint", std::string(""),
            "Name of the left drive-wheel joint (a driven rear wheel for ackermann). "
            "Its velocity is read from the JointState.").declare();

        params_->param(&right_wheel_joint_, "right_wheel_joint", std::string(""),
            "Name of the right drive-wheel joint (a driven rear wheel for ackermann). "
            "Its velocity is read from the JointState.").declare();

        params_->param(&steering_joint_, "steering_joint", std::string(""),
            "Name of the steering joint (ackermann only).  Its position is read as the "
            "effective road-wheel steer angle in radians.").declare();

        params_->param(&wheel_radius_, "wheel_radius", 0.1,
            "Drive-wheel radius (m); converts wheel joint velocity (rad/s) to linear "
            "velocity (m/s).").min(1e-6).declare();

        params_->param(&track_, "track", 1.0,
            "Lateral distance between the left and right wheels (m); used for the "
            "differential yaw rate.").min(1e-6).declare();

        params_->param(&wheelbase_, "wheelbase", 1.0,
            "Longitudinal distance between the front and rear axles (m); used for the "
            "ackermann yaw rate.").min(1e-6).declare();

        params_->param(&linear_variance_, "linear_variance", 1e-3,
            "Variance assigned to the linear-x (forward velocity) estimate.")
            .min(0.0).dynamic().declare();

        params_->param(&angular_variance_, "angular_variance", 1e-3,
            "Variance assigned to the angular-z (yaw rate) estimate.")
            .min(0.0).dynamic().declare();

        params_->param(&slip_variance_gain_, "slip_variance_gain", 0.0,
            "Turn-rate-dependent inflation of the forward-velocity (linear-x) variance "
            "to model wheel slip in turns. The reported linear-x variance becomes "
            "linear_variance + (slip_variance_gain * yaw_rate)^2, so the added standard "
            "deviation grows linearly with |yaw_rate| (variance with yaw_rate^2). Solid "
            "tires scrub and over-rotate in a tight turn, so the wheels over-report "
            "forward speed there while straight-line rolling is clean; inflating the "
            "variance in proportion to turn rate tells a downstream fuser to lean on an "
            "absolute source (e.g. GNSS position) more while turning and to trust wheel "
            "dead reckoning on the straightaways. Units: (m/s) per (rad/s), i.e. meters. "
            "Default 0 disables it.").min(0.0).dynamic().declare();

        params_->param(&lateral_variance_, "lateral_variance", 1e9,
            "Variance assigned to the linear-y (lateral velocity) estimate, which is "
            "always reported as 0. Leave large to treat vy as unobserved; set small "
            "(and configure the downstream fuser to use vy) to assert a non-holonomic "
            "no-lateral-slip constraint, which bounds an otherwise unobservable lateral "
            "position drift.").min(0.0).dynamic().declare();

        params_->param(&unmeasured_variance_, "unmeasured_variance", 1e9,
            "Variance assigned to the unobserved axes (linear z, angular x/y) so a "
            "downstream fuser discounts them.").min(0.0).declare();

        ackermann_ = (model_ == "ackermann");
        if (!ackermann_ && model_ != "differential") {
            RCLCPP_WARN(get_logger(),
                "unknown model '%s'; falling back to 'differential'.", model_.c_str());
        }
        if (left_wheel_joint_.empty() || right_wheel_joint_.empty()) {
            RCLCPP_ERROR(get_logger(),
                "left_wheel_joint and right_wheel_joint must both be set; no twist will "
                "be published until they are.");
        }
        if (ackermann_ && steering_joint_.empty()) {
            RCLCPP_ERROR(get_logger(),
                "ackermann model requires steering_joint to be set; no twist will be "
                "published until it is.");
        }

        pub_twist_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
            "twist", rclcpp::QoS(10));

        sub_joints_ = create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", rclcpp::QoS(10),
            std::bind(&JointStatesToTwist::onJointStates, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Converting joint states to twist (%s model).",
            ackermann_ ? "ackermann" : "differential");
    }

    void onJointStates(const sensor_msgs::msg::JointState::SharedPtr msg) {
        double w_left  = 0.0;
        double w_right = 0.0;
        if (!lookupVelocity(*msg, left_wheel_joint_, &w_left) ||
            !lookupVelocity(*msg, right_wheel_joint_, &w_right)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "left/right wheel joint velocities unavailable in joint_states; "
                "dropping message.");
            return;
        }

        const double v = wheel_radius_ * 0.5 * (w_left + w_right);

        double yaw = 0.0;
        if (ackermann_) {
            double steer = 0.0;
            if (!lookupPosition(*msg, steering_joint_, &steer)) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                    "steering joint position unavailable in joint_states; dropping "
                    "message.");
                return;
            }
            yaw = v * std::tan(steer) / wheelbase_;
        } else {
            yaw = wheel_radius_ * (w_right - w_left) / track_;
        }

        geometry_msgs::msg::TwistWithCovarianceStamped twist;
        twist.header.stamp = msg->header.stamp;
        twist.header.frame_id = frame_id_;
        twist.twist.twist.linear.x  = v;
        twist.twist.twist.angular.z = yaw;

        // Inflate the forward-velocity variance in proportion to turn rate to model
        // wheel slip: the wheels scrub and over-rotate in a tight turn, so the reported
        // vx is least trustworthy there (clean on the straightaway). The added standard
        // deviation is slip_variance_gain * |yaw|, so the variance grows with yaw^2.
        const double slip_std = slip_variance_gain_ * std::abs(yaw);
        twist.twist.covariance[kCovLx] = linear_variance_ + slip_std * slip_std;
        twist.twist.covariance[kCovAz] = angular_variance_;
        twist.twist.covariance[kCovLy] = lateral_variance_;
        twist.twist.covariance[kCovLz] = unmeasured_variance_;
        twist.twist.covariance[kCovAx] = unmeasured_variance_;
        twist.twist.covariance[kCovAy] = unmeasured_variance_;

        pub_twist_->publish(twist);
    }

    // Finds joint_name in msg.name and returns its velocity via out.  Returns false if the
    // joint is absent or the velocity array does not cover its index.
    static bool lookupVelocity(const sensor_msgs::msg::JointState& msg,
                               const std::string& joint_name, double* out) {
        const int idx = indexOf(msg.name, joint_name);
        if (idx < 0 || static_cast<size_t>(idx) >= msg.velocity.size()) {
            return false;
        }
        *out = msg.velocity[idx];
        return true;
    }

    // Finds joint_name in msg.name and returns its position via out.  Returns false if the
    // joint is absent or the position array does not cover its index.
    static bool lookupPosition(const sensor_msgs::msg::JointState& msg,
                               const std::string& joint_name, double* out) {
        const int idx = indexOf(msg.name, joint_name);
        if (idx < 0 || static_cast<size_t>(idx) >= msg.position.size()) {
            return false;
        }
        *out = msg.position[idx];
        return true;
    }

    static int indexOf(const std::vector<std::string>& names, const std::string& name) {
        const auto it = std::find(names.begin(), names.end(), name);
        if (it == names.end()) {
            return -1;
        }
        return static_cast<int>(std::distance(names.begin(), it));
    }

    rclcpp::TimerBase::SharedPtr                   init_timer_;
    std::shared_ptr<hatchbed_common::ParamHandler> params_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr                sub_joints_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_twist_;

    std::string frame_id_;
    std::string model_;
    std::string left_wheel_joint_;
    std::string right_wheel_joint_;
    std::string steering_joint_;
    double      wheel_radius_        = 0.1;
    double      track_               = 1.0;
    double      wheelbase_           = 1.0;
    double      linear_variance_     = 1e-3;
    double      angular_variance_    = 1e-3;
    double      slip_variance_gain_  = 0.0;
    double      lateral_variance_    = 1e9;
    double      unmeasured_variance_ = 1e9;
    bool        ackermann_           = false;
};

}  // namespace odometry
}  // namespace hatchbed_common

RCLCPP_COMPONENTS_REGISTER_NODE(hatchbed_common::odometry::JointStatesToTwist)
