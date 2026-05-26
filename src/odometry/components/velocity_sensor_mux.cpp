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

#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <hatchbed_common/param_handler.h>
#include <hatchbed_common/ros_names.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace hatchbed_common {
namespace odometry {

static constexpr int    kNumAxes        = 6;
static constexpr double kHighCovariance = 1e9;

static constexpr int kLX = 0, kLY = 1, kLZ = 2;
static constexpr int kAX = 3, kAY = 4, kAZ = 5;

static const std::array<const char*, kNumAxes> kAxisNames = {
    "lx", "ly", "lz", "ax", "ay", "az"
};

static int axisFromName(const std::string& name) {
    for (int i = 0; i < kNumAxes; ++i) {
        if (name == kAxisNames[i]) { return i; }
    }
    return -1;
}


/**
 * Multiplexes one or more twist-like sensor topics into a single
 * geometry_msgs/TwistWithCovarianceStamped output.
 *
 * Supported input types: imu, twist, twist_with_cov, vector3, odometry.
 *
 * For each output axis, the non-timed-out input with the lowest diagonal
 * covariance is selected.  An input is considered timed out when no message
 * has arrived within 2 / expected_rate seconds.  Each input declares exactly
 * which axes it provides via its 'axes' parameter.  If no active input covers
 * an axis the output value is 0 and covariance is kHighCovariance (1e9).
 *
 * Per-input configuration parameters (for each name N in 'inputs'):
 *   N.topic          (string)        -- topic to subscribe to
 *   N.type           (string)        -- imu | twist | twist_with_cov |
 *                                       vector3 | odometry
 *   N.expected_rate  (double, Hz)    -- timeout = 2 / expected_rate
 *   N.axes           (string[])      -- ordered list of output axes this
 *                                       input provides (lx ly lz ax ay az).
 *                                       Required; no default.  For vector3:
 *                                       axes[i] is the output axis for vector
 *                                       component i.  For other types: axes[i]
 *                                       selects which message field populates
 *                                       that axis.
 *   N.cov_override   (double[])      -- per-axis covariance overrides in the
 *                                       same order as N.axes.  cov_override[i]
 *                                       overrides the covariance for axes[i]
 *                                       when > 0; 0 means use message value.
 *                                       May be shorter than axes; remaining
 *                                       axes are not overridden.
 */
class VelocitySensorMux : public rclcpp::Node {
public:
    explicit VelocitySensorMux(const rclcpp::NodeOptions& options)
    : Node("velocity_sensor_mux", options)
    {
        init_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&VelocitySensorMux::onInit, this));
    }

private:
    struct AxisSample {
        double       value      = 0.0;
        double       covariance = kHighCovariance;
        rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
        bool         valid      = false;
    };

    struct InputEntry {
        std::string name;
        std::string type;
        double      expected_rate = 10.0;

        // Ordered list of output axes this input provides.  For vector3:
        // axes[i] is the output axis for message component i.
        std::vector<int> axes;

        // enabled_axes[a] is true when axis a receives updates from this input.
        std::array<bool,   kNumAxes> enabled_axes{};

        // Per-axis covariance override indexed by axis enum; 0 => use message cov.
        std::array<double, kNumAxes> cov_override{};

        std::array<AxisSample, kNumAxes> samples{};

        rclcpp::SubscriptionBase::SharedPtr sub;
    };

    // -------------------------------------------------------------------------
    // Initialization
    // -------------------------------------------------------------------------

    void onInit() {
        init_timer_->cancel();

        params_ = std::make_shared<hatchbed_common::ParamHandler>(shared_from_this());
        params_->register_verbose_logging_param();

        params_->param(&output_frame_, "output_frame", std::string("base_link"),
            "Frame ID for the output TwistWithCovarianceStamped header.").declare();

        params_->param(&output_rate_, "output_rate", 50.0,
            "Rate (Hz) at which the output topic is published.").min(1.0).max(1000.0).declare();

        std::vector<std::string> input_names;
        params_->param(&input_names, "inputs", std::vector<std::string>{},
            "Ordered list of input identifiers.  For each name N declare: "
            "N.topic, N.type, N.expected_rate, N.axes, N.cov_override.").declare();

        std::unordered_set<std::string> seen;
        for (const auto& name : input_names) {
            if (name.empty()) {
                RCLCPP_WARN(get_logger(), "inputs: empty name; skipping.");
                continue;
            }
            if (!hatchbed_common::isValidParamName(name)) {
                RCLCPP_WARN(get_logger(),
                    "inputs: '%s' is not a valid ROS parameter name component "
                    "(must start with a letter or underscore and contain only "
                    "alphanumeric characters and underscores); skipping.",
                    name.c_str());
                continue;
            }
            if (!seen.insert(name).second) {
                RCLCPP_WARN(get_logger(),
                    "inputs: duplicate name '%s'; skipping.", name.c_str());
                continue;
            }
            parseInput(name);
        }

        pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
            "twist", rclcpp::QoS(10));

        output_timer_ = rclcpp::create_timer(this, get_clock(),
                                             rclcpp::Duration::from_seconds(1.0 / output_rate_),
                                             std::bind(&VelocitySensorMux::onOutputTimer, this));


        RCLCPP_INFO(get_logger(), "VelocitySensorMux ready: %zu inputs, output at %.1f Hz.",
            inputs_.size(), output_rate_);
    }

    void parseInput(const std::string& name) {
        InputEntry entry;
        entry.name = name;
        entry.cov_override.fill(0.0);
        entry.enabled_axes.fill(false);

        const std::string pfx = name + ".";

        const std::string topic = params_->param(
            pfx + "topic", std::string(""),
            "Topic to subscribe to.").declare().value();

        entry.type = params_->param(
            pfx + "type", std::string(""),
            "Input type: imu | twist | twist_with_cov | vector3 | odometry.").declare().value();

        entry.expected_rate = params_->param(
            pfx + "expected_rate", 10.0,
            "Expected publish rate (Hz); timeout = 2 / expected_rate.")
            .min(0.001).declare().value();

        auto axis_names = params_->param(
            pfx + "axes", std::vector<std::string>{},
            "Ordered list of output axes this input provides (lx ly lz ax ay az). "
            "For vector3: axes[i] is the output axis for vector component i.").declare().value();

        const auto cov_vec = params_->param(
            pfx + "cov_override", std::vector<double>{},
            "Per-axis covariance overrides in the same order as axes. "
            "cov_override[i] overrides the covariance for axes[i] when > 0. "
            "May be shorter than axes; remaining axes are not overridden.").declare().value();

        if (topic.empty()) {
            RCLCPP_WARN(get_logger(), "input '%s' missing topic; skipping.", name.c_str());
            return;
        }
        if (entry.type.empty()) {
            RCLCPP_WARN(get_logger(), "input '%s' missing type; skipping.", name.c_str());
            return;
        }

        if (axis_names.empty()) {
            RCLCPP_WARN(get_logger(),
                "input '%s' requires 'axes' to be specified; skipping.", name.c_str());
            return;
        }

        for (const auto& an : axis_names) {
            const int a = axisFromName(an);
            if (a < 0) {
                RCLCPP_WARN(get_logger(),
                    "input '%s': unknown axis '%s'; ignoring.", name.c_str(), an.c_str());
            } else {
                entry.axes.push_back(a);
                entry.enabled_axes[a] = true;
            }
        }

        // cov_override is axes-indexed: cov_override[i] applies to axes[i].
        // May be shorter than axes; elements beyond the provided list are
        // treated as 0 (no override).
        if (cov_vec.size() > entry.axes.size()) {
            RCLCPP_WARN(get_logger(),
                "input '%s': cov_override has %zu elements but axes has %zu; "
                "extra elements ignored.",
                name.c_str(), cov_vec.size(), entry.axes.size());
        }
        for (size_t i = 0; i < cov_vec.size() && i < entry.axes.size(); ++i) {
            entry.cov_override[entry.axes[i]] = cov_vec[i];
        }

        if (!createSubscription(entry, topic)) {
            RCLCPP_WARN(get_logger(), "input '%s': unknown type '%s'; skipping.",
                name.c_str(), entry.type.c_str());
            return;
        }

        RCLCPP_INFO(get_logger(), "Registered input '%s' [%s] on '%s'.",
            name.c_str(), entry.type.c_str(), topic.c_str());
        inputs_.push_back(std::move(entry));
    }

    // Creates the appropriate subscription and stores it in entry.sub.
    // idx is the index this entry will occupy after inputs_.push_back().
    bool createSubscription(InputEntry& entry, const std::string& topic) {
        const size_t idx = inputs_.size();

        if (entry.type == "imu") {
            entry.sub = create_subscription<sensor_msgs::msg::Imu>(
                topic, rclcpp::QoS(10),
                [this, idx](const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
                    onImu(idx, msg);
                });
            return true;
        }
        if (entry.type == "twist") {
            entry.sub = create_subscription<geometry_msgs::msg::TwistStamped>(
                topic, rclcpp::QoS(10),
                [this, idx](const geometry_msgs::msg::TwistStamped::ConstSharedPtr& msg) {
                    onTwist(idx, msg);
                });
            return true;
        }
        if (entry.type == "twist_with_cov") {
            using MsgT = geometry_msgs::msg::TwistWithCovarianceStamped;
            entry.sub = create_subscription<MsgT>(
                topic, rclcpp::QoS(10),
                [this, idx](const MsgT::ConstSharedPtr& msg) {
                    onTwistWithCov(idx, msg);
                });
            return true;
        }
        if (entry.type == "vector3") {
            entry.sub = create_subscription<geometry_msgs::msg::Vector3Stamped>(
                topic, rclcpp::QoS(10),
                [this, idx](const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr& msg) {
                    onVector3(idx, msg);
                });
            return true;
        }
        if (entry.type == "odometry") {
            entry.sub = create_subscription<nav_msgs::msg::Odometry>(
                topic, rclcpp::QoS(10),
                [this, idx](const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
                    onOdometry(idx, msg);
                });
            return true;
        }
        return false;
    }

    // -------------------------------------------------------------------------
    // Input callbacks
    // -------------------------------------------------------------------------

    void onImu(size_t idx, const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
        auto& entry = inputs_[idx];
        const rclcpp::Time stamp = msg->header.stamp;

        // Natural mapping: angular_velocity.{x,y,z} -> {AX, AY, AZ}.
        // axes[] selects which of those to expose and in what order.
        const std::array<double, kNumAxes> all_vals = {
            0.0, 0.0, 0.0,
            msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z
        };
        const std::array<double, kNumAxes> all_cov = {
            -1.0, -1.0, -1.0,
            msg->angular_velocity_covariance[0],
            msg->angular_velocity_covariance[4],
            msg->angular_velocity_covariance[8]
        };

        for (const int a : entry.axes) {
            auto& s = entry.samples[a];
            s.value      = all_vals[a];
            s.covariance = resolveCovariance(entry, a, all_cov[a]);
            s.stamp      = stamp;
            s.valid      = true;
        }
    }

    void onTwist(
        size_t idx, const geometry_msgs::msg::TwistStamped::ConstSharedPtr& msg) {
        auto& entry = inputs_[idx];
        const rclcpp::Time stamp = msg->header.stamp;
        const std::array<double, kNumAxes> vals = {
            msg->twist.linear.x,  msg->twist.linear.y,  msg->twist.linear.z,
            msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z
        };
        for (const int a : entry.axes) {
            auto& s = entry.samples[a];
            s.value      = vals[a];
            s.covariance = resolveCovariance(entry, a, -1.0);  // no msg covariance
            s.stamp      = stamp;
            s.valid      = true;
        }
    }

    void onTwistWithCov(
        size_t idx,
        const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr& msg) {
        auto& entry = inputs_[idx];
        const rclcpp::Time stamp = msg->header.stamp;
        const std::array<double, kNumAxes> vals = {
            msg->twist.twist.linear.x,  msg->twist.twist.linear.y,
            msg->twist.twist.linear.z,  msg->twist.twist.angular.x,
            msg->twist.twist.angular.y, msg->twist.twist.angular.z
        };
        for (const int a : entry.axes) {
            auto& s = entry.samples[a];
            s.value      = vals[a];
            s.covariance = resolveCovariance(entry, a, msg->twist.covariance[a * 6 + a]);
            s.stamp      = stamp;
            s.valid      = true;
        }
    }

    void onVector3(
        size_t idx, const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr& msg) {
        auto& entry = inputs_[idx];
        const rclcpp::Time stamp = msg->header.stamp;
        const std::array<double, 3> components = {
            msg->vector.x, msg->vector.y, msg->vector.z
        };
        // axes[i] is the output axis for component i; extra components ignored.
        for (size_t i = 0; i < entry.axes.size() && i < 3; ++i) {
            const int a = entry.axes[i];
            auto& s = entry.samples[a];
            s.value      = components[i];
            s.covariance = resolveCovariance(entry, a, -1.0);  // no msg covariance
            s.stamp      = stamp;
            s.valid      = true;
        }
    }

    void onOdometry(size_t idx, const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
        auto& entry = inputs_[idx];
        const rclcpp::Time stamp = msg->header.stamp;
        const std::array<double, kNumAxes> vals = {
            msg->twist.twist.linear.x,  msg->twist.twist.linear.y,
            msg->twist.twist.linear.z,  msg->twist.twist.angular.x,
            msg->twist.twist.angular.y, msg->twist.twist.angular.z
        };
        for (const int a : entry.axes) {
            auto& s = entry.samples[a];
            s.value      = vals[a];
            s.covariance = resolveCovariance(entry, a, msg->twist.covariance[a * 6 + a]);
            s.stamp      = stamp;
            s.valid      = true;
        }
    }

    // -------------------------------------------------------------------------
    // Output
    // -------------------------------------------------------------------------

    void onOutputTimer() {
        const rclcpp::Time now = get_clock()->now();

        geometry_msgs::msg::TwistWithCovarianceStamped out;
        out.header.stamp    = now;
        out.header.frame_id = output_frame_;

        for (int a = 0; a < kNumAxes; ++a) {
            double best_cov = kHighCovariance;
            double best_val = 0.0;

            for (const auto& input : inputs_) {
                if (!input.enabled_axes[a]) { continue; }
                const auto& s = input.samples[a];
                if (!s.valid) { continue; }
                const double age     = (now - s.stamp).seconds();
                const double timeout = 2.0 / input.expected_rate;
                if (age > timeout) { continue; }
                if (s.covariance < best_cov) {
                    best_cov = s.covariance;
                    best_val = s.value;
                }
            }

            setAxisValue(out, a, best_val);
            out.twist.covariance[a * 6 + a] = best_cov;
        }

        pub_->publish(out);
    }

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------

    // Returns the effective covariance for axis a.
    // msg_cov: the diagonal element from the message; pass <= 0 if unavailable.
    static double resolveCovariance(const InputEntry& entry, int a, double msg_cov) {
        if (entry.cov_override[a] > 0.0) { return entry.cov_override[a]; }
        if (msg_cov > 0.0)               { return msg_cov; }
        return kHighCovariance;
    }

    static void setAxisValue(
        geometry_msgs::msg::TwistWithCovarianceStamped& msg, int axis, double val) {
        switch (axis) {
            case kLX: msg.twist.twist.linear.x  = val; break;
            case kLY: msg.twist.twist.linear.y  = val; break;
            case kLZ: msg.twist.twist.linear.z  = val; break;
            case kAX: msg.twist.twist.angular.x = val; break;
            case kAY: msg.twist.twist.angular.y = val; break;
            case kAZ: msg.twist.twist.angular.z = val; break;
            default: break;
        }
    }

    // -------------------------------------------------------------------------
    // Members
    // -------------------------------------------------------------------------

    rclcpp::TimerBase::SharedPtr init_timer_;
    rclcpp::TimerBase::SharedPtr output_timer_;
    std::shared_ptr<hatchbed_common::ParamHandler> params_;

    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_;

    std::vector<InputEntry> inputs_;

    std::string output_frame_ = "base_link";
    double      output_rate_  = 50.0;
};

}  // namespace odometry
}  // namespace hatchbed_common

RCLCPP_COMPONENTS_REGISTER_NODE(hatchbed_common::odometry::VelocitySensorMux)
