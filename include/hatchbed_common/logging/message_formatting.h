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

#pragma once

#include <fmt/core.h>
#include <fmt/format.h>

#include <hatchbed_common/logging/formatting.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>



// ============================================================================
// GEOMETRY & TF2 PRIMITIVES (Using Macros to reduce boilerplate)
// ============================================================================

#define HB_FMT_MSG_XYZ(Type) \
    template <> struct formatter<Type> : hatchbed_common::logging::FormatterDelegator<double> { \
        template <typename FormatContext> \
        auto format(const Type& v, FormatContext& ctx) const { \
            if (!has_custom_spec) return fmt::format_to(ctx.out(), "[{:.3f}, {:.3f}, {:.3f}]", \
                                                        v.x, v.y, v.z); \
            auto out = fmt::format_to(ctx.out(), "["); \
            ctx.advance_to(out); \
            out = format_elements(ctx, ", ", v.x, v.y, v.z); \
            return fmt::format_to(out, "]"); \
        } \
    }

#define HB_FMT_MSG_QUAT(Type) \
    template <> struct formatter<Type> : hatchbed_common::logging::FormatterDelegator<double> { \
        template <typename FormatContext> \
        auto format(const Type& q, FormatContext& ctx) const { \
            if (!has_custom_spec) { \
                return fmt::format_to(ctx.out(), "[{:.3f}, {:.3f}, {:.3f}, {:.3f}]", \
                                      q.x, q.y, q.z, q.w); \
            } \
            auto out = fmt::format_to(ctx.out(), "["); \
            ctx.advance_to(out); \
            out = format_elements(ctx, ", ", q.x, q.y, q.z, q.w); \
            return fmt::format_to(out, "]"); \
        } \
    }

namespace fmt {

HB_FMT_MSG_XYZ(geometry_msgs::msg::Point);
HB_FMT_MSG_XYZ(geometry_msgs::msg::Vector3);
HB_FMT_MSG_QUAT(geometry_msgs::msg::Quaternion);

template <> struct formatter<geometry_msgs::msg::Pose>
  : hatchbed_common::logging::FormatterDelegator<double>
{
    template <typename FormatContext>
    auto format(const geometry_msgs::msg::Pose& p, FormatContext& ctx) const {
        if (!has_custom_spec) return fmt::format_to(ctx.out(), "Pose(p: {}, q: {})", p.position,
                                                    p.orientation);

        auto out = fmt::format_to(ctx.out(), "Pose(p: [");
        ctx.advance_to(out);
        out = format_elements(ctx, ", ", p.position.x, p.position.y, p.position.z);
        out = fmt::format_to(out, "], q: [");
        ctx.advance_to(out);
        out = format_elements(ctx, ", ", p.orientation.x, p.orientation.y, p.orientation.z,
                              p.orientation.w);
        return fmt::format_to(out, "])");
    }
};

template <> struct formatter<geometry_msgs::msg::PoseStamped>
  : hatchbed_common::logging::FormatterDelegator<double>
{
    template <typename FormatContext>
    auto format(const geometry_msgs::msg::PoseStamped& p, FormatContext& ctx) const {
        if (!has_custom_spec) return fmt::format_to(ctx.out(), "Pose({} p: {}, q: {})", p.header,
                                                    p.pose.position, p.pose.orientation);
        auto out = fmt::format_to(ctx.out(), "Pose({} p: [", p.header);
        ctx.advance_to(out);
        out = format_elements(ctx, ", ", p.pose.position.x, p.pose.position.y, p.pose.position.z);
        out = fmt::format_to(out, "], q: [");
        ctx.advance_to(out);
        out = format_elements(ctx, ", ", p.pose.orientation.x, p.pose.orientation.y,
                              p.pose.orientation.z, p.pose.orientation.w);
        return fmt::format_to(out, "])");
    }
};

template <> struct formatter<geometry_msgs::msg::PoseWithCovariance>
  : hatchbed_common::logging::FormatterDelegator<double>
{
    template <typename FormatContext>
    auto format(const geometry_msgs::msg::PoseWithCovariance& p, FormatContext& ctx) const {
        if (!has_custom_spec) return fmt::format_to(ctx.out(),
                                                    "Pose(p: {}, q: {}, cov:[36 elements])",
                                                    p.pose.position, p.pose.orientation);
        auto out = fmt::format_to(ctx.out(), "Pose(p: [");
        ctx.advance_to(out);
        out = format_elements(ctx, ", ", p.pose.position.x, p.pose.position.y, p.pose.position.z);
        out = fmt::format_to(out, "], q: [");
        ctx.advance_to(out);
        out = format_elements(ctx, ", ", p.pose.orientation.x, p.pose.orientation.y,
                              p.pose.orientation.z, p.pose.orientation.w);
        return fmt::format_to(out, "], cov: [36 elements])");
    }
};

template <> struct formatter<geometry_msgs::msg::PoseWithCovarianceStamped>
  : hatchbed_common::logging::FormatterDelegator<double>
{
    template <typename FormatContext>
    auto format(const geometry_msgs::msg::PoseWithCovarianceStamped& p, FormatContext& ctx) const {
        if (!has_custom_spec) {
            return fmt::format_to(ctx.out(), "Pose({} p: {}, q: {}, cov: [36 elements])",
                                              p.header, p.pose.pose.position,
                                              p.pose.pose.orientation);
        }
        auto out = fmt::format_to(ctx.out(), "Pose({} p: [", p.header);
        ctx.advance_to(out);
        out = format_elements(ctx, ", ", p.pose.pose.position.x, p.pose.pose.position.y,
                              p.pose.pose.position.z);
        out = fmt::format_to(out, "], q: [");
        ctx.advance_to(out);
        out = format_elements(ctx, ", ", p.pose.pose.orientation.x, p.pose.pose.orientation.y,
                              p.pose.pose.orientation.z, p.pose.pose.orientation.w);
        return fmt::format_to(out, "], cov: [36 elements])");
    }
};

template <> struct formatter<geometry_msgs::msg::Transform>
  : hatchbed_common::logging::FormatterDelegator<double>
{
    template <typename FormatContext>
    auto format(const geometry_msgs::msg::Transform& t, FormatContext& ctx) const {
        if (!has_custom_spec) return fmt::format_to(ctx.out(), "Transform(t: {}, r: {})",
                                                    t.translation, t.rotation);
        auto out = fmt::format_to(ctx.out(), "Transform(t: [");
        ctx.advance_to(out);
        out = format_elements(ctx, ", ", t.translation.x, t.translation.y, t.translation.z);
        out = fmt::format_to(out, "], r: [");
        ctx.advance_to(out);
        out = format_elements(ctx, ", ", t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w);
        return fmt::format_to(out, "])");
    }
};

template <> struct formatter<geometry_msgs::msg::TransformStamped>
  : hatchbed_common::logging::FormatterDelegator<double>
{
    template <typename FormatContext>
    auto format(const geometry_msgs::msg::TransformStamped& t, FormatContext& ctx) const {
        if (!has_custom_spec) {
            return fmt::format_to(ctx.out(), "Transform({} child: '{}', t: {}, r: {})",
                                  t.header, t.child_frame_id, t.transform.translation,
                                  t.transform.rotation);
        }
        auto out = fmt::format_to(ctx.out(), "Transform({} child: '{}', t: [", t.header,
                                  t.child_frame_id);
        ctx.advance_to(out);
        out = format_elements(ctx, ", ", t.transform.translation.x, t.transform.translation.y,
                              t.transform.translation.z);
        out = fmt::format_to(out, "], r: [");
        ctx.advance_to(out);
        out = format_elements(ctx, ", ", t.transform.rotation.x, t.transform.rotation.y,
                              t.transform.rotation.z, t.transform.rotation.w);
        return fmt::format_to(out, "])");
    }
};

template <> struct formatter<geometry_msgs::msg::Twist>
  : hatchbed_common::logging::FormatterDelegator<double>
{
    template <typename FormatContext>
    auto format(const geometry_msgs::msg::Twist& t, FormatContext& ctx) const {
        if (!has_custom_spec) {
            return fmt::format_to(ctx.out(), "Twist(v: {}, w: {})", t.linear, t.angular);
        }

        auto out = fmt::format_to(ctx.out(), "Twist(v: [");
        ctx.advance_to(out);
        out = format_elements(ctx, ", ", t.linear.x, t.linear.y, t.linear.z);
        out = fmt::format_to(out, "], w: [");
        ctx.advance_to(out);
        out = format_elements(ctx, ", ", t.angular.x, t.angular.y, t.angular.z);
        return fmt::format_to(out, "])");
    }
};

template <> struct formatter<geometry_msgs::msg::TwistStamped>
  : hatchbed_common::logging::FormatterDelegator<double>
{
    template <typename FormatContext>
    auto format(const geometry_msgs::msg::TwistStamped& t, FormatContext& ctx) const {
        if (!has_custom_spec) {
            return fmt::format_to(ctx.out(), "Twist({} v: {}, w: {})",
                                  t.header, t.twist.linear, t.twist.angular);
        }
        auto out = fmt::format_to(ctx.out(), "Twist({} v: [", t.header);
        ctx.advance_to(out);
        out = format_elements(ctx, ", ", t.twist.linear.x, t.twist.linear.y, t.twist.linear.z);
        out = fmt::format_to(out, "], w: [");
        ctx.advance_to(out);
        out = format_elements(ctx, ", ", t.twist.angular.x, t.twist.angular.y, t.twist.angular.z);
        return fmt::format_to(out, "])");
    }
};

template <> struct formatter<geometry_msgs::msg::TwistWithCovariance>
  : hatchbed_common::logging::FormatterDelegator<double>
{
    template <typename FormatContext>
    auto format(const geometry_msgs::msg::TwistWithCovariance& t, FormatContext& ctx) const {
        if (!has_custom_spec) {
            return fmt::format_to(ctx.out(), "Twist(v: {}, w: {}, cov: [36 elements])",
                                  t.twist.linear, t.twist.angular);
        }
        auto out = fmt::format_to(ctx.out(), "Twist(v: [");
        ctx.advance_to(out);
        out = format_elements(ctx, ", ", t.twist.linear.x, t.twist.linear.y, t.twist.linear.z);
        out = fmt::format_to(out, "], w: [");
        ctx.advance_to(out);
        out = format_elements(ctx, ", ", t.twist.angular.x, t.twist.angular.y, t.twist.angular.z);
        return fmt::format_to(out, "], cov: [36 elements])");
    }
};

template <> struct formatter<geometry_msgs::msg::TwistWithCovarianceStamped>
  : hatchbed_common::logging::FormatterDelegator<double>
{
    template <typename FormatContext>
    auto format(const geometry_msgs::msg::TwistWithCovarianceStamped& t, FormatContext& ctx) const {
        if (!has_custom_spec) {
            return fmt::format_to(ctx.out(), "Twist({} v: {}, w: {}, cov: [36 elements])",
                                  t.header, t.twist.twist.linear, t.twist.twist.angular);
        }
        auto out = fmt::format_to(ctx.out(), "Twist({} v: [", t.header);
        ctx.advance_to(out);
        out = format_elements(ctx, ", ", t.twist.twist.linear.x, t.twist.twist.linear.y,
                                         t.twist.twist.linear.z);
        out = fmt::format_to(out, "], w: [");
        ctx.advance_to(out);
        out = format_elements(ctx, ", ", t.twist.twist.angular.x, t.twist.twist.angular.y,
                                         t.twist.twist.angular.z);
        return fmt::format_to(out, "], cov: [36 elements])");
    }
};

// ============================================================================
// LARGE SENSOR & NAV MESSAGES (Summary Formatters)
// ============================================================================

template <> struct formatter<tf2_msgs::msg::TFMessage> {
    constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }
    template <typename FormatContext>
    auto format(const tf2_msgs::msg::TFMessage& msg, FormatContext& ctx) const {
        return format_to(ctx.out(), "TFMessage({} transforms)", msg.transforms.size());
    }
};

template <> struct formatter<sensor_msgs::msg::Image> {
    constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }
    template <typename FormatContext>
    auto format(const sensor_msgs::msg::Image& msg, FormatContext& ctx) const {
        return format_to(ctx.out(), "Image({}, {}x{}, enc: '{}')",
            msg.header, msg.width, msg.height, msg.encoding);
    }
};

template <> struct formatter<sensor_msgs::msg::PointCloud2> {
    constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }
    template <typename FormatContext>
    auto format(const sensor_msgs::msg::PointCloud2& msg, FormatContext& ctx) const {
        return format_to(ctx.out(), "PointCloud2({}, {}x{}, is_dense: {})",
            msg.header, msg.width, msg.height, msg.is_dense);
    }
};

template <> struct formatter<sensor_msgs::msg::LaserScan> {
    constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }
    template <typename FormatContext>
    auto format(const sensor_msgs::msg::LaserScan& msg, FormatContext& ctx) const {
        return format_to(ctx.out(), "LaserScan({}, pts: {}, range: [{:.2f}, {:.2f}])",
            msg.header, msg.ranges.size(), msg.range_min, msg.range_max);
    }
};

template <> struct formatter<sensor_msgs::msg::Imu> {
    constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }
    template <typename FormatContext>
    auto format(const sensor_msgs::msg::Imu& msg, FormatContext& ctx) const {
        return format_to(ctx.out(), "Imu({} a: {}, w: {})",
            msg.header, msg.linear_acceleration, msg.angular_velocity);
    }
};

template <> struct formatter<nav_msgs::msg::Odometry>
  : hatchbed_common::logging::FormatterDelegator<double>
{
    template <typename FormatContext>
    auto format(const nav_msgs::msg::Odometry& msg, FormatContext& ctx) const {
        if (!has_custom_spec) {
            return fmt::format_to(ctx.out(),
                "\nOdometry({}, child: '{}',\n    {},\n    {})\n",
                msg.header, msg.child_frame_id, msg.pose, msg.twist);
        }

        // Custom specifier path (e.g. {:.2f})
        auto out = fmt::format_to(ctx.out(), "\nOdometry({}, child: '{}',",
                                  msg.header, msg.child_frame_id);

        // --- Pose Section ---
        out = fmt::format_to(out, "\n    Pose(p:[");
        ctx.advance_to(out);
        out = format_elements(ctx, ", ", msg.pose.pose.position.x, msg.pose.pose.position.y,
                              msg.pose.pose.position.z);
        out = fmt::format_to(out, "], q:[");
        ctx.advance_to(out);
        out = format_elements(ctx, ", ", msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                              msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
        out = fmt::format_to(out, "], cov: [36 elements]),");

        // --- Twist Section ---
        out = fmt::format_to(out, "\n    Twist(v:[");
        ctx.advance_to(out);
        out = format_elements(ctx, ", ", msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                              msg.twist.twist.linear.z);
        out = fmt::format_to(out, "], w:[");
        ctx.advance_to(out);
        out = format_elements(ctx, ", ", msg.twist.twist.angular.x, msg.twist.twist.angular.y,
                              msg.twist.twist.angular.z);

        return fmt::format_to(out, "], cov: [36 elements]))\n");
    }
};

template <> struct formatter<nav_msgs::msg::Path> {
    constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }
    template <typename FormatContext>
    auto format(const nav_msgs::msg::Path& msg, FormatContext& ctx) const {
        return format_to(ctx.out(), "Path({} poses: {})",
            msg.header, msg.poses.size());
    }
};

// ============================================================================
// VISUALIZATION MESSAGES
// ============================================================================

template <>
struct formatter<visualization_msgs::msg::Marker>
  : hatchbed_common::logging::FormatterDelegator<double>
{
    template <typename FormatContext>
    auto format(const visualization_msgs::msg::Marker& msg, FormatContext& ctx) const {
        auto get_type = [](int32_t t) {
            switch(t) {
                case 0: return "ARROW"; case 1: return "CUBE"; case 2: return "SPHERE";
                case 3: return "CYLINDER"; case 4: return "LINE_STRIP"; case 5: return "LINE_LIST";
                case 6: return "CUBE_LIST"; case 7: return "SPHERE_LIST"; case 8: return "POINTS";
                case 9: return "TEXT"; case 10: return "MESH"; case 11: return "TRI_LIST";
                default: return "UNKNOWN";
            }
        };
        auto get_action = [](int32_t a) {
            switch(a) {
                case 0: return "ADD/MOD"; case 2: return "DELETE";
                case 3: return "DELETEALL"; default: return "UNKNOWN";
            }
        };

        // --- Path 1: Default Formatting {} ---
        if (!has_custom_spec) {
            auto out = fmt::format_to(ctx.out(),
                "\nMarker({}, ns: '{}', id: {},\n"
                "    type: {}, action: {},\n"
                "    pose: p: {}, q: {},\n"
                "    scale: {},\n"
                "    color: {},\n"
                "    lifetime: {}s, locked: {}",
                msg.header, msg.ns, msg.id, get_type(msg.type), get_action(msg.action),
                msg.pose.position, msg.pose.orientation, msg.scale, msg.color, msg.lifetime,
                (msg.frame_locked ? "true" : "false"));

            if (!msg.text.empty()) {
                out = fmt::format_to(out, ",\n    text: '{}'", msg.text);
            }
            if (!msg.mesh_resource.empty()) {
                out = fmt::format_to(out, ",\n    mesh: '{}'", msg.mesh_resource);
            }

            return fmt::format_to(out, "\n)");
        }

        // --- Path 2: Custom Specifier Formatting (e.g. {:.2f}) ---
        auto out = fmt::format_to(ctx.out(),
                                  "\nMarker({}, ns: '{}', id: {},\n    type: {}, action: {},",
                                  msg.header, msg.ns, msg.id, get_type(msg.type),
                                  get_action(msg.action));

        // Delegate Pose (Position & Orientation)
        out = fmt::format_to(out, "\n    p:[");
        ctx.advance_to(out);
        out = format_elements(ctx, ", ", msg.pose.position.x, msg.pose.position.y,
                              msg.pose.position.z);
        out = fmt::format_to(out, "], q:[");
        ctx.advance_to(out);
        out = format_elements(ctx, ", ", msg.pose.orientation.x, msg.pose.orientation.y,
                              msg.pose.orientation.z, msg.pose.orientation.w);

        // Delegate Scale
        out = fmt::format_to(out, "],\n    scale:[");
        ctx.advance_to(out);
        out = format_elements(ctx, ", ", msg.scale.x, msg.scale.y, msg.scale.z);

        // Delegate Color
        out = fmt::format_to(out, "],\n    color:[");
        ctx.advance_to(out);
        out = format_elements(ctx, ", ", msg.color.r, msg.color.g, msg.color.b, msg.color.a);

        // Delegate Lifetime (convert Duration to double first)
        out = fmt::format_to(out, "],\n    lifetime: ");
        double total_sec = static_cast<double>(msg.lifetime.sec) +
                           (static_cast<double>(msg.lifetime.nanosec) / 1e9);
        ctx.advance_to(out);
        out = underlying.format(total_sec, ctx);

        out = fmt::format_to(out, "s, locked: {}", (msg.frame_locked ? "true" : "false"));

        // Optional fields
        if (!msg.text.empty()) {
            out = fmt::format_to(out, ",\n    text: '{}'", msg.text);
        }
        if (!msg.mesh_resource.empty()) {
            out = fmt::format_to(out, ",\n    mesh: '{}'", msg.mesh_resource);
        }

        return fmt::format_to(out, "\n)");
    }
};

template <> struct formatter<visualization_msgs::msg::MarkerArray> {
    constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }
    template <typename FormatContext>
    auto format(const visualization_msgs::msg::MarkerArray& msg, FormatContext& ctx) const {
        return format_to(ctx.out(), "MarkerArray({} markers)", msg.markers.size());
    }
};

}  // namespace fmt
