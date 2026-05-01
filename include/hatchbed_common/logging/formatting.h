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

#include <filesystem>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>

#include <fmt/chrono.h>
#include <fmt/core.h>
#include <fmt/format.h>
#include <fmt/ranges.h>

// ROS 2 Core
#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/color_rgba.hpp>

// Math & Geometry
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

namespace hatchbed_common {
namespace logging {

/**
 * @brief A reusable base class for formatters that delegate their format
 * specifier (like `:.2f`) to an underlying type.
 */
template <typename T>
struct FormatterDelegator {
    fmt::formatter<T> underlying;
    bool has_custom_spec = false;

    constexpr auto parse(fmt::format_parse_context& ctx) {
        auto it = ctx.begin(), end = ctx.end();
        if (it != end && *it != '}') {
            has_custom_spec = true;
            return underlying.parse(ctx);
        }
        return it;
    }

    /**
     * @brief Helper to cleanly format multiple elements using the parsed specifier.
     * Eliminates repetitive iterator updating.
     */
    template <typename FormatContext, typename... Args>
    auto format_elements(FormatContext& ctx, const char* sep, const Args&... args) const {
        auto out = ctx.out();
        size_t n = 0;

        // C++17 Fold expression with a lambda to safely interleave separators
        auto format_one = [&](const auto& arg) {
            if (n++ != 0) {
                out = fmt::format_to(out, "{}", sep);
            }
            ctx.advance_to(out);
            out = underlying.format(arg, ctx);
        };

        (format_one(args), ...);
        return out;
    }
};

/**
 * @brief Internal helper to format values inside containers.
 * Mimics fmt v10's behavior of quoting strings and chars.
 */
template <typename T, typename Context>
auto format_container_element(const T& val, Context& ctx) {
    using RawT = std::remove_cv_t<std::remove_reference_t<T>>;
    if constexpr (std::is_same_v<RawT, std::string> ||
                  std::is_same_v<RawT, const char*> ||
                  std::is_same_v<RawT, char*>) {
        return fmt::format_to(ctx.out(), "\"{}\"", val);
    } else if constexpr (std::is_same_v<RawT, char>) {
        return fmt::format_to(ctx.out(), "'{}'", val);
    } else {
        return fmt::format_to(ctx.out(), "{}", val);
    }
}

}  // namespace logging
}  // namespace hatchbed_common


// On Jazzy (fmt v10+), just pull in the official standard library formatters
#if FMT_VERSION >= 100000
#include <fmt/std.h>
#else
namespace fmt {

// --- POLYFILL: std::exception ---
template <>
struct formatter<std::exception> {
    constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }
    template <typename FormatContext>
    auto format(const std::exception& ex, FormatContext& ctx) const {
        return format_to(ctx.out(), "{}", ex.what());
    }
};

// --- POLYFILL: std::optional ---
template <typename T>
struct formatter<std::optional<T>> {
    constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }
    template <typename FormatContext>
    auto format(const std::optional<T>& opt, FormatContext& ctx) const {
        if (!opt) {
            return format_to(ctx.out(), "none");
        }
        auto out = format_to(ctx.out(), "optional(");
        ctx.advance_to(out);
        out = hatchbed_common::logging::format_container_element(*opt, ctx);
        return format_to(out, ")");
    }
};

// NOTE: std::tuple is handled by fmt/ranges.h (is_tuple_like) on fmt v9, so no polyfill needed.

// --- POLYFILL: std::variant ---
template <typename... T>
struct formatter<std::variant<T...>> {
    constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }
    template <typename FormatContext>
    auto format(const std::variant<T...>& v, FormatContext& ctx) const {
        auto out = format_to(ctx.out(), "variant(");
        ctx.advance_to(out);
        out = std::visit([&](const auto& val) {
            return hatchbed_common::logging::format_container_element(val, ctx);
        }, v);
        return format_to(out, ")");
    }
};


// --- POLYFILL: std::filesystem::path ---
template <>
struct formatter<std::filesystem::path> {
    constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }
    template <typename FormatContext>
    auto format(const std::filesystem::path& p, FormatContext& ctx) const {
        // v10 formats paths as quoted strings
        return format_to(ctx.out(), "\"{}\"", p.string());
    }
};

}  // namespace fmt
#endif

namespace fmt {

// ============================================================================
// RCLCPP CORE TYPES (Time, Duration)
// ============================================================================

template <> struct formatter<rclcpp::Time>
    : hatchbed_common::logging::FormatterDelegator<double>
{
    template <typename FormatContext>
    auto format(const rclcpp::Time& t, FormatContext& ctx) const {
        int64_t ns = t.nanoseconds();
        if (!has_custom_spec) {
            int64_t sec = ns / 1000000000LL;
            int64_t frac = std::abs(ns % 1000000000LL);
            if (ns < 0 && sec == 0) {
                return format_to(ctx.out(), "-0.{:09d}", frac);
            }
            return format_to(ctx.out(), "{}.{:09d}", sec, frac);
        }
        return underlying.format(static_cast<double>(ns) / 1e9, ctx);
    }
};

template <> struct formatter<rclcpp::Duration>
    : hatchbed_common::logging::FormatterDelegator<double>
{
    template <typename FormatContext>
    auto format(const rclcpp::Duration& d, FormatContext& ctx) const {
        int64_t ns = d.nanoseconds();
        if (!has_custom_spec) {
            int64_t sec = ns / 1000000000LL;
            int64_t frac = std::abs(ns % 1000000000LL);
            if (ns < 0 && sec == 0) {
                return format_to(ctx.out(), "-0.{:09d}", frac);
            }
            return format_to(ctx.out(), "{}.{:09d}", sec, frac);
        }
        return underlying.format(static_cast<double>(ns) / 1e9, ctx);
    }
};

template <> struct formatter<builtin_interfaces::msg::Time>
  : hatchbed_common::logging::FormatterDelegator<double>
{
    template <typename FormatContext>
    auto format(const builtin_interfaces::msg::Time& t, FormatContext& ctx) const {
        if (!has_custom_spec) {
            return format_to(ctx.out(), "{}.{:09d}", t.sec, t.nanosec);
        }

        // Convert ROS Time msg to double for custom formatting
        double total_seconds = static_cast<double>(t.sec) + (static_cast<double>(t.nanosec) / 1e9);

        ctx.advance_to(ctx.out());
        return underlying.format(total_seconds, ctx);
    }
};

template <> struct formatter<builtin_interfaces::msg::Duration>
  : hatchbed_common::logging::FormatterDelegator<double>
{
    template <typename FormatContext>
    auto format(const builtin_interfaces::msg::Duration& d, FormatContext& ctx) const {
        if (!has_custom_spec) {
            return format_to(ctx.out(), "{}.{:09d}", d.sec, d.nanosec);
        }
        double total_sec = static_cast<double>(d.sec) + (static_cast<double>(d.nanosec) / 1e9);
        ctx.advance_to(ctx.out());
        return underlying.format(total_sec, ctx);
    }
};

template <> struct formatter<std_msgs::msg::Header>
 : hatchbed_common::logging::FormatterDelegator<builtin_interfaces::msg::Time>
{
    template <typename FormatContext>
    auto format(const std_msgs::msg::Header& h, FormatContext& ctx) const {
        if (!has_custom_spec) {
            return format_to(ctx.out(), "[frame: '{}', stamp: {}]", h.frame_id, h.stamp);
        }

        auto out = format_to(ctx.out(), "[frame: '{}', stamp: ", h.frame_id);
        ctx.advance_to(out);
        // Delegate formatting of the stamp to our builtin_interfaces::msg::Time formatter
        out = underlying.format(h.stamp, ctx);
        return format_to(out, "]");
    }
};

template <> struct formatter<std_msgs::msg::ColorRGBA>
  : hatchbed_common::logging::FormatterDelegator<double> {
    template <typename FormatContext>
    auto format(const std_msgs::msg::ColorRGBA& c, FormatContext& ctx) const {
        if (!has_custom_spec) {
            return format_to(ctx.out(), "[r: {:.2f}, g: {:.2f}, b: {:.2f}, a: {:.2f}]", c.r, c.g,
                             c.b, c.a);
        }
        auto out = format_to(ctx.out(), "[r: ");
        ctx.advance_to(out); out = underlying.format(c.r, ctx);
        out = format_to(out, ", g: ");
        ctx.advance_to(out); out = underlying.format(c.g, ctx);
        out = format_to(out, ", b: ");
        ctx.advance_to(out); out = underlying.format(c.b, ctx);
        out = format_to(out, ", a: ");
        ctx.advance_to(out); out = underlying.format(c.a, ctx);
        return format_to(out, "]");
    }
};

// ============================================================================
// EIGEN TYPES (Universal SFINAE Formatter)
// ============================================================================
// This single block automatically catches ANY Eigen block type (Matrix3d, Vector3f, etc.)

// Disable fmt's built-in range formatter for all Eigen dense types to avoid
// ambiguity with our custom DenseBase formatter below.
template <typename T, typename Char>
struct range_format_kind<T, Char, std::enable_if_t<std::is_base_of_v<Eigen::DenseBase<T>, T>>>
    : std::integral_constant<range_format, range_format::disabled> {};

template <typename T>
struct formatter<T, char, std::enable_if_t<std::is_base_of_v<Eigen::DenseBase<T>, T>>>
  : hatchbed_common::logging::FormatterDelegator<typename T::Scalar>
{
    using Base = hatchbed_common::logging::FormatterDelegator<typename T::Scalar>;

    template <typename FormatContext>
    auto format(const T& mat, FormatContext& ctx) const {
        auto out = ctx.out();

        // 1. Handle Empty Matrices
        if (mat.rows() == 0 || mat.cols() == 0) {
            return fmt::format_to(out, "[]");
        }

        // 2. Formatting Logic
        bool is_vector = (mat.rows() == 1 || mat.cols() == 1);

        if (is_vector) {
            // Standard Vector format: [1.23, 4.56, 7.89]
            // Use 2D (r,c) access — linear coeff(i) is not valid for all expressions.
            out = fmt::format_to(out, "[");
            for (int i = 0; i < mat.size(); ++i) {
                if (i > 0) out = fmt::format_to(out, ", ");
                ctx.advance_to(out);
                int r = (mat.cols() == 1) ? i : 0;
                int c = (mat.cols() == 1) ? 0 : i;
                out = this->underlying.format(mat.derived()(r, c), ctx);
            }
            return fmt::format_to(out, "]");
        } else {
            // Standard Matrix format:
            // [[1.2, 3.4]
            //  [5.6, 7.8]]
            out = fmt::format_to(out, "\n[");
            for (int r = 0; r < mat.rows(); ++r) {
                if (r > 0) {
                    out = fmt::format_to(out, "\n [");
                } else {
                    out = fmt::format_to(out, "[");
                }

                for (int c = 0; c < mat.cols(); ++c) {
                    if (c > 0) out = fmt::format_to(out, ", ");
                    ctx.advance_to(out);
                    // derived() handles mapping/expressions safely
                    out = this->underlying.format(mat.derived()(r, c), ctx);
                }
                out = fmt::format_to(out, "]");
            }
            return fmt::format_to(out, "]");
        }
    }
};

/**
 * @brief Formatter for Eigen::Transform (Isometry, Affine, Projective)
 */
template <typename Scalar, int Dim, int Mode, int Options>
struct formatter<Eigen::Transform<Scalar, Dim, Mode, Options>>
    : hatchbed_common::logging::FormatterDelegator<Scalar>
{
    template <typename FormatContext>
    auto format(const Eigen::Transform<Scalar, Dim, Mode, Options>& tf, FormatContext& ctx) const {
        // ONLY format as "p, q" if it is an Isometry (Rigid body transform)
        if constexpr (Mode == Eigen::Isometry) {
            auto out = ctx.out();

            // 1. Translation
            out = fmt::format_to(out, "t: [");
            ctx.advance_to(out);
            const auto& translation = tf.translation();
            if constexpr (Dim == 3) {
                out = this->format_elements(ctx, ", ", translation.x(), translation.y(),
                                            translation.z());
            } else {
                out = this->format_elements(ctx, ", ", translation.x(), translation.y());
            }

            // 2. Rotation
            if constexpr (Dim == 3) {
                Eigen::Quaternion<Scalar> q(tf.rotation());
                out = fmt::format_to(out, "], r: [");
                ctx.advance_to(out);
                out = this->format_elements(ctx, ", ", q.x(), q.y(), q.z(), q.w());
            } else {
                Eigen::Rotation2D<Scalar> r2d(0);
                r2d.fromRotationMatrix(tf.rotation());
                out = fmt::format_to(out, "], angle: ");
                ctx.advance_to(out);
                return this->underlying.format(r2d.angle(), ctx);
            }
            return fmt::format_to(out, "]");
        } else {
            using MatrixType = typename Eigen::Transform<Scalar, Dim, Mode, Options>::MatrixType;
            fmt::formatter<MatrixType> mat_fmt;
            mat_fmt.underlying = this->underlying;
            mat_fmt.has_custom_spec = this->has_custom_spec;
            return mat_fmt.format(tf.matrix(), ctx);
        }
    }
};

/**
 * @brief Formatter for Eigen::Quaternion
 */
template <typename Scalar, int Options>
struct formatter<Eigen::Quaternion<Scalar, Options>>
    : hatchbed_common::logging::FormatterDelegator<Scalar>
{
    template <typename FormatContext>
    auto format(const Eigen::Quaternion<Scalar, Options>& q, FormatContext& ctx) const {
        if (!this->has_custom_spec) {
            return fmt::format_to(ctx.out(), "[{:.3f}, {:.3f}, {:.3f}, {:.3f}]", q.x(), q.y(),
                                  q.z(), q.w());
        }
        auto out = fmt::format_to(ctx.out(), "[");
        ctx.advance_to(out);
        // Note: Eigen stores quaternions internally, we access x, y, z, w specifically
        out = this->format_elements(ctx, ", ", q.x(), q.y(), q.z(), q.w());
        return fmt::format_to(out, "]");
    }
};

/**
 * @brief Formatter for Eigen::AngleAxis
 */
template <typename Scalar>
struct formatter<Eigen::AngleAxis<Scalar>>
    : hatchbed_common::logging::FormatterDelegator<Scalar>
{
    template <typename FormatContext>
    auto format(const Eigen::AngleAxis<Scalar>& aa, FormatContext& ctx) const {
        auto out = fmt::format_to(ctx.out(), "AngleAxis(angle: ");
        ctx.advance_to(out);
        out = this->underlying.format(aa.angle(), ctx);

        out = fmt::format_to(out, ", axis: [");
        ctx.advance_to(out);
        out = this->format_elements(ctx, ", ", aa.axis().x(), aa.axis().y(), aa.axis().z());

        return fmt::format_to(out, "])");
    }
};

// ============================================================================
// GEOMETRY & TF2 PRIMITIVES (Using Macros to reduce boilerplate)
// ============================================================================

#define HB_FMT_TF2_XYZ(Type) \
    template <> struct formatter<Type> : hatchbed_common::logging::FormatterDelegator<double> { \
        template <typename FormatContext> \
        auto format(const Type& v, FormatContext& ctx) const { \
            if (!has_custom_spec) return fmt::format_to(ctx.out(), "[{:.3f}, {:.3f}, {:.3f}]", \
                                                        v.x(), v.y(), v.z()); \
            auto out = fmt::format_to(ctx.out(), "["); \
            ctx.advance_to(out); \
            out = format_elements(ctx, ", ", v.x(), v.y(), v.z()); \
            return fmt::format_to(out, "]"); \
        } \
    }

#define HB_FMT_TF2_QUAT(Type) \
    template <> struct formatter<Type> : hatchbed_common::logging::FormatterDelegator<double> { \
        template <typename FormatContext> \
        auto format(const Type& q, FormatContext& ctx) const { \
            if (!has_custom_spec) { \
                return fmt::format_to(ctx.out(), "[{:.3f}, {:.3f}, {:.3f}, {:.3f}]", \
                                      q.x(), q.y(), q.z(), q.w()); \
            } \
            auto out = fmt::format_to(ctx.out(), "["); \
            ctx.advance_to(out); \
            out = format_elements(ctx, ", ", q.x(), q.y(), q.z(), q.w()); \
            return fmt::format_to(out, "]"); \
        } \
    }

HB_FMT_TF2_XYZ(tf2::Vector3);
HB_FMT_TF2_QUAT(tf2::Quaternion);

template <> struct formatter<tf2::Transform> : hatchbed_common::logging::FormatterDelegator<double>
{
    template <typename FormatContext>
    auto format(const tf2::Transform& t, FormatContext& ctx) const {
        const auto& origin = t.getOrigin();
        const auto& rot = t.getRotation();
        if (!has_custom_spec) return fmt::format_to(ctx.out(), "(t: {}, r: {})", origin,
                                                    rot);
        auto out = fmt::format_to(ctx.out(), "(t: [");
        ctx.advance_to(out);
        out = format_elements(ctx, ", ", origin.x(), origin.y(), origin.z());
        out = fmt::format_to(out, "], r: [");
        ctx.advance_to(out);
        out = format_elements(ctx, ", ", rot.x(), rot.y(), rot.z(), rot.w());
        return fmt::format_to(out, "])");
    }
};

// ============================================================================
// SMART POINTER FORMATTERS (Shared, Const Shared, Unique)
// ============================================================================
// This generic formatter intercepts ANY std::shared_ptr.
// It strips away the `const` (to handle msg::Type::ConstSharedPtr) and safely
// delegates the formatting to the underlying object's formatter.

template <typename T>
struct formatter<std::shared_ptr<T>> : formatter<std::remove_const_t<T>> {
    template <typename FormatContext>
    auto format(const std::shared_ptr<T>& ptr, FormatContext& ctx) const {
        if (ptr) {
            // Dereference the pointer and format the actual object
            return formatter<std::remove_const_t<T>>::format(*ptr, ctx);
        }
        // Fallback for empty pointers
        return format_to(ctx.out(), "[null]");
    }
};

// Also support std::unique_ptr (Heavily used in ROS 2 Intra-process comms)
template <typename T>
struct formatter<std::unique_ptr<T>> : formatter<std::remove_const_t<T>> {
    template <typename FormatContext>
    auto format(const std::unique_ptr<T>& ptr, FormatContext& ctx) const {
        if (ptr) {
            return formatter<std::remove_const_t<T>>::format(*ptr, ctx);
        }
        return format_to(ctx.out(), "[null]");
    }
};

}  // namespace fmt
