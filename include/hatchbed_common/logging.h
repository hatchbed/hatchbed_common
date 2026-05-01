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

#include <chrono>
#include <filesystem>  // Required for std::filesystem::path
#include <memory>
#include <string>
#include <type_traits>
#include <utility>

#include <fmt/core.h>
#include <hatchbed_common/logging/formatting.h>
#include <hatchbed_common/logging/macros.h>
#include <rclcpp/duration.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <rcutils/time.h>

namespace hatchbed_common {
namespace logging {

// Meyers Singleton to hold the global default logger safely
inline rclcpp::Logger& get_default_logger() {
    // Defaults to "default" if not explicitly set
    static rclcpp::Logger default_logger = rclcpp::get_logger("default");
    return default_logger;
}

// Call this at startup to override the default logger
inline void set_default_logger(const rclcpp::Logger& logger) {
    get_default_logger() = logger;
}

// User passed an actual rclcpp::Logger. Pass it through.
inline const rclcpp::Logger& resolve_logger(const rclcpp::Logger& logger) {
    return logger;
}

// User passed a rclcpp::Node pointer directly
inline rclcpp::Logger resolve_logger(rclcpp::Node* node) {
    return node->get_logger();
}

// User passed a shared_ptr<Node> directly
inline rclcpp::Logger resolve_logger(const std::shared_ptr<rclcpp::Node>& node) {
    return node->get_logger();
}

inline const rclcpp::Logger resolve_logger(const std::string& subsystem_name) {
    return get_default_logger().get_child(subsystem_name);
}

// User passed nullptr. Return the global default logger.
inline const rclcpp::Logger& resolve_logger(std::nullptr_t) {
    return get_default_logger();
}

inline rcutils_duration_value_t resolve_duration(int milliseconds) {
    return RCUTILS_MS_TO_NS(static_cast<rcutils_duration_value_t>(milliseconds));
}

inline rcutils_duration_value_t resolve_duration(const rclcpp::Duration& duration) {
    // rclcpp::Duration internally stores time as int64_t nanoseconds
    return duration.nanoseconds();
}

template <typename Rep, typename Period>
inline rcutils_duration_value_t resolve_duration(
    const std::chrono::duration<Rep, Period>& duration)
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
}

inline rclcpp::Clock& resolve_clock(rclcpp::Clock& clock) {
    return clock;
}

inline rclcpp::Clock& resolve_clock(const rclcpp::Clock::SharedPtr& clock) {
    return *clock;
}

inline rclcpp::Clock& resolve_clock(std::nullptr_t) {
    static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
    return steady_clock;
}

struct FormatOrForward {
    const char* file;
    int line;

private:
    // Helper for handling the formatting and safely catching format errors
    template <typename StringType, typename... Args>
    std::string do_format(StringType&& format_str, Args&&... args) const {
        try {
            return fmt::format(std::forward<StringType>(format_str), std::forward<Args>(args)...);
        }
        catch (const fmt::format_error& e) {
            fmt::string_view raw_fmt;
            if constexpr (std::is_same_v<std::decay_t<StringType>, fmt::basic_runtime<char>>) {
                raw_fmt = "[dynamic runtime string]";
            } else {
                raw_fmt = static_cast<fmt::string_view>(format_str);
            }
            return fmt::format(fmt::runtime("[LOGGING ERROR: {} | {}:{} | Format: {}]"),
                               e.what(), file, line, raw_fmt);
        }
        catch (const std::exception& e) {
            return fmt::format(fmt::runtime("[LOGGING ERROR: std::exception '{}' | {}:{}]"),
                               e.what(), file, line);
        }
        catch (...) {
            return fmt::format(fmt::runtime("[LOGGING ERROR: Unknown exception | {}:{}]"),
                               file, line);
        }
    }

public:
    // Overload 1: EXACTLY ONE argument (Primitives, standard strings)
    // We use enable_if to ensure this is NOT picked if the user passes fmt::runtime()
    template <typename T,
              typename std::enable_if_t<!std::is_same_v<std::decay_t<T>,
                                                        fmt::basic_runtime<char>>, int> = 0>
    inline decltype(auto) operator()(T&& arg) const {
        using DecayedT = std::decay_t<T>;

        if constexpr (std::is_same_v<DecayedT, const char*> ||
                      std::is_same_v<DecayedT, char*>) {
            return std::forward<T>(arg);
        } else if constexpr (std::is_same_v<DecayedT, std::string>) {
            return std::forward<T>(arg);
        } else {
            return fmt::to_string(std::forward<T>(arg));
        }
    }

    // Overload 2: Compile-time checked format string (1 or more args)
    template <typename... Args>
    inline std::string operator()(fmt::format_string<Args...> format_str, Args&&... args) const {
        return do_format(format_str, std::forward<Args>(args)...);
    }

    // Overload 3: Runtime format string bypass (1 or more args)
    template <typename... Args>
    inline std::string operator()(fmt::basic_runtime<char> format_str, Args&&... args) const {
        return do_format(format_str, std::forward<Args>(args)...);
    }
};

}  // namespace logging
}  // namespace hatchbed_common
