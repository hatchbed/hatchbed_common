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

// NOTE: Include via <hatchbed_common/logging.h>; resolve_logger() must be in scope.

#include <string>
#include <type_traits>
#include <utility>

#include <fmt/core.h>
#include <rcutils/logging.h>
#include <rcutils/logging_macros.h>

namespace hatchbed_common {
namespace logging {

// Low-level dispatch used by all per-level helpers below.
// Accepts any logger type supported by resolve_logger() plus an explicit source
// location so that ROS tooling (rqt_console, ros2 log) shows the real origin.
template <typename LoggerT>
inline void log_at(LoggerT&& logger, int severity,
                   const char* filename, const char* function, int line,
                   const std::string& message) {
    RCUTILS_LOGGING_AUTOINIT;
    auto && resolved = resolve_logger(std::forward<LoggerT>(logger));
    const char* name = resolved.get_name();
    if (rcutils_logging_logger_is_enabled_for(name, severity)) {
        rcutils_log_location_t loc = {function, filename, static_cast<size_t>(line)};
        rcutils_log(&loc, severity, name, "%s", message.c_str());
    }
}

// ---- Per-level functions: explicit logger + explicit source location ----
// Two overloads per level:
//   (1) pre-formatted std::string  -- used when the message is already built
//   (2) fmt format_string + args   -- compile-time checked; requires >= 1 format arg

// clang-format off

template <typename LoggerT>
inline void debug(LoggerT&& l, const char* file, const char* func, int line,
                  const std::string& msg) {
    log_at(std::forward<LoggerT>(l), RCUTILS_LOG_SEVERITY_DEBUG, file, func, line, msg);
}
template <typename LoggerT, typename... Args,
          std::enable_if_t<(sizeof...(Args) > 0), int> = 0>
inline void debug(LoggerT&& l, const char* file, const char* func, int line,
                  fmt::format_string<Args...> fmt_str, Args&&... args) {
    log_at(std::forward<LoggerT>(l), RCUTILS_LOG_SEVERITY_DEBUG, file, func, line,
           fmt::format(fmt_str, std::forward<Args>(args)...));
}

template <typename LoggerT>
inline void info(LoggerT&& l, const char* file, const char* func, int line,
                 const std::string& msg) {
    log_at(std::forward<LoggerT>(l), RCUTILS_LOG_SEVERITY_INFO, file, func, line, msg);
}
template <typename LoggerT, typename... Args,
          std::enable_if_t<(sizeof...(Args) > 0), int> = 0>
inline void info(LoggerT&& l, const char* file, const char* func, int line,
                 fmt::format_string<Args...> fmt_str, Args&&... args) {
    log_at(std::forward<LoggerT>(l), RCUTILS_LOG_SEVERITY_INFO, file, func, line,
           fmt::format(fmt_str, std::forward<Args>(args)...));
}

template <typename LoggerT>
inline void warn(LoggerT&& l, const char* file, const char* func, int line,
                 const std::string& msg) {
    log_at(std::forward<LoggerT>(l), RCUTILS_LOG_SEVERITY_WARN, file, func, line, msg);
}
template <typename LoggerT, typename... Args,
          std::enable_if_t<(sizeof...(Args) > 0), int> = 0>
inline void warn(LoggerT&& l, const char* file, const char* func, int line,
                 fmt::format_string<Args...> fmt_str, Args&&... args) {
    log_at(std::forward<LoggerT>(l), RCUTILS_LOG_SEVERITY_WARN, file, func, line,
           fmt::format(fmt_str, std::forward<Args>(args)...));
}

template <typename LoggerT>
inline void error(LoggerT&& l, const char* file, const char* func, int line,
                  const std::string& msg) {
    log_at(std::forward<LoggerT>(l), RCUTILS_LOG_SEVERITY_ERROR, file, func, line, msg);
}
template <typename LoggerT, typename... Args,
          std::enable_if_t<(sizeof...(Args) > 0), int> = 0>
inline void error(LoggerT&& l, const char* file, const char* func, int line,
                  fmt::format_string<Args...> fmt_str, Args&&... args) {
    log_at(std::forward<LoggerT>(l), RCUTILS_LOG_SEVERITY_ERROR, file, func, line,
           fmt::format(fmt_str, std::forward<Args>(args)...));
}

template <typename LoggerT>
inline void fatal(LoggerT&& l, const char* file, const char* func, int line,
                  const std::string& msg) {
    log_at(std::forward<LoggerT>(l), RCUTILS_LOG_SEVERITY_FATAL, file, func, line, msg);
}
template <typename LoggerT, typename... Args,
          std::enable_if_t<(sizeof...(Args) > 0), int> = 0>
inline void fatal(LoggerT&& l, const char* file, const char* func, int line,
                  fmt::format_string<Args...> fmt_str, Args&&... args) {
    log_at(std::forward<LoggerT>(l), RCUTILS_LOG_SEVERITY_FATAL, file, func, line,
           fmt::format(fmt_str, std::forward<Args>(args)...));
}

// ---- No-logger variants: use the default logger (nullptr -> resolve_logger) ----

inline void debug(const char* file, const char* func, int line, const std::string& msg) {
    log_at(nullptr, RCUTILS_LOG_SEVERITY_DEBUG, file, func, line, msg);
}
template <typename... Args, std::enable_if_t<(sizeof...(Args) > 0), int> = 0>
inline void debug(const char* file, const char* func, int line,
                  fmt::format_string<Args...> fmt_str, Args&&... args) {
    log_at(nullptr, RCUTILS_LOG_SEVERITY_DEBUG, file, func, line,
           fmt::format(fmt_str, std::forward<Args>(args)...));
}

inline void info(const char* file, const char* func, int line, const std::string& msg) {
    log_at(nullptr, RCUTILS_LOG_SEVERITY_INFO, file, func, line, msg);
}
template <typename... Args, std::enable_if_t<(sizeof...(Args) > 0), int> = 0>
inline void info(const char* file, const char* func, int line,
                 fmt::format_string<Args...> fmt_str, Args&&... args) {
    log_at(nullptr, RCUTILS_LOG_SEVERITY_INFO, file, func, line,
           fmt::format(fmt_str, std::forward<Args>(args)...));
}

inline void warn(const char* file, const char* func, int line, const std::string& msg) {
    log_at(nullptr, RCUTILS_LOG_SEVERITY_WARN, file, func, line, msg);
}
template <typename... Args, std::enable_if_t<(sizeof...(Args) > 0), int> = 0>
inline void warn(const char* file, const char* func, int line,
                 fmt::format_string<Args...> fmt_str, Args&&... args) {
    log_at(nullptr, RCUTILS_LOG_SEVERITY_WARN, file, func, line,
           fmt::format(fmt_str, std::forward<Args>(args)...));
}

inline void error(const char* file, const char* func, int line, const std::string& msg) {
    log_at(nullptr, RCUTILS_LOG_SEVERITY_ERROR, file, func, line, msg);
}
template <typename... Args, std::enable_if_t<(sizeof...(Args) > 0), int> = 0>
inline void error(const char* file, const char* func, int line,
                  fmt::format_string<Args...> fmt_str, Args&&... args) {
    log_at(nullptr, RCUTILS_LOG_SEVERITY_ERROR, file, func, line,
           fmt::format(fmt_str, std::forward<Args>(args)...));
}

inline void fatal(const char* file, const char* func, int line, const std::string& msg) {
    log_at(nullptr, RCUTILS_LOG_SEVERITY_FATAL, file, func, line, msg);
}
template <typename... Args, std::enable_if_t<(sizeof...(Args) > 0), int> = 0>
inline void fatal(const char* file, const char* func, int line,
                  fmt::format_string<Args...> fmt_str, Args&&... args) {
    log_at(nullptr, RCUTILS_LOG_SEVERITY_FATAL, file, func, line,
           fmt::format(fmt_str, std::forward<Args>(args)...));
}

// clang-format on

}  // namespace logging
}  // namespace hatchbed_common
