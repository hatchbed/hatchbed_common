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

#include <rcutils/logging_macros.h>

// --- LOGGING HELPER MACROS ---

#define HB_LOG_FMT(logger, severity, condition_before, condition_after, ...) \
  do { \
    RCUTILS_LOGGING_AUTOINIT; \
    auto && __hb_log_obj = hatchbed_common::logging::resolve_logger(logger); \
    const char* __hb_log_name = __hb_log_obj.get_name(); \
    if (rcutils_logging_logger_is_enabled_for(__hb_log_name, severity)) { \
      static rcutils_log_location_t __hb_log_loc = {__func__, __FILE__, __LINE__}; \
      condition_before \
      rcutils_log(&__hb_log_loc, severity, __hb_log_name, "%s", \
                  rclcpp::get_c_string( \
                    hatchbed_common::logging::format_or_forward(__VA_ARGS__))); \
      condition_after \
    } \
  } while (0)

#define HB_LOG_CONDITION_EMPTY

#define HB_LOG_CONDITION_ONCE_BEFORE { \
  static int __hb_logging_once = 0; \
  if (RCUTILS_UNLIKELY(0 == __hb_logging_once)) { \
    __hb_logging_once = 1;

#define HB_LOG_CONDITION_ONCE_AFTER }}

#define HB_LOG_CONDITION_EXPRESSION_BEFORE(expression) \
   if (expression) {
#define HB_LOG_CONDITION_EXPRESSION_AFTER }

#define HB_LOG_CONDITION_FUNCTION_BEFORE(function) \
   if ((*function)()) {
#define HB_LOG_CONDITION_FUNCTION_AFTER }

#define HB_LOG_CONDITION_SKIPFIRST_BEFORE { \
  static bool __hb_logging_first = true; \
  if (RCUTILS_UNLIKELY(true == __hb_logging_first)) { \
    __hb_logging_first = false; \
  } else {
#define HB_LOG_CONDITION_SKIPFIRST_AFTER }}

#define HB_LOG_CONDITION_THROTTLE_BEFORE(get_time_point, duration) { \
    static auto __hb_logging_duration = hatchbed_common::logging::resolve_duration(duration); \
    static rcutils_time_point_value_t __hb_logging_last_logged = 0; \
    rcutils_time_point_value_t __hb_logging_now = 0; \
    bool __hb_logging_condition = true; \
    if (get_time_point(&__hb_logging_now) != RCUTILS_RET_OK) { \
      rcutils_log( \
        &__hb_log_loc, RCUTILS_LOG_SEVERITY_ERROR, "", \
        "%s() at %s:%d getting current steady time failed\n", \
        __func__, __FILE__, __LINE__); \
    } else { \
      __hb_logging_condition = __hb_logging_now >= \
      __hb_logging_last_logged + __hb_logging_duration; \
    } \
    if (RCUTILS_LIKELY(__hb_logging_condition)) { \
      __hb_logging_last_logged = __hb_logging_now; \

#define HB_LOG_CONDITION_THROTTLE_AFTER }}

#define HB_LOG_TIME_POINT_FUNC(clock) \
  [&c = hatchbed_common::logging::resolve_clock(clock)](rcutils_time_point_value_t * time_point) \
      ->rcutils_ret_t { \
    try { \
      *time_point = c.now().nanoseconds(); \
    } catch (...) { \
      RCUTILS_SAFE_FWRITE_TO_STDERR( \
      "[hatchbed_common|logging.h] could not get current time stamp\n"); \
      return RCUTILS_RET_ERROR; \
    } \
    return RCUTILS_RET_OK; \
  }

// --- DEBUG ---

#if (RCUTILS_LOG_MIN_SEVERITY > RCUTILS_LOG_MIN_SEVERITY_DEBUG)

// empty logging macros for severity DEBUG when being disabled at compile time
#define HB_DEBUG(logger, ...)
#define HB_DEBUG_ONCE(logger, ...)
#define HB_DEBUG_SKIPFIRST(logger, ...)
#define HB_DEBUG_EXPRESSION(logger, expression, ...)
#define HB_DEBUG_FUNCTION(logger, function, ...)
#define HB_DEBUG_THROTTLE(logger, clock, duration, ...)

#else

#define HB_DEBUG(logger, ...) \
    HB_LOG_FMT(logger, RCUTILS_LOG_SEVERITY_DEBUG, HB_LOG_CONDITION_EMPTY, \
               HB_LOG_CONDITION_EMPTY, __VA_ARGS__)

#define HB_DEBUG_ONCE(logger, ...) \
    HB_LOG_FMT(logger, RCUTILS_LOG_SEVERITY_DEBUG, HB_LOG_CONDITION_ONCE_BEFORE, \
               HB_LOG_CONDITION_ONCE_AFTER, __VA_ARGS__)

#define HB_DEBUG_SKIPFIRST(logger, ...) \
    HB_LOG_FMT(logger, RCUTILS_LOG_SEVERITY_DEBUG, HB_LOG_CONDITION_SKIPFIRST_BEFORE, \
               HB_LOG_CONDITION_SKIPFIRST_AFTER, __VA_ARGS__)

#define HB_DEBUG_EXPRESSION(logger, expression, ...) \
    HB_LOG_FMT(logger, RCUTILS_LOG_SEVERITY_DEBUG, HB_LOG_CONDITION_EXPRESSION_BEFORE(expression), \
               HB_LOG_CONDITION_EXPRESSION_AFTER, __VA_ARGS__)

#define HB_DEBUG_FUNCTION(logger, function, ...) \
    HB_LOG_FMT(logger, RCUTILS_LOG_SEVERITY_DEBUG, HB_LOG_CONDITION_FUNCTION_BEFORE(function), \
               HB_LOG_CONDITION_FUNCTION_AFTER, __VA_ARGS__)

#define HB_DEBUG_THROTTLE(logger, clock, duration, ...) \
    HB_LOG_FMT(logger, RCUTILS_LOG_SEVERITY_DEBUG, \
               HB_LOG_CONDITION_THROTTLE_BEFORE(HB_LOG_TIME_POINT_FUNC(clock), duration), \
               HB_LOG_CONDITION_THROTTLE_AFTER, __VA_ARGS__)

#endif  // (RCUTILS_LOG_MIN_SEVERITY > RCUTILS_LOG_MIN_SEVERITY_DEBUG)

// --- ERROR ---

#if (RCUTILS_LOG_MIN_SEVERITY > RCUTILS_LOG_MIN_SEVERITY_ERROR)

// empty logging macros for severity ERROR when being disabled at compile time
#define HB_ERROR(logger, ...)
#define HB_ERROR_ONCE(logger, ...)
#define HB_ERROR_SKIPFIRST(logger, ...)
#define HB_ERROR_EXPRESSION(logger, expression, ...)
#define HB_ERROR_FUNCTION(logger, function, ...)
#define HB_ERROR_THROTTLE(logger, clock, duration, ...)

#else

#define HB_ERROR(logger, ...) \
    HB_LOG_FMT(logger, RCUTILS_LOG_SEVERITY_ERROR, HB_LOG_CONDITION_EMPTY, \
               HB_LOG_CONDITION_EMPTY, __VA_ARGS__)

#define HB_ERROR_ONCE(logger, ...) \
    HB_LOG_FMT(logger, RCUTILS_LOG_SEVERITY_ERROR, HB_LOG_CONDITION_ONCE_BEFORE, \
               HB_LOG_CONDITION_ONCE_AFTER, __VA_ARGS__)

#define HB_ERROR_SKIPFIRST(logger, ...) \
    HB_LOG_FMT(logger, RCUTILS_LOG_SEVERITY_ERROR, HB_LOG_CONDITION_SKIPFIRST_BEFORE, \
               HB_LOG_CONDITION_SKIPFIRST_AFTER, __VA_ARGS__)

#define HB_ERROR_EXPRESSION(logger, expression, ...) \
    HB_LOG_FMT(logger, RCUTILS_LOG_SEVERITY_ERROR, HB_LOG_CONDITION_EXPRESSION_BEFORE(expression), \
               HB_LOG_CONDITION_EXPRESSION_AFTER, __VA_ARGS__)

#define HB_ERROR_FUNCTION(logger, function, ...) \
    HB_LOG_FMT(logger, RCUTILS_LOG_SEVERITY_ERROR, HB_LOG_CONDITION_FUNCTION_BEFORE(function), \
               HB_LOG_CONDITION_FUNCTION_AFTER, __VA_ARGS__)

#define HB_ERROR_THROTTLE(logger, clock, duration, ...) \
    HB_LOG_FMT(logger, RCUTILS_LOG_SEVERITY_ERROR, \
               HB_LOG_CONDITION_THROTTLE_BEFORE(HB_LOG_TIME_POINT_FUNC(clock), duration), \
               HB_LOG_CONDITION_THROTTLE_AFTER, __VA_ARGS__)

#endif  // (RCUTILS_LOG_MIN_SEVERITY > RCUTILS_LOG_MIN_SEVERITY_ERROR)

// --- FATAL ---

#if (RCUTILS_LOG_MIN_SEVERITY > RCUTILS_LOG_MIN_SEVERITY_FATAL)

// empty logging macros for severity FATAL when being disabled at compile time
#define HB_FATAL(logger, ...)
#define HB_FATAL_ONCE(logger, ...)
#define HB_FATAL_SKIPFIRST(logger, ...)
#define HB_FATAL_EXPRESSION(logger, expression, ...)
#define HB_FATAL_FUNCTION(logger, function, ...)
#define HB_FATAL_THROTTLE(logger, clock, duration, ...)

#else

#define HB_FATAL(logger, ...) \
    HB_LOG_FMT(logger, RCUTILS_LOG_SEVERITY_FATAL, HB_LOG_CONDITION_EMPTY, \
               HB_LOG_CONDITION_EMPTY, __VA_ARGS__)

#define HB_FATAL_ONCE(logger, ...) \
    HB_LOG_FMT(logger, RCUTILS_LOG_SEVERITY_FATAL, HB_LOG_CONDITION_ONCE_BEFORE, \
               HB_LOG_CONDITION_ONCE_AFTER, __VA_ARGS__)

#define HB_FATAL_SKIPFIRST(logger, ...) \
    HB_LOG_FMT(logger, RCUTILS_LOG_SEVERITY_FATAL, HB_LOG_CONDITION_SKIPFIRST_BEFORE, \
               HB_LOG_CONDITION_SKIPFIRST_AFTER, __VA_ARGS__)

#define HB_FATAL_EXPRESSION(logger, expression, ...) \
    HB_LOG_FMT(logger, RCUTILS_LOG_SEVERITY_FATAL, HB_LOG_CONDITION_EXPRESSION_BEFORE(expression), \
               HB_LOG_CONDITION_EXPRESSION_AFTER, __VA_ARGS__)

#define HB_FATAL_FUNCTION(logger, function, ...) \
    HB_LOG_FMT(logger, RCUTILS_LOG_SEVERITY_FATAL, HB_LOG_CONDITION_FUNCTION_BEFORE(function), \
               HB_LOG_CONDITION_FUNCTION_AFTER, __VA_ARGS__)

#define HB_FATAL_THROTTLE(logger, clock, duration, ...) \
    HB_LOG_FMT(logger, RCUTILS_LOG_SEVERITY_FATAL, \
               HB_LOG_CONDITION_THROTTLE_BEFORE(HB_LOG_TIME_POINT_FUNC(clock), duration), \
               HB_LOG_CONDITION_THROTTLE_AFTER, __VA_ARGS__)

#endif  // (RCUTILS_LOG_MIN_SEVERITY > RCUTILS_LOG_MIN_SEVERITY_FATAL)

// --- INFO ---

#if (RCUTILS_LOG_MIN_SEVERITY > RCUTILS_LOG_MIN_SEVERITY_INFO)

// empty logging macros for severity INFO when being disabled at compile time
#define HB_INFO(logger, ...)
#define HB_INFO_ONCE(logger, ...)
#define HB_INFO_SKIPFIRST(logger, ...)
#define HB_INFO_EXPRESSION(logger, expression, ...)
#define HB_INFO_FUNCTION(logger, function, ...)
#define HB_INFO_THROTTLE(logger, clock, duration, ...)

#else

#define HB_INFO(logger, ...) \
    HB_LOG_FMT(logger, RCUTILS_LOG_SEVERITY_INFO, HB_LOG_CONDITION_EMPTY, \
               HB_LOG_CONDITION_EMPTY, __VA_ARGS__)

#define HB_INFO_ONCE(logger, ...) \
    HB_LOG_FMT(logger, RCUTILS_LOG_SEVERITY_INFO, HB_LOG_CONDITION_ONCE_BEFORE, \
               HB_LOG_CONDITION_ONCE_AFTER, __VA_ARGS__)

#define HB_INFO_SKIPFIRST(logger, ...) \
    HB_LOG_FMT(logger, RCUTILS_LOG_SEVERITY_INFO, HB_LOG_CONDITION_SKIPFIRST_BEFORE, \
               HB_LOG_CONDITION_SKIPFIRST_AFTER, __VA_ARGS__)

#define HB_INFO_EXPRESSION(logger, expression, ...) \
    HB_LOG_FMT(logger, RCUTILS_LOG_SEVERITY_INFO, HB_LOG_CONDITION_EXPRESSION_BEFORE(expression), \
               HB_LOG_CONDITION_EXPRESSION_AFTER, __VA_ARGS__)

#define HB_INFO_FUNCTION(logger, function, ...) \
    HB_LOG_FMT(logger, RCUTILS_LOG_SEVERITY_INFO, HB_LOG_CONDITION_FUNCTION_BEFORE(function), \
               HB_LOG_CONDITION_FUNCTION_AFTER, __VA_ARGS__)

#define HB_INFO_THROTTLE(logger, clock, duration, ...) \
    HB_LOG_FMT(logger, RCUTILS_LOG_SEVERITY_INFO, \
               HB_LOG_CONDITION_THROTTLE_BEFORE(HB_LOG_TIME_POINT_FUNC(clock), duration), \
               HB_LOG_CONDITION_THROTTLE_AFTER, __VA_ARGS__)

#endif  // (RCUTILS_LOG_MIN_SEVERITY > RCUTILS_LOG_MIN_SEVERITY_INFO)

// --- WARN ---

#if (RCUTILS_LOG_MIN_SEVERITY > RCUTILS_LOG_MIN_SEVERITY_WARN)

// empty logging macros for severity WARN when being disabled at compile time
#define HB_WARN(logger, ...)
#define HB_WARN_ONCE(logger, ...)
#define HB_WARN_SKIPFIRST(logger, ...)
#define HB_WARN_EXPRESSION(logger, expression, ...)
#define HB_WARN_FUNCTION(logger, function, ...)
#define HB_WARN_THROTTLE(logger, clock, duration, ...)

#else

#define HB_WARN(logger, ...) \
    HB_LOG_FMT(logger, RCUTILS_LOG_SEVERITY_WARN, HB_LOG_CONDITION_EMPTY, \
               HB_LOG_CONDITION_EMPTY, __VA_ARGS__)

#define HB_WARN_ONCE(logger, ...) \
    HB_LOG_FMT(logger, RCUTILS_LOG_SEVERITY_WARN, HB_LOG_CONDITION_ONCE_BEFORE, \
               HB_LOG_CONDITION_ONCE_AFTER, __VA_ARGS__)

#define HB_WARN_SKIPFIRST(logger, ...) \
    HB_LOG_FMT(logger, RCUTILS_LOG_SEVERITY_WARN, HB_LOG_CONDITION_SKIPFIRST_BEFORE, \
               HB_LOG_CONDITION_SKIPFIRST_AFTER, __VA_ARGS__)

#define HB_WARN_EXPRESSION(logger, expression, ...) \
    HB_LOG_FMT(logger, RCUTILS_LOG_SEVERITY_WARN, HB_LOG_CONDITION_EXPRESSION_BEFORE(expression), \
               HB_LOG_CONDITION_EXPRESSION_AFTER, __VA_ARGS__)

#define HB_WARN_FUNCTION(logger, function, ...) \
    HB_LOG_FMT(logger, RCUTILS_LOG_SEVERITY_WARN, HB_LOG_CONDITION_FUNCTION_BEFORE(function), \
               HB_LOG_CONDITION_FUNCTION_AFTER, __VA_ARGS__)

#define HB_WARN_THROTTLE(logger, clock, duration, ...) \
    HB_LOG_FMT(logger, RCUTILS_LOG_SEVERITY_WARN, \
               HB_LOG_CONDITION_THROTTLE_BEFORE(HB_LOG_TIME_POINT_FUNC(clock), duration), \
               HB_LOG_CONDITION_THROTTLE_AFTER, __VA_ARGS__)

#endif  // (RCUTILS_LOG_MIN_SEVERITY > RCUTILS_LOG_MIN_SEVERITY_WARN)
