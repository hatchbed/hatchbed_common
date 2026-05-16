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

#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <hatchbed_common/logging.h>
#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>

using hatchbed_common::logging::FormatOrForward;
using hatchbed_common::logging::resolve_duration;
using hatchbed_common::logging::resolve_logger;
using hatchbed_common::logging::set_default_logger;

// --- Helper for intercepting logs ---
struct LogRecord {
    int severity;
    std::string name;
    std::string message;
    std::string filename;
    std::string function;
    size_t line = 0;
};

static std::vector<LogRecord> g_log_records;

void custom_log_handler(
    const rcutils_log_location_t * location,
    int severity, const char * name, rcutils_time_point_value_t,
    const char * format, va_list * args)
{
    char buffer[1024];
    vsnprintf(buffer, sizeof(buffer), format, *args);
    g_log_records.push_back({
        severity,
        name ? name : "",
        buffer,
        location && location->file_name     ? location->file_name     : "",
        location && location->function_name ? location->function_name : "",
        location ? location->line_number : 0,
    });
}

class LoggingTest : public ::testing::Test {
protected:
    void SetUp() override {
        g_log_records.clear();
        rcutils_logging_set_output_handler(custom_log_handler);
        rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_DEBUG);
        set_default_logger(rclcpp::get_logger("default"));
    }

    void TearDown() override {
        rcutils_logging_set_output_handler(rcutils_logging_console_output_handler);
        rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_DEBUG);
        set_default_logger(rclcpp::get_logger("default"));
    }
};

// ============================================================================
// 1. RESOLUTION HELPERS TESTS
// ============================================================================

TEST_F(LoggingTest, ResolveLogger) {
    auto node = std::make_shared<rclcpp::Node>("test_node");

    // Test nullptr (Default)
    EXPECT_STREQ(resolve_logger(nullptr).get_name(), "default");

    // Test Node Pointer
    EXPECT_STREQ(resolve_logger(node.get()).get_name(), "test_node");

    // Test SharedPtr Node
    EXPECT_STREQ(resolve_logger(node).get_name(), "test_node");

    // Test Subsystem String
    EXPECT_STREQ(resolve_logger("sub").get_name(), "default.sub");

    // Test Actual Logger
    rclcpp::Logger explicit_logger = rclcpp::get_logger("explicit");
    EXPECT_STREQ(resolve_logger(explicit_logger).get_name(), "explicit");
}

TEST_F(LoggingTest, ResolveDuration) {
    // Test Integer ms
    EXPECT_EQ(resolve_duration(1000), 1000000000L);

    // Test rclcpp::Duration
    EXPECT_EQ(resolve_duration(rclcpp::Duration::from_seconds(1.5)), 1500000000L);

    // Test std::chrono
    EXPECT_EQ(resolve_duration(std::chrono::milliseconds(500)), 500000000L);
}

TEST_F(LoggingTest, FormatOrForward) {
    FormatOrForward format_or_forward{__FILE__, __LINE__};

    // Single Argument: String Literal
    EXPECT_STREQ(format_or_forward("hello"), "hello");

    // Single Argument: std::string
    std::string s = "world";
    EXPECT_EQ(format_or_forward(s), "world");

    // Single Argument: Numeric (should convert to string)
    EXPECT_EQ(format_or_forward(42), "42");

    // Multiple Arguments: fmt logic
    EXPECT_EQ(format_or_forward("val: {}", 123), "val: 123");
}

// ============================================================================
// 2. MACRO FUNCTIONALITY TESTS
// ============================================================================

TEST_F(LoggingTest, BasicLogging) {
    HB_INFO(nullptr, "Hello {}", "World");
    ASSERT_EQ(g_log_records.size(), 1u);
    EXPECT_EQ(g_log_records[0].message, "Hello World");
    EXPECT_EQ(g_log_records[0].severity, RCUTILS_LOG_SEVERITY_INFO);
}

TEST_F(LoggingTest, LogOnce) {
    for (int i = 0; i < 5; ++i) {
        HB_INFO_ONCE(nullptr, "Only once");
    }
    // Static variable in macro should restrict this to 1
    EXPECT_EQ(g_log_records.size(), 1u);
}

TEST_F(LoggingTest, LogSkipFirst) {
    for (int i = 0; i < 3; ++i) {
        HB_INFO_SKIPFIRST(nullptr, "Skip first: {}", i);
    }
    ASSERT_EQ(g_log_records.size(), 2u);
    EXPECT_EQ(g_log_records[0].message, "Skip first: 1");
    EXPECT_EQ(g_log_records[1].message, "Skip first: 2");
}

TEST_F(LoggingTest, LogExpression) {
    bool log_me = false;
    HB_INFO_EXPRESSION(nullptr, log_me, "Should not see this");
    EXPECT_EQ(g_log_records.size(), 0u);

    log_me = true;
    HB_INFO_EXPRESSION(nullptr, log_me, "Should see this");
    EXPECT_EQ(g_log_records.size(), 1u);
}

TEST_F(LoggingTest, LogThrottle) {
    auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);

    // Both calls share the same static throttle state because they're at the same source line.
    // The first iteration logs; subsequent iterations within the 1000 ms window are suppressed.
    for (int i = 0; i < 2; ++i) {
        HB_INFO_THROTTLE(nullptr, clock, 1000, "Throttled");
    }
    EXPECT_EQ(g_log_records.size(), 1u);
}

TEST_F(LoggingTest, DefaultLoggerOverride) {
    auto node = std::make_shared<rclcpp::Node>("new_default");
    set_default_logger(node->get_logger());
    g_log_records.clear();

    HB_INFO(nullptr, "Test default");
    ASSERT_EQ(g_log_records.size(), 1u);
    EXPECT_EQ(g_log_records[0].name, "new_default");
}

// ============================================================================
// 3. SEVERITY LEVELS
// ============================================================================

TEST_F(LoggingTest, SeverityDebug) {
    HB_DEBUG(nullptr, "Debug message");
    ASSERT_EQ(g_log_records.size(), 1u);
    EXPECT_EQ(g_log_records[0].severity, RCUTILS_LOG_SEVERITY_DEBUG);
    EXPECT_EQ(g_log_records[0].message, "Debug message");
}

TEST_F(LoggingTest, SeverityWarn) {
    HB_WARN(nullptr, "Warn message");
    ASSERT_EQ(g_log_records.size(), 1u);
    EXPECT_EQ(g_log_records[0].severity, RCUTILS_LOG_SEVERITY_WARN);
    EXPECT_EQ(g_log_records[0].message, "Warn message");
}

TEST_F(LoggingTest, SeverityError) {
    HB_ERROR(nullptr, "Error message");
    ASSERT_EQ(g_log_records.size(), 1u);
    EXPECT_EQ(g_log_records[0].severity, RCUTILS_LOG_SEVERITY_ERROR);
    EXPECT_EQ(g_log_records[0].message, "Error message");
}

TEST_F(LoggingTest, SeverityFatal) {
    HB_FATAL(nullptr, "Fatal message");
    ASSERT_EQ(g_log_records.size(), 1u);
    EXPECT_EQ(g_log_records[0].severity, RCUTILS_LOG_SEVERITY_FATAL);
    EXPECT_EQ(g_log_records[0].message, "Fatal message");
}

// ============================================================================
// 4. LOGGER ROUTING THROUGH MACROS
// ============================================================================

TEST_F(LoggingTest, LoggerRoutingNodePtr) {
    auto node = std::make_shared<rclcpp::Node>("route_ptr");
    g_log_records.clear();
    HB_INFO(node.get(), "via node ptr");
    ASSERT_EQ(g_log_records.size(), 1u);
    EXPECT_EQ(g_log_records[0].name, "route_ptr");
}

TEST_F(LoggingTest, LoggerRoutingSharedPtr) {
    auto node = std::make_shared<rclcpp::Node>("route_shared");
    g_log_records.clear();
    HB_INFO(node, "via shared ptr");
    ASSERT_EQ(g_log_records.size(), 1u);
    EXPECT_EQ(g_log_records[0].name, "route_shared");
}

TEST_F(LoggingTest, LoggerRoutingExplicit) {
    rclcpp::Logger logger = rclcpp::get_logger("explicit_route");
    HB_INFO(logger, "via explicit logger");
    ASSERT_EQ(g_log_records.size(), 1u);
    EXPECT_EQ(g_log_records[0].name, "explicit_route");
}

TEST_F(LoggingTest, LoggerRoutingSubsystem) {
    HB_INFO("my_sub", "via subsystem");
    ASSERT_EQ(g_log_records.size(), 1u);
    EXPECT_EQ(g_log_records[0].name, "default.my_sub");
}

// ============================================================================
// 5. FUNCTION CONDITION VARIANT
// ============================================================================

static bool always_log() { return true; }
static bool never_log() { return false; }

TEST_F(LoggingTest, LogFunctionTrue) {
    HB_INFO_FUNCTION(nullptr, always_log, "Function true");
    ASSERT_EQ(g_log_records.size(), 1u);
    EXPECT_EQ(g_log_records[0].message, "Function true");
}

TEST_F(LoggingTest, LogFunctionFalse) {
    HB_INFO_FUNCTION(nullptr, never_log, "Should not appear");
    EXPECT_EQ(g_log_records.size(), 0u);
}

// ============================================================================
// 6. SEVERITY FILTERING
// ============================================================================

TEST_F(LoggingTest, SeverityFiltering) {
    rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_WARN);

    HB_DEBUG(nullptr, "Filtered debug");
    HB_INFO(nullptr, "Filtered info");
    EXPECT_EQ(g_log_records.size(), 0u);

    HB_WARN(nullptr, "Passes filter");
    ASSERT_EQ(g_log_records.size(), 1u);
    EXPECT_EQ(g_log_records[0].message, "Passes filter");
}

// ============================================================================
// 7. THROTTLE CLOCK VARIANTS
// ============================================================================

TEST_F(LoggingTest, ThrottleNullptrClock) {
    for (int i = 0; i < 3; ++i) {
        HB_INFO_THROTTLE(nullptr, nullptr, 1000, "Null clock throttle");
    }
    EXPECT_EQ(g_log_records.size(), 1u);
}

TEST_F(LoggingTest, ThrottleClockByRef) {
    rclcpp::Clock clock(RCL_STEADY_TIME);
    for (int i = 0; i < 3; ++i) {
        HB_INFO_THROTTLE(nullptr, clock, 1000, "Clock ref throttle");
    }
    EXPECT_EQ(g_log_records.size(), 1u);
}

// ============================================================================
// 8. RESOLVE_DURATION EDGE CASES
// ============================================================================

TEST_F(LoggingTest, ResolveDurationZero) {
    EXPECT_EQ(resolve_duration(0), 0L);
}

TEST_F(LoggingTest, ResolveDurationChronoSeconds) {
    EXPECT_EQ(resolve_duration(std::chrono::seconds(2)), 2000000000L);
}

TEST_F(LoggingTest, ResolveDurationChronoMicroseconds) {
    EXPECT_EQ(resolve_duration(std::chrono::microseconds(500)), 500000L);
}

// ============================================================================
// 9. FORMAT_OR_FORWARD EDGE CASES
// ============================================================================

TEST_F(LoggingTest, FormatOrForwardEmptyString) {
    FormatOrForward format_or_forward{__FILE__, __LINE__};
    EXPECT_STREQ(format_or_forward(""), "");
}

TEST_F(LoggingTest, FormatOrForwardDouble) {
    FormatOrForward format_or_forward{__FILE__, __LINE__};
    EXPECT_EQ(format_or_forward(3.14), "3.14");
}

TEST_F(LoggingTest, FormatOrForwardThreeArgs) {
    FormatOrForward format_or_forward{__FILE__, __LINE__};
    EXPECT_EQ(format_or_forward("{} + {} = {}", 1, 2, 3), "1 + 2 = 3");
}

TEST_F(LoggingTest, FormatOrForwardErrorHandling) {
    // To test runtime error handling, we MUST use fmt::runtime().
    // If we pass a bad string literal directly, fmt::format_string will correctly
    // trigger a compile-time error, preventing the test from even building!

    // Here, we provide two `{}` brackets but only one argument (42).
    std::string bad_format_str = "Value 1: {}, Value 2: {}";

    // Manually pass in a fake file and line number for predictable testing
    FormatOrForward format_or_forward{"src/fake_node.cpp", 101};

    // Execute the formatter. It should catch the format_error internally.
    std::string result = format_or_forward(fmt::runtime(bad_format_str), 42);

    EXPECT_TRUE(result.find("src/fake_node.cpp:101") != std::string::npos);
}

// ============================================================================
// 10. MACRO RUNTIME ERROR HANDLING
// ============================================================================

TEST_F(LoggingTest, MacroRuntimeFormatError) {
    // Verifies that a standard log macro catches format errors and logs them
    std::string bad_format = "Missing arg: {} and {}";
    HB_INFO(nullptr, fmt::runtime(bad_format), "only_one");

    ASSERT_EQ(g_log_records.size(), 1u);
    // Verify the error message contains our custom error prefix
    EXPECT_TRUE(g_log_records[0].message.find("[LOGGING ERROR:") != std::string::npos);
    // Verify it correctly captured the file name from the macro expansion
    EXPECT_TRUE(g_log_records[0].message.find("test_logging.cpp") != std::string::npos);
}

TEST_F(LoggingTest, MacroRuntimeFormatErrorOnce) {
    // Verifies that even in stateful macros, the error is caught and state is respected
    std::string bad_format = "Once error: {}";
    for (int i = 0; i < 3; ++i) {
        HB_ERROR_ONCE(nullptr, fmt::runtime(bad_format));
    }

    // Should only log the error once despite being called in a loop
    ASSERT_EQ(g_log_records.size(), 1u);
    EXPECT_TRUE(g_log_records[0].message.find("[LOGGING ERROR:") != std::string::npos);
    EXPECT_EQ(g_log_records[0].severity, RCUTILS_LOG_SEVERITY_ERROR);
}

TEST_F(LoggingTest, MacroRuntimeFormatErrorThrottle) {
    // Verifies that throttled logs catch errors and respect the timer
    auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
    std::string bad_format = "Throttle error: {}";

    for (int i = 0; i < 3; ++i) {
        HB_WARN_THROTTLE(nullptr, clock, 1000, fmt::runtime(bad_format));
    }

    // Should only log the error once because of the 1000ms throttle
    ASSERT_EQ(g_log_records.size(), 1u);
    EXPECT_TRUE(g_log_records[0].message.find("[LOGGING ERROR:") != std::string::npos);
    EXPECT_EQ(g_log_records[0].severity, RCUTILS_LOG_SEVERITY_WARN);
}

TEST_F(LoggingTest, MacroRuntimeFormatErrorExpression) {
    // Verifies that error handling works inside conditional expression macros
    std::string bad_format = "Expression error: {}";
    bool should_log = true;

    HB_INFO_EXPRESSION(nullptr, should_log, fmt::runtime(bad_format));

    ASSERT_EQ(g_log_records.size(), 1u);
    EXPECT_TRUE(g_log_records[0].message.find("[LOGGING ERROR:") != std::string::npos);

    std::string bad_format2 = "Expression error: {} {}";
    HB_INFO_EXPRESSION(nullptr, should_log, fmt::runtime(bad_format2));

    ASSERT_EQ(g_log_records.size(), 2u);
    EXPECT_TRUE(g_log_records[1].message.find("[LOGGING ERROR:") != std::string::npos);
}

// ============================================================================
// 11. EXPLICIT-LOCATION FREE FUNCTIONS
// ============================================================================

namespace hbl = hatchbed_common::logging;

TEST_F(LoggingTest, LogAtPreformatted) {
    hbl::log_at(nullptr, RCUTILS_LOG_SEVERITY_INFO, "myfile.cpp", "myFunc", 42, "hello");
    ASSERT_EQ(g_log_records.size(), 1u);
    EXPECT_EQ(g_log_records[0].message,  "hello");
    EXPECT_EQ(g_log_records[0].severity, RCUTILS_LOG_SEVERITY_INFO);
    EXPECT_EQ(g_log_records[0].filename, "myfile.cpp");
    EXPECT_EQ(g_log_records[0].function, "myFunc");
    EXPECT_EQ(g_log_records[0].line,     42u);
}

TEST_F(LoggingTest, FreeFunctionAllLevels) {
    hbl::debug(nullptr, "f.cpp", "fn", 1, std::string("dbg"));
    hbl::info (nullptr, "f.cpp", "fn", 2, std::string("inf"));
    hbl::warn (nullptr, "f.cpp", "fn", 3, std::string("wrn"));
    hbl::error(nullptr, "f.cpp", "fn", 4, std::string("err"));
    hbl::fatal(nullptr, "f.cpp", "fn", 5, std::string("fat"));

    ASSERT_EQ(g_log_records.size(), 5u);
    EXPECT_EQ(g_log_records[0].severity, RCUTILS_LOG_SEVERITY_DEBUG);
    EXPECT_EQ(g_log_records[1].severity, RCUTILS_LOG_SEVERITY_INFO);
    EXPECT_EQ(g_log_records[2].severity, RCUTILS_LOG_SEVERITY_WARN);
    EXPECT_EQ(g_log_records[3].severity, RCUTILS_LOG_SEVERITY_ERROR);
    EXPECT_EQ(g_log_records[4].severity, RCUTILS_LOG_SEVERITY_FATAL);
}

TEST_F(LoggingTest, FreeFunctionFmtFormatting) {
    hbl::warn(nullptr, "src.cpp", "myFunc", 99, "val={} str={}", 7, std::string("hi"));
    ASSERT_EQ(g_log_records.size(), 1u);
    EXPECT_EQ(g_log_records[0].message,  "val=7 str=hi");
    EXPECT_EQ(g_log_records[0].severity, RCUTILS_LOG_SEVERITY_WARN);
    EXPECT_EQ(g_log_records[0].filename, "src.cpp");
    EXPECT_EQ(g_log_records[0].function, "myFunc");
    EXPECT_EQ(g_log_records[0].line,     99u);
}

TEST_F(LoggingTest, FreeFunctionExplicitLogger) {
    rclcpp::Logger logger = rclcpp::get_logger("free_fn_test");
    hbl::info(logger, "a.cpp", "fn", 1, std::string("routed"));
    ASSERT_EQ(g_log_records.size(), 1u);
    EXPECT_EQ(g_log_records[0].name, "free_fn_test");
    EXPECT_EQ(g_log_records[0].message, "routed");
}

TEST_F(LoggingTest, FreeFunctionNoLoggerUsesDefault) {
    auto node = std::make_shared<rclcpp::Node>("free_default");
    set_default_logger(node->get_logger());
    g_log_records.clear();

    hbl::info("b.cpp", "fn", 10, std::string("via default"));
    ASSERT_EQ(g_log_records.size(), 1u);
    EXPECT_EQ(g_log_records[0].name,    "free_default");
    EXPECT_EQ(g_log_records[0].message, "via default");
    EXPECT_EQ(g_log_records[0].line,    10u);
}

TEST_F(LoggingTest, FreeFunctionNoLoggerFmt) {
    hbl::warn("c.cpp", "fn", 5, "x={}", 42);
    ASSERT_EQ(g_log_records.size(), 1u);
    EXPECT_EQ(g_log_records[0].message, "x=42");
    EXPECT_EQ(g_log_records[0].severity, RCUTILS_LOG_SEVERITY_WARN);
    EXPECT_EQ(g_log_records[0].filename, "c.cpp");
    EXPECT_EQ(g_log_records[0].line,    5u);
}

TEST_F(LoggingTest, FreeFunctionSeverityFiltering) {
    rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_WARN);
    hbl::debug(nullptr, "f.cpp", "fn", 1, std::string("filtered"));
    hbl::info (nullptr, "f.cpp", "fn", 2, std::string("filtered"));
    EXPECT_EQ(g_log_records.size(), 0u);
    hbl::warn (nullptr, "f.cpp", "fn", 3, std::string("passes"));
    ASSERT_EQ(g_log_records.size(), 1u);
    EXPECT_EQ(g_log_records[0].message, "passes");
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
