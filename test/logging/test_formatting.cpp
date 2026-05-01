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

#include <gtest/gtest.h>

#include <hatchbed_common/logging/formatting.h>

// ============================================================================
// rclcpp::Time
// ============================================================================

TEST(RclcppTime, DefaultFormat) {
    // Zero timestamp
    EXPECT_EQ(fmt::format("{}", rclcpp::Time(0, 0u)), "0.000000000");

    // Whole seconds, no sub-second component
    EXPECT_EQ(fmt::format("{}", rclcpp::Time(5, 0u)), "5.000000000");

    // Sub-second component
    EXPECT_EQ(fmt::format("{}", rclcpp::Time(1, 500000000u)), "1.500000000");

    // Nanoseconds part is zero-padded to exactly 9 digits
    EXPECT_EQ(fmt::format("{}", rclcpp::Time(0, 1u)), "0.000000001");
    EXPECT_EQ(fmt::format("{}", rclcpp::Time(0, 100000000u)), "0.100000000");

    // All 9 nanosecond digits populated
    EXPECT_EQ(fmt::format("{}", rclcpp::Time(1, 123456789u)), "1.123456789");

    // Large timestamp
    EXPECT_EQ(fmt::format("{}", rclcpp::Time(1234567890, 999999999u)), "1234567890.999999999");
}

TEST(RclcppTime, CustomSpec) {
    const rclcpp::Time t(1, 500000000u);  // 1.5 seconds

    // Precision control (fractional seconds as double)
    EXPECT_EQ(fmt::format("{:.3f}", t), "1.500");
    EXPECT_EQ(fmt::format("{:.6f}", t), "1.500000");
    EXPECT_EQ(fmt::format("{:.0f}", rclcpp::Time(2, 0u)), "2");

    // Scientific notation
    EXPECT_EQ(fmt::format("{:e}", t), "1.500000e+00");
    EXPECT_EQ(fmt::format("{:.2e}", t), "1.50e+00");
}

// ============================================================================
// rclcpp::Duration
// ============================================================================

TEST(RclcppDuration, DefaultFormat) {
    // Zero duration
    EXPECT_EQ(fmt::format("{}", rclcpp::Duration(0, 0u)), "0.000000000");

    // Whole seconds
    EXPECT_EQ(fmt::format("{}", rclcpp::Duration(5, 0u)), "5.000000000");

    // Sub-second component
    EXPECT_EQ(fmt::format("{}", rclcpp::Duration(1, 500000000u)), "1.500000000");

    // Nanoseconds padding
    EXPECT_EQ(fmt::format("{}", rclcpp::Duration::from_nanoseconds(1)), "0.000000001");
    EXPECT_EQ(fmt::format("{}", rclcpp::Duration::from_nanoseconds(100000000)), "0.100000000");

    // Large duration
    EXPECT_EQ(fmt::format("{}", rclcpp::Duration(1234567, 0u)), "1234567.000000000");
}

TEST(RclcppDuration, NegativeDuration) {
    // Whole negative seconds
    EXPECT_EQ(fmt::format("{}", rclcpp::Duration::from_nanoseconds(-1000000000LL)), "-1.000000000");

    // Negative duration with sub-second component
    EXPECT_EQ(fmt::format("{}", rclcpp::Duration::from_nanoseconds(-1500000000LL)), "-1.500000000");

    // Sub-second negative: seconds == 0, sign must still appear
    EXPECT_EQ(fmt::format("{}", rclcpp::Duration::from_nanoseconds(-500000000LL)), "-0.500000000");
    EXPECT_EQ(fmt::format("{}", rclcpp::Duration::from_nanoseconds(-1LL)), "-0.000000001");
}

TEST(RclcppDuration, CustomSpec) {
    const rclcpp::Duration d(1, 500000000u);  // 1.5 seconds

    EXPECT_EQ(fmt::format("{:.3f}", d), "1.500");
    EXPECT_EQ(fmt::format("{:.1f}", d), "1.5");
    EXPECT_EQ(fmt::format("{:.6f}", d), "1.500000");

    // Scientific notation
    EXPECT_EQ(fmt::format("{:e}", d), "1.500000e+00");

    // Custom spec on a negative duration
    EXPECT_EQ(fmt::format("{:.2f}", rclcpp::Duration::from_nanoseconds(-1234567890LL)), "-1.23");

    // Large duration with grouping or total width
    EXPECT_EQ(fmt::format("{:10.2f}", rclcpp::Duration(123, 0u)), "    123.00");
}

// ============================================================================
// builtin_interfaces::msg::Time
// ============================================================================

TEST(BuiltinInterfacesTime, DefaultFormat) {
    builtin_interfaces::msg::Time t;

    // Zero timestamp
    t.sec = 0;
    t.nanosec = 0;
    EXPECT_EQ(fmt::format("{}", t), "0.000000000");

    // Whole seconds
    t.sec = 5;
    t.nanosec = 0;
    EXPECT_EQ(fmt::format("{}", t), "5.000000000");

    // Sub-second component
    t.sec = 1;
    t.nanosec = 500000000;
    EXPECT_EQ(fmt::format("{}", t), "1.500000000");

    // Nanoseconds padding
    t.sec = 0;
    t.nanosec = 1;
    EXPECT_EQ(fmt::format("{}", t), "0.000000001");

    t.sec = 0;
    t.nanosec = 100000000;
    EXPECT_EQ(fmt::format("{}", t), "0.100000000");

    // Large timestamp
    t.sec = 2147483647;  // Max int32
    t.nanosec = 999999999;
    EXPECT_EQ(fmt::format("{}", t), "2147483647.999999999");
}

TEST(BuiltinInterfacesTime, CustomSpec) {
    builtin_interfaces::msg::Time t;
    t.sec = 1600000000;
    t.nanosec = 900000000;

    // Precision control
    EXPECT_EQ(fmt::format("{:.2f}", t), "1600000000.90");
    EXPECT_EQ(fmt::format("{:.0f}", t), "1600000001");
}

// ============================================================================
// builtin_interfaces::msg::Duration
// ============================================================================

TEST(BuiltinInterfacesDuration, DefaultFormat) {
    builtin_interfaces::msg::Duration d;

    // Zero
    d.sec = 0; d.nanosec = 0;
    EXPECT_EQ(fmt::format("{}", d), "0.000000000");

    // Positive
    d.sec = 1; d.nanosec = 500000000;
    EXPECT_EQ(fmt::format("{}", d), "1.500000000");

    // Padding
    d.sec = 0; d.nanosec = 123;
    EXPECT_EQ(fmt::format("{}", d), "0.000000123");
}

TEST(BuiltinInterfacesDuration, NegativeDuration) {
    builtin_interfaces::msg::Duration d;

    // Standard ROS 2 normalization for -0.5s:
    // sec: -1, nanosec: 500,000,000
    d.sec = -1; d.nanosec = 500000000;

    // Note: The raw field formatter prints exactly what's in the struct.
    // If you want "pretty" negative math, use the custom spec path below.
    EXPECT_EQ(fmt::format("{}", d), "-1.500000000");
}

TEST(BuiltinInterfacesDuration, CustomSpec) {
    builtin_interfaces::msg::Duration d;
    d.sec = 2; d.nanosec = 750000000;  // 2.75s

    // Precision control via double conversion
    EXPECT_EQ(fmt::format("{:.2f}", d), "2.75");
    EXPECT_EQ(fmt::format("{:.1f}", d), "2.8");

    // Negative Duration Math Check
    // sec: -1, nanosec: 500,000,000 => -0.5s
    d.sec = -1; d.nanosec = 500000000;
    EXPECT_EQ(fmt::format("{:.1f}", d), "-0.5");
}

// ============================================================================
// std_msgs::msg::Header
// ============================================================================

TEST(StdMsgsHeader, DefaultFormat) {
    std_msgs::msg::Header h;

    // Empty Header
    h.frame_id = "";
    h.stamp.sec = 0;
    h.stamp.nanosec = 0;
    EXPECT_EQ(fmt::format("{}", h), "[frame: '', stamp: 0.000000000]");

    // Standard Header
    h.frame_id = "map";
    h.stamp.sec = 1600000000;
    h.stamp.nanosec = 123456789;
    EXPECT_EQ(fmt::format("{}", h), "[frame: 'map', stamp: 1600000000.123456789]");

    // Header with special characters in frame_id
    h.frame_id = "camera/left_link";
    h.stamp.sec = 0;
    h.stamp.nanosec = 500;
    EXPECT_EQ(fmt::format("{}", h), "[frame: 'camera/left_link', stamp: 0.000000500]");
}

TEST(StdMsgsHeader, NestedPointers) {
    auto h = std::make_shared<std_msgs::msg::Header>();
    h->frame_id = "odom";
    h->stamp.sec = 42;
    h->stamp.nanosec = 0;

    // Verifies that the universal pointer formatter correctly
    // delegates to the Header formatter
    EXPECT_EQ(fmt::format("{}", h), "[frame: 'odom', stamp: 42.000000000]");
}

TEST(StdMsgsHeader, CustomSpec) {
    std_msgs::msg::Header h;
    h.frame_id = "base_link";
    h.stamp.sec = 100;
    h.stamp.nanosec = 123456789;

    // Passing precision through the header to the stamp
    EXPECT_EQ(fmt::format("{:.3f}", h), "[frame: 'base_link', stamp: 100.123]");
    EXPECT_EQ(fmt::format("{:.1f}", h), "[frame: 'base_link', stamp: 100.1]");
}

// ============================================================================
// std_msgs::msg::ColorRGBA
// ============================================================================

TEST(StdMsgsColorRGBA, DefaultFormat) {
    std_msgs::msg::ColorRGBA c;

    // Test: Red with full alpha
    c.r = 1.0f; c.g = 0.0f; c.b = 0.0f; c.a = 1.0f;
    EXPECT_EQ(fmt::format("{}", c), "[r: 1.00, g: 0.00, b: 0.00, a: 1.00]");

    // Test: Half-transparent gray
    c.r = 0.5f; c.g = 0.5f; c.b = 0.5f; c.a = 0.5f;
    EXPECT_EQ(fmt::format("{}", c), "[r: 0.50, g: 0.50, b: 0.50, a: 0.50]");

    // Test: Specific rounding (default is .2f)
    c.r = 0.1234f; c.g = 0.5678f; c.b = 0.9012f; c.a = 1.0f;
    EXPECT_EQ(fmt::format("{}", c), "[r: 0.12, g: 0.57, b: 0.90, a: 1.00]");
}

TEST(StdMsgsColorRGBA, CustomSpec) {
    std_msgs::msg::ColorRGBA c;
    c.r = 0.1234f; c.g = 0.5678f; c.b = 0.9012f; c.a = 1.0f;

    // Test: Higher precision
    EXPECT_EQ(fmt::format("{:.3f}", c), "[r: 0.123, g: 0.568, b: 0.901, a: 1.000]");

    // Test: Low precision
    EXPECT_EQ(fmt::format("{:.1f}", c), "[r: 0.1, g: 0.6, b: 0.9, a: 1.0]");

    // Test: Scientific notation (delegation check)
    // Note: fmt::formatter<double> supports 'e'
    std::string s = fmt::format("{:.1e}", c);
    EXPECT_EQ(s, "[r: 1.2e-01, g: 5.7e-01, b: 9.0e-01, a: 1.0e+00]");
}

// ============================================================================
// Eigen: vectors
// ============================================================================

TEST(EigenDenseBase, Vectors) {
    // Standard Column Vector
    Eigen::Vector3d v(1.123, 2.456, 3.789);
    EXPECT_EQ(fmt::format("{}", v), "[1.123, 2.456, 3.789]");
    EXPECT_EQ(fmt::format("{:.1f}", v), "[1.1, 2.5, 3.8]");  // Checks rounding & custom spec

    // Standard Row Vector
    Eigen::RowVector2d rv(4.0, 5.0);
    EXPECT_EQ(fmt::format("{:.2f}", rv), "[4.00, 5.00]");

    // Integer Vector
    Eigen::Vector3i iv(1, -2, 3);
    EXPECT_EQ(fmt::format("{}", iv), "[1, -2, 3]");
    EXPECT_EQ(fmt::format("{:03d}", iv), "[001, -02, 003]");  // Custom int specifier
}

// ============================================================================
// Eigen: matrices
// ============================================================================

TEST(EigenDenseBase, Matrices) {
    Eigen::Matrix2d m;
    m << 1.1, 2.2,
         3.3, 4.4;

    // Note: The formatter inserts a leading newline for matrices
    std::string expected_default =
        "\n[[1.1, 2.2]\n"
          " [3.3, 4.4]]";
    EXPECT_EQ(fmt::format("{}", m), expected_default);

    std::string expected_custom =
        "\n[[1.100, 2.200]\n"
          " [3.300, 4.400]]";
    EXPECT_EQ(fmt::format("{:.3f}", m), expected_custom);

    // Empty Matrix
    Eigen::MatrixXd empty_mat(0, 0);
    EXPECT_EQ(fmt::format("{}", empty_mat), "[]");
}

TEST(EigenDenseBase, Expressions) {
    // Testing that mat.derived() evaluates correctly for Eigen block operations
    Eigen::Matrix3d m = Eigen::Matrix3d::Identity();
    m(0, 2) = 5.5;

    // Format just the top row
    EXPECT_EQ(fmt::format("{:.1f}", m.row(0)), "[1.0, 0.0, 5.5]");

    // Format a 2x2 block
    std::string expected_block =
        "\n[[1.0, 0.0]\n"
          " [0.0, 1.0]]";
    EXPECT_EQ(fmt::format("{:.1f}", m.block<2, 2>(0, 0)), expected_block);
}

TEST(EigenMatrix, Matrix3d) {
    Eigen::Matrix3d m;
    m << 1.1111, 2.2222, 3.3333,
         4.4444, 5.5555, 6.6666,
         7.7777, 8.8888, 9.9999;

    // Updated: Default formatting matches raw values (up to fmt's default precision)
    // Note: No trailing zeros are added in default {} mode.
    std::string expected_default =
        "\n[[1.1111, 2.2222, 3.3333]\n"
        " [4.4444, 5.5555, 6.6666]\n"
        " [7.7777, 8.8888, 9.9999]]";

    EXPECT_EQ(fmt::format("{}", m), expected_default);

    // Custom formatting (1 decimal place) still works as before
    // Rounding: 5.5555 -> 5.6, 9.9999 -> 10.0
    std::string expected_custom =
        "\n[[1.1, 2.2, 3.3]\n"
        " [4.4, 5.6, 6.7]\n"
        " [7.8, 8.9, 10.0]]";

    EXPECT_EQ(fmt::format("{:.1f}", m), expected_custom);
}

TEST(EigenMatrix, Matrix4f) {
    // Testing with float types
    Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
    m(0, 3) = 0.5f;
    m(1, 3) = -1.5f;

    std::string expected =
        "\n[[1.0, 0.0, 0.0, 0.5]\n"
        " [0.0, 1.0, 0.0, -1.5]\n"
        " [0.0, 0.0, 1.0, 0.0]\n"
        " [0.0, 0.0, 0.0, 1.0]]";

    EXPECT_EQ(fmt::format("{:.1f}", m), expected);
}

TEST(EigenMatrix, Empty) {
    // Zero-sized matrix
    Eigen::MatrixXd m_empty(0, 0);
    EXPECT_EQ(fmt::format("{}", m_empty), "[]");

    // Matrix with rows but no columns (still considered empty in our logic)
    Eigen::MatrixXd m_rows_only(3, 0);
    EXPECT_EQ(fmt::format("{}", m_rows_only), "[]");

    // Matrix with columns but no rows
    Eigen::MatrixXd m_cols_only(0, 3);
    EXPECT_EQ(fmt::format("{}", m_cols_only), "[]");
}

// ============================================================================
// Eigen: Transform
// ============================================================================

TEST(EigenTransform, Isometry3d) {
    Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
    iso.translation() << 1.5, -2.5, 3.0;

    // 90 degree rotation around Z axis
    iso.rotate(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));

    // Formats as p: [x,y,z], q: [x,y,z,w]
    EXPECT_EQ(fmt::format("{:.2f}", iso), "t: [1.50, -2.50, 3.00], r: [0.00, 0.00, 0.71, 0.71]");
}

TEST(EigenTransform, Isometry2d) {
    Eigen::Isometry2d iso = Eigen::Isometry2d::Identity();
    iso.translation() << 10.0, 20.0;

    // Rotate 90 degrees
    iso.rotate(Eigen::Rotation2Dd(M_PI / 2.0));

    // Formats as p: [x, y], angle: rad
    EXPECT_EQ(fmt::format("{:.2f}", iso), "t: [10.00, 20.00], angle: 1.57");
}

TEST(EigenTransform, Affine3d) {
    // Affine transforms should bypass the "p, q" formatting and print
    // the full 4x4 matrix to reveal any scaling/shearing.
    Eigen::Affine3d aff = Eigen::Affine3d::Identity();
    aff.translation() << 1.0, 2.0, 3.0;
    aff.scale(2.0);  // Adds uniform scale of 2.0

    std::string expected_matrix =
        "\n[[2.0, 0.0, 0.0, 1.0]\n"
          " [0.0, 2.0, 0.0, 2.0]\n"
          " [0.0, 0.0, 2.0, 3.0]\n"
          " [0.0, 0.0, 0.0, 1.0]]";

    // Use a custom specifier to make string matching perfectly predictable
    EXPECT_EQ(fmt::format("{:.1f}", aff), expected_matrix);
}

TEST(EigenTransform, AffineCompact3d) {
    // AffineCompact3d is stored as a 3x4 matrix (no bottom row [0, 0, 0, 1])
    Eigen::AffineCompact3d ac = Eigen::AffineCompact3d::Identity();
    ac.translation() << 1.0, 2.0, 3.0;
    ac.linear() *= 2.0;  // Apply a scale of 2.0

    // Because Mode != Isometry, it should print the 3x4 matrix
    // Note: Our matrix formatter adds a leading newline
    std::string expected =
        "\n[[2.0, 0.0, 0.0, 1.0]\n"
          " [0.0, 2.0, 0.0, 2.0]\n"
          " [0.0, 0.0, 2.0, 3.0]]";

    EXPECT_EQ(fmt::format("{:.1f}", ac), expected);
}

TEST(EigenTransform, Projective3d) {
    // Projective3d is a general 4x4 matrix, often used for camera projections
    Eigen::Projective3d pr = Eigen::Projective3d::Identity();

    // Manually set some projective values in the bottom row
    pr.matrix()(3, 0) = 0.1;
    pr.matrix()(3, 1) = 0.2;
    pr.translation() << 5.0, 6.0, 7.0;

    std::string expected =
        "\n[[1.0, 0.0, 0.0, 5.0]\n"
          " [0.0, 1.0, 0.0, 6.0]\n"
          " [0.0, 0.0, 1.0, 7.0]\n"
          " [0.1, 0.2, 0.0, 1.0]]";

    EXPECT_EQ(fmt::format("{:.1f}", pr), expected);
}

// ============================================================================
// Eigen: Quaternion
// ============================================================================

TEST(EigenQuaternion, DefaultAndCustom) {
    // Note: Eigen::Quaternion constructor takes (w, x, y, z).
    // Our formatter outputs[x, y, z, w].
    Eigen::Quaterniond q(1.0, 0.1, 0.2, 0.3);  // w=1.0, x=0.1, y=0.2, z=0.3

    // Default format uses hardcoded {:.3f}
    EXPECT_EQ(fmt::format("{}", q), "[0.100, 0.200, 0.300, 1.000]");

    // Custom format delegates to scalar
    EXPECT_EQ(fmt::format("{:.1f}", q), "[0.1, 0.2, 0.3, 1.0]");
}

// ============================================================================
// Eigen: AngleAxis
// ============================================================================

TEST(EigenAngleAxis, DefaultAndCustom) {
    Eigen::AngleAxisd aa(M_PI / 2.0, Eigen::Vector3d::UnitZ());

    // Custom specifier applies to both the angle and the axis vector
    EXPECT_EQ(fmt::format("{:.2f}", aa), "AngleAxis(angle: 1.57, axis: [0.00, 0.00, 1.00])");
    EXPECT_EQ(fmt::format("{:.4f}", aa),
        "AngleAxis(angle: 1.5708, axis: [0.0000, 0.0000, 1.0000])");
}

// ============================================================================
// tf2::Vector3
// ============================================================================

TEST(Tf2Vector3, DefaultSpec) {
    tf2::Vector3 v(1.2345, 2.3456, 3.4567);

    // Default format rounds to 3 decimal places.
    // 1.2345 stores as ~1.23449999... in double, so it rounds down.
    EXPECT_EQ(fmt::format("{}", v), "[1.234, 2.346, 3.457]");

    // Zeros
    tf2::Vector3 zeros(0.0, 0.0, 0.0);
    EXPECT_EQ(fmt::format("{}", zeros), "[0.000, 0.000, 0.000]");
}

TEST(Tf2Vector3, CustomSpec) {
    tf2::Vector3 v(1.2345, 2.3456, 3.4567);

    // Precision control
    EXPECT_EQ(fmt::format("{:.1f}", v), "[1.2, 2.3, 3.5]");
    EXPECT_EQ(fmt::format("{:.5f}", v), "[1.23450, 2.34560, 3.45670]");

    // Integer casting/formatting via general format
    EXPECT_EQ(fmt::format("{:.0f}", v), "[1, 2, 3]");
}

// ============================================================================
// tf2::Quaternion
// ============================================================================

TEST(Tf2Quaternion, DefaultSpec) {
    // tf2::Quaternion constructor takes (x, y, z, w)
    tf2::Quaternion q(0.0, 0.1, 0.2, 0.3);

    EXPECT_EQ(fmt::format("{}", q), "[0.000, 0.100, 0.200, 0.300]");
}

TEST(Tf2Quaternion, CustomSpec) {
    // 90 degree rotation around Z
    tf2::Quaternion q(0.0, 0.0, 0.70710678, 0.70710678);

    EXPECT_EQ(fmt::format("{:.2f}", q), "[0.00, 0.00, 0.71, 0.71]");
    EXPECT_EQ(fmt::format("{:.5f}", q), "[0.00000, 0.00000, 0.70711, 0.70711]");
}

// ============================================================================
// tf2::Transform
// ============================================================================

TEST(Tf2Transform, DefaultSpec) {
    tf2::Transform tf;
    tf.setOrigin(tf2::Vector3(1.5, 2.5, 3.5));
    tf.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

    EXPECT_EQ(fmt::format("{}", tf), "(t: [1.500, 2.500, 3.500], r: [0.000, 0.000, 0.000, 1.000])");
}

TEST(Tf2Transform, CustomSpec) {
    tf2::Transform tf;
    tf.setOrigin(tf2::Vector3(1.1234, 2.2345, 3.3456));
    tf.setRotation(tf2::Quaternion(0.0, 0.0, 0.707106, 0.707106));

    // Custom precision should recursively pass down to the Vector3 and Quaternion formatters
    EXPECT_EQ(fmt::format("{:.2f}", tf), "(t: [1.12, 2.23, 3.35], r: [0.00, 0.00, 0.71, 0.71])");
}

// ============================================================================
// Smart pointers
// ============================================================================

TEST(SharedPtr, NonNull) {
    auto ptr = std::make_shared<tf2::Vector3>(1.0, 2.0, 3.0);

    // Should gracefully dereference and format the underlying object
    EXPECT_EQ(fmt::format("{}", ptr), "[1.000, 2.000, 3.000]");

    // Custom specs should perfectly pass through the pointer wrapper
    EXPECT_EQ(fmt::format("{:.1f}", ptr), "[1.0, 2.0, 3.0]");
}

TEST(SharedPtr, Null) {
    std::shared_ptr<tf2::Vector3> ptr = nullptr;

    // Should catch the nullptr safely without segfaulting
    EXPECT_EQ(fmt::format("{}", ptr), "[null]");

    // Even if a custom spec is provided, a null pointer should just print [null]
    // (This requires `parse` to correctly consume the specifier even if ptr is null)
    EXPECT_EQ(fmt::format("{:.1f}", ptr), "[null]");
}

TEST(SharedPtr, ConstSharedPtr) {
    // Simulating a ROS 2 subscription callback message type: msg::Type::ConstSharedPtr
    std::shared_ptr<const tf2::Vector3> const_ptr =
        std::make_shared<const tf2::Vector3>(4.0, 5.0, 6.0);

    // The `std::remove_const_t` in our formatter should catch this perfectly
    EXPECT_EQ(fmt::format("{}", const_ptr), "[4.000, 5.000, 6.000]");
    EXPECT_EQ(fmt::format("{:.0f}", const_ptr), "[4, 5, 6]");
}

TEST(UniquePtr, NonNull) {
    auto ptr = std::make_unique<tf2::Vector3>(7.0, 8.0, 9.0);

    // Should gracefully dereference
    EXPECT_EQ(fmt::format("{}", ptr), "[7.000, 8.000, 9.000]");
    EXPECT_EQ(fmt::format("{:.2f}", ptr), "[7.00, 8.00, 9.00]");
}

TEST(UniquePtr, Null) {
    std::unique_ptr<tf2::Vector3> ptr = nullptr;

    EXPECT_EQ(fmt::format("{}", ptr), "[null]");
}

// ============================================================================
// Standard library polyfills / fmt/std.h types
// ============================================================================

TEST(StdOptional, HasValue) {
    std::optional<int> opt_int = 42;
    EXPECT_EQ(fmt::format("{}", opt_int), "optional(42)");

    std::optional<double> opt_double = 3.14159;
    // We can also test that custom specifiers pass through if our polyfill/fmt supports it
    // (Native fmt v10+ supports this, but our basic polyfill might just do default formatting.
    // We test the default here to be safe across all versions).
    EXPECT_EQ(fmt::format("{}", opt_double), "optional(3.14159)");
}

TEST(StdOptional, Empty) {
    std::optional<int> opt_empty = std::nullopt;
    EXPECT_EQ(fmt::format("{}", opt_empty), "none");

    // An uninitialized optional should also be 'none'
    std::optional<double> opt_uninit;
    EXPECT_EQ(fmt::format("{}", opt_uninit), "none");
}

TEST(StdTuple, Empty) {
    std::tuple<> empty_tuple;
    EXPECT_EQ(fmt::format("{}", empty_tuple), "()");
}

TEST(StdTuple, SingleElement) {
    std::tuple<int> single_tuple{42};
    EXPECT_EQ(fmt::format("{}", single_tuple), "(42)");
}

TEST(StdTuple, MultipleElements) {
    // Using int, double, and bool ensures perfectly consistent output across
    // both our fmt v8 polyfill and native fmt v10.
    std::tuple<int, double, bool> multi_tuple{1, 2.5, true};
    EXPECT_EQ(fmt::format("{}", multi_tuple), "(1, 2.5, true)");
}

TEST(StdVariant, DefaultSpec) {
    std::variant<int, double> v;

    // Initially holds the first alternative (int) initialized to 0
    EXPECT_EQ(fmt::format("{}", v), "variant(0)");

    // Assign an int
    v = 42;
    EXPECT_EQ(fmt::format("{}", v), "variant(42)");

    // Assign a double
    v = 3.14;
    EXPECT_EQ(fmt::format("{}", v), "variant(3.14)");
}

TEST(StdFilesystemPath, DefaultFormat) {
    // Standard absolute path
    std::filesystem::path p1 = "/opt/ros/humble/setup.bash";
    EXPECT_EQ(fmt::format("{}", p1), "\"/opt/ros/humble/setup.bash\"");

    // Standard relative path
    std::filesystem::path p2 = "config/params.yaml";
    EXPECT_EQ(fmt::format("{}", p2), "\"config/params.yaml\"");

    // Empty path
    std::filesystem::path p_empty = "";
    EXPECT_EQ(fmt::format("{}", p_empty), "\"\"");
}

// ============================================================================

int main(int argc, char ** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
