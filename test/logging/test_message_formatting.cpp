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

#include <hatchbed_common/logging/message_formatting.h>

// ============================================================================
// geometry_msgs::msg::Point
// ============================================================================

TEST(GeometryMsgsPoint, DefaultSpec) {
    geometry_msgs::msg::Point p;
    p.x = 1.1234; p.y = 2.2345; p.z = 3.3456;
    // Default rounds to 3 decimal places
    EXPECT_EQ(fmt::format("{}", p), "[1.123, 2.235, 3.346]");
}

TEST(GeometryMsgsPoint, CustomSpec) {
    geometry_msgs::msg::Point p;
    p.x = 1.1234; p.y = 2.2345; p.z = 3.3456;
    EXPECT_EQ(fmt::format("{:.2f}", p), "[1.12, 2.23, 3.35]");
    EXPECT_EQ(fmt::format("{:.1f}", p), "[1.1, 2.2, 3.3]");
}

// ============================================================================
// geometry_msgs::msg::Vector3
// ============================================================================

TEST(GeometryMsgsVector3, DefaultSpec) {
    geometry_msgs::msg::Vector3 v;
    v.x = -1.0; v.y = 0.5557; v.z = 100.1;
    EXPECT_EQ(fmt::format("{}", v), "[-1.000, 0.556, 100.100]");
}

TEST(GeometryMsgsVector3, CustomSpec) {
    geometry_msgs::msg::Vector3 v;
    v.x = 1.2345; v.y = 6.7891; v.z = 0.0;
    EXPECT_EQ(fmt::format("{:.2f}", v), "[1.23, 6.79, 0.00]");
}

// ============================================================================
// geometry_msgs::msg::Quaternion
// ============================================================================

TEST(GeometryMsgsQuaternion, DefaultSpec) {
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0; q.y = 0.0; q.z = 0.7071; q.w = 0.7071;
    EXPECT_EQ(fmt::format("{}", q), "[0.000, 0.000, 0.707, 0.707]");
}

TEST(GeometryMsgsQuaternion, CustomSpec) {
    geometry_msgs::msg::Quaternion q;
    q.x = 0.123; q.y = 0.456; q.z = 0.789; q.w = 1.0;
    EXPECT_EQ(fmt::format("{:.1f}", q), "[0.1, 0.5, 0.8, 1.0]");
}

// ============================================================================
// geometry_msgs::msg::Pose
// ============================================================================

TEST(GeometryMsgsPose, DefaultSpec) {
    geometry_msgs::msg::Pose p;
    p.position.x = 1.0; p.position.y = 2.0; p.position.z = 3.0;
    p.orientation.w = 1.0;
    // Default nested formatters for Point and Quat are used
    EXPECT_EQ(fmt::format("{}", p),
              "Pose(p: [1.000, 2.000, 3.000], q: [0.000, 0.000, 0.000, 1.000])");
}

TEST(GeometryMsgsPose, CustomSpec) {
    geometry_msgs::msg::Pose p;
    p.position.x = 1.111; p.position.y = 2.222; p.position.z = 3.333;
    p.orientation.x = 0.123; p.orientation.y = 0.456; p.orientation.z = 0.789;
    p.orientation.w = 1.0;

    // Custom spec applies to all 7 double components
    EXPECT_EQ(fmt::format("{:.2f}", p), "Pose(p: [1.11, 2.22, 3.33], q: [0.12, 0.46, 0.79, 1.00])");
}

// ============================================================================
// geometry_msgs::msg::PoseStamped
// ============================================================================

TEST(GeometryMsgsPoseStamped, DefaultSpec) {
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = "map";
    p.pose.position.x = 1.0;
    p.pose.orientation.w = 1.0;

    // Based on the code, output prefix is "Pose("
    EXPECT_EQ(fmt::format("{}", p),
              "Pose([frame: 'map', stamp: 0.000000000] "
              "p: [1.000, 0.000, 0.000], q: [0.000, 0.000, 0.000, 1.000])");
}

TEST(GeometryMsgsPoseStamped, CustomSpec) {
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = "odom";
    p.pose.position.x = 1.55;
    p.pose.orientation.w = 1.0;

    EXPECT_EQ(fmt::format("{:.1f}", p),
              "Pose([frame: 'odom', stamp: 0.000000000] "
              "p: [1.6, 0.0, 0.0], q: [0.0, 0.0, 0.0, 1.0])");
}

// ============================================================================
// geometry_msgs::msg::PoseWithCovariance
// ============================================================================

TEST(GeometryMsgsPoseWithCovariance, DefaultSpec) {
    geometry_msgs::msg::PoseWithCovariance p;
    p.pose.position.x = 1.0;
    p.pose.orientation.w = 1.0;

    EXPECT_EQ(fmt::format("{}", p),
              "Pose(p: [1.000, 0.000, 0.000], q: [0.000, 0.000, 0.000, 1.000], cov:[36 elements])");
}

TEST(GeometryMsgsPoseWithCovariance, CustomSpec) {
    geometry_msgs::msg::PoseWithCovariance p;
    p.pose.position.x = 1.234;
    p.pose.orientation.w = 1.0;

    EXPECT_EQ(fmt::format("{:.2f}", p),
              "Pose(p: [1.23, 0.00, 0.00], q: [0.00, 0.00, 0.00, 1.00], cov: [36 elements])");
}

// ============================================================================
// geometry_msgs::msg::PoseWithCovarianceStamped
// ============================================================================

TEST(GeometryMsgsPoseWithCovarianceStamped, DefaultSpec) {
    geometry_msgs::msg::PoseWithCovarianceStamped p;
    p.header.frame_id = "base_link";
    p.pose.pose.position.z = 5.0;
    p.pose.pose.orientation.w = 1.0;

    EXPECT_EQ(fmt::format("{}", p),
              "Pose([frame: 'base_link', stamp: 0.000000000] p: [0.000, 0.000, 5.000], "
              "q: [0.000, 0.000, 0.000, 1.000], cov: [36 elements])");
}

TEST(GeometryMsgsPoseWithCovarianceStamped, CustomSpec) {
    geometry_msgs::msg::PoseWithCovarianceStamped p;
    p.header.frame_id = "lidar";
    p.pose.pose.position.x = 10.123;
    p.pose.pose.orientation.w = 1.0;

    EXPECT_EQ(fmt::format("{:.1f}", p),
              "Pose([frame: 'lidar', stamp: 0.000000000] p: [10.1, 0.0, 0.0], "
              "q: [0.0, 0.0, 0.0, 1.0], cov: [36 elements])");
}
// ============================================================================
// geometry_msgs::msg::Transform
// ============================================================================

TEST(GeometryMsgsTransform, DefaultSpec) {
    geometry_msgs::msg::Transform tf;
    tf.translation.x = 1.5;
    tf.translation.y = 2.5;
    tf.translation.z = 3.5;
    tf.rotation.x = 0.0;
    tf.rotation.y = 0.0;
    tf.rotation.z = 0.0;
    tf.rotation.w = 1.0;

    EXPECT_EQ(fmt::format("{}", tf),
              "Transform(t: [1.500, 2.500, 3.500], r: [0.000, 0.000, 0.000, 1.000])");
}

TEST(GeometryMsgsTransform, CustomSpec) {
    geometry_msgs::msg::Transform tf;
    tf.translation.x = 1.1234;
    tf.translation.y = 2.2345;
    tf.translation.z = 3.3456;
    tf.rotation.x = 0.0;
    tf.rotation.y = 0.0;
    tf.rotation.z = 0.707106;
    tf.rotation.w = 0.707106;

    EXPECT_EQ(fmt::format("{:.2f}", tf),
              "Transform(t: [1.12, 2.23, 3.35], r: [0.00, 0.00, 0.71, 0.71])");
}

// ============================================================================
// geometry_msgs::msg::TransformStamped
// ============================================================================

TEST(GeometryMsgsTransformStamped, DefaultSpec) {
    geometry_msgs::msg::TransformStamped ts;
    ts.header.frame_id = "map";
    ts.child_frame_id = "odom";
    ts.transform.translation.x = 1.0;
    ts.transform.rotation.w = 1.0;

    // Output: TransformStamped([header], child: 'child', p: [x,y,z], q: [x,y,z,w])
    EXPECT_EQ(fmt::format("{}", ts),
              "Transform([frame: 'map', stamp: 0.000000000] child: 'odom', "
              "t: [1.000, 0.000, 0.000], r: [0.000, 0.000, 0.000, 1.000])");
}

TEST(GeometryMsgsTransformStamped, CustomSpec) {
    geometry_msgs::msg::TransformStamped ts;
    ts.header.frame_id = "world";
    ts.child_frame_id = "base_link";
    ts.transform.translation.x = 1.23456;
    ts.transform.rotation.z = 0.7071;
    ts.transform.rotation.w = 0.7071;

    // Precision propagates to both p and q
    EXPECT_EQ(fmt::format("{:.2f}", ts),
              "Transform([frame: 'world', stamp: 0.000000000] child: 'base_link', "
              "t: [1.23, 0.00, 0.00], r: [0.00, 0.00, 0.71, 0.71])");
}

// ============================================================================
// geometry_msgs::msg::Twist
// ============================================================================

TEST(GeometryMsgsTwist, DefaultSpec) {
    geometry_msgs::msg::Twist t;
    t.linear.x = 1.5;
    t.angular.z = 0.5;

    // Base Twist format: (v: [x,y,z], w: [x,y,z])
    EXPECT_EQ(fmt::format("{}", t), "Twist(v: [1.500, 0.000, 0.000], w: [0.000, 0.000, 0.500])");
}

TEST(GeometryMsgsTwist, CustomSpec) {
    geometry_msgs::msg::Twist t;
    t.linear.x = 1.1111;
    t.angular.z = 2.2222;

    EXPECT_EQ(fmt::format("{:.1f}", t), "Twist(v: [1.1, 0.0, 0.0], w: [0.0, 0.0, 2.2])");
}


// ============================================================================
// geometry_msgs::msg::TwistStamped
// ============================================================================

TEST(GeometryMsgsTwistStamped, DefaultSpec) {
    geometry_msgs::msg::TwistStamped ts;
    ts.header.frame_id = "base_link";
    ts.twist.linear.x = 10.0;

    EXPECT_EQ(fmt::format("{}", ts),
              "Twist([frame: 'base_link', stamp: 0.000000000] v: [10.000, 0.000, 0.000], "
              "w: [0.000, 0.000, 0.000])");
}

TEST(GeometryMsgsTwistStamped, CustomSpec) {
    geometry_msgs::msg::TwistStamped ts;
    ts.header.frame_id = "camera";
    ts.twist.linear.y = 1.234;

    EXPECT_EQ(fmt::format("{:.2f}", ts),
              "Twist([frame: 'camera', stamp: 0.000000000] v: [0.00, 1.23, 0.00], "
              "w: [0.00, 0.00, 0.00])");
}

// ============================================================================
// geometry_msgs::msg::TwistWithCovariance
// ============================================================================

TEST(GeometryMsgsTwistWithCovariance, DefaultSpec) {
    geometry_msgs::msg::TwistWithCovariance twc;
    twc.twist.linear.x = 1.0;

    EXPECT_EQ(fmt::format("{}", twc),
        "Twist(v: [1.000, 0.000, 0.000], w: [0.000, 0.000, 0.000], cov: [36 elements])");
}

TEST(GeometryMsgsTwistWithCovariance, CustomSpec) {
    geometry_msgs::msg::TwistWithCovariance twc;
    twc.twist.linear.z = 0.9876;

    EXPECT_EQ(fmt::format("{:.1f}", twc),
        "Twist(v: [0.0, 0.0, 1.0], w: [0.0, 0.0, 0.0], cov: [36 elements])");
}


// ============================================================================
// geometry_msgs::msg::TwistWithCovarianceStamped
// ============================================================================

TEST(GeometryMsgsTwistWithCovarianceStamped, DefaultSpec) {
    geometry_msgs::msg::TwistWithCovarianceStamped twcs;
    twcs.header.frame_id = "odom";
    twcs.twist.twist.linear.x = 5.5;

    EXPECT_EQ(fmt::format("{}", twcs),
              "Twist([frame: 'odom', stamp: 0.000000000] v: [5.500, 0.000, 0.000], "
              "w: [0.000, 0.000, 0.000], cov: [36 elements])");
}

TEST(GeometryMsgsTwistWithCovarianceStamped, CustomSpec) {
    geometry_msgs::msg::TwistWithCovarianceStamped twcs;
    twcs.header.frame_id = "odom";
    twcs.twist.twist.linear.x = 1.2345;

    EXPECT_EQ(fmt::format("{:.2f}", twcs),
              "Twist([frame: 'odom', stamp: 0.000000000] v: [1.23, 0.00, 0.00], "
              "w: [0.00, 0.00, 0.00], cov: [36 elements])");
}

// ============================================================================
// sensor_msgs::msg::Imu
// ============================================================================

TEST(SensorMsgsImu, DefaultFormat) {
    sensor_msgs::msg::Imu msg;
    msg.header.frame_id = "imu_link";
    msg.header.stamp.sec = 10;
    msg.linear_acceleration.z = 9.81;
    msg.angular_velocity.x = 0.1;

    EXPECT_EQ(fmt::format("{}", msg),
              "Imu([frame: 'imu_link', stamp: 10.000000000] a: [0.000, 0.000, 9.810], "
              "w: [0.100, 0.000, 0.000])");
}

// ============================================================================
// sensor_msgs::msg::Image
// ============================================================================

TEST(SensorMsgsImage, DefaultFormat) {
    sensor_msgs::msg::Image msg;
    msg.header.frame_id = "camera_depth";
    msg.width = 640;
    msg.height = 480;
    msg.encoding = "rgb8";

    EXPECT_EQ(fmt::format("{}", msg),
              "Image([frame: 'camera_depth', stamp: 0.000000000], 640x480, enc: 'rgb8')");
}

// ============================================================================
// sensor_msgs::msg::PointCloud2
// ============================================================================

TEST(SensorMsgsPointCloud2, DefaultFormat) {
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.frame_id = "velodyne";
    msg.width = 1024;
    msg.height = 1;
    msg.is_dense = true;

    EXPECT_EQ(fmt::format("{}", msg),
              "PointCloud2([frame: 'velodyne', stamp: 0.000000000], 1024x1, is_dense: true)");
}

// ============================================================================
// sensor_msgs::msg::LaserScan
// ============================================================================

TEST(SensorMsgsLaserScan, DefaultFormat) {
    sensor_msgs::msg::LaserScan msg;
    msg.header.frame_id = "laser";
    msg.ranges.resize(360);
    msg.range_min = 0.123;
    msg.range_max = 30.0;

    EXPECT_EQ(fmt::format("{}", msg),
              "LaserScan([frame: 'laser', stamp: 0.000000000], pts: 360, range: [0.12, 30.00])");
}

// ============================================================================
// nav_msgs::msg::Odometry
// ============================================================================

TEST(NavMsgsOdometry, DefaultSpec) {
    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = 1.0;
    odom.pose.pose.orientation.w = 1.0;
    odom.twist.twist.linear.x = 0.5;

    std::string expected =
        "\nOdometry([frame: 'odom', stamp: 0.000000000], child: 'base_link',\n"
        "    Pose(p: [1.000, 0.000, 0.000], q: [0.000, 0.000, 0.000, 1.000], cov:[36 elements]),\n"
        "    Twist(v: [0.500, 0.000, 0.000], w: [0.000, 0.000, 0.000], cov: [36 elements]))\n";

    EXPECT_EQ(fmt::format("{}", odom), expected);
}

TEST(NavMsgsOdometry, CustomSpec) {
    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = "map";
    odom.child_frame_id = "robot";
    odom.pose.pose.position.x = 1.111;
    odom.twist.twist.linear.x = 2.222;

    // Verify precision propagates while maintaining the new 4-line structure
    std::string expected =
        "\nOdometry([frame: 'map', stamp: 0.000000000], child: 'robot',\n"
        "    Pose(p:[1.1, 0.0, 0.0], q:[0.0, 0.0, 0.0, 1.0], cov: [36 elements]),\n"
        "    Twist(v:[2.2, 0.0, 0.0], w:[0.0, 0.0, 0.0], cov: [36 elements]))\n";

    EXPECT_EQ(fmt::format("{:.1f}", odom), expected);
}

// ============================================================================
// nav_msgs::msg::Path
// ============================================================================

TEST(NavMsgsPath, DefaultFormat) {
    nav_msgs::msg::Path msg;
    msg.header.frame_id = "map";
    msg.poses.resize(42);

    EXPECT_EQ(fmt::format("{}", msg), "Path([frame: 'map', stamp: 0.000000000] poses: 42)");
}

// ============================================================================
// tf2_msgs::msg::TFMessage
// ============================================================================

TEST(Tf2MsgsTFMessage, DefaultFormat) {
    tf2_msgs::msg::TFMessage msg;
    msg.transforms.resize(3);

    EXPECT_EQ(fmt::format("{}", msg), "TFMessage(3 transforms)");
}

// ============================================================================
// visualization_msgs::msg::Marker
// ============================================================================

TEST(VisualizationMsgsMarker, DefaultFormat) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.ns = "test_ns";
    m.id = 42;
    m.type = visualization_msgs::msg::Marker::CUBE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = 1.0;
    m.scale.x = 0.5; m.scale.y = 0.5; m.scale.z = 0.5;
    m.color.r = 1.0; m.color.a = 1.0;
    m.lifetime.sec = 5;

    std::string expected =
        "\nMarker([frame: 'map', stamp: 0.000000000], ns: 'test_ns', id: 42,\n"
        "    type: CUBE, action: ADD/MOD,\n"
        "    pose: p: [1.000, 0.000, 0.000], q: [0.000, 0.000, 0.000, 1.000],\n"
        "    scale: [0.500, 0.500, 0.500],\n"
        "    color: [r: 1.00, g: 0.00, b: 0.00, a: 1.00],\n"
        "    lifetime: 5.000000000s, locked: false\n"
        ")";

    EXPECT_EQ(fmt::format("{}", m), expected);
}

TEST(VisualizationMsgsMarker, CustomSpec) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "base_link";
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.pose.position.x = 1.234;
    m.pose.orientation.w = 1.0;
    m.color.g = 0.555;
    m.scale.x = 1.111;
    m.lifetime.sec = 2; m.lifetime.nanosec = 500000000;

    std::string expected =
        "\nMarker([frame: 'base_link', stamp: 0.000000000], ns: '', id: 0,\n"
        "    type: SPHERE, action: ADD/MOD,\n"
        "    p:[1.2, 0.0, 0.0], q:[0.0, 0.0, 0.0, 1.0],\n"
        "    scale:[1.1, 0.0, 0.0],\n"
        "    color:[0.0, 0.6, 0.0, 0.0],\n"
        "    lifetime: 2.5s, locked: false\n"
        ")";

    EXPECT_EQ(fmt::format("{:.1f}", m), expected);
}


// ============================================================================
// visualization_msgs::msg::MarkerArray
// ============================================================================

TEST(VisualizationMsgsMarkerArray, DefaultFormat) {}

// ============================================================================

int main(int argc, char ** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
