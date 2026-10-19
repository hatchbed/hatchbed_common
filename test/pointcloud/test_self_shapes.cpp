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

#include <algorithm>
#include <cmath>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include <hatchbed_common/pointcloud/self_shapes.hpp>

using hatchbed_common::pointcloud::buildSelfShapes;
using hatchbed_common::pointcloud::insideConvexPrism;
using hatchbed_common::pointcloud::SelfShape;
using hatchbed_common::pointcloud::SelfShapeConfig;

namespace {

// Two links. "body" carries collision AND visual geometry, so the collision must win --
// they are deliberately different sizes so a test can tell which one was used. "rails" has
// two separated visual posts and no collision, which is the case that decides whether
// combining fills the gap between them.
constexpr const char* kUrdf = R"(<?xml version="1.0"?>
<robot name="test">
  <link name="body">
    <collision>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry><box size="2 2 1"/></geometry>
    </collision>
    <visual>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry><box size="8 8 1"/></geometry>
    </visual>
  </link>
  <link name="rails">
    <visual>
      <origin xyz="-3 0 1" rpy="0 0 0"/>
      <geometry><box size="0.2 0.2 2"/></geometry>
    </visual>
    <visual>
      <origin xyz="3 0 1" rpy="0 0 0"/>
      <geometry><box size="0.2 0.2 2"/></geometry>
    </visual>
  </link>
  <joint name="rails_joint" type="fixed">
    <parent link="body"/>
    <child link="rails"/>
    <origin xyz="0 0 1" rpy="0 0 0"/>
  </joint>
</robot>)";

void ignore(const std::string&) {}

// Places every shape at the identity, which is what a link sitting at the frame the prisms
// are held in amounts to.
void placeAtIdentity(std::vector<SelfShape>& shapes) {
    for (auto& shape : shapes) {
        ASSERT_TRUE(shape.update([](const std::string&) {
            return std::optional<Eigen::Isometry3f>(Eigen::Isometry3f::Identity());
        }));
    }
}

bool anyContains(const std::vector<SelfShape>& shapes, const Eigen::Vector3f& point) {
    return std::any_of(shapes.begin(), shapes.end(), [&](const SelfShape& shape) {
        return insideConvexPrism(point, shape.prism());
    });
}

SelfShapeConfig noPadding() {
    SelfShapeConfig config;
    config.padding = 0.0f;
    return config;
}

}  // namespace

// The default, and the reason it is the default: collision geometry on a real model is
// frequently the cruder of the two, so taking it alone leaves the difference uncovered.
TEST(BuildSelfShapes, BothCoversTheLargerOfCollisionAndVisual) {
    auto shapes = buildSelfShapes(kUrdf, noPadding(), ignore);
    placeAtIdentity(shapes);

    EXPECT_TRUE(anyContains(shapes, {0.5f, 0.5f, 0.5f}));    // in the 2 x 2 collision box
    EXPECT_TRUE(anyContains(shapes, {3.5f, 3.5f, 0.5f}));    // in the 8 x 8 visual only
    EXPECT_FALSE(anyContains(shapes, {4.5f, 4.5f, 0.5f}));   // outside both
}

TEST(BuildSelfShapes, CollisionOnlyIgnoresTheVisual) {
    SelfShapeConfig config = noPadding();
    config.geometry = SelfShapeConfig::Geometry::Collision;

    auto shapes = buildSelfShapes(kUrdf, config, ignore);
    placeAtIdentity(shapes);

    EXPECT_TRUE(anyContains(shapes, {0.5f, 0.5f, 0.5f}));
    EXPECT_FALSE(anyContains(shapes, {3.5f, 3.5f, 0.5f}));
}

// Collision-only still has to fall back, or a link the model never gave a collision shape --
// which is most of a guardrail -- would not be covered at all.
TEST(BuildSelfShapes, CollisionOnlyFallsBackToVisualWhereThereIsNoCollision) {
    SelfShapeConfig config = noPadding();
    config.geometry = SelfShapeConfig::Geometry::Collision;

    auto shapes = buildSelfShapes(kUrdf, config, ignore);
    placeAtIdentity(shapes);

    EXPECT_TRUE(anyContains(shapes, {-3.0f, 0.0f, 2.0f}));   // a rail post, visual only
}

TEST(BuildSelfShapes, VisualOnlyIgnoresTheCollision) {
    SelfShapeConfig config = noPadding();
    config.geometry = SelfShapeConfig::Geometry::Visual;

    auto shapes = buildSelfShapes(kUrdf, config, ignore);
    placeAtIdentity(shapes);

    // The visual is the larger here, so what it proves is the count: one element, not two.
    EXPECT_EQ(shapes.size(), 3u);
    EXPECT_TRUE(anyContains(shapes, {3.5f, 3.5f, 0.5f}));
}

TEST(BuildSelfShapes, APerLinkGeometryOverrideBeatsTheGlobalOne) {
    SelfShapeConfig config = noPadding();
    config.geometry = SelfShapeConfig::Geometry::Collision;
    config.link_geometry["body"] = SelfShapeConfig::Geometry::Visual;

    auto shapes = buildSelfShapes(kUrdf, config, ignore);
    placeAtIdentity(shapes);

    EXPECT_TRUE(anyContains(shapes, {3.5f, 3.5f, 0.5f}));
}

TEST(BuildSelfShapes, OneShapePerElementByDefault) {
    auto shapes = buildSelfShapes(kUrdf, noPadding(), ignore);
    // The body's collision and its visual, plus one for each rail post.
    EXPECT_EQ(shapes.size(), 4u);

    placeAtIdentity(shapes);
    EXPECT_TRUE(anyContains(shapes, {-3.0f, 0.0f, 2.0f}));   // in a post
    EXPECT_TRUE(anyContains(shapes, {3.0f, 0.0f, 2.0f}));    // in the other
    // Between them, above the body: this is the view a combined link would have taken away.
    EXPECT_FALSE(anyContains(shapes, {0.0f, 0.0f, 2.0f}));
}

TEST(BuildSelfShapes, CombiningALinkFillsBetweenItsElements) {
    SelfShapeConfig config = noPadding();
    config.link_combine["rails"] = true;

    auto shapes = buildSelfShapes(kUrdf, config, ignore);
    EXPECT_EQ(shapes.size(), 3u);

    placeAtIdentity(shapes);
    EXPECT_TRUE(anyContains(shapes, {0.0f, 0.0f, 2.0f}));
}

TEST(BuildSelfShapes, AGroupIsOnePrismAndItsLinksAreNotEvaluatedAgain) {
    SelfShapeConfig config = noPadding();
    config.groups.push_back({"everything", {"body", "rails"}});

    auto shapes = buildSelfShapes(kUrdf, config, ignore);
    ASSERT_EQ(shapes.size(), 1u);
    EXPECT_EQ(shapes.front().name(), "group:everything");
    // One prism, but still one source per member, since the members can move apart.
    EXPECT_EQ(shapes.front().sources().size(), 2u);
}

TEST(BuildSelfShapes, ADisabledLinkContributesNothing) {
    SelfShapeConfig config = noPadding();
    config.disabled_links.insert("rails");

    auto shapes = buildSelfShapes(kUrdf, config, ignore);
    ASSERT_EQ(shapes.size(), 2u);

    placeAtIdentity(shapes);
    EXPECT_FALSE(anyContains(shapes, {-3.0f, 0.0f, 2.0f}));
}

TEST(BuildSelfShapes, PaddingGrowsEveryShape) {
    SelfShapeConfig config = noPadding();
    config.padding = 0.5f;
    // The collision box alone, so what this measures is the padding rather than which of the
    // body's two very differently sized elements happened to cover the point.
    config.geometry = SelfShapeConfig::Geometry::Collision;

    auto shapes = buildSelfShapes(kUrdf, config, ignore);
    placeAtIdentity(shapes);

    // 0.4 outside the 2 x 2 collision box, inside the padding.
    EXPECT_TRUE(anyContains(shapes, {1.4f, 0.0f, 0.5f}));
    EXPECT_FALSE(anyContains(shapes, {1.6f, 0.0f, 0.5f}));
}

TEST(BuildSelfShapes, GarbageDescriptionYieldsNothingRatherThanThrowing) {
    std::string reported;
    auto shapes = buildSelfShapes("not a urdf", noPadding(),
                                  [&](const std::string& warning) { reported = warning; });
    EXPECT_TRUE(shapes.empty());
    EXPECT_FALSE(reported.empty());
}

// The reason the vertices are kept rather than only the prism: a prism carries no record of
// what it was extruded from, so tilting one can only ever grow it, while rebuilding from the
// vertices describes the tilted geometry itself.
TEST(SelfShapeUpdate, TiltingRebuildsRatherThanGrowing) {
    // A tight budget, so any real tilt forces the rebuild rather than the carry-across. It
    // also grows every extent by twice itself, which the heights below account for.
    constexpr float kPadding = 0.01f;
    SelfShapeConfig config = noPadding();
    config.padding = kPadding;
    config.geometry = SelfShapeConfig::Geometry::Collision;  // the 2 x 2 x 1 box alone
    config.disabled_links.insert("rails");

    auto shapes = buildSelfShapes(kUrdf, config, ignore);
    ASSERT_EQ(shapes.size(), 1u);
    SelfShape& body = shapes.front();

    placeAtIdentity(shapes);
    const float upright_height = body.prism().max_z - body.prism().min_z;

    // The 2 x 2 x 1 box laid on its side: 1 tall where it was, 2 wide where it was not.
    Eigen::Isometry3f tipped = Eigen::Isometry3f::Identity();
    tipped.linear() = Eigen::AngleAxisf(static_cast<float>(M_PI_2),
                                        Eigen::Vector3f::UnitY()).toRotationMatrix();
    ASSERT_TRUE(body.update([&](const std::string&) {
        return std::optional<Eigen::Isometry3f>(tipped);
    }));

    const float tipped_height = body.prism().max_z - body.prism().min_z;
    EXPECT_NEAR(upright_height, 1.0f + 2.0f * kPadding, 1e-3f);
    EXPECT_NEAR(tipped_height, 2.0f + 2.0f * kPadding, 1e-3f);
    // Its own top corner, which a rebuild follows and a grown prism would only enclose.
    EXPECT_TRUE(insideConvexPrism({0.4f, 0.0f, 0.9f}, body.prism()));
}

TEST(SelfShapeUpdate, ATurnAboutZIsCarriedAcrossExactly) {
    SelfShapeConfig config = noPadding();
    config.geometry = SelfShapeConfig::Geometry::Collision;  // the 2 x 2 x 1 box alone
    config.disabled_links.insert("rails");

    auto shapes = buildSelfShapes(kUrdf, config, ignore);
    ASSERT_EQ(shapes.size(), 1u);
    SelfShape& body = shapes.front();
    placeAtIdentity(shapes);

    Eigen::Isometry3f turned = Eigen::Isometry3f::Identity();
    turned.linear() = Eigen::AngleAxisf(static_cast<float>(M_PI_4),
                                        Eigen::Vector3f::UnitZ()).toRotationMatrix();
    turned.translation() = Eigen::Vector3f(5.0f, 0.0f, 0.0f);
    ASSERT_TRUE(body.update([&](const std::string&) {
        return std::optional<Eigen::Isometry3f>(turned);
    }));

    // A square turned 45 degrees about its own centre and moved: the corner now reaches
    // sqrt(2) along x from the new centre, and the height is untouched.
    EXPECT_NEAR(body.prism().min_z, 0.0f, 1e-3f);
    EXPECT_NEAR(body.prism().max_z, 1.0f, 1e-3f);
    EXPECT_TRUE(insideConvexPrism({5.0f + 1.3f, 0.0f, 0.5f}, body.prism()));
    EXPECT_FALSE(insideConvexPrism({5.0f + 1.5f, 0.0f, 0.5f}, body.prism()));
}

TEST(SelfShapeUpdate, AMissingTransformLeavesTheShapeAsItWas) {
    auto shapes = buildSelfShapes(kUrdf, noPadding(), ignore);
    ASSERT_FALSE(shapes.empty());
    EXPECT_FALSE(shapes.front().update([](const std::string&) {
        return std::optional<Eigen::Isometry3f>();
    }));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
