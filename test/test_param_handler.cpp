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
#include <hatchbed_common/param_handler.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Set a parameter and return whether the handler accepted it.
template <typename NodeT, typename T>
static bool setParam(std::shared_ptr<NodeT> node, const std::string & name, T value)
{
  auto result = node->set_parameter(rclcpp::Parameter(name, value));
  return result.successful;
}

// ---------------------------------------------------------------------------
// Tests: rclcpp::Node
// ---------------------------------------------------------------------------

TEST(ParamHandlerNode, DeclareBool)
{
  auto node = std::make_shared<rclcpp::Node>("ph_node_bool");
  hatchbed_common::ParamHandler params(node);

  bool val = false;
  params.param(&val, "flag", true, "a bool").declare();
  EXPECT_TRUE(val);
}

TEST(ParamHandlerNode, DeclareInt)
{
  auto node = std::make_shared<rclcpp::Node>("ph_node_int");
  hatchbed_common::ParamHandler params(node);

  int val = 0;
  params.param(&val, "count", 42, "an int").declare();
  EXPECT_EQ(val, 42);
}

TEST(ParamHandlerNode, DeclareDouble)
{
  auto node = std::make_shared<rclcpp::Node>("ph_node_double");
  hatchbed_common::ParamHandler params(node);

  double val = 0.0;
  params.param(&val, "speed", 1.5, "a double").declare();
  EXPECT_DOUBLE_EQ(val, 1.5);
}

TEST(ParamHandlerNode, DeclareString)
{
  auto node = std::make_shared<rclcpp::Node>("ph_node_string");
  hatchbed_common::ParamHandler params(node);

  std::string val;
  params.param(&val, "frame", std::string("base_link"), "a string").declare();
  EXPECT_EQ(val, "base_link");
}

TEST(ParamHandlerNode, DynamicDoubleUpdatesOnSet)
{
  auto node = std::make_shared<rclcpp::Node>("ph_node_dyn_double");
  hatchbed_common::ParamHandler params(node);

  double val = 0.0;
  params.param(&val, "speed", 1.0, "speed").min(0.0).max(10.0).dynamic().declare();
  ASSERT_DOUBLE_EQ(val, 1.0);

  EXPECT_TRUE(setParam(node, "speed", 5.0));
  EXPECT_DOUBLE_EQ(val, 5.0);
}

TEST(ParamHandlerNode, DynamicBoolUpdatesOnSet)
{
  auto node = std::make_shared<rclcpp::Node>("ph_node_dyn_bool");
  hatchbed_common::ParamHandler params(node);

  bool val = false;
  params.param(&val, "enabled", false, "toggle").dynamic().declare();
  ASSERT_FALSE(val);

  EXPECT_TRUE(setParam(node, "enabled", true));
  EXPECT_TRUE(val);
}

TEST(ParamHandlerNode, StaticParamRejectsUpdate)
{
  auto node = std::make_shared<rclcpp::Node>("ph_node_static");
  hatchbed_common::ParamHandler params(node);

  double val = 0.0;
  params.param(&val, "fixed", 3.14, "read-only").declare();
  ASSERT_DOUBLE_EQ(val, 3.14);

  // Static params must not be updated after initialization.
  EXPECT_FALSE(setParam(node, "fixed", 9.99));
  EXPECT_DOUBLE_EQ(val, 3.14);
}

TEST(ParamHandlerNode, MinMaxClampDefault)
{
  auto node = std::make_shared<rclcpp::Node>("ph_node_clamp");
  hatchbed_common::ParamHandler params(node);

  // Default value above max should be clamped to max at declare time.
  double val = 0.0;
  params.param(&val, "rate", 200.0, "rate").min(0.0).max(100.0).declare();
  EXPECT_DOUBLE_EQ(val, 100.0);
}

TEST(ParamHandlerNode, MinMaxRejectsDynamicOutOfRange)
{
  auto node = std::make_shared<rclcpp::Node>("ph_node_range");
  hatchbed_common::ParamHandler params(node);

  double val = 0.0;
  params.param(&val, "gain", 1.0, "gain").min(0.0).max(5.0).dynamic().declare();

  EXPECT_FALSE(setParam(node, "gain", 10.0));
  EXPECT_DOUBLE_EQ(val, 1.0);  // unchanged
}

TEST(ParamHandlerNode, OwnedStoreReturnsDeclaredValue)
{
  auto node = std::make_shared<rclcpp::Node>("ph_node_owned");
  hatchbed_common::ParamHandler params(node);

  auto declared = params.param("threshold", 0.5, "threshold").declare();
  EXPECT_DOUBLE_EQ(declared.value(), 0.5);
}

// ---------------------------------------------------------------------------
// Tests: rclcpp_lifecycle::LifecycleNode
// ---------------------------------------------------------------------------

TEST(ParamHandlerLifecycle, DeclareBool)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("ph_lc_bool");
  hatchbed_common::ParamHandler params(node);

  bool val = false;
  params.param(&val, "flag", true, "a bool").declare();
  EXPECT_TRUE(val);
}

TEST(ParamHandlerLifecycle, DeclareDouble)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("ph_lc_double");
  hatchbed_common::ParamHandler params(node);

  double val = 0.0;
  params.param(&val, "speed", 2.5, "a double").declare();
  EXPECT_DOUBLE_EQ(val, 2.5);
}

TEST(ParamHandlerLifecycle, DeclareString)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("ph_lc_string");
  hatchbed_common::ParamHandler params(node);

  std::string val;
  params.param(&val, "frame", std::string("odom"), "a string").declare();
  EXPECT_EQ(val, "odom");
}

TEST(ParamHandlerLifecycle, DynamicDoubleUpdatesOnSet)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("ph_lc_dyn_double");
  hatchbed_common::ParamHandler params(node);

  double val = 0.0;
  params.param(&val, "speed", 1.0, "speed").min(0.0).max(10.0).dynamic().declare();
  ASSERT_DOUBLE_EQ(val, 1.0);

  EXPECT_TRUE(setParam(node, "speed", 7.0));
  EXPECT_DOUBLE_EQ(val, 7.0);
}

TEST(ParamHandlerLifecycle, StaticParamRejectsUpdate)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("ph_lc_static");
  hatchbed_common::ParamHandler params(node);

  double val = 0.0;
  params.param(&val, "fixed", 3.14, "read-only").declare();
  ASSERT_DOUBLE_EQ(val, 3.14);

  EXPECT_FALSE(setParam(node, "fixed", 9.99));
  EXPECT_DOUBLE_EQ(val, 3.14);
}

TEST(ParamHandlerLifecycle, MinMaxRejectsDynamicOutOfRange)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("ph_lc_range");
  hatchbed_common::ParamHandler params(node);

  double val = 0.0;
  params.param(&val, "gain", 1.0, "gain").min(0.0).max(5.0).dynamic().declare();

  EXPECT_FALSE(setParam(node, "gain", 10.0));
  EXPECT_DOUBLE_EQ(val, 1.0);
}

TEST(ParamHandlerLifecycle, OwnedStoreReturnsDeclaredValue)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("ph_lc_owned");
  hatchbed_common::ParamHandler params(node);

  auto declared = params.param("threshold", 0.75, "threshold").declare();
  EXPECT_DOUBLE_EQ(declared.value(), 0.75);
}

// ---------------------------------------------------------------------------

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
