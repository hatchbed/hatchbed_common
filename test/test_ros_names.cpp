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

#include <gtest/gtest.h>
#include <hatchbed_common/ros_names.h>

using hatchbed_common::isValidParamName;

// ---------------------------------------------------------------------------
// Valid names
// ---------------------------------------------------------------------------

TEST(IsValidParamName, SingleLetter) {
    EXPECT_TRUE(isValidParamName("a"));
    EXPECT_TRUE(isValidParamName("Z"));
}

TEST(IsValidParamName, LeadingUnderscore) {
    EXPECT_TRUE(isValidParamName("_"));
    EXPECT_TRUE(isValidParamName("_name"));
}

TEST(IsValidParamName, TypicalNames) {
    EXPECT_TRUE(isValidParamName("wheel_odom"));
    EXPECT_TRUE(isValidParamName("imu"));
    EXPECT_TRUE(isValidParamName("sensor_0"));
    EXPECT_TRUE(isValidParamName("MyInput"));
    EXPECT_TRUE(isValidParamName("input_A1_b2"));
}

TEST(IsValidParamName, MaxLength) {
    // 255 characters is the maximum allowed length.
    EXPECT_TRUE(isValidParamName(std::string(255, 'a')));
}

// ---------------------------------------------------------------------------
// Invalid names
// ---------------------------------------------------------------------------

TEST(IsValidParamName, EmptyString) {
    EXPECT_FALSE(isValidParamName(""));
}

TEST(IsValidParamName, ExceedsMaxLength) {
    EXPECT_FALSE(isValidParamName(std::string(256, 'a')));
}

TEST(IsValidParamName, StartsWithDigit) {
    EXPECT_FALSE(isValidParamName("0name"));
    EXPECT_FALSE(isValidParamName("9"));
}

TEST(IsValidParamName, ContainsDot) {
    EXPECT_FALSE(isValidParamName("wheel.odom"));
    EXPECT_FALSE(isValidParamName(".name"));
    EXPECT_FALSE(isValidParamName("name."));
}

TEST(IsValidParamName, ContainsForwardSlash) {
    EXPECT_FALSE(isValidParamName("wheel/odom"));
    EXPECT_FALSE(isValidParamName("/name"));
}

TEST(IsValidParamName, ContainsHyphen) {
    EXPECT_FALSE(isValidParamName("wheel-odom"));
}

TEST(IsValidParamName, ContainsSpace) {
    EXPECT_FALSE(isValidParamName("wheel odom"));
    EXPECT_FALSE(isValidParamName(" name"));
}

TEST(IsValidParamName, ContainsTilde) {
    EXPECT_FALSE(isValidParamName("~name"));
    EXPECT_FALSE(isValidParamName("na~me"));
}

TEST(IsValidParamName, ContainsCurlyBrace) {
    EXPECT_FALSE(isValidParamName("{name}"));
    EXPECT_FALSE(isValidParamName("na{me"));
}
