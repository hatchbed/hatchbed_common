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

#include <chrono>
#include <functional>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <hatchbed_common/param_handler.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace hatchbed_common {
namespace transforms {

/**
 * Bridges two otherwise disconnected TF trees to broadcast parent_frame ->
 * child_frame on the main /tf topic.
 *
 * source_tf_topic carries transforms that are kept off the main /tf tree
 * (typically by remapping /tf to a private topic in the publisher's launch
 * configuration).  The node automatically finds a pivot frame -- any frame
 * reachable from child_frame in one buffer and from parent_frame in the other
 * -- then computes:
 *
 *   T_parent_child = T_parent_pivot * inv(T_child_pivot)
 *
 * Either child_frame or parent_frame may come from the source topic; the node
 * tries both buffers for each endpoint when searching for a pivot.  The only
 * hard requirement is that child_frame must not already have a parent in the
 * main TF tree.
 */
class TfReroot : public rclcpp::Node {
public:
    explicit TfReroot(const rclcpp::NodeOptions& options)
    : Node("tf_reroot", options)
    {
        init_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TfReroot::onInit, this));
    }

private:
    void onInit() {
        init_timer_->cancel();

        params_ = std::make_shared<hatchbed_common::ParamHandler>(shared_from_this());
        params_->register_verbose_logging_param();

        params_->param(&child_frame_, "child_frame", std::string(""),
            "Frame to broadcast as a child of parent_frame.  Must be reachable "
            "via source_tf_topic or already present in the main TF tree.").declare();

        params_->param(&parent_frame_, "parent_frame", std::string(""),
            "Frame that will become the parent of child_frame in the main TF "
            "tree.  Typically already present in the main TF tree.").declare();

        if (child_frame_.empty() || parent_frame_.empty()) {
            RCLCPP_ERROR(get_logger(), "child_frame and parent_frame must both be set.");
            return;
        }

        // Separate buffer fed only from source_tf_topic.
        source_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        source_buffer_->setUsingDedicatedThread(true);

        // Main buffer fed from /tf and /tf_static via TransformListener.
        main_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
        main_listener_ = std::make_shared<tf2_ros::TransformListener>(*main_buffer_, this);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        sub_tf_ = create_subscription<tf2_msgs::msg::TFMessage>(
            "tf_source", rclcpp::QoS(100),
            std::bind(&TfReroot::onSourceTf, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
            "Broadcasting '%s' -> '%s'. Pivot frame will be detected automatically.",
            parent_frame_.c_str(), child_frame_.c_str());
    }

    void onSourceTf(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
        for (const auto& tf : msg->transforms) {
            try {
                source_buffer_->setTransform(tf, "source_tf_topic");
            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN(get_logger(),
                    "Failed to ingest source transform: %s", ex.what());
            }
        }

        if (pivot_frame_.empty() && !detectPivot()) {
            return;
        }

        // Look up child_frame -> pivot from whichever buffer it was found in.
        geometry_msgs::msg::TransformStamped T_child_pivot_msg;
        try {
            auto& buf = child_in_source_ ? source_buffer_ : main_buffer_;
            T_child_pivot_msg = buf->lookupTransform(
                child_frame_, pivot_frame_, tf2::TimePointZero);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_DEBUG(get_logger(),
                "Lost '%s' -> '%s': %s -- re-detecting pivot.",
                child_frame_.c_str(), pivot_frame_.c_str(), ex.what());
            pivot_frame_.clear();
            return;
        }

        // Look up parent_frame -> pivot from whichever buffer it was found in.
        geometry_msgs::msg::TransformStamped T_parent_pivot_msg;
        try {
            auto& buf = parent_in_main_ ? main_buffer_ : source_buffer_;
            T_parent_pivot_msg = buf->lookupTransform(
                parent_frame_, pivot_frame_, tf2::TimePointZero);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_DEBUG(get_logger(),
                "Lost '%s' -> '%s': %s -- re-detecting pivot.",
                parent_frame_.c_str(), pivot_frame_.c_str(), ex.what());
            pivot_frame_.clear();
            return;
        }

        // T_parent_child = T_parent_pivot * inv(T_child_pivot)
        const tf2::Transform T_child_pivot  = fromMsg(T_child_pivot_msg.transform);
        const tf2::Transform T_parent_pivot = fromMsg(T_parent_pivot_msg.transform);
        const tf2::Transform T_parent_child = T_parent_pivot * T_child_pivot.inverse();

        geometry_msgs::msg::TransformStamped out;
        out.header.stamp    = T_child_pivot_msg.header.stamp;
        out.header.frame_id = parent_frame_;
        out.child_frame_id  = child_frame_;
        out.transform       = toMsg(T_parent_child);
        tf_broadcaster_->sendTransform(out);
    }

    // Searches all frames known to either buffer for one that is reachable from
    // child_frame (tried source buffer first, then main) AND from parent_frame
    // (tried main buffer first, then source).  Records which buffer each
    // endpoint was found in and caches the result in pivot_frame_.
    bool detectPivot() {
        const std::vector<std::string> source_frames = source_buffer_->getAllFrameNames();
        const std::vector<std::string> main_frames   = main_buffer_->getAllFrameNames();

        std::set<std::string> all_frames(source_frames.begin(), source_frames.end());
        all_frames.insert(main_frames.begin(), main_frames.end());

        struct Candidate {
            std::string frame;
            bool child_in_source;
            bool parent_in_main;
        };
        std::vector<Candidate> candidates;

        for (const auto& frame : all_frames) {
            if (frame == child_frame_ || frame == parent_frame_) {
                continue;
            }

            // Try child_frame -> frame: source buffer first, then main.
            bool child_found     = false;
            bool child_in_source = true;
            for (bool try_source : {true, false}) {
                auto& buf = try_source ? source_buffer_ : main_buffer_;
                try {
                    buf->lookupTransform(child_frame_, frame, tf2::TimePointZero);
                    child_found     = true;
                    child_in_source = try_source;
                    break;
                } catch (const tf2::TransformException&) {}
            }
            if (!child_found) { continue; }

            // Try parent_frame -> frame: main buffer first, then source.
            bool parent_found   = false;
            bool parent_in_main = true;
            for (bool try_main : {true, false}) {
                auto& buf = try_main ? main_buffer_ : source_buffer_;
                try {
                    buf->lookupTransform(parent_frame_, frame, tf2::TimePointZero);
                    parent_found   = true;
                    parent_in_main = try_main;
                    break;
                } catch (const tf2::TransformException&) {}
            }
            if (!parent_found) { continue; }

            candidates.push_back({frame, child_in_source, parent_in_main});
        }

        if (candidates.empty()) {
            RCLCPP_DEBUG(get_logger(),
                "No pivot frame found yet -- no frame is reachable from both "
                "'%s' and '%s' across the two TF buffers.",
                child_frame_.c_str(), parent_frame_.c_str());
            return false;
        }

        if (candidates.size() > 1) {
            std::string list;
            for (const auto& c : candidates) { list += "'" + c.frame + "' "; }
            RCLCPP_WARN(get_logger(),
                "Multiple pivot frames found: %sUsing '%s'. "
                "Consider verifying your TF setup.",
                list.c_str(), candidates.front().frame.c_str());
        }

        pivot_frame_     = candidates.front().frame;
        child_in_source_ = candidates.front().child_in_source;
        parent_in_main_  = candidates.front().parent_in_main;
        RCLCPP_INFO(get_logger(),
            "Pivot frame detected: '%s' "
            "(child via %s buffer, parent via %s buffer).",
            pivot_frame_.c_str(),
            child_in_source_ ? "source" : "main",
            parent_in_main_  ? "main"   : "source");
        return true;
    }

    static tf2::Transform fromMsg(const geometry_msgs::msg::Transform& t) {
        return tf2::Transform(
            tf2::Quaternion(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w),
            tf2::Vector3(t.translation.x, t.translation.y, t.translation.z));
    }

    static geometry_msgs::msg::Transform toMsg(const tf2::Transform& t) {
        geometry_msgs::msg::Transform msg;
        msg.translation.x = t.getOrigin().x();
        msg.translation.y = t.getOrigin().y();
        msg.translation.z = t.getOrigin().z();
        msg.rotation.x    = t.getRotation().x();
        msg.rotation.y    = t.getRotation().y();
        msg.rotation.z    = t.getRotation().z();
        msg.rotation.w    = t.getRotation().w();
        return msg;
    }

    rclcpp::TimerBase::SharedPtr                   init_timer_;
    std::shared_ptr<hatchbed_common::ParamHandler> params_;

    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_tf_;
    std::shared_ptr<tf2_ros::Buffer>                          source_buffer_;
    std::shared_ptr<tf2_ros::Buffer>                          main_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>               main_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>            tf_broadcaster_;

    std::string child_frame_;
    std::string parent_frame_;
    std::string pivot_frame_;       // detected at runtime; empty until found
    bool        child_in_source_;   // set by detectPivot
    bool        parent_in_main_;    // set by detectPivot
};

}  // namespace transforms
}  // namespace hatchbed_common

RCLCPP_COMPONENTS_REGISTER_NODE(hatchbed_common::transforms::TfReroot)
