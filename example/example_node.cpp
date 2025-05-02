#include <chrono>
#include <string>
#include <vector>

#include <hatchbed_common/param_handler.h>
#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("param_handler_example");

    // The param handler needs to be initialized with a node handle
    hatchbed_common::ParamHandler params(node);

    // add 'verbose' dynamic parameter to handle enabling debug log level
    params.register_verbose_logging_param();

    //
    // Static parameters
    //
    int num_tries = params.param("num_tries", 1, "Number of tries").min(1).max(50).declare().value();
    // When using the short form (non pointer overload) for int64_t, you have to specify the value as a 64 bit integer 
    // (using L or LL suffix) if using literals. Otherwise the value will be interpreted as an int
    int64_t num_tries_long = params.param("num_tries_long", 1L, "Number of tries").min(1).max(50).declare().value();
    std::vector<int64_t> num_tries_list = params.param("num_tries_list", std::vector<int64_t>{1, 2, 3}, "Number of tries list").min(0).max(10).declare().value();
    std::string frame_id = params.param("frame_id", std::string("base_link"), "TF frame").declare().value();
    std::vector<std::string> topic_list = params.param("topic_list", std::vector<std::string>{"/topic1", "topic2", "/namespace/topic3"}, "Topic list").declare().value();
    bool debug = params.param("debug", false, "Enable debug mode").declare().value();
    std::vector<bool> debug_list = params.param("debug_list", std::vector<bool>{true, false, true}, "Enable debug mode list").declare().value();
    double threshold = params.param("threshold", 0.75, "Threshold value").min(0.0).max(1.0).declare().value();
    std::vector<double> thresholds = params.param("thresholds", std::vector<double>{0.75, 0.1, 0.24}, "Thresholds").min(0.0).max(1.0).declare().value();

    // Sub-grouped
    double coeff_a = params.param("internal.coeff_a", 1.0, "Coefficient A").min(0.0).max(1.0).declare().value();
    double coeff_b = params.param("internal.coeff_b", 0.5, "Coefficient B").min(0.0).max(1.0).declare().value();
    double coeff_c = params.param("internal.coeff_c", 55.47, "Coefficient C").min(-100.0).max(100.0).declare().value();

    //
    // Dynamic parameters
    //
    auto enable_feedback = params.param("enable_feedback", false, "Enable feedback").dynamic().declare();
    int even_numbers = params.param("even_numbers", 0, "Even numbers").min(0).max(20).step(2).dynamic().declare().value();
    int mode = params.param("mode", 0, "Operating mode").enumerate({
        {0, "Default", "Default operating mode"},
        {1, "Advanced", "Advanced operating mode"},
        {2, "Legacy", "Legacy operating mode"}}).dynamic().declare().value();
    std::vector<int64_t> mode_list = params.param("mode_list", std::vector<int64_t>{0, 1, 2}, "Operating mode list").enumerate({
        {0, "Default", "Default operating mode"},
        {1, "Advanced", "Advanced operating mode"},
        {2, "Legacy", "Legacy operating mode"}}).dynamic().declare().value();
    auto target_exposure = params.param("exposure", 2000.0, "Exposure (microseconds)").min(0.0).max(10000.0).step(0.01).callback([&](double value){
        RCLCPP_INFO(node->get_logger(), "Handling change in exposure parameter.  New value is: %lf microseconds", value);
    }).declare();
    std::vector<double> thresholds_dynamic = params.param("thresholds_dynamic",
                                                          std::vector<double>{0.85, 0.9, 0.03},
                                                          "A list of dynamic thresholds").min(0.0).max(1.0).step(0.01).dynamic().declare().value();

    target_exposure.update(500);

    auto timer = node->create_wall_timer(1.0s, [&](){
        RCLCPP_DEBUG(node->get_logger(), "Debug log message.");
        if (enable_feedback.value()) {
            RCLCPP_INFO(node->get_logger(), "Feedback is currently enabled.");
        }
        else {
            RCLCPP_INFO(node->get_logger(), "Feedback is currently not enabled.");

        }
    });

    rclcpp::spin(node);

    return 0;
}
