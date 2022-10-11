#include <chrono>
#include <string>

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
    std::string frame_id = params.param("frame_id", std::string("base_link"), "TF frame").declare().value();
    bool debug = params.param("debug", false, "Enable debug mode").declare().value();
    double threshold = params.param("threshold", 0.75, "Threshold value").min(0.0).max(1.0).declare().value();

    // Sub-grouped
    double coeff_a = params.param("internal.coeff_a", 1.0, "Coefficient A").min(0.0).max(1.0).declare().value();
    double coeff_b = params.param("internal.coeff_b", 0.5, "Coefficient B").min(0.0).max(1.0).declare().value();
    double coeff_c = params.param("internal.coeff_c", 55.47, "Coefficient C").min(-100.0).max(100.0).declare().value();

    //
    // Dynamic parameters
    //
    auto enable_feedback = params.param("enable_feedback", false, "Enable feedback").dynamic().declare();
    int mode = params.param("mode", 0, "Operating mode").enumerate({
        {0, "Default", "Default operating mode"},
        {1, "Advanced", "Advanced operating mode"},
        {2, "Legacy", "Legacy operating mode"}}).dynamic().declare().value();
    auto target_exposure = params.param("exposure", 2000.0, "Exposure (microseconds)").min(0.0).max(10000.0).callback([&](double value){
        RCLCPP_INFO(node->get_logger(), "Handling change in exposure parameter.  New value is: %lf microseconds", value);
    }).declare();

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
