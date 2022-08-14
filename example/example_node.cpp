#include <chrono>
#include <string>

#include <param_util/param_handler.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "param_util_example");
    ros::NodeHandle priv("~");

    // The param handler needs to be initialized with a private node handle to
    // correctly namespace the dynamic reconfig topics.  It's also possible to
    // provide a handle to sub-namespaces of the private namespace.
    param_util::ParamHandler params(priv);

    //
    // Static parameters
    //
    int num_tries = params.param("num_tries", 1, "Number of tries").min(1).max(50).value();
    std::string frame_id = params.param("frame_id", std::string("base_link"), "TF frame").value();
    bool debug = params.param("debug", false, "Enable debug mode").value();
    double threshold = params.param("threshold", 0.75, "Threshold value").min(0.0).max(1.0).value();
    int mode = params.param("mode", 0, "Operating mode").enumerate({
        {0, "Default", "Default operating mode"},
        {1, "Advanced", "Advanced operating mode"},
        {20, "Legacy", "Legacy operating mode"}}).value();

    // Sub-grouped
    double coeff_a = params.param("coeff_a", 1.0, "Coefficient A").group("internal").min(0.0).max(1.0).value();
    double coeff_b = params.param("coeff_b", 0.5, "Coefficient B").group("internal").min(0.0).max(1.0).value();
    double coeff_c = params.param("coeff_c", 55.47, "Coefficient C").group("internal").min(-100.0).max(100.0).value();

    //
    // Dynamic parameters
    //
    auto enable_feedback = params.param("enable_feedback", false, "Enable feedback").dynamic();
    auto target_exposure = params.param("exposure", 2000.0, "Exposure (microseconds)").min(0.0).max(1000.0).callback([](double value){
        ROS_INFO("Handling change in exposure parameter.  New value is: %lf microseconds", value);
    });

    ros::Timer timer = priv.createTimer(ros::Duration(1.0), [&](const ros::TimerEvent& e){
        if (enable_feedback.value()) {
            ROS_INFO("Feedback is currently enabled.");
        }
        else {
            ROS_INFO("Feedback is currently not enabled.");
        }
    });

    ros::spin();

    return 0;
}
