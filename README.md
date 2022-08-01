# param_util

Utility class for registering and handling updates to ros2 parameters.

This is a header only library which provides the ParamHandler class.  The
primary purpose is to reduce boilerplate code related to:
  - declaring parameters
  - defining parameter descriptions
  - setting up parameters for dynamic update
  - setting range limits on dynamic parameters
  - adding a 'verbose' parameter to dynamically turn on and off the debug log
    level

## Static Parameters

Static parameters can be registered and loaded with the following function

```
  /**
   * Register a non-dynamic parameter and return it's value.
   *
   * @param[in] name         Parameter name
   * @param[in] default_val  Default parameter value
   * @param[in] description  Parameter description
   *
   * @returns the value of the parameter.
   */
  template <class T>
  T ParamHandler::param(const std::string& name, const T& default_val, const std::string& description);
```

Even though the parameter is configured to be read only, it will still show up in rqt_reconfigure where the description
can be accessed as a tooltip.

## Dynamic Parameters

Dynamic parameters can be registered in a similar way, but instead of returning the current value, a pointer to where
the parameter should be stored is passed in.   ParamHandler will then handled the callbacks for parameter updates and
update the variable referenced by the pointer.

Note: ParamHandler assumes single threaded access to the provided parameter storage variable.

```
  /**
   * Register a dynamic parameter without any range constraint and populate
   * the parameter variable with the current value.
   *
   * @param[out] param       Reference to parameter variable
   * @param[in] name         Parameter name
   * @param[in] default_val  Default parameter value
   * @param[in] description  Parameter description
   */
  template <class T>
  void register_param(T* param, const std::string& name, const T& default_val, const std::string& description)
```

Range constraints can also be specified for `int` and `double` parameters.  See the examples below.

## Example Usage:
```
#include <param_util/param_handler.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("node_name");
  auto params = std::make_shared<param_util::ParamHandler>(node);

  // declare and get static parameters
  int int_param = params->param("int_param_name", 10, "description of parameter");
  double double_param = params->param("double_param_name", 10.0, "description of parameter");
  bool bool_param = params->param("bool_param_name", false, "description of parameter");
  std::string string_param = params->param("string_param_name", std::string("default"), "description of parameter");

  // register dynamic parameters
  int dyn_int_param;
  params->register(&dyn_int_param, "dyn_int_param_name", 55, "description of parameter");

  double dyn_double_param;
  params->register(&dyn_double_param, "dyn_double_param_name", 0.1, "description of parameter");

  double dyn_bool_param;
  params->register(&dyn_bool_param, "dyn_bool_param_name", true, "description of parameter");

  // dynamic parameters can have range constraints with min, max, and step parameters
  int int_range_param;
  params->register(&int_range_param, "int_range_param_name", 55, "description of parameter", 0, 100, 1);

  double double_range_param;
  params->register(&double_range_param, "double_range_param_name", 0.1, "description of parameter", -1.0, 1.0, 0.1);

  rclcpp::spin();
  return 0;

```