# hatchbed_common

Common Hatchbed C++ utility code for ROS, such registering and handling updates to ros parameters.

## Param Handler

The functionality and design are similar to [ddynamic_reconfigure](https://github.com/pal-robotics/ddynamic_reconfigure)
and [swri_roscpp](https://github.com/swri-robotics/marti_common/tree/master/swri_roscpp#dynamic-parameters)
in that dynamic parameters can be created and managed programatically without
needing to define a .cfg file.

The objectives are to:
  - minimize boilerplate code for defining and accessing parameters
  - support code clarity when defining parameters
  - provide a similar interface for both static and dynamic parameters
  - provide a similar interface for both ros1 and ros2
  - add minor quality of life improvements like:
     - logging parameter values at startup and on change
     - enforcing range constraints
     - publishing static (readonly) parameters to dynamic reconfig for easier runtime inspection

### API

The `ParamHandler` is a convenience class for managing static and dynamic ROS 
parameters.  It will automatically send parameter config description messages
when new parameters are registered with the handler and will handle receiving
and sending parameter updates.

Both static and dynamic parameters are included in the config description,
but static parameters will be labeled as '(readonly)' and prevent any updates
that might come in for them.

#### Registering Parameters

When registering a new parameter the param handler will return a parameter
object which can be used to access the parameter value in a thread safe way.

All parameters require a name, default value, and description.

Optionally, a pointer to an existing variable can be passed in when registering
a parameter.  In this case that variable is used to store the parameter value, 
but access to it is not protected, so should only be used in single threaded
applications.
 
When registering a parameter it is possible to chain additional configuration
items to the parameter, such as:
  - `.callback()`: provide a callback function when the parameter changes, implies `.dynamic()`
  - `.dynamic()`: allow the parameter to by modified with dynamic reconfig
  - `.enum()`: specify an enumeration for integer parameters
  - `.max()`: specify a maximum value for numeric parameters
  - `.min()`: specify a minimun value for numeric parameters
  - `.step()`: specify a step size for numeric parameters

Once the parameter has been configured, it's necessary to call the `.declare()` method.

#### Static Parameters

For static parameters it's generally sufficient to just immediately store the 
value using the `.value()` method.

```
auto node = std::make_shared<rclcpp::Node>("param_handler_example");
hatchbed_common::ParamHandler params(node);

// integer parameter
int num_tries = params.param("num_tries", 1, "Number of tries").min(1).max(50).declare().value();

// integer array parameter
std::vector<int64_t> int_params = params.param("int_params", std::vector<int64_t>{1, 2, 3}, "Integer array").declare().value();

// string parameter
std::string frame_id = params.param("frame_id", std::string("base_link"), "TF frame").declare().value();

// string array parameter
std::vector<std::string> string_params = params.param("string_params", std::vector<std::string>{"a", "b", "c"}, "String array").declare().value();

// bool parameter
bool debug = params.param("debug", false, "Enable debug mode").value();

// bool array parameter
std::vector<bool> bool_params = params.param("bool_params", std::vector<bool>{true, false, true}, "Boolean array").declare().value();

// double parameter
double threshold = params.param("threshold", 0.75, "Threshold value").min(0.0).max(1.0).declare().value();

// double array parameter
std::vector<double> double_params = params.param("double_params", std::vector<double>{0.1, 0.2, 0.3}, "Double array").declare().value();

// enum parameter
int mode = params.param("mode", 0, "Operating mode").enumerate({
    {0, "Default", "Default operating mode"},
    {1, "Advanced", "Advanced operating mode"},
    {20, "Legacy", "Legacy operating mode"}}).declare().value();
```

#### Dynamic Parameters

For dynamic parameters, there are several options.

In a single threaded use case it's possible to pass in a pointer to where the 
parameter should be stored:

```
int num_tries = 0;
params.param(&num_tries, "num_tries", 1, "Number of tries").min(1).max(50).dynamic().declare();

while (rclcpp::ok()) {
    process.execute(num_tries);
    rclcpp::spin_some(node);
}

```

Here the `num_tries` int variable will be automatically updated.

When multi-threading is involved the above method is not recommended.  Instead
the parameter object returned by the handler should be used to ensure thread-safe
data access.

```
auto num_tries = params.param("num_tries", 1, "Number of tries").min(1).max(50).dynamic().declare();

std::thread t([&](){
    while (rclcpp::ok()) {
        process.execute(num_tries.value());
    }
});

rclcpp::spin(node);
t.join();

```

The different parameter types are:
  - `hatchbed_common::BoolParameter`
  - `hatchbed_common::DoubleParameter`
  - `hatchbed_common::IntParameter`
  - `hatchbed_common::StringParameter`

In addition to accessing the current value, the parameter object can be used to 
publish an update to the parameter using the `.update()` method.

In some cases a direct callback may be desired to notify the process
that the value has changed:

```
params.param("num_tries", 1, "Number of tries").min(1).max(50).callback([](int value){
    process.setNumTries(value);
}).declare();

while (rclcpp::ok()) {
    process.exectute();
    rclcpp::spin_some(node);
}
```

Finally, it's possible to register a callback to notify the process that any of the registered 
dynamic parameters has been updated:

```
    params.setCallback([]() {
        // do something
    });
```

These different approaches are not mutually exclusive and can be used in concert.

#### Differences Between ROS1 and ROS2
 - ros2: `.declare()` must be called after configuring a parameter
 - ros2: there is no `.group()` configuration
 - ros2: parameters are ordered alphabetically in dynamic_reconfigure
 - ros2: `.register_verbose_logging_param()` helper function added to enable dynamic parameter for log-devel
 - ros1: there is no `.step()` configuration for numeric parameters
 - ros1: parameters are ordered in configuration order in dynamic_reconfigure

 ## Simple Profiler

 The `simple_profiler.h` header file provides a simple, low overhead, time profiling utility.

 ```

// enable/disable profiling.
::profile::set_enabled(true);

// start a new named scope at current level
profile_scope("new_scope");

// do work

// open a new scope layer
::profile::push();

// start a child scope
profile_scope("child_scope1");

// do work

// start a second child scope, stopping previous scope: "child_scope1".
profile_scope("child_scope2");

// do work

// close the current scope layer, stopping any open child scope
::profile::pop();

// do work

// start a new scope, stopping "new_scope".
profile_scope("new_scope2");

// do work

// close any open scopes and increment count
::profile::finalize();

// print timings
std::cout << ::profile::get();
 ```

Profile scopes will stop automatically when:
 - going out of the scope they were executed in
 - a new scope on the same level is opened
 - the current scope level is closed with a `::profile::pop()`
 - `::profile::finalize()` is called

Calling `::profile::finalize()` will increment the timing count for tracking the average timings
over multiple iterations.

Calling `::profile::set_enabled(true)` can dynamically enabled or disable the profiling, though
even with disabled a single comparison is made in each call to check the state of this flag. Most of
this small overhead can be removed by setting the `-DPROFILE_DISABLED` compiler flag.

Profiling is performed in a thread_local manner, so it is thread-safe, but timings aren't
aggregated across threads.

The timing profile is stream formatted like this:

```
timing profile (ms)   elapsed     avg
=====================================
handle_points ........ 282.22  239.91
| get_transforms ....... 0.05    0.05
| transform ............ 2.33    2.24
| fit ................ 279.70  237.48
| | sample ............. 3.21    3.07
| | index .............. 3.53    3.16
| | init ............... 0.41    0.35
| | setup ............. 28.05   25.90
| |_solve ............ 205.69  168.50
|_publish .............. 0.01    0.01

```