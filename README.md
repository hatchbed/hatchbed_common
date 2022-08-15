# param_util

Utility C++ code for registering and handling updates to ROS parameters.

## Motivation

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

## API

The `ParamHandler` is a convenience class for managing static and dynamic ROS 
parameters.  It will automatically send parameter config description messages
when new parameters are registered with the handler and will handle receiving
and sending parameter updates.

Both static and dynamic parameters are included in the config description,
but static parameters will be labeled as '(readonly)' and prevent any updates
that might come in for them.

### Registering Parameters

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
  - `.group()`: place the parameter in a sub-group
  - `.max()`: specify a maximum value for numeric parameters
  - `.min()`: specify a minimun value for numeric parameters

### Static Parameters

For static parameters it's generally sufficient to just immediately store the 
value using the `.value()` method.

```
param_util::ParamHandler params(ros::NodeHandle("~"));

// integer parameter
int num_tries = params.param("num_tries", 1, "Number of tries").min(1).max(50).value();

// string parameter
std::string frame_id = params.param("frame_id", std::string("base_link"), "TF frame").value();

// bool parameter
bool debug = params.param("debug", false, "Enable debug mode").value();

// double parameter
double threshold = params.param("threshold", 0.75, "Threshold value").min(0.0).max(1.0).value();

// enum parameter
int mode = params.param("mode", 0, "Operating mode").enumerate({
    {0, "Default", "Default operating mode"},
    {1, "Advanced", "Advanced operating mode"},
    {20, "Legacy", "Legacy operating mode"}}).value();
```

### Dynamic Parameters

For dynamic parameters, there are several options.

In a single threaded use case it's possible to pass in a pointer to where the 
parameter should be stored:

```
int num_tries = 0;
params.param(&num_tries, "num_tries", 1, "Number of tries").min(1).max(50).dynamic();

while (ros::ok()) {
    process.execute(num_tries);
    ros::spinOnce();
}

```

Here the `num_tries` int variable will be automatically updated.

When multi-threading is involved the above method is not recommended.  Instead
the parameter object returned by the handler should be used to ensure thread-safe
data access.

```
auto num_tries = params.param("num_tries", 1, "Number of tries").min(1).max(50).dynamic();

std::thread t([&](){
    while (ros::ok()) {
        process.execute(num_tries.value());
    }
});

ros::spin();
t.join();

```

The different parameter types are:
  - `param_util::BoolParameter`
  - `param_util::DoubleParameter`
  - `param_util::IntParameter`
  - `param_util::StringParameter`

In addition to accessing the current value, the parameter object can be used to 
publish an update to the parameter using the `.update()` method.

Finally, in some cases a direct callback may be desired to notify the process
that the value has changed:

```
params.param("num_tries", 1, "Number of tries").min(1).max(50).callback([](int value){
    process.setNumTries(value);
});

while (ros::ok()) {
    process.exectute();
    ros::spinOnce();
}
```

These different approaches are not mutually exclusive and can be used in concert.

### Differences Between ROS1 and ROS2
 - ros2: `.declare()` must be called after configuring a parameter
 - ros2: there is no `.group()` configuration
 - ros2: parameters are ordered alphabetically in dynamic_reconfigure
 - ros2: `.register_verbose_logging_param()` helper function added to enable dynamic parameter for log-devel
 - ros1: there is no `.step()` configuration for numeric parameters
 - ros1: parameters are ordered in configuration order in dynamic_reconfigure