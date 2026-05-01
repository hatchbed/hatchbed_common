# Parameter Utilities

The `param_handler.h` and `parameter.h` headers provide a programmatic interface for
declaring and managing ROS 2 parameters without needing `.cfg` files. The design is
similar to [ddynamic_reconfigure](https://github.com/pal-robotics/ddynamic_reconfigure)
and [swri_roscpp](https://github.com/swri-robotics/marti_common/tree/master/swri_roscpp#dynamic-parameters).

**Objectives:**
- Minimize boilerplate for defining and accessing parameters
- Provide a uniform interface for static and dynamic parameters
- Log parameter values at startup and on change
- Enforce range constraints at the ROS layer
- Publish static (read-only) parameters to dynamic reconfigure for easy runtime inspection

---

## CMake dependency

```cmake
find_package(hatchbed_common REQUIRED)
target_link_libraries(my_target hatchbed_common::common)
```

---

## Quick start

```cpp
#include <hatchbed_common/param_handler.h>

auto node = std::make_shared<rclcpp::Node>("example");
hatchbed_common::ParamHandler params(node);

// Static — read the value once at startup
int num_tries = params.param("num_tries", 1, "Number of tries")
    .min(1).max(50).declare().value();

// Dynamic — value updates automatically when the parameter is changed
double threshold = 0.0;
params.param(&threshold, "threshold", 0.75, "Detection threshold")
    .min(0.0).max(1.0).dynamic().declare();
```

---

## ParamHandler

`hatchbed_common::ParamHandler` manages the parameter lifecycle for a node.

```cpp
// Constructed with the node it will manage parameters for
hatchbed_common::ParamHandler params(shared_from_this());
```

### Registering parameters

All parameters require a name, default value, and description string.  The fluent
builder returned by `param()` accepts optional configuration before `.declare()` finalizes
the registration.

```
params.param([ptr,] name, default, description)
    [.min(value)]
    [.max(value)]
    [.step(value)]          // ROS 2 only
    [.enumerate({...})]     // integer types only
    [.dynamic()]
    [.callback(fn)]         // implies .dynamic()
    .declare()
```

Calling `.declare()` is required in ROS 2.  The method returns a `Declared<Parameter<T>>`
wrapper from which `.value()` retrieves the current value.

### Global change callback

Register a single callback fired whenever any dynamic parameter changes:

```cpp
params.setCallback([]() {
    RCLCPP_INFO(logger, "A parameter changed.");
});
```

### Verbose logging helper

Adds a dynamic `verbose_logging` bool parameter that, when set to `true`, switches the
node's effective log level to DEBUG:

```cpp
params.register_verbose_logging_param();
```

---

## Static parameters

Read the value immediately via `.declare().value()` — no further updates are expected.

```cpp
// int (internally int64_t; cast checked at declaration)
int num_tries = params.param("num_tries", 1, "Number of tries")
    .min(1).max(50).declare().value();

// int64_t (use L suffix on literals to avoid ambiguity)
int64_t big_count = params.param("big_count", 1L, "Large counter")
    .min(0L).declare().value();

// double
double speed = params.param("max_speed", 1.0, "Maximum speed (m/s)")
    .min(0.01).max(5.0).declare().value();

// string
std::string frame = params.param("frame_id", std::string("base_link"),
    "Output frame").declare().value();

// bool
bool debug = params.param("debug", false, "Enable debug output").declare().value();

// enum (integer)
int mode = params.param("mode", 0, "Operating mode")
    .enumerate({
        {0, "Default",  "Standard operating mode"},
        {1, "Advanced", "Advanced operating mode"},
        {2, "Legacy",   "Deprecated legacy mode"},
    }).declare().value();
```

## Array parameters

```cpp
// integer array
std::vector<int64_t> ids = params.param("ids", std::vector<int64_t>{1, 2, 3},
    "Sensor IDs").declare().value();

// double array
std::vector<double> gains = params.param("gains",
    std::vector<double>{1.0, 0.5, 0.1}, "PID gains").declare().value();

// string array
std::vector<std::string> topics = params.param("input_topics",
    std::vector<std::string>{"/camera/image"}, "Input topics").declare().value();

// bool array
std::vector<bool> flags = params.param("enabled_channels",
    std::vector<bool>{true, false, true}, "Active channels").declare().value();
```

Range constraints (`min`, `max`, `step`) apply per-element for arrays. ROS 2 does not
currently expose array constraints in the parameter description, but the handler enforces
them on write.

---

## Dynamic parameters

### Pointer form (single-threaded)

Bind the parameter directly to an existing variable.  The variable is updated in-place
when the parameter changes; no locking is applied.

```cpp
double threshold = 0.0;
params.param(&threshold, "threshold", 0.75, "Detection threshold")
    .min(0.0).max(1.0).dynamic().declare();

std::vector<double> weights;
params.param(&weights, "weights", std::vector<double>{0.5, 0.5}, "Blend weights")
    .min(0.0).max(1.0).dynamic().declare();

// threshold and weights are updated transparently by the ROS executor
while (rclcpp::ok()) {
    process(threshold, weights);
    rclcpp::spin_some(node);
}
```

### Parameter object (multi-threaded)

Store the returned `Declared` object and call `.value()` whenever the current value is
needed.  Access is mutex-protected.

```cpp
auto speed_param = params.param("max_speed", 1.0, "Maximum speed (m/s)")
    .min(0.01).max(5.0).dynamic().declare();

// in another thread:
double current = speed_param.value();
```

The parameter types available as named aliases:

| Alias | Underlying type |
|---|---|
| `BoolParameter` | `bool` |
| `DoubleParameter` | `double` |
| `IntParameter` | `int64_t` |
| `SystemIntParameter` | `int64_t` (read as `int`) |
| `StringParameter` | `std::string` |
| `BoolArrayParameter` | `std::vector<bool>` |
| `DoubleArrayParameter` | `std::vector<double>` |
| `IntArrayParameter` | `std::vector<int64_t>` |
| `StringArrayParameter` | `std::vector<std::string>` |

### Callback form

Execute a function immediately when the parameter value changes:

```cpp
params.param("num_tries", 1, "Number of tries")
    .min(1).max(50)
    .callback([](int value) {
        process.setNumTries(value);
    }).declare();
```

The callback implies `.dynamic()` — the parameter is automatically modifiable at runtime.

---

## Differences between ROS 1 and ROS 2

| Feature | ROS 1 | ROS 2 |
|---|---|---|
| `.declare()` required | No | Yes |
| `.step()` for numeric params | No | Yes |
| `.group()` grouping | Yes | No |
| Parameter order in UI | Declaration order | Alphabetical |
| `register_verbose_logging_param()` | No | Yes |

---

[Back to README](../README.md)
