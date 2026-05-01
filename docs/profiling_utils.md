# Profiling Utilities

`hatchbed_common/simple_profiler.h` provides a low-overhead hierarchical timing
profiler.  Scopes nest naturally, timings are thread-local, and both per-call and
rolling-average figures are reported.

---

## CMake dependency

The profiler is header-only and part of the core library:

```cmake
find_package(hatchbed_common REQUIRED)
target_link_libraries(my_target hatchbed_common::common)
```

To eliminate all profiling overhead at compile time (not just the enabled-check branch):

```cmake
target_compile_definitions(my_target PRIVATE PROFILE_DISABLED)
```

---

## Basic usage

```cpp
#include <hatchbed_common/simple_profiler.h>

::profile::set_enabled(true);

void process_frame()
{
    profile_scope("process_frame");       // times the whole function

    fetch_data();

    ::profile::push();                    // open a child scope level
    {
        profile_scope("preprocess");      // times preprocess()
        preprocess();
    }
    {
        profile_scope("detect");          // times detect(); stops "preprocess"
        detect();
    }
    ::profile::pop();                     // close child level; stops "detect"

    publish_results();

    ::profile::finalize();                // close open scopes; increment frame counter
    RCLCPP_DEBUG_STREAM(logger, ::profile::get());
}
```

---

## Scope lifecycle

A `profile_scope("name")` timer stops automatically when any of the following occur:

- The enclosing C++ scope exits (RAII destructor)
- A new `profile_scope` opens at the same level
- `::profile::pop()` closes the current level
- `::profile::finalize()` closes all open scopes

---

## API reference

### Macros

| Macro / function | Description |
|---|---|
| `profile_scope("name")` | RAII timer; stops when scope exits or a new peer scope starts |
| `::profile::push()` | Open a new child scope level |
| `::profile::pop()` | Close the current child scope level |
| `::profile::finalize()` | Close all scopes, record timings, increment frame count |
| `::profile::set_enabled(bool)` | Enable or disable profiling at runtime |
| `::profile::get()` | Return a reference to the thread-local `SimpleProfiler` for printing |

### Direct class access

```cpp
auto & prof = hatchbed_common::SimpleProfiler::instance();
prof.setEnabled(true);
prof.pushScope();
prof.popScope();
prof.finalize();
prof.print(std::cout);
```

---

## Output format

```
timing profile (ms)   elapsed     avg
=====================================
process_frame ........ 282.22  239.91
| preprocess ........... 0.05    0.05
| detect ............... 2.33    2.24
| | sample ............. 0.41    0.35
| | match ............. 28.05   25.90
| |_solve ............. 16.69   14.50
|_publish .............. 0.01    0.01
```

- **elapsed** — time for the most recent call
- **avg** — rolling average over all calls since the last `set_enabled(true)`
- Indentation shows parent-child relationships; `|_` marks the last child

---

## Thread safety

Profiling is per-thread (`thread_local` storage).  Timings are not aggregated across
threads.  Each thread has its own independent `SimpleProfiler` instance.

---

## Integration with fmt / logging

The `SimpleProfiler` is streamable and also supports direct use with `fmt` and the
`HB_*` logging macros when `logging/message_formatting.h` is included:

```cpp
HB_INFO(logger, "Frame timings:\n{}", ::profile::get());
```

---

[Back to README](../README.md)
