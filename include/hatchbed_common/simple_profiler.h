#pragma once

#include <chrono>
#include <iostream>
#include <stack>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#ifndef PROFILE_DISABLED

namespace profile {

/**
 * Represents a level in the profiling scope hierarchy.
 */
struct ProfileScopeLevel {
    std::string path;
    std::vector<std::string> children;

    ProfileScopeLevel(const std::string& path_)
        : path(path_) {}
};

/**
 * Singleton class for managing profiling scopes and reporting elapsed times.
 */
class SimpleProfiler {
public:
    /**
     * Access the singleton instance of the Profiler.
     */
    static SimpleProfiler& instance() {
        thread_local SimpleProfiler profiler;
        return profiler;
    }

    /**
     * Push a new scope level onto the stack.
     */
    void pushScope() {
        if (!enabled) {
            return;
        }

        if (scope_stack.empty()) {
            scope_stack.emplace("/");
            return;
        }

        auto& current_level = scope_stack.top();
        if (!current_level.children.empty()) {
            scope_stack.emplace(current_level.path + "/" + current_level.children.back());
        }
    }

    /**
     * Pop the current scope level from the stack.
     */
    void popScope() {
        if (!enabled) {
            return;
        }

        if (scope_stack.empty()) {
            return;
        }

        // close any remaining open scope at this level
        auto& current_level = scope_stack.top();
        if (!current_level.children.empty()) {
            stopScope(current_level.path + "/" + current_level.children.back());
        }
        scope_stack.pop();
    }

    /**
     * Finalize profiling for the current frame, recording elapsed times.
     */
    void finalize() {
        if (!enabled) {
            return;
        }

        // close all open scopes
        while (!scope_stack.empty()) {
            popScope();
        }

        last_elapsed = elapsed_times;
        for (const auto& [name, time] : last_elapsed) {
            total_times[name] += time;
        }

        elapsed_times.clear();
        count++;
    }

    /**
     * Print detailed timing data in a formatted tree structure.
     * 
     * @param[in] os  The output stream to write to.
     */
    void print(std::ostream& os) const {    
        if (!enabled) {
            return;
        }

        // Helper to get indentation & scope label
        auto get_indent_and_label = [](const std::string& full_name) -> std::pair<int, std::string> {
            size_t last = full_name.find_last_of('/');
            if (last == std::string::npos || last == 0)
                return {0, full_name.substr(last + 1)};
            int indent = std::count(full_name.begin(), full_name.end(), '/');
            return {std::max(0, indent - 2), full_name.substr(last + 1)};
        };

        // Build scope tree
        std::map<std::string, std::vector<std::string>> children;
        for (const auto& scope : scope_order) {
            size_t parent_pos = scope.find_last_of('/');
            if (parent_pos != std::string::npos && parent_pos > 0) {
                std::string parent = scope.substr(0, parent_pos);
                children[parent].push_back(scope);
            }
        }

        // Dynamically compute the label width
        size_t max_label_length = 0;
        for (const auto& scope : scope_order) {
            auto [indent, label] = get_indent_and_label(scope);
            size_t len = indent * 2 + label.length();
            max_label_length = std::max(max_label_length, len);
        }
        const int name_width = std::max(static_cast<int>(max_label_length + 2), 20);

        // Dynamically compute the elapsed column width
        int max_elapsed_digits = 0;
        for (const auto& [scope, val] : last_elapsed) {
            double abs_val = std::abs(val);
            // log10(0) is undefined, so handle zero explicitly
            int whole_digits = (abs_val < 1e-3) ? 1 : static_cast<int>(std::log10(abs_val)) + 1;
            max_elapsed_digits = std::max(max_elapsed_digits, whole_digits);
        }
        max_elapsed_digits += 3;
        const int elapsed_width = std::max(max_elapsed_digits + 1, 7);

        // Dynamically compute the avg column width
        int max_avg_digits = 0;
        for (const auto& [scope, val] : total_times) {
            double abs_val = count > 0 ? std::abs(val / count) : 0.0;
            // log10(0) is undefined, so handle zero explicitly
            int whole_digits = (abs_val < 1e-3) ? 1 : static_cast<int>(std::log10(abs_val)) + 1;
            max_avg_digits = std::max(max_avg_digits, whole_digits);
        }
        max_avg_digits += 3;
        const int avg_width = std::max(max_avg_digits + 2, 4);


        // Header
        os << "\n";
        os << std::left << std::setw(name_width) << "timing profile (ms)"
           << std::right << std::setw(elapsed_width) << "elapsed"
           << std::setw(avg_width) << "avg" << '\n';
        size_t total_width = name_width + elapsed_width + avg_width;
        os << std::string(total_width, '=');
        os << '\n';

        // Data
        for (size_t i = 0; i < scope_order.size(); ++i) {
            const std::string& scope = scope_order[i];
            auto [indent, label] = get_indent_and_label(scope);
            std::ostringstream name_stream;
            for (int j = 0; j + 1 < indent; j++) {
                name_stream << "| ";
            }

            // Tree branches
            if (indent > 0) {
                const std::string& parent = scope.substr(0, scope.find_last_of('/'));
                const auto& siblings = children[parent];
                bool is_last = (scope == siblings.back());
                name_stream << (is_last ? "|_" : "| ");
            }

            name_stream << label;

            std::string label_str = name_stream.str();
            int dot_count = std::max(0, name_width - static_cast<int>(label_str.size()) - 1);
            label_str += " " + std::string(dot_count, '.');

            os << std::left << std::setw(name_width) << label_str;

            double elapsed = last_elapsed.count(scope) ? last_elapsed.at(scope) : 0.0;
            double total = total_times.count(scope) ? total_times.at(scope) : 0.0;
            double avg = count > 0 ? total / count : 0.0;

            std::ostringstream temp;
            temp << " " << std::fixed << std::setprecision(2) << elapsed;
            std::string elapsed_str = temp.str();

            os << std::right << std::fixed << std::setprecision(2)
               << std::setfill('.') << std::setw(elapsed_width) << elapsed_str
               << std::setfill(' ') << std::setw(avg_width) << avg << '\n';
        }
    }

    /**
     * Enable or disable profiling globally.
     * 
     * @param[in] is_enabled  True to enable profiling, false to disable.
     */
    void setEnabled(bool is_enabled) {
        enabled = is_enabled;
    }

    /**
     * A RAII timer that starts on construction and stops on destruction.
     *        
     * Intended for automatic profiling of scoped code blocks.
     */
    class ScopedTimer {
    public:
        ScopedTimer(const std::string& name)
            : name_(name), scope_depth_(SimpleProfiler::instance().currentScopeDepth()) 
        {
            SimpleProfiler::instance().start(name_);
        }

        ~ScopedTimer() {
            // Ensure the scope stack is restored to its previous depth
            while (SimpleProfiler::instance().scope_stack.size() > scope_depth_) {
                SimpleProfiler::instance().popScope();
            }

            SimpleProfiler::instance().stop(name_);
        }

    private:
        std::string name_;
        size_t scope_depth_;
    };

private:
    SimpleProfiler() = default;

    /**
     * Get the current scope depth.
     */
    size_t currentScopeDepth() const { 
        return scope_stack.size(); 
    }

    /**
     * Start timing a named profiling scope.
     * 
     * @param[in] name  The name of the scope to start.
     */
    void start(const std::string& name) {
        if (!enabled) {
            return;
        }

        if (scope_stack.empty()) {
            pushScope();
        }

        auto& current_level = scope_stack.top();

        // close current active scope if one exists
        if (!current_level.children.empty()) {
            stopScope(current_level.path + "/" + current_level.children.back());
        }

        current_level.children.push_back(name);
        
        std::string full_name = current_level.path + "/" + name;
        start_times[full_name] = std::chrono::high_resolution_clock::now();

        if (scope_set.find(full_name) == scope_set.end()) {
            scope_set.insert(full_name);
            scope_order.push_back(full_name);
        }
    }

    /**
     * Stop timing a named profiling scope.
     * 
     * @param[in] name  The name of the scope to stop.
     */
    void stop(const std::string& name) {
        if (!enabled) {
            return;
        }

        if (scope_stack.empty()) {
            return;
        }

        auto& current_level = scope_stack.top();
        if (current_level.children.empty()) {
            return;
        }

        if (name == current_level.children.back()) {
            stopScope(current_level.path + "/" + name);
        }
    }

    /**
     * Internal helper to stop and record timing for a given scope.
     * 
     * @param[in] name  The full name (path) of the scope.
     */
    void stopScope(const std::string& name) {
        auto it = start_times.find(name);
        if (it == start_times.end()) return;

        auto now = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration<double, std::milli>(now - it->second).count();
        elapsed_times[name] += elapsed;
        start_times.erase(it);
    }

    std::stack<ProfileScopeLevel> scope_stack;
    std::unordered_map<std::string, std::chrono::high_resolution_clock::time_point> start_times;
    std::unordered_map<std::string, double> elapsed_times;
    std::unordered_map<std::string, double> last_elapsed;
    std::unordered_map<std::string, double> total_times;

    std::vector<std::string> scope_order;
    std::unordered_set<std::string> scope_set;
    size_t count = 0;
    bool enabled = true;
};

/**
 * Overload for printing the profiler summary to an output stream.
 */
inline std::ostream& operator<<(std::ostream& os, const SimpleProfiler& profiler) {
    profiler.print(os);
    return os;
}

inline void push() {
    SimpleProfiler::instance().pushScope();
}

inline void pop() {
    SimpleProfiler::instance().popScope();
}

inline void finalize() {
    SimpleProfiler::instance().finalize();
}

inline SimpleProfiler& get() {
    return SimpleProfiler::instance();
}

inline void set_enabled(bool enabled) {
    SimpleProfiler::instance().setEnabled(enabled);
}

}  // namespace profile

/**
 * Helper macros for concatenating names.
 */
#define PROFILE_CONCAT_IMPL(x, y) x##y
#define PROFILE_CONCAT(x, y) PROFILE_CONCAT_IMPL(x, y)

/**
 * Helper macro for creating a ScopedTimer with a unique name.
 */
#define profile_scope(name) profile::SimpleProfiler::ScopedTimer PROFILE_CONCAT(timer, __COUNTER__)(name)

#else

namespace profile {

inline void push() {}

inline void pop() {}

inline void finalize() {}

inline std::string get() {
    return "profiling disabled";
}

inline void set_enabled(bool enabled) {}

}  // namespace profile

#define profile_scope(name) do {} while(0)

#endif