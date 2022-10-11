/**
 * Copyright (c) 2022, Hatchbed
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <mutex>
#include <string>
#include <unordered_map>

#include <hatchbed_common/parameter.h>
#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>

namespace hatchbed_common {

/**
 * The param handler is a convenience class for managing static and dynamic ROS
 * parameters.  It will handle receiving and sending parameter updates.
 *
 * When registering a new parameter the param handler will return a parameter
 * object which can be used to access the parameter value in a thread safe way
 * if the application is using multiple threads.
 *
 * All parameters require a name, default value, and description.
 *
 * Optionally, a pointer to an existing variable can be passed in when
 * registering a parameter.  In this case that variable is used to store the
 * parameter value, but access to it is not protected, so should only be used
 * in single threaded applications.
 *
 * When registering a parameter it is possible to chain additional configuration
 * items to the parameter, such as:
 *   * .dynamic() - allow the parameter to by modified with dynamic reconfig
 *   * .callback(func) - provide a callback function when the parameter changes, implies .dynamic()
 *   * .min(val) - specify a minimun value for numeric parameters
 *   * .max(val) - specify a maximum value for numeric parameters
 *   * .step(val) - specify step size for numeric parameters
 *   * .enum(list) - specify an enumeration for integer parameters
 *
 * Once the parameter has been configured, it's necessary to call the `.declare()` method.
 *
 * The parameter objects values can be accessed in a thread safe way with the
 * .value() method and updated with the .update(val) method.
 *
 * Assigning parameter objects results in a shallow copy.
 */
class ParamHandler {
  public:
    ParamHandler(rclcpp::Node::SharedPtr node) :
      node_(node),
      namespace_(node_->get_fully_qualified_name()),
      verbose_param_(false)
    {
      params_callback_handle_ = node_->add_on_set_parameters_callback(
        std::bind(&ParamHandler::parametersCallback, this, std::placeholders::_1));
    };

    ~ParamHandler() = default;

    /**
     * Register a boolean 'verbose' dynamic parameter to enable or disable debug
     * level logging.
     */
    void register_verbose_logging_param() {
        if (verbose_param_) {
            return;
        }
        verbose_param_ = true;

        // determine the default verbose setting based on the current log level
        auto log_level = rcutils_logging_get_logger_level(node_->get_logger().get_name());
        bool is_verbose = log_level == RCUTILS_LOG_SEVERITY_DEBUG;
        if (is_verbose) {
            non_verbose_level_ = log_level;
        }

        param("verbose", is_verbose, "Enable debug logging").callback([this](const bool& value) {
            RCLCPP_INFO_STREAM(node_->get_logger(), "setting verbose logging: " << value);
            if (value) {
                auto ret = rcutils_logging_set_logger_level(node_->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
                if (ret != RCUTILS_RET_OK) {
                    RCLCPP_WARN_STREAM(node_->get_logger(), "Failed to set log level to debug");
                }
            }
            else {
                auto ret = rcutils_logging_set_logger_level(node_->get_logger().get_name(), non_verbose_level_);
                if (ret != RCUTILS_RET_OK) {
                    RCLCPP_WARN_STREAM(node_->get_logger(), "Failed to set log level");
                }
            }
        }).declare();
    }

    /**
     * Register a bool parameter and return it's value.
     *
     * @param[in] name         Parameter name
     * @param[in] default_val  Default parameter value
     * @param[in] description  Parameter description
     *
     * @returns the value of the parameter.
     */
    BoolParameter& param(const std::string& name, const bool& default_val, const std::string& description) {
        std::scoped_lock lock(mutex_);
        bool_params_[name] = BoolParameter(nullptr, namespace_, name, default_val, description, node_);
        return bool_params_[name];
    }

    /**
     * Register a bool parameter and return it's value.
     *
     * NOTE: This version is only recommended for single threaded applications since the user provided parameter
     *       pointer won't be fully guarded from concurrent usage.
     *
     *       The point should also remain valid for the life of the handler.
     *
     * @param[out] param       Reference to parameter variable
     * @param[in] name         Parameter name
     * @param[in] default_val  Default parameter value
     * @param[in] description  Parameter description
     *
     * @returns the value of the parameter.
     */
    BoolParameter& param(bool* param, const std::string& name, const bool& default_val, const std::string& description) {
        std::scoped_lock lock(mutex_);
        bool_params_[name] = BoolParameter(param, namespace_, name, default_val, description, node_);
        return bool_params_[name];
    }

    /**
     * Register an integer parameter and return it's value.
     *
     * @param[in] name         Parameter name
     * @param[in] default_val  Default parameter value
     * @param[in] description  Parameter description
     *
     * @returns the value of the parameter.
     */
    IntParameter& param(const std::string& name, const int& default_val, const std::string& description) {
        std::scoped_lock lock(mutex_);
        int_params_[name] = IntParameter(nullptr, namespace_, name, default_val, description, node_);
        return int_params_[name];
    }

    /**
     * Register an integer parameter and return it's value.
     *
     * NOTE: This version is only recommended for single threaded applications since the user provided parameter
     *       pointer won't be fully guarded from concurrent usage.
     *
     *       The point should also remain valid for the life of the handler.
     *
     * @param[out] param       Reference to parameter variable
     * @param[in] name         Parameter name
     * @param[in] default_val  Default parameter value
     * @param[in] description  Parameter description
     *
     * @returns the value of the parameter.
     */
    IntParameter& param(int* param, const std::string& name, const int& default_val, const std::string& description) {
        std::scoped_lock lock(mutex_);
        int_params_[name] = IntParameter(param, namespace_, name, default_val, description, node_);
        return int_params_[name];
    }

    /**
     * Register a double parameter and return it's value.
     *
     * @param[in] name         Parameter name
     * @param[in] default_val  Default parameter value
     * @param[in] description  Parameter description
     *
     * @returns the value of the parameter.
     */
    DoubleParameter& param(const std::string& name, const double& default_val, const std::string& description) {
        std::scoped_lock lock(mutex_);
        double_params_[name] = DoubleParameter(nullptr, namespace_, name, default_val, description, node_);
        return double_params_[name];
    }

    /**
     * Register a double parameter and return it's value.
     *
     * NOTE: This version is only recommended for single threaded applications since the user provided parameter
     *       pointer won't be fully guarded from concurrent usage.
     *
     *       The point should also remain valid for the life of the handler.
     *
     * @param[out] param       Reference to parameter variable
     * @param[in] name         Parameter name
     * @param[in] default_val  Default parameter value
     * @param[in] description  Parameter description
     *
     * @returns the value of the parameter.
     */
    DoubleParameter& param(double* param, const std::string& name, const double& default_val, const std::string& description) {
        std::scoped_lock lock(mutex_);
        double_params_[name] = DoubleParameter(param, namespace_, name, default_val, description, node_);
        return double_params_[name];
    }

    /**
     * Register a string parameter and return it's value.
     *
     * @param[in] name         Parameter name
     * @param[in] default_val  Default parameter value
     * @param[in] description  Parameter description
     *
     * @returns the value of the parameter.
     */
    StringParameter& param(const std::string& name, const std::string& default_val, const std::string& description) {
        std::scoped_lock lock(mutex_);
        string_params_[name] = StringParameter(nullptr, namespace_, name, default_val, description, node_);
        return string_params_[name];
    }

    /**
     * Register a string parameter and return it's value.
     *
     * NOTE: This version is only recommended for single threaded applications since the user provided parameter
     *       pointer won't be fully guarded from concurrent usage.
     *
     *       The point should also remain valid for the life of the handler.
     *
     * @param[out] param       Reference to parameter variable
     * @param[in] name         Parameter name
     * @param[in] default_val  Default parameter value
     * @param[in] description  Parameter description
     *
     * @returns the value of the parameter.
     */
    StringParameter& param(std::string* param, const std::string& name, const std::string& default_val, const std::string& description) {
        std::scoped_lock lock(mutex_);
        string_params_[name] = StringParameter(param, namespace_, name, default_val, description, node_);
        return string_params_[name];
    }

private:
  rclcpp::Node::SharedPtr node_;
  std::string namespace_;
  bool verbose_param_;
  int non_verbose_level_ = RCUTILS_LOG_SEVERITY_INFO;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

  std::mutex mutex_;
  std::unordered_map<std::string, BoolParameter> bool_params_;
  std::unordered_map<std::string, DoubleParameter> double_params_;
  std::unordered_map<std::string, IntParameter> int_params_;
  std::unordered_map<std::string, StringParameter> string_params_;

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    for (const auto &param: parameters) {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
            auto bool_param = bool_params_.find(param.get_name());
            if (bool_param != bool_params_.end()) {
                if (!bool_param->second.update(param.as_bool(), true)) {

                    result.successful = false;
                    result.reason = "Failed to update parameter: " + bool_param->first;
                }
            }
        }
        else if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            auto double_param = double_params_.find(param.get_name());
            if (double_param != double_params_.end()) {
                if (!double_param->second.update(param.as_double(), true)) {
                    result.successful = false;
                    result.reason = "Failed to update parameter: " + double_param->first;
                }
            }
        }
        else if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
            auto int_param = int_params_.find(param.get_name());
            if (int_param != int_params_.end()) {
                if (!int_param->second.update(param.as_int(), true)) {
                    result.successful = false;
                    result.reason = "Failed to update parameter: " + int_param->first;
                }
            }
        }
        else if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
            auto string_param = string_params_.find(param.get_name());
            if (string_param != string_params_.end()) {
                if (!string_param->second.update(param.as_string(), true)) {
                    result.successful = false;
                    result.reason = "Failed to update parameter: " + string_param->first;
                }
            }
        }
    }

    if (!result.successful) {
        RCLCPP_WARN_STREAM(node_->get_logger(), result.reason);
    }

    return result;
  }
};

}  // hatchbed_common
