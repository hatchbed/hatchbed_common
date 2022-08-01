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

#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>

namespace param_util {

class ParamHandler {
public:
  ParamHandler(rclcpp::Node::SharedPtr node) :
    node_(node),
    verbose_param_(false)
  {
    params_callback_handle_ = node_->add_on_set_parameters_callback(
      std::bind(&ParamHandler::parametersCallback, this, std::placeholders::_1));
  };

  ~ParamHandler() = default;

  /**
   * Register a boolean 'verbose' parameter to enable or disable debug level
   * logging.
   */
  void register_verbose_logging_param() {
    if (verbose_param_) {
      return;
    }
    verbose_param_ = true;

    // determine the default verbose setting based on the current log level
    auto log_level = rcutils_logging_get_logger_level(node_->get_logger().get_name());
    verbose_ = log_level == RCUTILS_LOG_SEVERITY_DEBUG;
    if (!verbose_) {
      non_verbose_level_ = log_level;
    }

    // register the verbose parameter and get the configured value
    register_param(&verbose_, "verbose", verbose_, "Enable debug level logging.");

    // set the log level to debug if verbose is enabled
    if (verbose_ && log_level != RCUTILS_LOG_SEVERITY_DEBUG) {
      auto ret = rcutils_logging_set_logger_level(node_->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
      if (ret != RCUTILS_RET_OK) {
          RCLCPP_WARN_STREAM(node_->get_logger(), "Failed to set log level to debug");
      }
    }
  }

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
  T param(const std::string& name, const T& default_val, const std::string& description) {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = description;
    descriptor.read_only = true;
    node_->declare_parameter(name, rclcpp::ParameterValue(default_val), descriptor);
    T value = node_->get_parameter(name).get_value<T>();
    RCLCPP_INFO_STREAM(node_->get_logger(), name << ": " << value);
    return value;
  }

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
  void register_param(T* param, const std::string& name, const T& default_val, const std::string& description) {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = description;
    descriptor.read_only = false;
    node_->declare_parameter(name, rclcpp::ParameterValue(default_val), descriptor);
    auto p = node_->get_parameter(name);

    if (p.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
      bool_params_[name] = static_cast<bool*>(static_cast<void*>(param));
    }
    else if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      double_params_[name] = static_cast<double*>(static_cast<void*>(param));
    }
    else if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      int_params_[name] = static_cast<int*>(static_cast<void*>(param));
    }
    else {
      RCLCPP_ERROR(node_->get_logger(), "Unsupported dynamic parameter type: %s for parameter: %s", p.get_type_name().c_str(), name.c_str());
    }

    *param = p.get_value<T>();
    RCLCPP_INFO_STREAM(node_->get_logger(), name << ": " << *param);
  }

  /**
   * Register a dynamic bool parameter stored in an integer variable and
   * populate the parameter variable with the current value.
   *
   * @param[out] param       Reference to parameter variable
   * @param[in] name         Parameter name
   * @param[in] default_val  Default parameter value
   * @param[in] description  Parameter description
   */
  void register_param(int* param, const std::string& name, bool default_val, const std::string& description) {
    int_params_[name] = param;
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = description;
    descriptor.read_only = false;
    node_->declare_parameter(name, rclcpp::ParameterValue(default_val), descriptor);
    *param = node_->get_parameter(name).as_bool();
    RCLCPP_INFO_STREAM(node_->get_logger(), name << ": " << *param);
  }

  /**
   * Register a dynamic integer parameter with a range constraint and populate
   * the parameter variable with the current value.
   *
   * @param[out] param       Reference to parameter variable
   * @param[in] name         Parameter name
   * @param[in] default_val  Default parameter value
   * @param[in] description  Parameter description
   * @param[in] min          Min value
   * @param[in] max          Max value
   */
  void register_param(int* param, const std::string& name, int default_val, const std::string& description, int min, int max, int step = 0) {
    int_params_[name] = param;
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = description;
    descriptor.read_only = false;
    descriptor.integer_range.resize(1);
    descriptor.integer_range[0].from_value = min;
    descriptor.integer_range[0].to_value = max;
    descriptor.integer_range[0].step = 1;
    node_->declare_parameter(name, default_val, descriptor);
    *param = node_->get_parameter(name).as_int();
    RCLCPP_INFO_STREAM(node_->get_logger(), name << ": " << *param);
  }

  /**
   * Register a dynamic double parameter with a range constraint and populate
   * the parameter variable with the current value.
   *
   * @param[out] param       Reference to parameter variable
   * @param[in] name         Parameter name
   * @param[in] default_val  Default parameter value
   * @param[in] description  Parameter description
   * @param[in] min          Min value
   * @param[in] max          Max value
   */
  void register_param(double* param, const std::string& name, double default_val, const std::string& description, double min, double max, double step = 0.0) {
    double_params_[name] = param;
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = description;
    descriptor.read_only = false;
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = min;
    descriptor.floating_point_range[0].to_value = max;
    descriptor.floating_point_range[0].step = step;
    node_->declare_parameter(name, default_val, descriptor);
    *param = node_->get_parameter(name).as_double();
    RCLCPP_INFO_STREAM(node_->get_logger(), name << ": " << *param);
  }

private:
  rclcpp::Node::SharedPtr node_;
  bool verbose_param_;
  bool verbose_ = false;
  int non_verbose_level_ = RCUTILS_LOG_SEVERITY_INFO;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
  std::unordered_map<std::string, bool*> bool_params_;
  std::unordered_map<std::string, double*> double_params_;
  std::unordered_map<std::string, int*> int_params_;
  std::unordered_map<std::string, std::string*> string_params_;

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    for (const auto &param: parameters) {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
        auto bool_param = bool_params_.find(param.get_name());
        if (bool_param != bool_params_.end()) {
          *bool_param->second = param.as_bool();
          RCLCPP_INFO(node_->get_logger(), "Parameter updated: %s = %s", param.get_name().c_str(), (param.as_bool() ? "true": "false"));

          if (verbose_param_ && param.get_name() == "verbose") {
            if (verbose_) {
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
          }
        }
        else {
          auto int_param = int_params_.find(param.get_name());
          if (int_param != int_params_.end()) {
            *int_param->second = param.as_bool();
            RCLCPP_INFO(node_->get_logger(), "Parameter updated: %s = %s", param.get_name().c_str(), (param.as_bool() ? "true": "false"));
          }
        }
      }
      else if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        auto double_param = double_params_.find(param.get_name());
        if (double_param != double_params_.end()) {
          *double_param->second = param.as_double();
          RCLCPP_INFO(node_->get_logger(), "Parameter updated: %s = %lf", param.get_name().c_str(), param.as_double());
        }
      }
      else if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        auto int_param = int_params_.find(param.get_name());
        if (int_param != int_params_.end()) {
          *int_param->second = param.as_int();
          RCLCPP_INFO(node_->get_logger(), "Parameter updated: %s = %ld", param.get_name().c_str(), param.as_int());
        }
      }
    }

    return result;
  }
};

}  // param_util
