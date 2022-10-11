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

#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/GroupState.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <hatchbed_common/parameter.h>
#include <ros/node_handle.h>

namespace hatchbed_common {

/**
 * The param handler is a convenience class for managing static and dynamic ROS
 * parameters.  It will send parameter config description messages when
 * new parameters are registered with the handler and will handle receiving
 * and sending parameter updates.
 *
 * Both static and dynamic parameters are included in the config description,
 * but static parameters will be labeled as '(readonly)' and ignore any updates
 * that might come in for them.
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
 *   * .group(string) - place the parameter in a sub-group
 *   * .min(val) - specify a minimun value for numeric parameters
 *   * .max(val) - specify a maximum value for numeric parameters
 *   * .enum(list) - specify an enumeration for integer parameters
 *
 * The parameter objects values can be accessed in a thread safe way with the
 * .value() method and updated with the .update(val) method.
 *
 * Assigning parameter objects results in a shallow copy.
 */
class ParamHandler {
  public:
    using Ptr = std::shared_ptr<ParamHandler>;

    /**
     * Construct and initialize param handler.
     *
     * @param[in] node  Node handle.
     */
    ParamHandler(ros::NodeHandle node) :
      node_(node)
    {
        description_pub_ = std::make_shared<ros::Publisher>(
            node_.advertise<dynamic_reconfigure::ConfigDescription>("parameter_descriptions", 1, true));
        update_pub_ = std::make_shared<ros::Publisher>(
            node_.advertise<dynamic_reconfigure::Config>("parameter_updates", 1, true));
        config_service_ = node_.advertiseService("set_parameters", &ParamHandler::configCallback, this);
        description_timer_ = node_.createWallTimer(ros::WallDuration(0.1), &ParamHandler::publishDescription, this);
    };

    ~ParamHandler() = default;

    void shutdown() {
        description_timer_.stop();
        config_service_.shutdown();
        update_pub_ = {};
        description_pub_ = {};
    }

    bool hasBool(const std::string& name) {
        return bool_params_.count(name) > 0;
    }

    bool getBool(const std::string& name) {
        auto bool_param_it = bool_params_.find(name);
        if (bool_param_it == bool_params_.end()) {
            return false;
        }

        return bool_param_it->second.value();
    }

    bool hasDouble(const std::string& name) {
        return double_params_.count(name) > 0;
    }

    double getDouble(const std::string& name) {
        auto double_param_it = double_params_.find(name);
        if (double_param_it == double_params_.end()) {
            return std::numeric_limits<double>::quiet_NaN();
        }

        return double_param_it->second.value();
    }

    bool hasInteger(const std::string& name) {
        return int_params_.count(name) > 0;
    }

    int getInteger(const std::string& name) {
        auto int_param_it = int_params_.find(name);
        if (int_param_it == int_params_.end()) {
            return 0;
        }

        return int_param_it->second.value();
    }

    bool hasString(const std::string& name) {
        return string_params_.count(name) > 0;
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

        if (registered_.count(name) != 0) {
            throw std::runtime_error("Parameter [" + name + "] already registered.");
        }
        registered_.insert(name);

        bool_params_[name] = BoolParameter(nullptr, node_.getNamespace(), name, default_val, description);
        bool value = node_.param(name, default_val);
        bool_params_[name].update(value);
        bool_params_[name].setPublishCallback([this](const std::string& name){ publishUpdate(name); });
        bool_params_[name].setConfigChangeCallback([this](const std::string& name){ descriptionChanged(); });
        param_order_.push_back(name);

        std::string resolved = node_.resolveName(name);
        ROS_INFO("%s: %s", resolved.c_str(), value ? "true" : "false");

        description_changed_ = true;
        return bool_params_[name];
    }

    /**
     * Register a bool parameter and return it's value.
     *
     * NOTE: This version is only recommended for single threaded applications since the user provided parameter
     *       pointer won't be fully guarded from concurrent usage.
     *
     *       The pointer should also remain valid for the life of the handler.
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

        if (registered_.count(name) != 0) {
            throw std::runtime_error("Parameter [" + name + "] already registered.");
        }
        registered_.insert(name);

        bool_params_[name] = BoolParameter(param, node_.getNamespace(), name, default_val, description);
        bool value = node_.param(name, default_val);
        bool_params_[name].update(value);
        bool_params_[name].setPublishCallback([this](const std::string& name){ publishUpdate(name); });
        bool_params_[name].setConfigChangeCallback([this](const std::string& name){ descriptionChanged(); });
        param_order_.push_back(name);

        std::string resolved = node_.resolveName(name);
        ROS_INFO("%s: %s", resolved.c_str(), value ? "true" : "false");

        description_changed_ = true;
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

        if (registered_.count(name) != 0) {
            throw std::runtime_error("Parameter [" + name + "] already registered.");
        }
        registered_.insert(name);

        int_params_[name] = IntParameter(nullptr, node_.getNamespace(), name, default_val, description);
        int value = node_.param(name, default_val);
        int_params_[name].update(value);
        int_params_[name].setPublishCallback([this](const std::string& name){ publishUpdate(name); });
        int_params_[name].setConfigChangeCallback([this](const std::string& name){ descriptionChanged(); });
        param_order_.push_back(name);

        std::string resolved = node_.resolveName(name);
        ROS_INFO("%s: %d", resolved.c_str(), value);

        description_changed_ = true;
        return int_params_[name];
    }

    /**
     * Register an integer parameter and return it's value.
     *
     * NOTE: This version is only recommended for single threaded applications since the user provided parameter
     *       pointer won't be fully guarded from concurrent usage.
     *
     *       The pointer should also remain valid for the life of the handler.
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

        if (registered_.count(name) != 0) {
            throw std::runtime_error("Parameter [" + name + "] already registered.");
        }
        registered_.insert(name);

        int_params_[name] = IntParameter(param, node_.getNamespace(), name, default_val, description);
        int value = node_.param(name, default_val);
        int_params_[name].update(value);
        int_params_[name].setPublishCallback([this](const std::string& name){ publishUpdate(name); });
        int_params_[name].setConfigChangeCallback([this](const std::string& name){ descriptionChanged(); });
        param_order_.push_back(name);

        std::string resolved = node_.resolveName(name);
        ROS_INFO("%s: %d", resolved.c_str(), value);

        description_changed_ = true;
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

        if (registered_.count(name) != 0) {
            throw std::runtime_error("Parameter [" + name + "] already registered.");
        }
        registered_.insert(name);

        double_params_[name] = DoubleParameter(nullptr, node_.getNamespace(), name, default_val, description);
        double value = node_.param(name, default_val);
        double_params_[name].update(value);
        double_params_[name].setPublishCallback([this](const std::string& name){ publishUpdate(name); });
        double_params_[name].setConfigChangeCallback([this](const std::string& name){ descriptionChanged(); });
        param_order_.push_back(name);

        std::string resolved = node_.resolveName(name);
        ROS_INFO("%s: %lf", resolved.c_str(), value);

        description_changed_ = true;
        return double_params_[name];
    }

    /**
     * Register a double parameter and return it's value.
     *
     * NOTE: This version is only recommended for single threaded applications since the user provided parameter
     *       pointer won't be fully guarded from concurrent usage.
     *
     *       The pointer should also remain valid for the life of the handler.
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

        if (registered_.count(name) != 0) {
            throw std::runtime_error("Parameter [" + name + "] already registered.");
        }
        registered_.insert(name);

        double_params_[name] = DoubleParameter(param, node_.getNamespace(), name, default_val, description);
        double value = node_.param(name, default_val);
        double_params_[name].update(value);
        double_params_[name].setPublishCallback([this](const std::string& name){ publishUpdate(name); });
        double_params_[name].setConfigChangeCallback([this](const std::string& name){ descriptionChanged(); });
        param_order_.push_back(name);

        std::string resolved = node_.resolveName(name);
        ROS_INFO("%s: %lf", resolved.c_str(), value);

        description_changed_ = true;
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

        if (registered_.count(name) != 0) {
            throw std::runtime_error("Parameter [" + name + "] already registered.");
        }
        registered_.insert(name);

        string_params_[name] = StringParameter(nullptr, node_.getNamespace(), name, default_val, description);
        std::string value = node_.param(name, default_val);
        string_params_[name].update(value);
        string_params_[name].setPublishCallback([this](const std::string& name){ publishUpdate(name); });
        string_params_[name].setConfigChangeCallback([this](const std::string& name){ descriptionChanged(); });
        param_order_.push_back(name);

        std::string resolved = node_.resolveName(name);
        ROS_INFO("%s: %s", resolved.c_str(), value.c_str());

        description_changed_ = true;
        return string_params_[name];
    }

    /**
     * Register a string parameter and return it's value.
     *
     * NOTE: This version is only recommended for single threaded applications since the user provided parameter
     *       pointer won't be fully guarded from concurrent usage.
     *
     *       The pointer should also remain valid for the life of the handler.
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

        if (registered_.count(name) != 0) {
            throw std::runtime_error("Parameter [" + name + "] already registered.");
        }
        registered_.insert(name);

        string_params_[name] = StringParameter(param, node_.getNamespace(), name, default_val, description);
        std::string value = node_.param(name, default_val);
        string_params_[name].update(value);
        string_params_[name].setPublishCallback([this](const std::string& name){ publishUpdate(name); });
        string_params_[name].setConfigChangeCallback([this](const std::string& name){ descriptionChanged(); });
        param_order_.push_back(name);

        std::string resolved = node_.resolveName(name);
        ROS_INFO("%s: %s", resolved.c_str(), value.c_str());

        description_changed_ = true;
        return string_params_[name];
    }

  private:
    void publishDescription(const ros::WallTimerEvent& e) {
        std::scoped_lock lock(mutex_);

        if (!description_pub_) {
            ROS_WARN("Description publisher has been shutdown.");
            return;
        }

        if (!description_changed_) {
            if (values_changed_) {
                publishUpdate();
            }
            return;
        }

        dynamic_reconfigure::ConfigDescription description;
        dynamic_reconfigure::Group default_group;
        default_group.name = "default";
        default_group.type = "";
        default_group.id = 0;
        default_group.parent = 0;

        dynamic_reconfigure::GroupState default_state;
        default_state.name = "default";
        default_state.state = true;
        default_state.id = 0;
        default_state.parent = 0;

        description.max.groups.push_back(default_state);
        description.min.groups.push_back(default_state);
        description.dflt.groups.push_back(default_state);

        description.groups.push_back(default_group);

        std::unordered_map<std::string, size_t> group_map;
        group_map["default"] = 0;

        for (const auto& name: param_order_) {
            auto bool_param_it = bool_params_.find(name);
            if (bool_param_it != bool_params_.end()) {
                dynamic_reconfigure::BoolParameter param_min;
                param_min.name = bool_param_it->second.qualifiedName();
                param_min.value = true;
                description.min.bools.push_back(param_min);
                auto param_max = param_min;
                param_max.value = false;
                description.max.bools.push_back(param_max);
                auto param_default = param_min;
                param_default.value = bool_param_it->second.defaultValue();
                description.dflt.bools.push_back(param_default);

                dynamic_reconfigure::ParamDescription param_desc;
                param_desc.name = bool_param_it->second.qualifiedName();
                param_desc.type = "bool";
                param_desc.level = 0;
                param_desc.description = bool_param_it->second.description();

                std::string group_name = bool_param_it->second.group();
                if (group_name.empty()) {
                    group_name = "default";
                }
                if (group_map.count(group_name) == 0) {
                    size_t group_id = group_map.size();
                    group_map[group_name] = group_id;

                    dynamic_reconfigure::Group group;
                    group.name = group_name;
                    group.type = "";
                    group.id = group_id;
                    group.parent = 0;

                    dynamic_reconfigure::GroupState group_state;
                    group_state.name = group_name;
                    group_state.state = true;
                    group_state.id = group_id;
                    group_state.parent = 0;

                    description.max.groups.push_back(group_state);
                    description.min.groups.push_back(group_state);
                    description.dflt.groups.push_back(group_state);

                    description.groups.push_back(group);
                }
                size_t group_id = group_map[group_name];
                description.groups[group_id].parameters.push_back(param_desc);

                continue;
            }

            auto double_param_it = double_params_.find(name);
            if (double_param_it != double_params_.end()) {

                dynamic_reconfigure::DoubleParameter param_min;
                param_min.name = double_param_it->second.qualifiedName();
                if (double_param_it->second.hasMin()) {
                    param_min.value = double_param_it->second.min();
                }
                else {
                    param_min.value = -10000;
                }
                description.min.doubles.push_back(param_min);

                auto param_max = param_min;
                if (double_param_it->second.hasMax()) {
                    param_max.value = double_param_it->second.max();
                }
                else {
                    param_max.value = 10000;
                }
                description.max.doubles.push_back(param_max);

                auto param_default = param_min;
                param_default.value = double_param_it->second.defaultValue();
                description.dflt.doubles.push_back(param_default);

                dynamic_reconfigure::ParamDescription param_desc;
                param_desc.name = double_param_it->second.qualifiedName();
                param_desc.type = "double";
                param_desc.level = 0;
                param_desc.description = double_param_it->second.description();

                std::string group_name = double_param_it->second.group();
                if (group_name.empty()) {
                    group_name = "default";
                }
                if (group_map.count(group_name) == 0) {
                    size_t group_id = group_map.size();
                    group_map[group_name] = group_id;

                    dynamic_reconfigure::Group group;
                    group.name = group_name;
                    group.type = "";
                    group.id = group_id;
                    group.parent = 0;

                    dynamic_reconfigure::GroupState group_state;
                    group_state.name = group_name;
                    group_state.state = true;
                    group_state.id = group_id;
                    group_state.parent = 0;

                    description.max.groups.push_back(group_state);
                    description.min.groups.push_back(group_state);
                    description.dflt.groups.push_back(group_state);

                    description.groups.push_back(group);
                }
                size_t group_id = group_map[group_name];
                description.groups[group_id].parameters.push_back(param_desc);

                continue;
            }

            auto int_param_it = int_params_.find(name);
            if (int_param_it != int_params_.end()) {

                dynamic_reconfigure::IntParameter param_min;
                param_min.name = int_param_it->second.qualifiedName();
                if (int_param_it->second.hasMin()) {
                    param_min.value = int_param_it->second.min();
                }
                else {
                    param_min.value = -10000;
                }
                description.min.ints.push_back(param_min);

                auto param_max = param_min;
                if (int_param_it->second.hasMax()) {
                    param_max.value = int_param_it->second.max();
                }
                else {
                    param_max.value = 10000;
                }
                description.max.ints.push_back(param_max);

                auto param_default = param_min;
                param_default.value = int_param_it->second.defaultValue();
                description.dflt.ints.push_back(param_default);

                dynamic_reconfigure::ParamDescription param_desc;
                param_desc.name = int_param_it->second.qualifiedName();
                param_desc.type = "int";
                param_desc.level = 0;
                param_desc.description = int_param_it->second.description();
                param_desc.edit_method = getEditMethod(int_param_it->second);

                std::string group_name = int_param_it->second.group();
                if (group_name.empty()) {
                    group_name = "default";
                }
                if (group_map.count(group_name) == 0) {
                    size_t group_id = group_map.size();
                    group_map[group_name] = group_id;

                    dynamic_reconfigure::Group group;
                    group.name = group_name;
                    group.type = "";
                    group.id = group_id;
                    group.parent = 0;

                    dynamic_reconfigure::GroupState group_state;
                    group_state.name = group_name;
                    group_state.state = true;
                    group_state.id = group_id;
                    group_state.parent = 0;

                    description.max.groups.push_back(group_state);
                    description.min.groups.push_back(group_state);
                    description.dflt.groups.push_back(group_state);

                    description.groups.push_back(group);
                }
                size_t group_id = group_map[group_name];
                description.groups[group_id].parameters.push_back(param_desc);

                continue;
            }

            auto string_param_it = string_params_.find(name);
            if (string_param_it != string_params_.end()) {
                dynamic_reconfigure::StrParameter param_min;
                param_min.name = string_param_it->second.qualifiedName();
                param_min.value = "";
                description.min.strs.push_back(param_min);
                description.max.strs.push_back(param_min);
                auto param_default = param_min;
                param_default.value = string_param_it->second.defaultValue();
                description.dflt.strs.push_back(param_default);

                dynamic_reconfigure::ParamDescription param_desc;
                param_desc.name = string_param_it->second.qualifiedName();
                param_desc.type = "str";
                param_desc.level = 0;
                param_desc.description = string_param_it->second.description();

                std::string group_name = string_param_it->second.group();
                if (group_name.empty()) {
                    group_name = "default";
                }
                if (group_map.count(group_name) == 0) {
                    size_t group_id = group_map.size();
                    group_map[group_name] = group_id;

                    dynamic_reconfigure::Group group;
                    group.name = group_name;
                    group.type = "";
                    group.id = group_id;
                    group.parent = 0;

                    dynamic_reconfigure::GroupState group_state;
                    group_state.name = group_name;
                    group_state.state = true;
                    group_state.id = group_id;
                    group_state.parent = 0;

                    description.max.groups.push_back(group_state);
                    description.min.groups.push_back(group_state);
                    description.dflt.groups.push_back(group_state);

                    description.groups.push_back(group);
                }
                size_t group_id = group_map[group_name];
                description.groups[group_id].parameters.push_back(param_desc);

                continue;
            }
        }

        group_states_ = description.dflt.groups;
        description_pub_->publish(description);

        publishUpdate();

        description_changed_ = false;
    }

    bool configCallback(dynamic_reconfigure::Reconfigure::Request &req, dynamic_reconfigure::Reconfigure::Response &rsp) {
        // update the parameters
        for (const auto& param: req.config.bools) {
            mutex_.lock();
            if (bool_params_.count(param.name) != 0) {
                auto bool_param = bool_params_[param.name];
                mutex_.unlock();
                bool_param.update(param.value, true);
            }
            else {
                mutex_.unlock();
            }
        }

        for (const auto& param: req.config.doubles) {
            mutex_.lock();
            if (double_params_.count(param.name) != 0) {
                auto double_param = double_params_[param.name];
                mutex_.unlock();
                double_param.update(param.value, true);
            }
            else {
                mutex_.unlock();
            }
        }

        for (const auto& param: req.config.ints) {
            mutex_.lock();
            if (int_params_.count(param.name) != 0) {
                auto int_param = int_params_[param.name];
                mutex_.unlock();
                int_param.update(param.value, true);
            }
            else {
                mutex_.unlock();
            }
        }

        for (const auto& param: req.config.strs) {
            mutex_.lock();
            if (string_params_.count(param.name) != 0) {
                auto string_param = string_params_[param.name];
                mutex_.unlock();
                string_param.update(param.value, true);
            }
            else {
                mutex_.unlock();
            }
        }

        std::scoped_lock lock(mutex_);
        publishUpdate();

        return true;
    }

    void publishUpdate() {
        if (!update_pub_) {
            ROS_WARN("Update publisher has been shutdown.");
            return;
        }

        ROS_DEBUG("sending param update");
        dynamic_reconfigure::Config config;

        for (const auto& p: bool_params_) {
            dynamic_reconfigure::BoolParameter param;
            param.name = p.second.qualifiedName();
            param.value = p.second.value();
            config.bools.push_back(param);
        }

        for (const auto& p: double_params_) {
            dynamic_reconfigure::DoubleParameter param;
            param.name = p.second.qualifiedName();
            param.value = p.second.value();
            config.doubles.push_back(param);
        }

        for (const auto& p: int_params_) {
            dynamic_reconfigure::IntParameter param;
            param.name = p.second.qualifiedName();
            param.value = p.second.value();
            config.ints.push_back(param);
        }

        for (const auto& p: string_params_) {
            dynamic_reconfigure::StrParameter param;
            param.name = p.second.qualifiedName();
            param.value = p.second.value();
            config.strs.push_back(param);
        }

        config.groups = group_states_;
        update_pub_->publish(config);

        values_changed_ = false;
    }

    void publishUpdate(const std::string& name) {
        std::scoped_lock lock(mutex_);
        values_changed_ = true;
    }

    void descriptionChanged() {
        std::scoped_lock lock(mutex_);
        description_changed_ = true;
    }

    static std::string getEditMethod(const IntParameter& param) {
        if (param.enums().empty()) {
            return {};
        }

        // Based on https://github.com/awesomebytes/ddynamic_reconfigure's implementation
        std::stringstream ret;
        ret << "{";
        ret << "'enum_description': '" << param.description() << "', ";
        ret << "'enum': [";
        const auto& enums = param.enums();
        auto it = enums.cbegin();
        ret << getEnumEntry(*it);
        for (it++; it != enums.cend(); it++) {
            ret << ", " << getEnumEntry(*it);
        }
        ret << "]";
        ret << "}";
        return ret.str();
    }

    static std::string getEnumEntry(const EnumOption& option) {
        std::stringstream ret;
        ret << "{";
        ret << "'srcline': 0, ";
        ret << "'description': '" << option.description << "', ";
        ret << "'srcfile': '', ";
        ret << "'cconsttype': 'const int', ";
        ret << "'value': " << option.value << ", ";
        ret << "'ctype': 'int', ";
        ret << "'type': 'int', ";
        ret << "'name': '" << option.name << "'";
        ret << "}";
        return ret.str();
    }

    ros::NodeHandle node_;
    bool description_changed_ = false;
    bool values_changed_ = false;
    std::mutex mutex_;

    ros::WallTimer description_timer_;
    ros::ServiceServer config_service_;
    std::shared_ptr<ros::Publisher> description_pub_;
    std::shared_ptr<ros::Publisher> update_pub_;

    std::vector<std::string> param_order_;
    std::unordered_set<std::string> registered_;
    std::vector<dynamic_reconfigure::GroupState> group_states_;
    std::unordered_map<std::string, BoolParameter> bool_params_;
    std::unordered_map<std::string, DoubleParameter> double_params_;
    std::unordered_map<std::string, IntParameter> int_params_;
    std::unordered_map<std::string, StringParameter> string_params_;
};

}  // hatchbed_common
