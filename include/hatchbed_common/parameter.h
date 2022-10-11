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

#include <functional>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>

namespace hatchbed_common {

template <class T>
struct OwnedStore {
    using Ptr = std::shared_ptr<OwnedStore>;
    OwnedStore(const T& value) : value(value) {}

    T value;
    std::mutex mutex;
};

template <class T>
struct BorrowedStore {
    using Ptr = std::shared_ptr<BorrowedStore>;
    BorrowedStore(T* value) : value(value) {}

    T* value;
    std::mutex mutex;
};

struct EnumOption {
    int value;
    std::string name;
    std::string description;
};

class ParamHandler;

template <class T>
class Parameter {
    public:
    class Declared {
        public:
        Declared(const Parameter& param) : param_(param) {}

        T value() { return param_.value(); }
        bool update(const T& value) { return param_.update(value, false); }

        private:
        Parameter param_;
    };

    Parameter(T* store, const std::string& ns, const std::string& name, T default_val, const std::string& description,
              std::shared_ptr<rclcpp::Node> node)
      : namespace_(ns),
        name_(name),
        default_val_(default_val),
        description_(description),
        node_(node)
    {
        if (!store) {
            owned_store_ = std::make_shared<OwnedStore<T>>(default_val_);
        }
        else {
            borrowed_store_ = std::make_shared<BorrowedStore<T>>(store);
            *(borrowed_store_->value) = default_val_;
        }
    }

    Parameter() = default;
    Parameter(const Parameter& parameter) = default;
    virtual ~Parameter() = default;

    virtual Declared declare() {
        registerParam();
        return Declared(*this);
    }

    virtual Parameter<T>& callback(std::function<void(T)> callback) {
        is_dynamic_ = true;
        user_callback_ = callback;
        return *this;
    }

    virtual Parameter<T>& dynamic() {
        is_dynamic_ = true;
        return *this;
    }

    T defaultValue() const {
        return default_val_;
    }

    std::string qualifiedName() const {
        if (!is_dynamic_) {
            return name_ + " (readonly)";
        }
        return name_;
    }

    std::string description() const {
        return description_;
    }

    bool isDynamic() const {
        return is_dynamic_;
    }

    protected:
    T value() const {
        if (owned_store_) {
            std::scoped_lock lock(owned_store_->mutex);
            return owned_store_->value;
        }

        std::scoped_lock lock(borrowed_store_->mutex);
        return *borrowed_store_->value;
    }

    virtual bool update(const T& value, bool from_callback = false) {
        if (initialized_ && !is_dynamic_) {
            RCLCPP_WARN_STREAM(node_->get_logger(), namespace_ << "/" << name_ << " is static and can't be updated.");
            return false;
        }

        bool did_change = false;
        if (owned_store_) {
            std::scoped_lock lock(owned_store_->mutex);
            did_change = owned_store_->value != value;
            if (did_change && from_callback) {
                RCLCPP_INFO_STREAM(node_->get_logger(), "updated <" << namespace_ << "/" << name_ << ">: "
                    << toString(owned_store_->value) << " to " << toString(value));
            }
            owned_store_->value = value;
        }
        else {
            std::scoped_lock lock(borrowed_store_->mutex);
            did_change = *borrowed_store_->value != value;
            if (did_change && from_callback) {
                RCLCPP_INFO_STREAM(node_->get_logger(), "updated <" << namespace_ << "/" << name_ << ">: "
                    << toString(*borrowed_store_->value) <<" to " << toString(value));
            }
            *borrowed_store_->value = value;
        }

        if (!from_callback && did_change) {
            node_->set_parameter({ name_, rclcpp::ParameterValue(this->value()) });
        }

        if (user_callback_ && did_change) {
            user_callback_(value);
        }

        return true;
    }

    virtual std::string toString(const T& value) const {
        std::stringstream ss;
        ss << value;
        return ss.str();
    }

    virtual void registerParam() {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = description_;
        descriptor.read_only = !is_dynamic_;
        node_->declare_parameter(name_, rclcpp::ParameterValue(default_val_), descriptor);

        update(getParameter());
        RCLCPP_INFO_STREAM(this->node_->get_logger(), this->namespace_ << "/" << this->name_ << ": " << toString(this->value()));
        initialized_ = true;
    }

    T getParameter() {
        auto p = node_->get_parameter(name_);
        return p.get_value<T>();
    }

    std::string namespace_;
    std::string name_;
    T default_val_;
    std::string description_;
    std::shared_ptr<rclcpp::Node> node_;
    typename BorrowedStore<T>::Ptr borrowed_store_;
    typename OwnedStore<T>::Ptr owned_store_;

    bool initialized_ = false;
    bool is_dynamic_ = false;
    std::function<void(T)> user_callback_;

    friend class ParamHandler;
};

class BoolParameter : public Parameter<bool> {
    public:
    BoolParameter(bool* store, const std::string& ns, const std::string& name, bool default_val,
                  const std::string& description, std::shared_ptr<rclcpp::Node> node)
      : Parameter<bool>(store, ns, name, default_val, description, node) {}

    BoolParameter() = default;
    BoolParameter(const BoolParameter& parameter) = default;
    virtual ~BoolParameter() = default;

    protected:
    virtual std::string toString(const bool& value) const override {
        if (value) {
            return "true";
        }
        return "false";
    }
};

template <typename T>
class NumericParameter : public Parameter<T> {
    public:
    NumericParameter(T* store, const std::string& ns, const std::string& name, T default_val,
                     const std::string& description, std::shared_ptr<rclcpp::Node> node)
      : Parameter<T>(store, ns, name, default_val, description, node) {}

    NumericParameter() = default;
    NumericParameter(const NumericParameter& parameter) = default;
    virtual ~NumericParameter() = default;

    virtual NumericParameter<T>& callback(std::function<void(T)> callback) override {
        Parameter<T>::callback(callback);
        return *this;
    }

    virtual NumericParameter<T>& dynamic() override {
        Parameter<T>::dynamic();
        return *this;
    }

    virtual NumericParameter<T>& min(T min) {
        min_ = min;
        has_range_ = true;
        if (max_ < min_) {
            max_ = min_;
        }
        this->default_val_ = clamp(this->default_val_);
        return *this;
    }

    virtual NumericParameter<T>& max(T max) {
        max_ = max;
        has_range_ = true;
        if (min_ > max_) {
            min_ = max_;
        }
        this->default_val_ = clamp(this->default_val_);
        return *this;
    }

    virtual NumericParameter<T>& step(T step) {
        step_ = step;
        return *this;
    }

    T min() const {
        return min_;
    }

    bool hasRange() const {
        return has_range_;
    }


    T max() const {
        return max_;
    }

    T clamp(const T& value) const {
        T clamped = value;

        if (has_range_ && clamped < min_) {
            clamped = min_;
        }
        if (has_range_ && clamped > max_) {
            clamped = max_;
        }

        return clamped;
    }

    protected:
    virtual bool update(const T& value, bool from_callback = false) {
        if (has_range_ && (value < min_ || value > max_)) {
            RCLCPP_WARN_STREAM(this->node_->get_logger(),  this->namespace_ << "/" << this->name_
                               << ": value out of range <" << value << ">");
            return false;
        }

        return Parameter<T>::update(value, from_callback);
    }

    virtual std::string toString(const T& value) const override {
        return std::to_string(value);
    }

    virtual void registerParam() override {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = this->description_;
        descriptor.read_only = !this->is_dynamic_;
        if (has_range_) {
            descriptor.floating_point_range.resize(1);
            descriptor.floating_point_range[0].from_value = min_;
            descriptor.floating_point_range[0].to_value = max_;
            descriptor.floating_point_range[0].step = step_;
        }
        this->node_->declare_parameter(this->name_, rclcpp::ParameterValue(this->default_val_), descriptor);
        this->update(this->getParameter());

        RCLCPP_INFO_STREAM(this->node_->get_logger(), this->namespace_ << "/" << this->name_ << ": " << toString(this->value()));
        this->initialized_ = true;
    }

    T min_ = -10000;
    T max_ = 10000;
    T step_ = 0;
    bool has_range_ = false;

    friend class ParamHandler;
};

class IntParameter : public NumericParameter<int> {
    public:
    IntParameter(int* store, const std::string& ns, const std::string& name, int default_val,
                 const std::string& description, std::shared_ptr<rclcpp::Node> node)
      : NumericParameter<int>(store, ns, name, default_val, description, node) {}

    IntParameter() = default;
    IntParameter(const IntParameter& parameter) = default;
    virtual ~IntParameter() = default;

    virtual IntParameter& callback(std::function<void(int)> callback) override {
        NumericParameter<int>::callback(callback);
        return *this;
    }

    virtual IntParameter& dynamic() override {
        NumericParameter<int>::dynamic();
        return *this;
    }

    virtual IntParameter& min(int min) override {
        if (enums_.empty()) {
            NumericParameter<int>::min(min);
        }
        return *this;
    }

    virtual IntParameter& max(int max) override {
        if (enums_.empty()) {
            NumericParameter<int>::max(max);
        }
        return *this;
    }

    virtual IntParameter& step(int step) {
        if (enums_.empty()) {
            NumericParameter<int>::step(step);
        }
        return *this;
    }

    virtual IntParameter& enumerate(const std::vector<EnumOption>& enums) {
        enums_ = enums;
        if (!enums_.empty()) {
            this->has_range_ = false;
            this->step_ = 0;
            this->min_ = enums_.front().value;
            this->max_ = enums_.front().value;
            for (const auto& option: enums_) {
                this->min_ = std::min(this->min_, option.value);
                this->max_ = std::max(this->max_, option.value);
            }
        }
        return *this;
    }

    const std::vector<EnumOption>& enums() const {
        return enums_;
    }

    protected:
    virtual bool update(const int& value, bool from_callback = false) {
        if (enums_.empty()) {
            return NumericParameter<int>::update(value, from_callback);
        }

        bool valid = false;
        for (const auto& option: enums_) {
            if (option.value == value) {
                valid = true;
                break;
            }
        }

        if (valid) {
            return Parameter<int>::update(value, from_callback);
        }

        return false;
    }

    virtual std::string toString(const int& value) const override {
        std::string str = std::to_string(value);

        for (const auto& option: enums_) {
            if (option.value == value) {
                str += " (" + option.name + ")";
            }
        }
        return str;
    }

    virtual void registerParam() override {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = description_;

        // generate enum description
        if (!enums_.empty()) {
            descriptor.description += "\nEnumeration:";
            for (const auto& option: enums_) {
                descriptor.description += "\n  " + option.name + "(" + std::to_string(option.value) + "): " + option.description;
            }
        }

        descriptor.read_only = !is_dynamic_;

        if (has_range_ || !enums_.empty()) {
            descriptor.integer_range.resize(1);
            descriptor.integer_range[0].from_value = min_;
            descriptor.integer_range[0].to_value = max_;
            descriptor.integer_range[0].step = step_;
        }
        node_->declare_parameter(name_, rclcpp::ParameterValue(default_val_), descriptor);

        update(getParameter());

        RCLCPP_INFO_STREAM(node_->get_logger(), namespace_ << "/" << name_ << ": " << toString(value()));
        initialized_ = true;
    }

    std::vector<EnumOption> enums_;

    friend class ParamHandler;
};

template class Parameter<bool>;
template class Parameter<std::string>;
template class NumericParameter<int>;
template class NumericParameter<double>;

typedef Parameter<std::string> StringParameter;
typedef NumericParameter<double> DoubleParameter;

}  // namespace hatchbed_common
