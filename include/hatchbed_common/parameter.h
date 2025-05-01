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
#include <type_traits>
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

template <class T>
struct EnumOption {
    T value;
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

    // NOTE: This class uses method chaining (i.e. modifiers to the parameter return the object reference so that you can chain the modifiers together)
    //   e.g. params.param("param_name", 0).min(0).max(10).step(1).callback([](int value) { ... }).declare();
    //   Any child class that creates a new method should override any parent class chaining methods to return the child class type.
    //   e.g. class ChildParameter : public Parameter<int> {
    //       virtual ChildParameter& dynamic() override {
    //           ParameterBase<int>::dynamic();
    //           return *this;
    //       }
    //   };
    //   Otherwise the return value will be of type ParameterBase<T> and not the child class type and any child class methods will not be available.

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

    // general case ss works for all basic types
    template<typename U>
    std::string toString(const U& value) const {
        std::stringstream ss;
        ss << value;
        return ss.str();
    }

    // overload for bool (ss will just set 1 or 0 for true/false)
    // instead, this overload returns true/false
    std::string toString(const bool& value) const {
        return value ? "true" : "false";
    }

    // overload for vector
    template<typename U>
    std::string toString(const std::vector<U>& values) const {
        std::stringstream ss;
        ss << "[";
        for (const auto& value : values) {
            ss << toString(value);
            if (value != values.back()) {
                ss << ", ";
            }
        }
        ss << "]";
        return ss.str();
    }

    virtual void registerParam() {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = description_;
        descriptor.read_only = !is_dynamic_;

        try {
            node_->declare_parameter(name_, rclcpp::ParameterValue(default_val_), descriptor);
        }
        catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException& e) {
            RCLCPP_WARN(node_->get_logger(), "Parameter [%s] already declared: %s", name_.c_str(),
                        e.what());
        }

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

template <typename T, typename U=T>
class NumericParameter : public Parameter<T> {
    public:
    NumericParameter(T* store, const std::string& ns, const std::string& name, T default_val,
                     const std::string& description, std::shared_ptr<rclcpp::Node> node)
      : Parameter<T>(store, ns, name, default_val, description, node) {}

    NumericParameter() = default;
    NumericParameter(const NumericParameter& parameter) = default;
    virtual ~NumericParameter() = default;

    virtual NumericParameter<T, U>& callback(std::function<void(T)> callback) override {
        Parameter<T>::callback(callback);
        return *this;
    }

    virtual NumericParameter<T, U>& dynamic() override {
        Parameter<T>::dynamic();
        return *this;
    }

    virtual NumericParameter<T, U>& min(U min) {
        min_ = min;
        has_range_ = true;
        if (max_ < min_) {
            max_ = min_;
        }
        this->default_val_ = clamp(this->default_val_);
        return *this;
    }

    virtual NumericParameter<T, U>& max(U max) {
        max_ = max;
        has_range_ = true;
        if (min_ > max_) {
            min_ = max_;
        }
        this->default_val_ = clamp(this->default_val_);
        return *this;
    }

    virtual NumericParameter<T, U>& step(U step) {
        step_ = step;
        return *this;
    }

    U min() const {
        return min_;
    }

    bool hasRange() const {
        return has_range_;
    }


    U max() const {
        return max_;
    }

    U clamp(const U& value) const {
        U clamped = value;

        if (has_range_ && clamped < min_) {
            clamped = min_;
        }
        if (has_range_ && clamped > max_) {
            clamped = max_;
        }

        return clamped;
    }

    std::vector<U> clamp(const std::vector<U>& values) const {
        std::vector<U> clamped;
        for (const auto& value : values) {
            clamped.push_back(clamp(value));
        }
        return clamped;
    }

    protected:
    bool checkRange(const U& value) const {
        if (has_range_ && (value < min_ || value > max_)) {
            RCLCPP_WARN_STREAM(this->node_->get_logger(),  this->namespace_ << "/" << this->name_
                               << ": value out of range <" << value << ">");
            return false;
        }
        return true;
    }

    bool checkRange(const std::vector<U>& values) const {
        for (const auto& value : values) {
            if (!checkRange(value)) {
                return false;
            }
        }
        return true;
    }

    virtual bool update(const T& value, bool from_callback = false) {
        if (!checkRange(value)) {
            return false;
        }

        return Parameter<T>::update(value, from_callback);
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
        try {
            this->node_->declare_parameter(this->name_, rclcpp::ParameterValue(this->default_val_), descriptor);
        }
        catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException& e) {
            RCLCPP_WARN(this->node_->get_logger(), "Parameter [%s] already declared: %s", 
                        this->name_.c_str(), e.what());
        }

        this->update(this->getParameter());

        RCLCPP_INFO_STREAM(this->node_->get_logger(), this->namespace_ << "/" << this->name_ << ": " << this->toString(this->value()));
        this->initialized_ = true;
    }

    U min_ = -10000;
    U max_ = 10000;
    U step_ = 0;
    bool has_range_ = false;

    friend class ParamHandler;
};

// Specialization for int (extends enum handling)
template <typename T, typename U=T>
class NumericIntParameter : public NumericParameter<T, U> {
    public:
    NumericIntParameter(T* store, const std::string& ns, const std::string& name, T default_val,
                        const std::string& description, std::shared_ptr<rclcpp::Node> node)
      : NumericParameter<T, U>(store, ns, name, default_val, description, node) {}

    NumericIntParameter() = default;
    NumericIntParameter(const NumericIntParameter& parameter) = default;
    virtual ~NumericIntParameter() = default;

    virtual NumericIntParameter<T, U>& callback(std::function<void(T)> callback) override {
        NumericParameter<T, U>::callback(callback);
        return *this;
    }

    virtual NumericIntParameter<T, U>& dynamic() override {
        NumericParameter<T, U>::dynamic();
        return *this;
    }

    virtual NumericIntParameter<T, U>& min(U min) override {
        if (enums_.empty()) {
            NumericParameter<T, U>::min(min);
        }
        return *this;
    }

    virtual NumericIntParameter<T, U>& max(U max) override {
        if (enums_.empty()) {
            NumericParameter<T, U>::max(max);
        }
        return *this;
    }

    virtual NumericIntParameter<T, U>& step(U step) {
        if (enums_.empty()) {
            NumericParameter<T, U>::step(step);
        }
        return *this;
    }

    virtual NumericIntParameter<T, U>& enumerate(const std::vector<EnumOption<U>>& enums) {
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

    const std::vector<EnumOption<U>>& enums() const {
        return enums_;
    }

    bool checkEnum(const U& value) const {
        for (const auto& option: enums_) {
            if (option.value == value) {
                return true;
            }
        }
        return false;
    }

    // overload for vector
    bool checkEnum(const std::vector<U>& values) const {
        for (const auto& value : values) {
            if (!checkEnum(value)) {
                return false;
            }
        }
        return true;
    }

    protected:
    virtual bool update(const T& value, bool from_callback = false) {
        if (enums_.empty()) {
            return NumericParameter<T, U>::update(value, from_callback);
        }

        if (!checkEnum(value)) {
            return false;
        }

        return Parameter<T>::update(value, from_callback);
    }

    // overload for U
    std::string toString(const U& value) const {
        std::stringstream ss;
        ss << value;
        for (const auto& option: enums_) {
            if (option.value == value) {
                ss << " (" + option.name + ")";
            }
        }
        return ss.str();
    }

    // overload for vector U
    std::string toString(const std::vector<U>& values) const {
        std::stringstream ss;
        ss << "[";
        for (const auto& value : values) {
            ss << toString(value);
            if (value != values.back()) {
                ss << ", ";
            }
        }
        ss << "]";
        return ss.str();
    }

    virtual void registerParam() override {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = this->description_;

        // generate enum description
        if (!enums_.empty()) {
            descriptor.description += "\nEnumeration:";
            for (const auto& option: enums_) {
                descriptor.description += "\n  " + option.name + "(" + std::to_string(option.value) + "): " + option.description;
            }
        }

        descriptor.read_only = !this->is_dynamic_;

        if (this->has_range_ || !enums_.empty()) {
            descriptor.integer_range.resize(1);
            descriptor.integer_range[0].from_value = this->min_;
            descriptor.integer_range[0].to_value = this->max_;
            descriptor.integer_range[0].step = this->step_;
        }

        try {
            this->node_->declare_parameter(this->name_, rclcpp::ParameterValue(this->default_val_), descriptor);
        }
        catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException& e) {
            RCLCPP_INFO(this->node_->get_logger(), "Parameter [%s] already declared: %s", this->name_.c_str(), 
                        e.what());
        }

        this->update(this->getParameter());

        RCLCPP_INFO_STREAM(this->node_->get_logger(), this->namespace_ << "/" << this->name_ << ": " << this->toString(this->value()));
        this->initialized_ = true;
    }

    std::vector<EnumOption<U>> enums_;

    friend class ParamHandler;
};

template class Parameter<bool>;
template class Parameter<std::vector<bool>>;
template class Parameter<std::string>;
template class Parameter<std::vector<std::string>>;
template class NumericIntParameter<int>;
template class NumericIntParameter<int64_t>;
template class NumericIntParameter<std::vector<int64_t>, int64_t>;
template class NumericParameter<double>;
template class NumericParameter<std::vector<double>, double>;

typedef Parameter<bool> BoolParameter;
typedef Parameter<std::vector<bool>> BoolArrayParameter;
typedef Parameter<std::string> StringParameter;
typedef Parameter<std::vector<std::string>> StringArrayParameter;
typedef NumericIntParameter<int> SystemIntParameter; // support for legacy int parameters, they are stil int64_t in ROS, though
typedef NumericIntParameter<int64_t> IntParameter;
typedef NumericIntParameter<std::vector<int64_t>, int64_t> IntArrayParameter;
typedef NumericParameter<double> DoubleParameter;
typedef NumericParameter<std::vector<double>, double> DoubleArrayParameter;

}  // namespace hatchbed_common
