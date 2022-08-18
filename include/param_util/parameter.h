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

#include <ros/ros.h>

namespace param_util {

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

template <class T>
class Parameter {
    public:
    Parameter(T* store, const std::string& ns, const std::string& name, T default_val, const std::string& description)
      : namespace_(ns),
        name_(name),
        default_val_(default_val),
        description_(description)
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

    virtual Parameter<T>& group(const std::string& group) {
        group_ = group;
        configChanged();
        return *this;
    }

    virtual Parameter<T>& callback(std::function<void(T)> callback) {
        is_dynamic_ = true;
        user_callback_ = callback;
        configChanged();
        return *this;
    }

    virtual Parameter<T>& dynamic() {
        is_dynamic_ = true;
        configChanged();
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

    std::string group() const {
        return group_;
    }

    bool isDynamic() const {
        return is_dynamic_;
    }

    T value() const {
        if (owned_store_) {
            std::scoped_lock lock(owned_store_->mutex);
            return owned_store_->value;
        }

        std::scoped_lock lock(borrowed_store_->mutex);
        return *borrowed_store_->value;
    }

    void setPublishCallback(std::function<void(const std::string&)> callback) {
        publish_callback_ = callback;
    }

    void setConfigChangeCallback(std::function<void(const std::string&)> callback) {
        config_change_callback_ = callback;
    }

    virtual bool update(const T& value, bool from_callback = false) {
        // don't update if this is from a callback and not a dynamic parameter
        if (from_callback && !is_dynamic_) {
            return false;
        }

        bool did_change = false;
        if (owned_store_) {
            std::scoped_lock lock(owned_store_->mutex);
            did_change = owned_store_->value != value;
            if (did_change) {
                ROS_INFO_STREAM("updated <" << namespace_ << "/" << name_ << ">: " << toString(owned_store_->value)
                    << " to " << toString(value));
            }
            owned_store_->value = value;
        }
        else {
            std::scoped_lock lock(borrowed_store_->mutex);
            did_change = *borrowed_store_->value != value;
            if (did_change) {
                ROS_INFO_STREAM("updated <" << namespace_ << "/" << name_ << ">: " << toString(*borrowed_store_->value)
                    <<" to " << toString(value));
            }
            *borrowed_store_->value = value;
        }

        if (publish_callback_ && (!from_callback && did_change)) {
            publish_callback_(name_);
        }

        if (user_callback_ && did_change) {
            user_callback_(value);
        }

        return true;
    }

    protected:
    virtual std::string toString(const T& value) const {
        std::stringstream ss;
        ss << value;
        return ss.str();
    }

    void configChanged() {
        if (config_change_callback_) {
            config_change_callback_(name_);
        }
    }

    std::string namespace_;
    std::string name_;
    T default_val_;
    std::string description_;
    typename BorrowedStore<T>::Ptr borrowed_store_;
    typename OwnedStore<T>::Ptr owned_store_;
    std::string group_;

    bool is_dynamic_ = false;
    std::function<void(T)> user_callback_;
    std::function<void(const std::string&)> publish_callback_;
    std::function<void(const std::string&)> config_change_callback_;
};

class BoolParameter : public Parameter<bool> {
    public:
    BoolParameter(bool* store, const std::string& ns, const std::string& name, bool default_val, const std::string& description)
      : Parameter<bool>(store, ns, name, default_val, description) {}

    BoolParameter() = default;
    BoolParameter(const BoolParameter& parameter) = default;
    virtual ~BoolParameter() = default;

    virtual BoolParameter& group(const std::string& group) override {
        Parameter<bool>::group(group);
        return *this;
    }

    virtual BoolParameter& callback(std::function<void(bool)> callback) override {
        Parameter<bool>::callback(callback);
        return *this;
    }

    virtual BoolParameter& dynamic() override {
        Parameter<bool>::dynamic();
        return *this;
    }

    std::string group() const {
        return Parameter<bool>::group();
    }

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
    NumericParameter(T* store, const std::string& ns, const std::string& name, T default_val, const std::string& description)
      : Parameter<T>(store, ns, name, default_val, description) {}

    NumericParameter() = default;
    NumericParameter(const NumericParameter& parameter) = default;
    virtual ~NumericParameter() = default;

    virtual NumericParameter<T>& group(const std::string& group) override {
        Parameter<T>::group(group);
        return *this;
    }

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
        has_min_ = true;
        if (max_ < min_) {
            max_ = min_;
        }
        T clamped = clamp(Parameter<T>::value());
        if (clamped != Parameter<T>::value()) {
            ROS_INFO_STREAM("clamped <" << this->namespace_ << "/" << this->name_ << ">: " << this->toString(Parameter<T>::value())
                <<" to " << this->toString(clamped));
            update(clamped);
        }
        this->configChanged();
        return *this;
    }

    virtual NumericParameter<T>& max(T max) {
        max_ = max;
        has_max_ = true;
        if (min_ > max_) {
            min_ = max_;
        }

        T clamped = clamp(Parameter<T>::value());
        if (clamped != Parameter<T>::value()) {
            ROS_INFO_STREAM("clamped <" << this->namespace_ << "/" << this->name_ << ">: " << this->toString(Parameter<T>::value())
                <<" to " << this->toString(clamped));
            update(clamped);
        }
        this->configChanged();
        return *this;
    }

    std::string group() const {
        return Parameter<T>::group();
    }

    T min() const {
        return min_;
    }

    bool hasMin() const {
        return has_min_;
    }


    T max() const {
        return max_;
    }

    bool hasMax() const {
        return has_max_;
    }

    T clamp(const T& value) const {
        T clamped = value;

        if (has_min_ && clamped < min_) {
            clamped = min_;
        }
        if (has_max_ && clamped > max_) {
            clamped = max_;
        }

        return clamped;
    }

    virtual bool update(const T& value, bool from_callback = false) override {
        T clamped = clamp(value);
        return Parameter<T>::update(clamped, from_callback);
    }

    protected:
    T min_ = -10000;
    T max_ = 10000;
    bool has_max_ = false;
    bool has_min_ = false;
};

class IntParameter : public NumericParameter<int> {
    public:
    IntParameter(int* store, const std::string& ns, const std::string& name, int default_val, const std::string& description)
      : NumericParameter<int>(store, ns, name, default_val, description) {}

    IntParameter() = default;
    IntParameter(const IntParameter& parameter) = default;
    virtual ~IntParameter() = default;

    virtual IntParameter& group(const std::string& group) override {
        NumericParameter<int>::group(group);
        return *this;
    }

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

    virtual IntParameter& enumerate(const std::vector<EnumOption>& enums) {
        enums_ = enums;
        configChanged();
        return *this;
    }

    std::string group() const {
        return NumericParameter<int>::group();
    }

    int min() const {
        return NumericParameter<int>::min();
    }

    int max() const {
        return NumericParameter<int>::max();
    }

    const std::vector<EnumOption>& enums() const {
        return enums_;
    }

    virtual bool update(const int& value, bool from_callback = false) override {
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

    protected:
    virtual std::string toString(const int& value) const override {
        for (const auto& option: enums_) {
            if (option.value == value) {
                return std::to_string(value) + " (" + option.name + ")";
            }
        }
        return std::to_string(value);
    }

    std::vector<EnumOption> enums_;
};

template class Parameter<bool>;
template class Parameter<std::string>;
template class NumericParameter<int>;
template class NumericParameter<double>;

typedef Parameter<std::string> StringParameter;
typedef NumericParameter<double> DoubleParameter;

}  // namespace param_util
