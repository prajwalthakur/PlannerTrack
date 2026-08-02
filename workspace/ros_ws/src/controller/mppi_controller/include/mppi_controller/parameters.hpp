#pragma once
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <variant>
#include <string>

namespace controller::mppi_controller
{

// using ParamValue = std::variant<int64_t, float, bool, std::string, std::vector<std::string>>;

struct Parameters
{
    // std::unordered_map<std::string, ParamValue> params;

    Parameters(rclcpp::Node& node):mNode{node}
    {
    }

    auto getParamGetter(const std::string & prefix)
    {
        return [this, prefix](auto & value,
                            const std::string & name,
                            const auto & default_val)
        {
            using T = std::decay_t<decltype(default_val)>;

            std::string full = prefix.empty() ? name : prefix + "." + name;
            //std::string full = prefix + "." + name;
            if (!mNode.has_parameter(full)) {
                value = mNode.declare_parameter<T>(full, default_val);
            } else {
                value = mNode.get_parameter(full).get_value<T>();
            }
        };
    }

    template<typename T>
    void addParamCallback(const std::string & name, T && callback)
    {
        get_param_callbacks_[name] = callback;
    }

    // template<typename T>
    // T getParameter(const std::string& paramName) const
    // {
    //     auto itr = params.find(paramName);

    //     // if (itr == params.end())
    //     //     return std::nullopt;

    //     // if (!std::holds_alternative<T>(itr->second))
    //     //     return std::nullopt;
    //     if (itr == params.end())
    //         throw std::runtime_error("Parameter not found: " + paramName);
    //     return std::get<T>(itr->second);
    // }

    // template<typename T>
    // bool updateParameter(const std::string& paramName, const T& value)
    // {
    //     auto itr = params.find(paramName);
        
    //     if(itr == params.end())
    //         return false;

    //     if (!std::holds_alternative<T>(itr->second))
    //         return false;

    //     itr->second = value;
    //     return true;

    // }

    // float declareFloat(rclcpp::Node& node, const std::string& name, double default_val)
    // {
    //     return static_cast<float>(node.declare_parameter<double>(name, default_val));
    // }

    std::unordered_map<std::string, std::function<void(const rclcpp::Parameter &)>> get_param_callbacks_;
    rclcpp::Node& mNode;
};

} // namespace controller::mppi_controller

namespace mppi_controller = controller::mppi_controller;