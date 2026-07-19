// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace mpl::rclcpp_utils
{
  class Parameters{
      public:
        Parameters(rclcpp::Node& node):mNode{node}{}

        inline auto getParamGetter(const std::string & prefix)
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
      private:
        rclcpp::Node& mNode;
  };
  template <class T>
  T get_or_declare_parameter(rclcpp::Node & node, const std::string & name, const T& defaultVal)
  {
    return node.declare_parameter<T>(name, defaultVal);
  }


  template <class T>
  bool update_param(
    const std::vector<rclcpp::Parameter> & params, const std::string & name, T & value)
  {
    const auto itr = std::find_if(
      params.cbegin(), params.cend(),
      [&name](const rclcpp::Parameter & p) { return p.get_name() == name; });

    // Not found
    if (itr == params.cend()) {
      return false;
    }

    value = itr->template get_value<T>();
    return true;
  }

  // NOTE: This function does not appear to be used.
  template <class T>
  [[deprecated]] T wait_for_param(
    rclcpp::Node * node, const std::string & remote_node_name, const std::string & param_name)
  {
    std::chrono::seconds sec(1);

    auto param_client = std::make_shared<rclcpp::SyncParametersClient>(node, remote_node_name);

    while (!param_client->wait_for_service(sec)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service.");
        return {};
      }
      RCLCPP_INFO_THROTTLE(
        node->get_logger(), *node->get_clock(), 1000 /* ms */, "waiting for node: %s, param: %s\n",
        remote_node_name.c_str(), param_name.c_str());
    }

    if (param_client->has_parameter(param_name)) {
      return param_client->get_parameter<T>(param_name);
    }

    return {};
  }

}  // namespace mpl_rclcpp_utils
