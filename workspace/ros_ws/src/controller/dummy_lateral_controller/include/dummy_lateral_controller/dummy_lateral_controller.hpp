// Copyright 2026 Prajwal Thakur
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
#include <optional>
#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>

// #include "trajectory_follower_base/lateral_controller_base.h"
#include "trajectory_follower_base/trajectory_follower_base_collection.h"

// msgs
// ros 
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
// custom msgs
#include "project_utils_msgs/msg/lateral.hpp"
#include "project_utils_msgs/msg/trajectory.hpp"


using mpl::control::trajectory_follower::InputData;
using mpl::control::trajectory_follower::LateralControllerBase;
using mpl::control::trajectory_follower::LateralOutput;
using project_utils_msgs::msg::Lateral;
using project_utils_msgs::msg::Trajectory;
using project_utils_msgs::msg::TrajectoryPoint;



namespace dummy_lateral_controller
{

    class DummyLateralController : public LateralControllerBase
    {

        public:
            /// \param node Reference to the node used only for the component and parameter initialization.
            explicit DummyLateralController([[maybe_unused]] rclcpp::Node & node);
            ~DummyLateralController() override =default;
        private:
            /**
            * @brief compute control command for path follow with a constant control period
            */
            bool isReady([[maybe_unused]] const InputData & input_data) override;

            LateralOutput run(const InputData & input_data) override;
        private:
            rclcpp::Clock::SharedPtr mClock{nullptr};

        
    };

}//namespace dummy_lateral_controller