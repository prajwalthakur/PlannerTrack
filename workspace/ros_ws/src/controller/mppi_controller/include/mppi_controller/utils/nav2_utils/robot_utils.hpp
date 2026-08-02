// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2019 Steven Macenski
// Copyright (c) 2019 Samsung Research America
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

#include <string>
#include <cmath>
#include <memory>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2/utils.hpp"
#include "mppi_controller/utils/logger_utils.hpp"
namespace controller::mppi_controller::utils
{

    bool validateTwist(const geometry_msgs::msg::Twist & msg);

    bool getCurrentPose(
    geometry_msgs::msg::PoseStamped & global_pose,
    tf2_ros::Buffer & tf_buffer, const std::string global_frame,
    const std::string robot_frame, const double transform_timeout,
    const rclcpp::Time stamp, Logger& logger);
        
    bool transformPoseInTargetFrame(
    const geometry_msgs::msg::PoseStamped & input_pose,
    geometry_msgs::msg::PoseStamped & transformed_pose,
    tf2_ros::Buffer & tf_buffer, const std::string target_frame,
    const double transform_timeout,Logger& logger);
}  // end namespace nav2_util

namespace mppi_utils = controller::mppi_controller::utils;