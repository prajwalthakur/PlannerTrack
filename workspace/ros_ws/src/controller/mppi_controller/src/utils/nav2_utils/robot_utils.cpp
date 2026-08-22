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
#include "mppi_controller/utils/nav2_utils/robot_utils.hpp"

/** \file
 * \brief `Twist` validation and TF pose-lookup/transform helpers implementation.
 */
#include "tf2/convert.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav2_util/robot_utils.hpp"
#include "rclcpp/logger.hpp"
namespace controller::mppi_controller::utils
{

    bool validateTwist(const geometry_msgs::msg::Twist & msg)
    {
        if (std::isinf(msg.linear.x) || std::isnan(msg.linear.x)) {
            return false;
        }

        if (std::isinf(msg.linear.y) || std::isnan(msg.linear.y)) {
            return false;
        }

        if (std::isinf(msg.linear.z) || std::isnan(msg.linear.z)) {
            return false;
        }

        if (std::isinf(msg.angular.x) || std::isnan(msg.angular.x)) {
            return false;
        }

        if (std::isinf(msg.angular.y) || std::isnan(msg.angular.y)) {
            return false;
        }

        if (std::isinf(msg.angular.z) || std::isnan(msg.angular.z)) {
            return false;
        }

        return true;
    }
    // get the position of the robot in the global frame
    bool getCurrentPose(
    geometry_msgs::msg::PoseStamped & global_pose,
    tf2_ros::Buffer & tf_buffer, const std::string global_frame,
    const std::string robot_frame, const double transform_timeout,
    const rclcpp::Time stamp, Logger& logger)
    {
        tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
        global_pose.header.frame_id = robot_frame;
        global_pose.header.stamp = stamp;
        return transformPoseInTargetFrame(global_pose, 
            global_pose, 
            tf_buffer, 
            global_frame, 
            transform_timeout, logger);
    }
        
    bool transformPoseInTargetFrame(
    const geometry_msgs::msg::PoseStamped & input_pose,
    geometry_msgs::msg::PoseStamped & transformed_pose,
    tf2_ros::Buffer & tf_buffer, const std::string target_frame,
    const double transform_timeout, Logger& logger)
    {
        if(input_pose.header.frame_id == target_frame)
        {
            transformed_pose = input_pose;
            return true;
        }
        try {
            transformed_pose = tf_buffer.transform(input_pose,target_frame,tf2::durationFromSec(transform_timeout));
            return true;
        } catch (tf2::LookupException& ex){
            logger.error("No Transform available Error looking up target frame: %s\n", ex.what());

        } catch (tf2::ConnectivityException & ex){
            logger.error("Connectivity Error looking up target frame: %s\n", ex.what());
        } catch (tf2::ExtrapolationException & ex) {
            logger.error(
            "Extrapolation Error looking up target frame: %s\n", ex.what());
         } catch (tf2::TimeoutException & ex) {
            logger.error(
            "Transform timeout with tolerance: %.4f", transform_timeout);
        } catch (tf2::TransformException & ex) {
            logger.error( "Failed to transform from %s to %s",
            input_pose.header.frame_id.c_str(), target_frame.c_str());
        }
        return false;
    }
    
}  // end namespace nav2_util

namespace mppi_utils = controller::mppi_controller::utils;