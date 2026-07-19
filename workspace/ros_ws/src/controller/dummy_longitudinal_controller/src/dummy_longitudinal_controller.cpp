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
#include "dummy_longitudinal_controller/dummy_longitudinal_controller.hpp"

namespace dummy_longitudinal_controller
{
    DummyLongitudinalController::DummyLongitudinalController([[maybe_unused]] rclcpp::Node & node)
    :mClock(node.get_clock())
    {
        auto tempLogger = node.get_logger();
        mLogger = mpl::rclcpp_utils::Logger(node.get_logger(),"Longitudinal_Controller");
        mLogger.info("Dummy longitudinal controller is initiated");
    }

    ////////////////////////////////////////////////////////////////////////////////

    bool DummyLongitudinalController::isReady([[maybe_unused]] const InputData & input_data)
    {
        return true;
    }
    
    ////////////////////////////////////////////////////////////////////////////////

    LongitudinalOutput DummyLongitudinalController::run([[maybe_unused]] const InputData & input_data)
    {

        LongitudinalOutput output;
        mLogger.info("running long controller");
        [[maybe_unused]] auto current_pose_ = input_data.mCurrentOdometry.pose.pose;
        mLogger.info("running long controller current pose of robot %.3f, %.3f, %.3f",current_pose_.position.x, current_pose_.position.y, current_pose_.position.z);
        // [[maybe_unused]] auto current_pose_ = input_data.mCurrentOdometry->pose.pose;
        // [[maybe_unused]] auto trajectory_ = input_data.mCurrentTrajectory;
        // [[maybe_unused]] auto current_odometry_ = input_data.mCurrentOdometry;
        // [[maybe_unused]] auto current_steering_ = input_data.mCurrentSteering;
        project_utils_msgs::msg::Longitudinal  cmd_;
        cmd_.stamp         =    mClock->now();
        cmd_.velocity      =    0.0;
        cmd_.acceleration  =    0.0;
        output.mControlCmd =    cmd_;
        return output;
    }

}//namespace dummy_longitudinal_controller