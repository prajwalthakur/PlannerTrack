#pragma once 
#include "trajectory_follower_base/control_horizon.h"
#include "trajectory_follower_base/input_data.h"
#include "trajectory_follower_base/sync_data.h"
#include "rclcpp/rclcpp.hpp"

namespace mpl::control::trajectory_follower
{
    class ControllerBase
    {
        public:
            ControllerBase()=default;
            virtual ~ControllerBase()=default;
            virtual bool isReady(const InputData& inputData)=0;

    };
}
