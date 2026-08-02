#pragma once
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "mppi_controller/utils/types.hpp"
namespace controller::mppi_controller
{
    struct OutputData
    {
        
        bool  mPublishZeroVelocity{false};
        geometry_msgs::msg::TwistStamped mControlCommand;
        mppi_mt::ArrayXX mOptimalTrajectory;
    };
} // controller::mppi_controller