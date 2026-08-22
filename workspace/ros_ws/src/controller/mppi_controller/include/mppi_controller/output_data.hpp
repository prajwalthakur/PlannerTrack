#pragma once
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "mppi_controller/utils/types.hpp"
namespace controller::mppi_controller
{
    /**
     * \brief Per-cycle output of \c controller::mppi_controller::Optimizer::computeControl --
     * the commanded twist plus the optimized trajectory (for visualization),
     * converted back to the shared \c HybridOutput by
     * \c MPPIController::toHybridOutput.
     */
    struct OutputData
    {
        
        bool  mPublishZeroVelocity{false};
        geometry_msgs::msg::TwistStamped mControlCommand;
        mppi_mt::ArrayXX mOptimalTrajectory;
    };
} // controller::mppi_controller