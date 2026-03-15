#pragma once
#include "project_utils/msg/trajectory.hpp"
#include "project_utils/msg/steering_report.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace mpl::control::trajectory_follower
{
    struct InputData
    {
        project_utils::msg::Trajectory mCurrentTrajectory;
        nav_msgs::msg::Odometry mCurrentOdometry;
        geometry_msgs::msg::AccelWithCovarianceStamped mCurrentAcc;
        project_utils::msg::SteeringReport mCurrentSteering;
    };
} // mpl::control::trajectory_follower
