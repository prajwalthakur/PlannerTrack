#pragma once
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "mppi_controller/utils/types.hpp"

namespace controller::mppi_controller
{
    struct InputData
    {
        
        //geometry_msgs::msg::PoseArray mPlanToFollow;
        nav_msgs::msg::Path mPlanToFollow;
        geometry_msgs::msg::PoseStamped mGoalPose;
        //nav_msgs::msg::Odometry mCurrentOdom; // curent pose and speed 
        geometry_msgs::msg::Twist mSpeed;
        geometry_msgs::msg::PoseStamped mRobotPose;
        std::string mBaseFrameID;

        // optional not uses currently
        geometry_msgs::msg::AccelWithCovarianceStamped mCurrentAcc;
        //mppi_cpp::msg::SteeringReport mCurrentSteering;
    };
} // controller::mppi_controller