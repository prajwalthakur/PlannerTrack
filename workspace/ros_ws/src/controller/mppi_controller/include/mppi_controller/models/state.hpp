#pragma once
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include "mppi_controller/utils/types.hpp"
namespace controller::mppi_controller::models
{
    struct State
    {
        virtual ~State()=default;

        virtual void reset(unsigned int batch_size, unsigned int time_steps)=0;
        
        mppi_mt::ArrayXX vx;
        mppi_mt::ArrayXX wz;
        mppi_mt::ArrayXX cvx;
        mppi_mt::ArrayXX cwz;

        geometry_msgs::msg::PoseStamped pose;
        geometry_msgs::msg::Twist speed;
        float local_path_length;
        float distance_to_goal;

    };
}//namespace controller::mppi_controller

namespace model = controller::mppi_controller::models;
