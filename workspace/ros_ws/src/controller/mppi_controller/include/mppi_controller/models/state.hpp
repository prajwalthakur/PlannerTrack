#pragma once
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include "mppi_controller/utils/types.hpp"
namespace controller::mppi_controller::models
{
    /**
     * \brief Per-rollout velocity state for a batch of MPPI trajectories:
     * `[batch_size x time_steps]` arrays of commanded (\ref cvx / \ref cwz)
     * and noised/actual (\ref vx / \ref wz) linear/angular velocity, plus
     * the current robot pose/speed and progress-along-path summary values.
     */
    struct State
    {
        virtual ~State()=default;

        /// \brief Resize all rollout arrays to `[batch_size x time_steps]` and zero them.
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
