#pragma once
#include "mppi_controller/utils/types.hpp"
namespace controller::mppi_controller::models
{

    /**
    * @struct mppi::models::Trajectories
    * @brief The batch of rollout (x, y, yaw) trajectories produced by
    * integrating \ref State's velocities over time -- one row per rollout,
    * one column per time step. What the critics score and what
    * \c TrajectoryVisualizer draws.
    */
    struct Trajectories
    {
        virtual ~Trajectories()=default;

        /**
        * @brief Reset trajectory data
        */
        virtual void reset(unsigned int batch_size, unsigned int time_steps)=0;
        
        mppi_mt::ArrayXX x;
        mppi_mt::ArrayXX y;
        mppi_mt::ArrayXX yaws;
    };

}//namespace controller::mppi_controller::models
namespace models  = controller::mppi_controller::models;