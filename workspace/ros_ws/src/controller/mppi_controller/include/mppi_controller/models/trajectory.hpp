#pragma once
#include "mppi_controller/utils/types.hpp"
namespace controller::mppi_controller::models
{

    /**
    * @struct mppi::models::Trajectories
    * @brief Trajectories represented as a tensor
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