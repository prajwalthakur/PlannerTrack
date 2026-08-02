#pragma once
#include "mppi_controller/models/trajectory.hpp"
#include "mppi_controller/utils/types.hpp"
namespace controller::mppi_controller::models
{

    struct AckermannTrajectories: public Trajectories
    {

        ~AckermannTrajectories() override=default;

        void reset(unsigned int batch_size, unsigned int time_steps) override
        {
            vx.setZero(batch_size, time_steps);
            wz.setZero(batch_size, time_steps);

            x.setZero(batch_size, time_steps);
            y.setZero(batch_size, time_steps);
            yaws.setZero(batch_size, time_steps);
        }
        
        mppi_mt::ArrayXX vx;
        mppi_mt::ArrayXX wz;

        
    };

}//namespace controller::mppi_controller::models