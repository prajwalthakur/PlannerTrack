#pragma once
#include "mppi_controller/models/state.hpp"
#include "mppi_controller/utils/types.hpp"

namespace controller::mppi_controller::models
{
    class State;
    struct AckermannState : public State
    {
        ~AckermannState() override=default;

        void reset(unsigned int batch_size, unsigned int time_steps) override;
    };
}//namespace controller::mppi_controller
