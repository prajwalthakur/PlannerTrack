#pragma once
#include "mppi_controller/models/state.hpp"
#include "mppi_controller/utils/types.hpp"

namespace controller::mppi_controller::models
{
    class State;
    /// \brief \ref State for \ref AckermannModel (no additional fields beyond the base -- Ackermann is fully described by `{vx, wz}`).
    struct AckermannState : public State
    {
        ~AckermannState() override=default;

        void reset(unsigned int batch_size, unsigned int time_steps) override;
    };
}//namespace controller::mppi_controller
