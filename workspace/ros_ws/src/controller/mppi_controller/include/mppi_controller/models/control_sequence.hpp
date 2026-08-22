#pragma once
#include "mppi_controller/utils/types.hpp"
namespace controller::mppi_controller::models
{

    /// \brief Abstract single-timestep control command (concrete layout is model-specific, e.g. Ackermann's `{vx, wz}`).
    struct Control
    {
        virtual ~Control()=default;

    };

    /// \brief Abstract nominal control sequence the optimizer perturbs/updates over \p time_steps steps.
    struct ControlSequence
    {
        virtual ~ControlSequence()=default;
        /// \brief Resize to \p time_steps and reset to a default (e.g. zero) sequence.
        virtual void reset(unsigned int time_steps)=0;

    };

} //controller::mppi_controller::models

namespace models = controller::mppi_controller::models;