#pragma once
#include "mppi_controller/utils/types.hpp"
namespace controller::mppi_controller::models
{

    struct Control
    {
        virtual ~Control()=default;

    };

    struct ControlSequence
    {
        virtual ~ControlSequence()=default;
        virtual void reset(unsigned int time_steps)=0;

    };

} //controller::mppi_controller::models

namespace models = controller::mppi_controller::models;