#pragma once

// Get the actual model of the robot

#include "mppi_controller/models/model_factory.hpp"

namespace  controller::mppi_controller
{

    models::AckermannModel& getConcreteModel(models::Model& model)
    {
        return static_cast<models::AckermannModel&>(model);
    }


}//namespace  controller::mppi_controller