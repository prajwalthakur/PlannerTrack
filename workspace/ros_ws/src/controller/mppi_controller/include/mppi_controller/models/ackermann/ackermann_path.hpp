#pragma once
#include "mppi_controller/models/path.hpp"
#include "mppi_controller/utils/types.hpp"
namespace controller::mppi_controller::models
{

    struct AckermannPath: public Path
    {

        ~AckermannPath() override=default;
    };

}//namespace controller::mppi_controller::models