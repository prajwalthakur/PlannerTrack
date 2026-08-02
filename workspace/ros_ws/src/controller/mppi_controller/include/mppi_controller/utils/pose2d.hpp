#pragma once

namespace controller::mppi_controller::utils
{
    // A struct to hold pose data in floating point resolution
    struct Pose2D
    {
        float x, y, theta;
    };
}
namespace mppi_utils = controller::mppi_controller::utils;