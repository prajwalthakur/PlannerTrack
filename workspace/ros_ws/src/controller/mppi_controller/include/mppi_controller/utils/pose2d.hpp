#pragma once

namespace controller::mppi_controller::utils
{
    /// \brief A lightweight float 2D pose, used for \ref controller::mppi_controller::models::Path::path_pose_2d "Path::path_pose_2d".
    struct Pose2D
    {
        float x, y, theta;
    };
}
namespace mppi_utils = controller::mppi_controller::utils;