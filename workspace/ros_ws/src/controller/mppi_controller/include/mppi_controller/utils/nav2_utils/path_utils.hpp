#pragma once 
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2/utils.hpp"
#include "mppi_controller/utils/geometry_utils.hpp"

// #include "angles/angles.h"
// #include "nav2_util/geometry_utils.hpp"
// #include "nav2_util/robot_utils.hpp"
namespace controller::mppi_controller::utils
{

    /**
     * @brief Find and remove poses after the first constraint in the path
     * constraint : (maximaum angle after which we do inplace, rather than running the standard controller)
     * @param path to check for inversion or rotation
     * @param enforce_path_inversion Whether to enable check for inversion
     * @param rotation_threshold Minimum rotation angle to consider an in-place rotation (0 to disable rotation check)
     * @return The location of the inversion or rotation, return 0 if none exist
     */
    size_t removePosesAfterFirstConstraint(nav_msgs::msg::Path& path, bool enforcePathInversion, float rotationThreshold);

    /// \brief Locate the first inversion/in-place-rotation point in \p path without modifying it (see \ref removePosesAfterFirstConstraint).
    /// \return Index of the constraint, or 0 if none exists.
    size_t findFirstPathConstraint(
    nav_msgs::msg::Path & path,
    bool enforce_path_inversion,
    float rotation_threshold);
}//namespace controller::mppi_controller::utils