/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#pragma once

#include "min_curv_planner/types.hpp"

#include <string>

namespace min_curv_planner
{

/**
 * \brief Load a closed track from a global_waypoints.json file (the format
 * written by planner/global_planner, e.g.
 * scenarios/ground_vehicle_racing/map/global_waypoints.json).
 *
 * Reads `<key>.wpnts[*].{x_m, y_m, d_right, d_left}`. JSON is parsed with
 * yaml-cpp (JSON is valid YAML), so no extra dependency is needed.
 * A repeated closing point (last == first) is dropped.
 *
 * \throws std::runtime_error if the file or key is missing or has < 3 points.
 */
Track loadTrackFromWaypointsJson(
    const std::string & filePath, const std::string & key = "centerline_waypoints");

}  // namespace min_curv_planner
