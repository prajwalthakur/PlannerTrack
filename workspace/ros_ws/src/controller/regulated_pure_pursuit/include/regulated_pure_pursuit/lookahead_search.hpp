// Copyright 2026 Prajwal Thakur
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#pragma once

#include <geometry_msgs/msg/pose.hpp>

#include <cstdint>
#include <vector>

/** \file
 * \brief Pure-pursuit lookahead-point search on open and closed (racing-line) paths.
 *
 * On a closed path the search wraps from the last waypoint to the first, so the
 * target keeps following the curve across the start/finish seam instead of the
 * straight extension of the last segment.
 */

namespace regulatedpp_controller::lookahead
{

/**
 * \brief Number of distinct waypoints of a closed path, or 0 if the path is open.
 *
 * The path is closed if the gap from the last waypoint back to the first is at
 * most \p closeGapRatio times the mean waypoint spacing. A last waypoint that
 * duplicates the first (gap below 1e-6 m) is not counted, so the loop never has
 * a zero-length segment.
 */
int32_t closedLoopSize(const std::vector<geometry_msgs::msg::Pose> & waypoints,
    double closeGapRatio = 2.0);

/// \brief Waypoint before \p idx: wraps on a closed path (\p loopSize > 0), plain idx - 1 otherwise.
int32_t previousIdx(int32_t idx, int32_t loopSize);

/**
 * \brief Index of the first waypoint from \p searchStartIdx on that lies ahead of
 * \p currPose (in the path direction) and farther than \p lookAheadDistance from it.
 *
 * Open path (\p loopSize == 0): stops at the last waypoint and returns it if no
 * waypoint is far enough (the original behaviour). Closed path: searches one full
 * loop, wrapping the index.
 *
 * \param dirThDist Minimum segment length used to detect the path direction
 *   (see mpl::planning_utils::getPathDirection).
 * \return The waypoint index, or -1 if none was found.
 */
int32_t findNextPointIdx(const geometry_msgs::msg::Pose & currPose,
    const std::vector<geometry_msgs::msg::Pose> & waypoints, double lookAheadDistance,
    int32_t searchStartIdx, int32_t loopSize, double dirThDist);

}  // namespace regulatedpp_controller::lookahead
