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
#include "regulated_pure_pursuit/lookahead_search.hpp"

#include "project_utils/geometry_utils.hpp"
#include "project_utils/planning_utils.hpp"

#include <cmath>

namespace regulatedpp_controller::lookahead
{

int32_t closedLoopSize(
    const std::vector<geometry_msgs::msg::Pose> & waypoints, double closeGapRatio)
{
	const auto n = static_cast<int32_t>(waypoints.size());
	if (n < 3) {
		return 0;
	}
	double length = 0.0;
	for (int32_t i = 1; i < n; ++i) {
		length += std::sqrt(mpl::geometry_utils::calcDistance2dSquare(
		    waypoints[i - 1].position, waypoints[i].position));
	}
	const double meanSpacing = length / (n - 1);
	const double gap = std::sqrt(
	    mpl::geometry_utils::calcDistance2dSquare(waypoints[n - 1].position, waypoints[0].position));
	if (gap > closeGapRatio * meanSpacing) {
		return 0;
	}
	constexpr double kDuplicateGap = 1e-6;  // [m]
	return gap < kDuplicateGap ? n - 1 : n;
}

int32_t previousIdx(int32_t idx, int32_t loopSize)
{
	return loopSize > 0 ? (idx - 1 + loopSize) % loopSize : idx - 1;
}

int32_t findNextPointIdx(const geometry_msgs::msg::Pose & currPose,
    const std::vector<geometry_msgs::msg::Pose> & waypoints, double lookAheadDistance,
    int32_t searchStartIdx, int32_t loopSize, double dirThDist)
{
	// if waypoints are not given, do nothing.
	if (waypoints.empty() || searchStartIdx == -1) {
		return -1;
	}

	// check if the waypoint direction is forward
	const int8_t wpDir = mpl::planning_utils::getPathDirection(waypoints, dirThDist);
	const auto n = static_cast<int32_t>(waypoints.size());
	const bool closed = loopSize > 0;
	const int32_t count = closed ? loopSize : n - searchStartIdx;
	for (int32_t k = 0; k < count; ++k) {
		const int32_t i = closed ? (searchStartIdx + k) % loopSize : searchStartIdx + k;
		// open path: if the first waypoint is the last
		if (!closed && i == n - 1) {
			return i;
		}
		const auto rel =
		    mpl::planning_utils::transformToRelativeCoordinate2D(waypoints.at(i).position, currPose);
		if (wpDir == 0) {
			// forward path: skip waypoints behind the car
			if (rel.x < 0) {
				continue;
			}
		} else if (wpDir == 1) {
			// backward path: skip waypoints in front of the car
			if (rel.x > 0) {
				continue;
			}
		} else {
			return -1;
		}
		// if there exists an effective waypoint
		const double ds =
		    mpl::geometry_utils::calcDistance2dSquare(waypoints.at(i).position, currPose.position);
		if (ds > lookAheadDistance * lookAheadDistance) {
			return i;
		}
	}
	return -1;
}

}  // namespace regulatedpp_controller::lookahead
