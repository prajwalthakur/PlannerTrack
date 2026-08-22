/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#include "ssc_planner/reference_lane_gen.hpp"

namespace ssc_planner
{

void constructReferenceLane(const InputData & inputData,
    mpl::interpolation::SplineInterpolationPoints2d * refLane, mpl::rclcpp_utils::Logger & logger)
{
	refLane->initPoints(inputData.mEgoPath.poses, logger);
}

std::pair<double, double> projectOntoReferenceLane(
    const mpl::interpolation::SplineInterpolationPoints2d & refLane, double x, double y)
{
	return refLane.projectPointOntoSpline(x, y);
}

}  // namespace ssc_planner
