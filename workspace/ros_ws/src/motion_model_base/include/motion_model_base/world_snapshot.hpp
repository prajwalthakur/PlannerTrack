/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#pragma once
#include "motion_model_base/geometric_model/geometric_model.hpp"
#include <vector>

// Per-tick, read-only view of everything a sensor might ray-cast against:
// other agents' true (rendered) shapes plus static obstacles. Built once per
// lidar tick by the simulator and shared across all agents'
// SensorModel::step() calls -- O(N) to build, not the O(N^2) it would be if
// every agent rebuilt its own obstacle list from scratch.
struct WorldSnapshot
{
	std::vector<const GeometricModel *> agents;
	std::vector<ShapeDescriptor> staticObstacles;
};
