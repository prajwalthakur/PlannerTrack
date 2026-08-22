/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#pragma once
#include "motion_model_base/geometric_model/geometric_model.hpp"
#include "motion_model_base/occupancy_grid_map.hpp"
#include <vector>

/**
 * \brief Per-tick, read-only view of everything a sensor might ray-cast
 * against: other agents' true (rendered) shapes, static obstacles, and
 * (optionally) a map-derived occupancy grid.
 *
 * Built once per lidar tick by the simulator and shared across all agents'
 * \c SensorModel::step() calls -- O(N) to build, not the O(N^2) it would be
 * if every agent rebuilt its own obstacle list from scratch.
 */
struct WorldSnapshot
{
	std::vector<const GeometricModel *> agents;
	std::vector<ShapeDescriptor> staticObstacles;
	// Non-owning; null until the simulator has received a map (e.g. before
	// the first /map message arrives). Points at a grid owned/kept alive by
	// the simulator for as long as the map is loaded.
	const OccupancyGridMap * mapGrid{nullptr};
};
