/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#pragma once
#include <cstdint>
#include <vector>

/**
 * \brief Lightweight, ROS-message-agnostic occupancy grid used by sensor
 * models (e.g. \c LidarSensorModel) to ray-march against map-derived
 * obstacles -- track walls baked into a `map.pgm` and loaded by
 * `nav2_map_server` -- without this package or `motion_model_sensors`
 * depending on `nav_msgs`.
 *
 * The owning node (`agent_sim`) subscribes to `/map` and fills one of these
 * once; it is then shared read-only via \ref WorldSnapshot for the lifetime
 * of the map.
 *
 * Layout mirrors `nav_msgs/OccupancyGrid`: row-major, cell (0,0) at
 * (\ref originX, \ref originY), no rotation of the grid frame relative to
 * the world frame is represented (matches every `map.yaml` in this repo
 * today, which all declare yaw 0).
 */
struct OccupancyGridMap
{
	unsigned int width{0};
	unsigned int height{0};
	float resolution{0.05f};  // meters/cell
	double originX{0.0};
	double originY{0.0};
	/// Per-cell occupancy probability, ROS convention: 0 = free, 100 =
	/// occupied, -1 = unknown. A cell is treated as occupied iff value >= 50.
	std::vector<int8_t> data;

	/// \brief Whether a map has actually been received/filled yet.
	bool valid() const { return width > 0 && height > 0 && !data.empty(); }

	/**
	 * \brief Whether the cell at (\p cellX, \p cellY) is occupied.
	 * \return `false` for any cell outside the map extent (treated as
	 * unwalled, not occupied); otherwise `true` iff the cell's occupancy
	 * value is >= 50.
	 */
	bool isOccupied(int cellX, int cellY) const
	{
		if (cellX < 0 || cellY < 0 || cellX >= static_cast<int>(width) ||
		    cellY >= static_cast<int>(height)) {
			return false;  // outside the map extent -- not walled in
		}
		return data[static_cast<std::size_t>(cellY) * width + static_cast<std::size_t>(cellX)] >= 50;
	}
};
