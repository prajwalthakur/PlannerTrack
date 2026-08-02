#pragma once
#include "mppi_controller/input_data.hpp"
#include "mppi_controller/models/model.hpp"
#include "mppi_controller/models/state.hpp"
#include "mppi_controller/utils/types.hpp"

#include "project_utils_msgs/msg/critics_stats.hpp"  //TODO:

namespace controller::mppi_controller
{
// namespace models = ::controller::mppi_controller::models;
struct CriticData
{
	// Do not change order.
	// If changing then change the initialization in mppi_class.hpp.
	models::Model * model;
	const geometry_msgs::msg::PoseStamped & goal;
	mppi_mt::ArrayX & costs;
	float & model_dt;

	bool fail_flag{false};
	size_t invalid_pts_count{0};

	std::optional<std::vector<bool>> path_pts_valid{std::nullopt};
	std::optional<size_t> furthest_reached_path_point{std::nullopt};
	std::vector<bool> trajectories_in_collision;

	// stat-msgs
	std::unique_ptr<project_utils_msgs::msg::CriticsStats> critic_stat_msg{nullptr};

	void reset()
	{
		fail_flag = false;
		path_pts_valid.reset();
		furthest_reached_path_point.reset();
	}
};  // CriticData

}  // namespace controller::mppi_controller

namespace mppi = controller::mppi_controller;
