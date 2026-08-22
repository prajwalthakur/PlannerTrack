/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#pragma once
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "project_utils_msgs/msg/trajectory.hpp"

struct InputData
{
	// Ego's seed path for which we generate the trajectory. Assumed already
	// densified/corner-smoothed -- see mpl_route_planner's path_converter,
	// which is what's expected to have produced it.
	nav_msgs::msg::Path mEgoPath;
	// Ego's current pose.
	geometry_msgs::msg::PoseStamped mEgoPose;
	project_utils_msgs::msg::Trajectory mEgoInitTraj;
};
