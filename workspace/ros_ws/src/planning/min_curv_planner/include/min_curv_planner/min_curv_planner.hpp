/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#pragma once

#include "planner_base/planner_base.hpp"
#include "min_curv_planner/params.hpp"
#include "min_curv_planner/types.hpp"

#include <rclcpp/rclcpp.hpp>

#include "project_utils_msgs/msg/trajectory.hpp"
#include "project_utils_msgs/msg/wpnt_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <yaml-cpp/yaml.h>

#include <string>

/** \file
 * \brief Global raceline planner (PlannerBase plugin).
 *
 * configure() loads this agent's vehicle parameters (agents.yaml), the
 * planner config ("min_curv_planner" block of plannerConfigFile) and the track
 * centerline (global_waypoints.json). The first computeTrajectory() call runs
 * min_curv_planner::computeRaceline() and publishes the result latched on
 *   ~/trajectory  project_utils_msgs/Trajectory  (what controllers follow)
 *   ~/waypoints   project_utils_msgs/WpntArray   (adds s, widths, kappa)
 *   ~/markers     visualization_msgs/MarkerArray (speed-colored line)
 * Later calls do nothing: the track is static and the outputs are latched.
 *
 * Needs no PlannerNode inputs (requiredInputs() is all false): it produces
 * the reference other planners and controllers consume.
 */
class MinCurvPlanner : public PlannerBase
{
  public:
	MinCurvPlanner() = default;
	~MinCurvPlanner() override = default;

	void configure(mpl::rclcpp_utils::Logger & logger, const YAML::Node & agentsConfig,
		const std::string & graphFile, const std::string & plannerConfigFile,
		const UniqueId & id, const rclcpp::Node::SharedPtr & node) override;

	// ego centric planning.
	// Doesnt need any other agents predicted references.
	void receivePredictedTrajectory() override {}

	PlannerInputs requiredInputs() const override;

	void computeTrajectory(const InputData & inputData) override;

  private:
	void publish(const min_curv_planner::Raceline & raceline);

  private:
	rclcpp::Node::SharedPtr mNode;
	std::string mFrameId{"map"};
	min_curv_planner::RacelineParams mParams;
	min_curv_planner::Track mCenterline;
	bool mComputed{false};

	rclcpp::Publisher<project_utils_msgs::msg::Trajectory>::SharedPtr mTrajectoryPub;
	rclcpp::Publisher<project_utils_msgs::msg::WpntArray>::SharedPtr mWaypointsPub;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mMarkersPub;
};
