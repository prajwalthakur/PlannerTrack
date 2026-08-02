/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */

// Copyright 2026 Prajwal Thakur
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <cmath>
#include <memory>
#include <string>

#include "trajectory_follower_base/trajectory_follower_base_collection.h"

// msgs
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
// custom msgs
#include "project_utils_msgs/msg/control.hpp"
#include "project_utils_msgs/msg/trajectory.hpp"

// MPPI optimizer
#include "mppi_controller/input_data.hpp"
#include "mppi_controller/output_data.hpp"
#include "mppi_controller/parameters.hpp"
#include "mppi_controller/utils/logger_utils.hpp"
#include "mppi_controller/utils/mppi_utils/trajectory_visualizer.hpp"
#include "mppi_controller/utils/nav2_utils/feasible_path_handler.hpp"
#include "mppi_controller/utils/ros_namespace.hpp"
#include "mppi_controller/vanilla_mppi_class.hpp"

using mpl::control::trajectory_follower::HybridControllerBase;
using mpl::control::trajectory_follower::HybridOutput;
using project_utils_msgs::msg::Control;
using project_utils_msgs::msg::Trajectory;
using project_utils_msgs::msg::TrajectoryPoint;

// Named distinctly from controller::mppi_controller::InputData (mppi_controller/input_data.hpp):
// both are called "InputData", and code inside namespace controller::mppi_controller that wrote
// unqualified InputData was silently binding to the wrong one (the mppi-internal struct shadows
// the trajectory-follower one for any unqualified lookup from within that namespace).
using HybridInputData = mpl::control::trajectory_follower::InputData;

namespace controller::mppi_controller
{

class MPPIController : public HybridControllerBase
{
  public:
	/// \param node Reference to the node used only for the component and parameter initialization.
	explicit MPPIController(rclcpp::Node & node);
	~MPPIController() override = default;

  private:
	bool isReady([[maybe_unused]] const HybridInputData & input_data) override;

	HybridOutput run(const HybridInputData & input_data) override;

	// Converts the shared trajectory-follower InputData into the MPPI optimizer's own
	// InputData: robot pose/speed come from odometry, and the path-to-follow is
	// rebuilt from the reference trajectory and pushed into the path handler only
	// when its timestamp changes (see mLastPathTimestamp) -- not on every cycle.
	controller::mppi_controller::InputData toMppiInputData(const HybridInputData & input_data);

	// Converts the optimizer's OutputData back into the shared HybridOutput.
	// The model still only produces a diff-drive (vx, wz) command, so wz is converted
	// into an equivalent bicycle-model steering angle via steering = atan(L * wz / vx),
	// using mWheelBase as L.
	HybridOutput toHybridOutput(const controller::mppi_controller::OutputData & output_data) const;

  private:
	rclcpp::Node & mNode;
	rclcpp::Clock::SharedPtr mClock;

	std::shared_ptr<controller::mppi_controller::Parameters> mParameters{nullptr};
	controller::mppi_controller::Logger mMppiLogger;
	std::unique_ptr<controller::mppi_controller::Optimizer> mOptimizer{nullptr};

	// Costmap + path handler backing the path-to-follow conversion in toMppiInputData().
	std::shared_ptr<CostMapRos> mCostmapRos{nullptr};
	std::unique_ptr<mppi_utils::FeasiblePathHandler> mPathHandler{nullptr};

	// Timestamp of the last reference trajectory pushed into mPathHandler via setPlan();
	// toMppiInputData() only re-registers the plan when a new trajectory's stamp differs
	// from this, instead of re-doing the costmap-frame transform/pruning every cycle.
	builtin_interfaces::msg::Time mLastPathTimestamp{};

	// Wheelbase used to convert the model's yaw-rate command into a steering angle
	// in toHybridOutput(); read once from the "wheel_base" parameter at construction.
	float mWheelBase{1.4f};

	// Rollout/optimal-trajectory rviz visualization, published from run() when
	// enabled (FollowPath.visualize / FollowPath.visualize_optimal_trajectory).
	TrajectoryVisualizer mVisualizer;
	bool mVisualizeRollouts{false};
	bool mVisualizeOptimalTrajectory{false};

	// Publishes the costmap-frame pruned segment of the reference plan (see
	// toMppiInputData()) for rviz, on "pruned_plan" (-> /agent_N/pruned_plan).
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mPrunedPlanPub;

	static constexpr double loggerThrottleInterval = 5000;  // in ms -> 5 sec
};

}  // namespace controller::mppi_controller
