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

#include <memory>
#include <optional>
#include <vector>

// #include "trajectory_follower_base/lateral_controller_base.h"
#include "project_utils/parameter.hpp"
#include "trajectory_follower_base/trajectory_follower_base_collection.h"
// msgs
// ros
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
// custom msgs
#include "project_utils_msgs/msg/control.hpp"
#include "project_utils_msgs/msg/trajectory.hpp"
// include optimizer
#include "regulated_pure_pursuit/optimizer.hpp"
// Visualization
#include <visualization_msgs/msg/marker_array.hpp>
// Debug messages
#include <project_utils_msgs/msg/float32_multi_array_stamped.hpp>


using mpl::control::trajectory_follower::HybridControllerBase;
using mpl::control::trajectory_follower::HybridOutput;
using mpl::control::trajectory_follower::InputData;
using project_utils_msgs::msg::Control;
using project_utils_msgs::msg::Trajectory;
using project_utils_msgs::msg::TrajectoryPoint;

namespace regulatedpp_controller
{
// struct stVehConfigs
// {
// };

/**
 * \brief \c HybridControllerBase regulated-pure-pursuit controller: a
 * geometric pure-pursuit lateral tracker (\ref
 * regulatedpp_controller::optimizer::Optimizer "optimizer::Optimizer")
 * whose lookahead distance and target speed are "regulated" down for high
 * curvature/lateral error.
 *
 * Registered via `rclcpp_components_register_node` as
 * `regulatedpp_controller_exe` -- see \ref plugin_architecture (section 3).
 */
class RegulatedPP : public HybridControllerBase
{
  public:
	/// \param node Reference to the node used only for the component and parameter initialization.
	explicit RegulatedPP([[maybe_unused]] rclcpp::Node & node);
	~RegulatedPP() = default;

  private:
	/// \brief Always ready -- this controller has no warm-up/data dependency.
	bool isReady([[maybe_unused]] const InputData & input_data) override;

	/// \brief Compute this cycle's combined steering + speed command via \ref mOptimizer.
	HybridOutput run(const InputData & input_data) override;

  private:
  	rclcpp::Node& mNode;
	rclcpp::Clock::SharedPtr mClock;
	// configs
	// stVehConfigs mVehConfigs;
	std::shared_ptr<mpl::rclcpp_utils::Parameters> mParameters{nullptr};
	std::unique_ptr<regulatedpp_controller::optimizer::Optimizer> mOptimizer{nullptr};

	// debug markers
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_debug_marker_;
	// Predicted Trajectory publish
	rclcpp::Publisher<project_utils_msgs::msg::Trajectory>::SharedPtr pub_predicted_trajectory_;
	
	static constexpr double loggerThrottleInterval = 5000;  // in ms -> 5 sec
};

}  // namespace regulatedpp_controller
