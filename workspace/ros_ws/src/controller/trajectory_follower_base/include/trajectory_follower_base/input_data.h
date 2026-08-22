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
#pragma once

// msgs
// ros
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
// custom msgs
#include "project_utils_msgs/msg/path.hpp"
#include "project_utils_msgs/msg/steering_report.hpp"
#include "project_utils_msgs/msg/trajectory.hpp"
#include "project_utils_msgs/msg/wpnt_array.hpp"

/**
 * \file
 * \brief Defines the shared input-data structure passed to every
 * trajectory-follower controller (lateral, longitudinal, hybrid).
 */
namespace mpl::control::trajectory_follower
{
/**
 * \brief Everything a controller derived from \ref ControllerBase needs to
 * compute one control command: the trajectory to follow, current odometry,
 * and current steering feedback.
 *
 * Shared by lateral, longitudinal, and hybrid controllers so all three see
 * the same snapshot of vehicle state per control cycle. Can also be used
 * for path tracking controllers if needed.
 */
struct InputData
{
	nav_msgs::msg::Odometry mCurrentOdometry;
	//project_utils_msgs::msg::WpntArray mLocalWpArray;
	//std_msgs::msg::String mStateMachine;
	project_utils_msgs::msg::Trajectory mCurrentTrajectory;
	// nav_msgs::msg::Odometry mCurrentOdometry;
	// project_utils_msgs::msg::Path mPathToFollow;
	// geometry_msgs::msg::AccelWithCovarianceStamped mCurrentAcc;
	project_utils_msgs::msg::SteeringReport mCurrentSteering;
};
}  // namespace mpl::control::trajectory_follower

namespace trajectory_follower = mpl::control::trajectory_follower;
