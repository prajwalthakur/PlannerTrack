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
#include "f110_msgs/msg/wpnt_array.hpp"
#include "project_utils_msgs/msg/path.hpp"
#include "project_utils_msgs/msg/steering_report.hpp"
#include "project_utils_msgs/msg/trajectory.hpp"

// This file defines the input data structure for the trajectory follower
// controllers.
// It includes the current trajectory, odometry,
// acceleration, and steering report of the vehicle.
// The controllers will use this data to compute the control
// commands to follow the trajectory.
// This can also be used for path tracking controllers if needed.
namespace mpl::control::trajectory_follower
{
struct InputData
{
	nav_msgs::msg::Odometry mCurrentOdometry;
	f110_msgs::msg::WpntArray mLocalWpArray;
	std_msgs::msg::String mStateMachine;
	// project_utils_msgs::msg::Trajectory mCurrentTrajectory;
	// nav_msgs::msg::Odometry mCurrentOdometry;
	// project_utils_msgs::msg::Path mPathToFollow;
	// geometry_msgs::msg::AccelWithCovarianceStamped mCurrentAcc;
	project_utils_msgs::msg::SteeringReport mCurrentSteering;
};
}  // namespace mpl::control::trajectory_follower

namespace trajectory_follower = mpl::control::trajectory_follower;
