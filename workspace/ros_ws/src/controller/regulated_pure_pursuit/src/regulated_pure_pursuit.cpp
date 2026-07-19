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
///////////////////////////////////////////////////////////////////////////
#include "regulated_pure_pursuit/regulated_pure_pursuit.hpp"

namespace regulatedpp_controller
{
RegulatedPP::RegulatedPP([[maybe_unused]] rclcpp::Node & node)
    : mNode{node}, mClock(node.get_clock())
{
	mName = "regulated_pp";
	mParameters = std::make_shared<mpl::rclcpp_utils::Parameters>(node);
	auto tempLogger = node.get_logger();
	mLogger = mpl::rclcpp_utils::Logger(node.get_logger(), mName);
	// optimizer set
	mpl::rclcpp_utils::Logger logger(mLogger, "optimizer");
	mOptimizer = std::make_unique<regulatedpp_controller::optimizer::Optimizer>(
	    mParameters.get(), logger, mName, "optimizer", mNode);

	// Debug Publishers
	pub_debug_marker_ =
	    node.create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/markers", 0);
	// Publish predicted trajectory
	pub_predicted_trajectory_ = node.create_publisher<project_utils_msgs::msg::Trajectory>(
	    "~/output/predicted_trajectory", 1);

	mLogger.info("regulated pp controller is initiated");  // pp ;)
}

////////////////////////////////////////////////////////////////////////////////

bool RegulatedPP::isReady([[maybe_unused]] const InputData & input_data)
{
	return true;
}

////////////////////////////////////////////////////////////////////////////////

HybridOutput RegulatedPP::run([[maybe_unused]] const InputData & input_data)
{
	
	mLogger.info_throttle(*mNode.get_clock(), loggerThrottleInterval,  " In regulated pure pursuit run function.");
	HybridOutput output;
	[[maybe_unused]] auto current_pose_ = input_data.mCurrentOdometry.pose.pose;
	// mLogger.info_throttle(*mNode.get_clock(), loggerThrottleInterval,"running regulated pp controller current pose of robot %.3f, %.3f, %.3f",
	//     current_pose_.position.x, current_pose_.position.y, current_pose_.position.z);
	mOptimizer->optimize(input_data, output);
	//  [[maybe_unused]] auto current_pose_ = input_data.mCurrentOdometry->pose.pose;
	//  [[maybe_unused]] auto trajectory_ = input_data.mCurrentTrajectory;
	//  [[maybe_unused]] auto current_odometry_ = input_data.mCurrentOdometry;
	//  [[maybe_unused]] auto current_steering_ = input_data.mCurrentSteering;
	// project_utils_msgs::msg::Control cmd_;
	// cmd_.stamp = mClock->now();
	// cmd_.longitudinal.velocity = 0.0;
	// cmd_.longitudinal.acceleration = 0.0;
	// cmd_.lateral.steering_tire_angle = 0.0;
	// cmd_.lateral.steering_tire_rotation_rate = 0.0;
	// output.mControlCmd = cmd_;
	return output;
}

}  // namespace regulatedpp_controller
