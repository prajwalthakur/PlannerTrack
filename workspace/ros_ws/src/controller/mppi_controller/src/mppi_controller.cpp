/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
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
#include "mppi_controller/mppi_controller.hpp"

/** \file
 * \brief \c MPPIController implementation: bridges the shared
 * trajectory-follower types and the MPPI \c Optimizer.
 */

namespace controller::mppi_controller
{

//////////////////////////////////////////////////////////////////////////

MPPIController::MPPIController(rclcpp::Node & node) : mNode{node}, mClock(node.get_clock())
{
	mName = "mppi_controller";
	mLogger = mpl::rclcpp_utils::Logger(node.get_logger(), mName);
	mMppiLogger = controller::mppi_controller::Logger(node.get_logger(), mName);

	mParameters = std::make_shared<controller::mppi_controller::Parameters>(node);
	auto getParam = mParameters->getParamGetter("");
	getParam(mWheelBase, "wheel_base", 1.4f);

	// optimizer setup
	controller::mppi_controller::Logger optimizerLogger(mMppiLogger, "mppi_optimizer");

	// 1. Initialize the Costmap2DROS object
	mCostmapRos = std::make_shared<CostMapRos>(
	    "local_costmap", mNode.get_namespace(), "local_costmap", false);
	// 2. Trigger Lifecycle transitions
	// In a real app, use a Lifecycle Manager, but for a standalone node:
	mCostmapRos->on_configure(rclcpp_lifecycle::State());
	mCostmapRos->on_activate(rclcpp_lifecycle::State());

	// parentName="" + name="PathHandler" matches the yaml's top-level PathHandler:
	// block (see FeasiblePathHandler::initialize()'s full_name construction).
	mPathHandler = std::make_unique<mppi_utils::FeasiblePathHandler>();
	mPathHandler->initialize(mParameters.get(), mMppiLogger, mCostmapRos, "", "PathHandler",
	    mCostmapRos->getTfBuffer());
	mOptimizer = std::make_unique<controller::mppi_controller::Optimizer>(
	    mParameters, mCostmapRos, optimizerLogger, std::string("FollowPath"), mClock);
	mOptimizer->onConfigure();
	auto getFollowPathParam = mParameters->getParamGetter("FollowPath");
	getFollowPathParam(mVisualizeRollouts, "visualize", false);
	getFollowPathParam(mVisualizeOptimalTrajectory, "visualize_optimal_trajectory", false);
	mVisualizer.configure(mNode, mCostmapRos->getGlobalFrameID(), mParameters.get(), "FollowPath");

	mPrunedPlanPub = mNode.create_publisher<nav_msgs::msg::Path>("pruned_plan", 10);

	mMppiLogger.info("mppi-controller is initiated");
}

//////////////////////////////////////////////////////////////////////////

bool MPPIController::isReady([[maybe_unused]] const HybridInputData & input_data)
{
	return true;
}

//////////////////////////////////////////////////////////////////////////

controller::mppi_controller::InputData MPPIController::toMppiInputData(const HybridInputData & input_data)
{
	controller::mppi_controller::InputData mppiInput;

	mppiInput.mBaseFrameID = input_data.mCurrentOdometry.header.frame_id;
	mppiInput.mRobotPose.header = input_data.mCurrentOdometry.header;
	mppiInput.mRobotPose.pose = input_data.mCurrentOdometry.pose.pose;
	mppiInput.mSpeed = input_data.mCurrentOdometry.twist.twist;


	nav_msgs::msg::Path newPath;
	const auto & newStamp = input_data.mCurrentTrajectory.header.stamp;
	if ((newStamp.sec - mLastPathTimestamp.sec) > 1e-6) {
		newPath.header = input_data.mCurrentTrajectory.header;
		newPath.poses.reserve(input_data.mCurrentTrajectory.points.size());
		for (const auto & point : input_data.mCurrentTrajectory.points) {
			geometry_msgs::msg::PoseStamped poseStamped;
			poseStamped.header = input_data.mCurrentTrajectory.header;
			poseStamped.pose = point.pose;
			newPath.poses.push_back(poseStamped);
		}
		mPathHandler->setPlan(newPath);
		
		mLastPathTimestamp = newStamp;
	}


	auto transformedGlobalPlan = mPathHandler->setPathForRacing(mppiInput.mRobotPose);
	geometry_msgs::msg::PoseStamped goal =
	    mPathHandler->getTransformedGoal(mppiInput.mRobotPose.header.stamp);
	mppiInput.mPlanToFollow = transformedGlobalPlan;
	mPrunedPlanPub->publish(transformedGlobalPlan);
	mLogger.info(" path size %zu ", mppiInput.mPlanToFollow.poses.size());
	mppiInput.mGoalPose = goal;
	mppiInput.mBaseFrameID = mCostmapRos->getBaseFrameID();
	return mppiInput;
}

//////////////////////////////////////////////////////////////////////////

HybridOutput MPPIController::toHybridOutput(
    const controller::mppi_controller::OutputData & output_data) const
{
	HybridOutput output;
	output.mControlCmd.stamp = mNode.now();
	const double vx = output_data.mControlCommand.twist.linear.x;
	const double wz = output_data.mControlCommand.twist.angular.z;
	output.mControlCmd.longitudinal.velocity = vx;
	// steering_tire_angle = atan(L * wz / vx); atan2 additionally stays well-defined as vx -> 0.
	output.mControlCmd.lateral.steering_tire_angle = std::atan2(mWheelBase * wz, std::max(vx,0.05));
	return output;
}

//////////////////////////////////////////////////////////////////////////

HybridOutput MPPIController::run(const HybridInputData & input_data)
{
	
	mLogger.info_throttle(
	    *mNode.get_clock(), loggerThrottleInterval, "In mppi_controller run function.");

	controller::mppi_controller::InputData mppiInput = toMppiInputData(input_data);
	controller::mppi_controller::OutputData mppiOutput = mOptimizer->computeControl(mppiInput);

	if (mVisualizeOptimalTrajectory || mVisualizeRollouts) {
		const auto stamp = mNode.now();
		if (mVisualizeOptimalTrajectory) {
			mVisualizer.add(mppiOutput.mOptimalTrajectory, "optimal_trajectory", stamp);
		}
		if (mVisualizeRollouts) {
			const auto * trajectories = mOptimizer->getTrajectories();
			if (trajectories) {
				mVisualizer.add(
				    *trajectories, mOptimizer->getCosts(), mOptimizer->getCollisionFlags(), stamp);
			}
		}
		mVisualizer.visualize();
	}

	return toHybridOutput(mppiOutput);
}
}  // namespace controller::mppi_controller
