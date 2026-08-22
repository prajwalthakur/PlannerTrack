/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#include "planner_base/planner_node.hpp"

#include <chrono>

//////////////////////////////////////////////////////////////////////////

PlannerNode::PlannerNode(const rclcpp::NodeOptions & nodeOptions) : Node("planner", nodeOptions)
{
	mLogger = mpl::rclcpp_utils::Logger(this->get_logger());
	mLogger.info("PlannerNode constructor");
	mConstructedAt = this->now();

	mParameters = std::make_unique<mpl::rclcpp_utils::Parameters>(*this);
	auto getParam = mParameters->getParamGetter("");

	getParam(mAgentNumber, "agent_number", 0);
	getParam(mAgentsConfigFile, "agents_config_file", std::string(""));
	getParam(mGraphFile, "graph_file", std::string(""));
	getParam(mSscConfigFile, "ssc_config_file", std::string(""));
	// Hardcoded default for now
	getParam(mPlannerPluginName, "planner_plugin", std::string("SscPlanner"));
	getParam(mPlannerFrequencyHz, "planner_frequency", 2.0f);
	getParam(mStartupDelaySec, "startup_delay_sec", 2.0);

	// Loading the plugin itself doesn't need a real shared_ptr -- only
	// handing it to mPlanner->configure() does (see configure()'s own
	// comment) -- so this stays in the constructor.
	mPlanner = mPlannerLoader.createSharedInstance(mPlannerPluginName);
}

//////////////////////////////////////////////////////////////////////////

void PlannerNode::configure()
{
	const YAML::Node agentsYaml = YAML::LoadFile(mAgentsConfigFile);
	const YAML::Node agentsConfig = agentsYaml["agents"];
	const UniqueId id("planning_agent", mAgentNumber);


	mPlanner->configure(mLogger, agentsConfig, mGraphFile, mSscConfigFile, id, shared_from_this());

	const auto periodNs = std::chrono::duration_cast<std::chrono::nanoseconds>(
	    std::chrono::duration<double, std::ratio<1, 1>>(1.0 / mPlannerFrequencyHz));
	mTimer = rclcpp::create_timer(this, get_clock(), periodNs, [this]() { this->callbackTimer(); });
}

//////////////////////////////////////////////////////////////////////////

bool PlannerNode::buildInputData(InputData * inputData)
{
	const auto pathData = mSubReferencePath.takeData();
	const auto trajData = mSubReferenceTrajectory.takeData();
	const auto odomData = mSubOdometry.takeData();

	if (!pathData) {
		mLogger.info_throttle(*get_clock(), loggerThrottleIntervalMs, "waiting for reference_path");
	}
	if (!trajData) {
		mLogger.info_throttle(
		    *get_clock(), loggerThrottleIntervalMs, "waiting for reference_trajectory");
	}
	if (!odomData) {
		mLogger.info_throttle(*get_clock(), loggerThrottleIntervalMs, "waiting for odom");
	}
	if (!pathData || !trajData || !odomData) {
		return false;
	}

	inputData->mEgoPath = *pathData;
	inputData->mEgoInitTraj = *trajData;
	inputData->mEgoPose.header = odomData->header;
	inputData->mEgoPose.pose = odomData->pose.pose;
	return true;
}

//////////////////////////////////////////////////////////////////////////

void PlannerNode::callbackTimer()
{
	if ((this->now() - mConstructedAt).seconds() < mStartupDelaySec) {
		// Deliberately silent (no throttled log) -- this is expected on
		// every tick during the startup window, not a stuck/waiting state
		// worth surfacing the way buildInputData()'s gate is.
		return;
	}

	InputData inputData;
	if (!buildInputData(&inputData)) {
		return;
	}
	// planning to bring the other agents trajectory, odom in input data itself
	mPlanner->computeTrajectory(inputData);

	// One-shot for now: mStartTime gets re-based to ego's *current* odom
	// stamp on every computeTrajectory() call, but this scenario's
	// reference_trajectory is published once, latched -- its header.stamp
	// is frozen at publish time. Calling computeTrajectory() repeatedly
	// lets those two clocks drift apart (mStartTime keeps advancing,
	// header.stamp doesn't), pushing FrenetState::t further negative every
	// cycle. Computing once, promptly after the readiness gate first
	// clears, keeps that gap small. Revisit once there's an actual
	// closed loop (ego moving, trajectories genuinely re-published) that
	// needs continuous re-planning -- this timer is what re-enables that.
	mLogger.info("computeTrajectory() done -- one-shot, stopping timer.");
	mTimer->cancel();
}
