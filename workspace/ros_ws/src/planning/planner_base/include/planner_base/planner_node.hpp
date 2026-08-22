/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#pragma once
#include "planner_base/input_data.hpp"
#include "planner_base/planner_base.hpp"
#include "project_utils/logger.hpp"
#include "project_utils/parameter.hpp"
#include "project_utils/polling_subscriber.hpp"
#include "project_utils/unique_id.hpp"

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "project_utils_msgs/msg/trajectory.hpp"

#include <memory>
#include <string>

//////////////////////////////////////////////////////////////////////////

// Hosts any PlannerBase plugin as a live ROS2 node, one agent per process --
// mirrors trajectory_follower_node's Controller (same params-via-
// mpl::rclcpp_utils::Parameters style, same InternalProcessPollingSubscriber
// readiness-gated pattern). PlannerBase::configure() wires up everything the
// plugin needs to watch *other* agents internally (see SscPlanner::
// configure() -> receivePredictedTrajectoryStatic()) -- this node's own job
// is only to supply this agent's *own* InputData (mEgoPath/mEgoInitTraj/
// mEgoPose), which the plugin has no subscription of its own for.
//
// The timer is currently one-shot, not a re-plan loop: it polls (at
// planner_frequency) until all three inputs are ready, calls
// computeTrajectory() exactly once, then cancels itself -- see
// callbackTimer()'s own comment for why (this scenario's reference
// trajectory is published once, latched, so repeated computeTrajectory()
// calls would let FrenetState::t drift further negative each cycle).
//
// Deliberately a plain node (not rclcpp_components) -- see
// planner_base/CMakeLists.txt's build-target comment for why: this node
// itself does pluginlib-based plugin loading (of PlannerBase), and nesting
// that inside an rclcpp_components-generated dlopen wrapper is a known
// failure mode already documented (and avoided) in controller_node's own
// CMakeLists.txt.
class PlannerNode : public rclcpp::Node
{
  public:
	explicit PlannerNode(const rclcpp::NodeOptions & nodeOptions);
	~PlannerNode() override = default;

	// Two-phase init: the constructor only reads params and loads the
	// plugin (neither needs a real shared_ptr to this node). This method
	// hands the plugin a real rclcpp::Node::SharedPtr via shared_from_this()
	// and starts the timer -- must be called after this node is already
	// owned by a shared_ptr (i.e. from main.cpp, right after
	// std::make_shared<PlannerNode>(...) returns), since shared_from_this()
	// is undefined behavior any earlier than that.
	void configure();

  private:
	// Polls all three input subscribers; returns false (and throttle-logs
	// which ones are still missing) until every one has delivered at least
	// one message. Must never let computeTrajectory() run against a
	// partially-empty InputData -- an empty mEgoPath in particular breaks
	// constructReferenceLane downstream.
	bool buildInputData(InputData * inputData);
	void callbackTimer();

	mpl::rclcpp_utils::Logger mLogger;
	std::unique_ptr<mpl::rclcpp_utils::Parameters> mParameters;

	pluginlib::ClassLoader<PlannerBase> mPlannerLoader{"planner_base", "PlannerBase"};
	std::shared_ptr<PlannerBase> mPlanner;

	// Per-agent identity + config file paths, set by the launch file (one
	// process per agent, identity comes entirely from these params -- same
	// convention as controller_node's mAgentNumber/mAgentsConfigFile).
	int mAgentNumber{0};
	std::string mAgentsConfigFile;
	std::string mGraphFile;
	std::string mSscConfigFile;
	std::string mPlannerPluginName;
	float mPlannerFrequencyHz{2.0f};

	rclcpp::TimerBase::SharedPtr mTimer;
	// Ego's own 3 inputs (mSub*) can be ready before SscPlanner's *internal*
	// subscriptions to every other agent's reference_trajectory have -- a
	// separate discovery race this node has no visibility into (that state
	// lives inside SscPlanner's own WorldSnapshot). Skip callbackTimer()
	// entirely until this delay has elapsed, giving both races a moment to
	// settle before the one-shot compute fires -- without this, a lucky-
	// but-incomplete first tick computes a corridor missing another agent's
	// occupancy and never gets a chance to retry.
	rclcpp::Time mConstructedAt;
	double mStartupDelaySec{2.0};

	mpl_rclcpp_utils::InternalProcessPollingSubscriber<nav_msgs::msg::Path> mSubReferencePath{
	    this, "~/input/reference_path", rclcpp::QoS{1}.transient_local()};
	mpl_rclcpp_utils::InternalProcessPollingSubscriber<project_utils_msgs::msg::Trajectory>
	    mSubReferenceTrajectory{
	        this, "~/input/reference_trajectory", rclcpp::QoS{1}.transient_local()};
	mpl_rclcpp_utils::InternalProcessPollingSubscriber<nav_msgs::msg::Odometry> mSubOdometry{
	    this, "~/input/odom"};

	static constexpr double loggerThrottleIntervalMs = 5000;  // 5 sec
};
