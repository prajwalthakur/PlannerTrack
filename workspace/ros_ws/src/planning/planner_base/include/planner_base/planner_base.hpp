/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#pragma once
#include "planner_base/world_snapshot.hpp"
#include "project_utils/logger.hpp"
#include "project_utils/unique_id.hpp"
#include "planner_base/input_data.hpp"
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include <string>

// Which InputData fields a planner needs before PlannerNode calls
// computeTrajectory(). PlannerNode only waits for (and only fills) the
// fields marked true here.
struct PlannerInputs
{
	bool referencePath{true};  // InputData::mEgoPath
	bool referenceTrajectory{true};  // InputData::mEgoInitTraj
	bool odometry{true};  // InputData::mEgoPose
};

// Base interface for ego trajectory planners (e.g. ssc_planner,
// min_curv_planner). Every planner needs its own agent identity (to
// know which agent to skip when watching others), the shared agents.yaml
// "agents" map (to discover who those others are, and its own vehicle
// parameters), an optional route graph file, its own config file, and a
// node handle (to create its own subscriptions/publishers).
class PlannerBase
{
  public:
	PlannerBase() = default;
	virtual ~PlannerBase() = default;

	// agentsConfig is the "agents" map from agents.yaml (i.e. the caller
	// passes agentsYaml["agents"], not the whole file). plannerConfigFile
	// points at this planner's own dedicated config (e.g.
	// scenarios/intersection/params/ssc_planner.yaml) -- separate from
	// agents.yaml/graphFile, which describe the scenario/route graph,
	// not the planner's own tuning. graphFile may be empty for planners
	// that don't use a route graph.
	virtual void configure(mpl::rclcpp_utils::Logger & logger, const YAML::Node & agentsConfig,
	    const std::string & graphFile, const std::string & plannerConfigFile,
	    const UniqueId & id, const rclcpp::Node::SharedPtr & node) = 0;

	// Subscribes to every other agent's predicted trajectory + current pose
	// (never this planner's own -- see agentNum below), writing each into
	// mWorldSnapshot as messages arrive.
	virtual void receivePredictedTrajectory() = 0;

	// Inputs PlannerNode must receive before calling computeTrajectory().
	virtual PlannerInputs requiredInputs() const { return PlannerInputs{}; }

	virtual void computeTrajectory(const InputData &) = 0;

	// Latest trajectory produced by computeTrajectory() (empty before the
	// first successful call).
	virtual const project_utils_msgs::msg::Trajectory & getTrajectory() const
	{
		return mTrajectory;
	}

  protected:
	// Owned exclusively by this planner instance -- never shared with
	// another planner or thread, so no mutex guards it. See
	// world_snapshot.hpp for why that's safe as long as the owning node
	// stays single-threaded.
	WorldSnapshot mWorldSnapshot;
	mpl::rclcpp_utils::Logger mLogger;
	int mAgentNum{0};
	InputData mInputData;
	project_utils_msgs::msg::Trajectory mTrajectory;
};
