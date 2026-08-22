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

// Base interface for ego trajectory planners (e.g. ssc_planner). Still
// settling -- see .docs/project.md M3/M4 -- but configure()'s arguments are
// no longer speculative: every planner needs its own agent identity (to
// know which agent to skip when watching others), the shared agents.yaml
// "agents" map (to discover who those others are), the route graph file
// (its seed geometry), and a node handle (to create its own subscriptions).
class PlannerBase
{
  public:
	PlannerBase() = default;
	virtual ~PlannerBase() = default;

	// agentsConfig is the "agents" map from agents.yaml (i.e. the caller
	// passes agentsYaml["agents"], not the whole file). sscConfigFilePath
	// points at this planner's own dedicated config (e.g.
	// scenarios/intersection/params/ssc_planner.yaml) -- separate from
	// agents.yaml/graphFilepath, which describe the scenario/route graph,
	// not the planner's own tuning.
	virtual void configure(mpl::rclcpp_utils::Logger & logger, const YAML::Node & agentsConfig,
	    const std::string & graphFilepath, const std::string & sscConfigFilePath,
	    const UniqueId & id, const rclcpp::Node::SharedPtr & node) = 0;

	// Subscribes to every other agent's predicted trajectory + current pose
	// (never this planner's own -- see agentNum below), writing each into
	// mWorldSnapshot as messages arrive.
	virtual void receivePredictedTrajectory() = 0;

	virtual void computeTrajectory(const InputData&) = 0;

  protected:
	// Owned exclusively by this planner instance -- never shared with
	// another planner or thread, so no mutex guards it. See
	// world_snapshot.hpp for why that's safe as long as the owning node
	// stays single-threaded.
	WorldSnapshot mWorldSnapshot;
	mpl::rclcpp_utils::Logger mLogger;
	int mAgentNum{0};
	InputData mInputData;
};
