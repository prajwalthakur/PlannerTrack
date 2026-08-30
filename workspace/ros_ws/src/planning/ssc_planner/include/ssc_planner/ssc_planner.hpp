/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#pragma once
#include "builtin_interfaces/msg/duration.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "motion_model_base/geometric_model/geometric_model.hpp"
#include "planner_base/planner_base.hpp"
#include "project_utils/types.hpp"
#include "ssc_planner/bezier_traj_gen/bezier_curve.hpp"
#include "ssc_planner/reference_lane_gen.hpp"
#include "ssc_planner/ssc_map.hpp"
#include "ssc_planner/ssc_map_config.hpp"
#include "ssc_planner/ssc_visualizer.hpp"

#include <rclcpp/rclcpp.hpp>

#include "nav_msgs/msg/odometry.hpp"
#include "project_utils_msgs/msg/trajectory.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <pluginlib/class_loader.hpp>
#include <yaml-cpp/yaml.h>

#include <memory>
#include <string>
#include <vector>

//////////////////////////////////////////////////////////////////////////

// configure() and receivePredictedTrajectory() are real: they load the
// route graph and subscribe to every other agent's reference trajectory +
// odom into mWorldSnapshot. computeTrajectory() constructs the reference
// lane (reference_lane_gen.hpp) but computeSSCCorridor()/
// computeBezierTrajectory() are still stubs -- see .docs/project.md M3/M4 --
// they need cube-inflation corridor generation (Ding, Zhang, Chen, Shen,
// RA-L 2019, Algorithm 1) around the ego seed trajectory, then a
// piecewise-Bezier QP trajectory optimization inside the resulting
// corridor.
class SscPlanner : public PlannerBase
{
	using BaseType = PlannerBase;

  public:
	// mGeometryLoader needs a real member-initializer-list (a
	// pluginlib::ClassLoader has no default constructor), so this can't
	// stay "= default" -- defined in ssc_planner.cpp.
	SscPlanner();
	~SscPlanner() override = default;

	void configure(mpl::rclcpp_utils::Logger & logger, const YAML::Node & agentsConfig,
	    const std::string & graphFilepath, const std::string & sscConfigFilePath,
	    const UniqueId & id, const rclcpp::Node::SharedPtr & node) override;
	void receivePredictedTrajectory() override {};
	void receivePredictedTrajectoryStatic();  // for now -its just recieved the predicited
	                                          // trajectory only once
	void computeTrajectory(const InputData & inputData) override;
	// Projects every other agent's agentPredTrajectory through mRefLane
	// into mWorldSnapshot.mAgentsFrenetSnapShot -- must run after mRefLane
	// is built (constructReferenceLane) and before computeSSCCorridor(),
	// which is what actually consumes it.
	void projectAgentsToFrenet();
	// Same, but for ego's own seed trajectory (inputData.mEgoInitTraj ->
	// mWorldSnapshot.egoFrenetSnapShot) -- the corridor's seed-generation
	// input. inputData is a parameter (not read from mWorldSnapshot) because
	// this trajectory arrives fresh via computeTrajectory()'s argument, not
	// through a persistent subscription like the other agents'.
	void projectEgoToFrenet(const InputData & inputData);
	ssc_planner::ErrorType computeSSCCorridor();
	// corridorStatus: computeSSCCorridor()'s own result -- skips solving
	// entirely (logs, returns) rather than handing a possibly-empty/invalid
	// corridor to the Bezier QP.
	void computeBezierTrajectory(ssc_planner::ErrorType corridorStatus);
	// mStartTime is the one shared time origin every agent's trajectory gets
	// re-based against -- set once per planning cycle (ego's own "now",
	// e.g. mNode->now())
	void setStartTime(const rclcpp::Time & t) { mStartTime = t; }
	// initialTime: the trajectory's own header.stamp (a different, real
	// wall-clock moment per agent, since global_waypoints_publisher.py
	// publishes each agent's Trajectory sequentially
	// durationFromInitial: that trajectory point's own time_from_start.
	// Returns seconds relative to mStartTime -- this is FrenetState::t.
	double getRelTime(const builtin_interfaces::msg::Time & initialTime,
	    const builtin_interfaces::msg::Duration & durationFromInitial) const
	{
		const rclcpp::Time absoluteTime =
		    rclcpp::Time(initialTime) + rclcpp::Duration(durationFromInitial);
		return (absoluteTime - mStartTime).seconds();
	}

  private:
	YAML::Node mAgentsConfig;
	UniqueId mId;
	rclcpp::Node::SharedPtr mNode;
	// Held only to keep the subscriptions alive -- callbacks write directly
	// into mWorldSnapshot, nothing else reads this vector.
	std::vector<rclcpp::SubscriptionBase::SharedPtr> mSubscriptions;
	mpl::interpolation::SplineInterpolationPoints2d mRefLane;
	rclcpp::Time mStartTime;
	pluginlib::ClassLoader<GeometricModel> mGeometryLoader;
	// Loaded in configure() from sscConfigFilePath (scenarios/intersection/
	// params/ssc_planner.yaml) -- this planner's own tuning, separate from
	// the scenario's agents.yaml/route graph.
	ssc_planner::SscMapConfig mMapConfig;
	// Owns the occupancy grid + driving-corridor state, analogous to
	// EPSILON's SscPlanner::p_ssc_map_. Constructed in configure() once
	// mMapConfig is loaded.
	std::unique_ptr<ssc_planner::SscMap> mSscMap;
	// Debug-only: renders mSscMap->finalCorridorVec() as a MarkerArray, see
	// ssc_visualizer.hpp. Frame is a standalone debug frame
	// ("agent_<n>/ssc_debug"), not "map" -- (s,d,t) has no real spatial
	// meaning.
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mCorridorMarkerPub;
	// Real-map-frame overlay (buildCorridorMarkersCartesian) -- projects the
	// corridor's (s,d) footprint through mRefLane, for showing alongside the
	// real scene (map/agents/routes) instead of the abstract debug view.
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mCorridorMarkerCartesianPub;
	// Result of computeBezierTrajectory()'s solve -- only meaningful when
	// mBezierSplineValid is true (SolveBezierSpline() succeeded).
	BezierSpline<kBezierOrder, kBezierDim> mBezierSpline;
	bool mBezierSplineValid{false};
	// The solved spline, sampled and published as waypoints -- see
	// computeBezierTrajectory()'s own comment for the sampling scheme.
	rclcpp::Publisher<project_utils_msgs::msg::Trajectory>::SharedPtr mBezierTrajectoryPub;
	// RViz LINE_STRIP of the same waypoints -- own publisher/topic so it can
	// be toggled independently of the corridor markers.
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mBezierTrajectoryMarkerPub;
};

//////////////////////////////////////////////////////////////////////////
