/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#pragma once

#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "motion_model_base/geometric_model/geometric_model.hpp"
#include "mpl_route_planner/types.hpp"
#include "project_utils/macros_expression.hpp"
#include "project_utils/types.hpp"
#include "project_utils/unique_id.hpp"
#include "project_utils_msgs/msg/trajectory.hpp"

// Everything a planner needs to plan one cycle, in one place: the route
// graph, this agent's own current pose, and every other agent's predicted
// trajectory + current pose.
//
// Ownership/threading: one WorldSnapshot belongs exclusively to one planner
// instance (see PlannerBase::mWorldSnapshot) -- it's a plain member, never a
// pointer handed to more than one owner. routeGraph is loaded once in
// configure() and never mutated again; mAgentsSnapShot is only ever written
// from this planner's own subscription callbacks and only ever read from
// this planner's own computeTrajectory(). As long as the owning ROS2 node
// uses the default single-threaded executor (true everywhere else in this
// project), those callbacks and computeTrajectory() all run on one thread
// and never truly overlap -- so nothing here needs a mutex. That stops being
// true the moment a planner is driven by a MultiThreadedExecutor or a
// WorldSnapshot is shared across planner instances; don't do either without
// adding synchronization first.

// One other agent's latest known state. "Predicted" trajectory is a
// placeholder for now: that agent's own committed ground-truth route,
// sourced from its existing /agent_i/reference_trajectory topic -- see
// .docs/project.md's M3 execution-order note. Swapping in real
// MPDM-predicted trajectories (M2/M5) later only changes which topic feeds
// agentPredTrajectory, not this struct.

struct FrenetState
{
	float s{0.0};
	float d{0.0};
	float t{0.0};
	// Approximates ds/dt at this point -- taken directly from the
	// trajectory point's own longitudinal_velocity_mps (body-frame forward
	// speed), not a proper Frenet velocity decomposition (that would need
	// the reference lane's curvature/heading-error at this point, which
	// this project deliberately doesn't compute -- see ssc_map.cpp's
	// inflateCubeIn3dGrid, the one consumer of this field, for the
	// reasoning on why the approximation is acceptable there).
	float sDot{0.0};
};

// Mirrors EPSILON's common::FsVehicle (core/common/inc/common/basics/semantics.h:275)
// field-for-field: frenet_state -> frenetState, vec_E<Vec2f> vertices ->
// vec_E<mt::Vecf<2>> vertices. One per predicted-trajectory point -- vertices
// are this agent's rotated footprint corners (same rotation formula as
// common::SemanticsUtils::GetVehicleVertices / RectangularGeometry::calcVertices),
// each already projected through the reference lane into (s, d), not left in
// world frame.
struct FsVehicle
{
	FrenetState frenetState;
	mt::vec_E<mt::Vecf<2>> vertices;
};

struct AgentSnapshot
{
	UniqueId id;
	project_utils_msgs::msg::Trajectory agentPredTrajectory;
	geometry_msgs::msg::PoseStamped agentPose;
	//FrenetState frenetState;
	// Forward-propagated by SscPlanner's odom callback (geomModel->step(pose))
	// each time agentPose updates -- describe() then gives this agent's
	// current rotated footprint (center/length/width/heading, or exact
	// vertices) without us hand-rolling that geometry ourselves. Loaded via
	// the same geometry_plugin every agent already declares in agents.yaml.
	ptSharedPtr<GeometricModel> geomModel{nullptr};
};

struct WorldSnapshot
{
	// Loaded once in configure() from route_graph.geojson, then read-only
	// for the planner's lifetime.
	mpl_route::Graph routeGraph;
	mpl_route::GraphToIDMap routeGraphIdMap;

	geometry_msgs::msg::PoseStamped egoPose;
	// This planner's own geometry model -- same reasoning as
	// AgentSnapshot::geomModel above, stepped from the ego odom callback.
	ptSharedPtr<GeometricModel> geomModel{nullptr};

	// Every agent except this planner's own (see PlannerBase::agentNum),
	// keyed by UniqueId("agent", i).
	std::unordered_map<UniqueId, AgentSnapshot> mAgentsSnapShot;
	// Mirrors EPSILON's sur_vehicle_trajs_fs_ (std::unordered_map<int,
	// vec_E<common::FsVehicle>> in ssc_planner.cc) -- one FsVehicle per
	// predicted-trajectory point, per agent.
	std::unordered_map<UniqueId, mt::vec_E<FsVehicle>> mAgentsFrenetSnapShot;
	// Ego's own seed trajectory (InputData::mEgoInitTraj), Frenet-projected
	// the same way -- mirrors EPSILON's forward_trajs_fs_[i] for this
	// planner's single fixed route (no per-behavior loop, see project.md
	// M3/M4 notes). Used for corridor seed generation, not occupancy --
	// ego doesn't need to avoid itself -- but kept as FsVehicle rather than
	// bare FrenetState for the same structural reason as mAgentsFrenetSnapShot.
	mt::vec_E<FsVehicle> egoFrenetSnapShot;
};
