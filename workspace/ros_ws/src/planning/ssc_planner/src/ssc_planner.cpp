/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#include "ssc_planner/ssc_planner.hpp"

#include "mpl_route_planner/graph_loader.hpp"
#include "project_utils/geometry_utils.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <array>
#include <cmath>

//////////////////////////////////////////////////////////////////////////

namespace
{
// GeometricModel::step() takes an stPose (x, y, z, yaw), not a
// geometry_msgs::msg::Pose (quaternion orientation) -- this is the
// conversion, using the same yaw-extraction helper the rest of the
// codebase already uses (see e.g. SingleTrackDynStateModel's odom
// publishing for the inverse direction).

//////////////////////////////////////////////////////////////////////////

stPose toStPose(const geometry_msgs::msg::Pose & pose)
{
	return stPose(pose.position.x, pose.position.y, pose.position.z,
	    mpl::geometry_utils::getYawFromQuaternion(pose.orientation));
}

//////////////////////////////////////////////////////////////////////////

std::array<mt::Vecf<2>, 4> rectangleVertices(
    double x, double y, double yaw, double length, double width)
{
	const double cosTheta = std::cos(yaw);
	const double sinTheta = std::sin(yaw);
	const double dWx = width / 2.0 * sinTheta;
	const double dWy = width / 2.0 * cosTheta;
	const double dLx = length / 2.0 * cosTheta;
	const double dLy = length / 2.0 * sinTheta;

	return {
	    mt::Vecf<2>(x - dWx + dLx, y + dWy + dLy),
	    mt::Vecf<2>(x - dWx - dLx, y - dLy + dWy),
	    mt::Vecf<2>(x + dWx - dLx, y - dWy - dLy),
	    mt::Vecf<2>(x + dWx + dLx, y + dLy - dWy),
	};
}
}  // namespace

//////////////////////////////////////////////////////////////////////////

// mGeometryLoader needs its (package, base_class) args in a real
// member-initializer-list -- pluginlib::ClassLoader has no default
// constructor, so this can't live in configure() (member-initializer-lists
// are constructor-only syntax).

//////////////////////////////////////////////////////////////////////////

SscPlanner::SscPlanner() : mGeometryLoader("motion_model_base", "GeometricModel")
{
}

//////////////////////////////////////////////////////////////////////////

void SscPlanner::configure(mpl::rclcpp_utils::Logger & logger, const YAML::Node & agentsConfig,
    const std::string & graphFilepath, const std::string & sscConfigFilePath, const UniqueId & id,
    const rclcpp::Node::SharedPtr & node)
{
	mLogger = logger;
	mAgentsConfig = agentsConfig;
	mId = id;
	mNode = node;
	mAgentNum = static_cast<int>(id.value());
	mWorldSnapshot.mAgentsSnapShot.clear();
	if (!mpl_route::GraphLoader().loadGraphFromFile(
	        mWorldSnapshot.routeGraph, mWorldSnapshot.routeGraphIdMap, graphFilepath)) {
		mLogger.error("SscPlanner: failed to load route graph from %s", graphFilepath.c_str());
	}
	mMapConfig = ssc_planner::loadSscMapConfig(sscConfigFilePath);
	mSscMap = std::make_unique<ssc_planner::SscMap>(mMapConfig, mLogger);
	// Latched: computeTrajectory() currently runs once (see PlannerNode's
	// one-shot timer), so a late-connecting RViz subscriber still needs to
	// receive that one publish.
	mCorridorMarkerPub = mNode->create_publisher<visualization_msgs::msg::MarkerArray>(
	    "~/ssc/corridor_markers", rclcpp::QoS(1).transient_local());
	mCorridorMarkerCartesianPub = mNode->create_publisher<visualization_msgs::msg::MarkerArray>(
	    "~/ssc/corridor_markers_map", rclcpp::QoS(1).transient_local());

	// Own geometry model
	const YAML::Node egoConfig = mAgentsConfig["agent_" + std::to_string(mAgentNum)];
	const std::string geomType = egoConfig["geometry_plugin"].as<std::string>();
	auto geom = mGeometryLoader.createSharedInstance(geomType);
	geom->initialize(egoConfig["geometry_params"], mId);
	mWorldSnapshot.geomModel = std::move(geom);

	// Own state -- plain odom, not latched (matches the publisher's own
	// QoS in motion_model_ground_vehicles).
	mSubscriptions.push_back(mNode->create_subscription<nav_msgs::msg::Odometry>(
	    "/agent_" + std::to_string(mAgentNum) + "/odom", rclcpp::QoS(1),
	    [this](nav_msgs::msg::Odometry::SharedPtr msg) {
		    mWorldSnapshot.egoPose.header = msg->header;
		    mWorldSnapshot.egoPose.pose = msg->pose.pose;
		    // mWorldSnapshot.geomModel->step(toStPose(mWorldSnapshot.egoPose.pose));
	    }));

	receivePredictedTrajectoryStatic();
}

//////////////////////////////////////////////////////////////////////////

void SscPlanner::receivePredictedTrajectoryStatic()
{
	// Transient-local to match global_waypoints_publisher.py's publisher
	// QoS for /agent_i/reference_trajectory -- without this, a late
	// subscriber (this one) never receives the one retained latched
	// message.
	const rclcpp::QoS latchedQos = rclcpp::QoS(1).transient_local().reliable();

	for (const auto & entry : mAgentsConfig) {
		const std::string agentName = entry.first.as<std::string>();
		const int othermAgentNum = std::stoi(agentName.substr(agentName.find_last_of('_') + 1));
		if (othermAgentNum == mAgentNum) {
			continue;  // never subscribe to self
		}
		const UniqueId otherId("planning_agent", othermAgentNum);
		const std::string ns = "/agent_" + std::to_string(othermAgentNum);

		// Geometric model for this other agent, so we can forward-propagate
		// its footprint the same way we do our own above.
		const std::string geomType = entry.second["geometry_plugin"].as<std::string>();
		auto geom = mGeometryLoader.createSharedInstance(geomType);
		geom->initialize(entry.second["geometry_params"], otherId);

		AgentSnapshot & initialSnapshot = mWorldSnapshot.mAgentsSnapShot[otherId];
		initialSnapshot.id = otherId;
		initialSnapshot.geomModel = std::move(geom);

		mSubscriptions.push_back(mNode->create_subscription<project_utils_msgs::msg::Trajectory>(
		    ns + "/reference_trajectory", latchedQos,
		    [this, otherId](project_utils_msgs::msg::Trajectory::SharedPtr msg) {
			    AgentSnapshot & snapshot = mWorldSnapshot.mAgentsSnapShot[otherId];
			    snapshot.id = otherId;
			    snapshot.agentPredTrajectory = *msg;
		    }));

		mSubscriptions.push_back(mNode->create_subscription<nav_msgs::msg::Odometry>(
		    ns + "/odom", rclcpp::QoS(1), [this, otherId](nav_msgs::msg::Odometry::SharedPtr msg) {
			    AgentSnapshot & snapshot = mWorldSnapshot.mAgentsSnapShot[otherId];
			    snapshot.id = otherId;
			    snapshot.agentPose.header = msg->header;
			    snapshot.agentPose.pose = msg->pose.pose;
			    // snapshot.geomModel->step(toStPose(snapshot.agentPose.pose));
		    }));
	}
}

//////////////////////////////////////////////////////////////////////////

void SscPlanner::computeTrajectory(const InputData & inputData)
{
	setStartTime(inputData.mEgoPose.header.stamp);
	ssc_planner::constructReferenceLane(inputData, &mRefLane, mLogger);
	projectAgentsToFrenet();
	projectEgoToFrenet(inputData);
	mSscMap->resetSscMap(mWorldSnapshot.egoFrenetSnapShot.at(0).frenetState);
	computeSSCCorridor();
	computeBezierTrajectory();
}

//////////////////////////////////////////////////////////////////////////

void SscPlanner::projectAgentsToFrenet()
{
	for (auto & [id, snapshot] : mWorldSnapshot.mAgentsSnapShot) {
		mt::vec_E<FsVehicle> & frenetTraj = mWorldSnapshot.mAgentsFrenetSnapShot[id];
		frenetTraj.clear();
		frenetTraj.reserve(snapshot.agentPredTrajectory.points.size());

		// Assumes rectangle shape
		const auto shape = snapshot.geomModel->describe();
		double length = shape.rect.length;
		double width = shape.rect.width;

		if (mMapConfig.egoInflation.simple) {
			// Grow this agent's footprint by ego's own size before
			// computing corners -- ego-footprint-margin decision, see
			// ssc_map_config.hpp: stands in for the occupancy-based margin
			// the SSC reference gets from InflateObstacleGrid, which this
			// project doesn't have (no static-obstacle rasterization). Not
			// a true Minkowski sum of two arbitrarily-rotated rectangles --
			// grows the box uniformly by ego's extent before rotating by
			// this agent's own heading, slightly conservative (never
			// smaller than the true swept region).
			const auto egoShape = mWorldSnapshot.geomModel->describe();
			length += egoShape.rect.length;
			width += egoShape.rect.width;
		} else if (mMapConfig.egoInflation.minkowski) {
			// TODO: true Minkowski-sum inflation of two arbitrarily-rotated
			// rectangles -- not yet implemented.
			// or use nav2 cosmaps
		}

		for (const auto & point : snapshot.agentPredTrajectory.points) {
			FsVehicle fsVehicle;

			const auto [s, d] = ssc_planner::projectOntoReferenceLane(
			    mRefLane, point.pose.position.x, point.pose.position.y);
			fsVehicle.frenetState.s = static_cast<float>(s);
			fsVehicle.frenetState.d = static_cast<float>(d);
			fsVehicle.frenetState.t = static_cast<float>(
			    getRelTime(snapshot.agentPredTrajectory.header.stamp, point.time_from_start));
			// Assumes body-frame longitudinal speed ~= ds/dt along the
			// reference lane, not a real Frenet velocity decomposition --
			// see FrenetState::sDot's doc comment for why.
			fsVehicle.frenetState.sDot = point.longitudinal_velocity_mps;

			// Rotated footprint corners at this predicted point (world
			// frame)
			const double yaw = mpl::geometry_utils::getYawFromQuaternion(point.pose.orientation);
			const auto corners =
			    rectangleVertices(point.pose.position.x, point.pose.position.y, yaw, length, width);
			for (const auto & corner : corners) {
				const auto [cs, cd] =
				    ssc_planner::projectOntoReferenceLane(mRefLane, corner(0), corner(1));
				fsVehicle.vertices.push_back(
				    mt::Vecf<2>(static_cast<float>(cs), static_cast<float>(cd)));
			}

			frenetTraj.push_back(fsVehicle);
		}
	}
}

//////////////////////////////////////////////////////////////////////////

void SscPlanner::projectEgoToFrenet(const InputData & inputData)
{
	mt::vec_E<FsVehicle> & frenetTraj = mWorldSnapshot.egoFrenetSnapShot;
	frenetTraj.clear();
	frenetTraj.reserve(inputData.mEgoInitTraj.points.size());

	// Assumes rectangle shape, matching projectAgentsToFrenet().
	const auto shape = mWorldSnapshot.geomModel->describe();
	const double length = shape.rect.length;
	const double width = shape.rect.width;

	for (const auto & point : inputData.mEgoInitTraj.points) {
		FsVehicle fsVehicle;

		const auto [s, d] = ssc_planner::projectOntoReferenceLane(
		    mRefLane, point.pose.position.x, point.pose.position.y);
		fsVehicle.frenetState.s = static_cast<float>(s);
		fsVehicle.frenetState.d = static_cast<float>(d);
		fsVehicle.frenetState.t = static_cast<float>(
		    getRelTime(inputData.mEgoInitTraj.header.stamp, point.time_from_start));
		// Assumes body-frame longitudinal speed ~= ds/dt along the
		// reference lane, not a real Frenet velocity decomposition -- see
		// FrenetState::sDot's doc comment for why.
		fsVehicle.frenetState.sDot = point.longitudinal_velocity_mps;

		// Rotated footprint corners at this predicted point (world frame).
		const double yaw = mpl::geometry_utils::getYawFromQuaternion(point.pose.orientation);
		const auto corners =
		    rectangleVertices(point.pose.position.x, point.pose.position.y, yaw, length, width);
		for (const auto & corner : corners) {
			const auto [cs, cd] =
			    ssc_planner::projectOntoReferenceLane(mRefLane, corner(0), corner(1));
			fsVehicle.vertices.push_back(
			    mt::Vecf<2>(static_cast<float>(cs), static_cast<float>(cd)));
		}

		frenetTraj.push_back(fsVehicle);
	}
}

//////////////////////////////////////////////////////////////////////////

void SscPlanner::computeSSCCorridor()
{
	auto status = mSscMap->constructSscMap(mWorldSnapshot.mAgentsFrenetSnapShot);
	if (status == ssc_planner::kWrongStatus) {
		mLogger.error("[SscPlanner] failed to construct ssc map");
		return;
	}
	mSscMap->constructCorridorUsingInitialTrajectory(mWorldSnapshot.egoFrenetSnapShot);
	if (mSscMap->getFinalGlobalMetricCubesList() == ssc_planner::kWrongStatus) {
		mLogger.error("[SscPlanner] failed to get final corridor");
	}

	const auto markerArray = ssc_planner::buildCorridorMarkers(mSscMap->finalCorridorVec(),
	    mSscMap->ifCorridorValid(), "agent_" + std::to_string(mAgentNum) + "/ssc_debug",
	    mNode->now());
	mCorridorMarkerPub->publish(markerArray);

	// Real-map-frame overlay, for showing the corridor alongside the actual
	// scene (map/agents/routes) rather than the abstract debug view above.
	const auto markerArrayCartesian = ssc_planner::buildCorridorMarkersCartesian(
	    mSscMap->finalCorridorVec(), mSscMap->ifCorridorValid(), mRefLane, "map", mNode->now());
	mCorridorMarkerCartesianPub->publish(markerArrayCartesian);
}

//////////////////////////////////////////////////////////////////////////

void SscPlanner::computeBezierTrajectory()
{
}

PLUGINLIB_EXPORT_CLASS(SscPlanner, PlannerBase)
