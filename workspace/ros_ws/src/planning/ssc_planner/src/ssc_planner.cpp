/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#include "ssc_planner/ssc_planner.hpp"

#include "mpl_route_planner/graph_loader.hpp"
#include "project_utils/geometry_utils.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
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
	mBezierTrajectoryPub = mNode->create_publisher<project_utils_msgs::msg::Trajectory>(
	    "~/ssc/bezier_trajectory", rclcpp::QoS(1).transient_local());
	mBezierTrajectoryMarkerPub = mNode->create_publisher<visualization_msgs::msg::MarkerArray>(
	    "~/ssc/bezier_trajectory_markers", rclcpp::QoS(1).transient_local());

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
	const ssc_planner::ErrorType status = computeSSCCorridor();
	computeBezierTrajectory(status);
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

ssc_planner::ErrorType SscPlanner::computeSSCCorridor()
{
	auto status = mSscMap->constructSscMap(mWorldSnapshot.mAgentsFrenetSnapShot);
	if (status == ssc_planner::kWrongStatus) {
		mLogger.error("[SscPlanner] failed to construct ssc map");
		return ssc_planner::kWrongStatus;
	}
	mSscMap->constructCorridorUsingInitialTrajectory(mWorldSnapshot.egoFrenetSnapShot);
	if (mSscMap->getFinalGlobalMetricCubesList() == ssc_planner::kWrongStatus) {
		mLogger.error("[SscPlanner] failed to get final corridor");
		return ssc_planner::kWrongStatus;
	}

	const auto markerArray =
	    ssc_planner::buildCorridorMarkers(mSscMap->finalCorridorVec(), mSscMap->ifCorridorValid(),
	        "agent_" + std::to_string(mAgentNum) + "/ssc_debug", mNode->now());
	mCorridorMarkerPub->publish(markerArray);

	// Real-map-frame overlay, for showing the corridor alongside the actual
	// scene (map/agents/routes) rather than the abstract debug view above.
	const auto markerArrayCartesian = ssc_planner::buildCorridorMarkersCartesian(
	    mSscMap->finalCorridorVec(), mSscMap->ifCorridorValid(), mRefLane, "map", mNode->now());
	mCorridorMarkerCartesianPub->publish(markerArrayCartesian);
	return ssc_planner::kSuccess;
}

//////////////////////////////////////////////////////////////////////////

namespace
{
// Estimates (velocity, acceleration) in both s and d 
void estimateVelAccel(const FsVehicle & p0, const FsVehicle & p1, const FsVehicle & p2,
    mt::Vecf<2> * vel, mt::Vecf<2> * acc)
{
	const float dt01 = p1.frenetState.t - p0.frenetState.t;
	const float dt12 = p2.frenetState.t - p1.frenetState.t;
	const mt::Vecf<2> v01(
	    (p1.frenetState.s - p0.frenetState.s) / dt01, (p1.frenetState.d - p0.frenetState.d) / dt01);
	const mt::Vecf<2> v12(
	    (p2.frenetState.s - p1.frenetState.s) / dt12, (p2.frenetState.d - p1.frenetState.d) / dt12);
	*vel = v01;
	*acc = (v12 - v01) / (0.5f * (dt01 + dt12));
}

// velocity_singularity_eps guard (ssc_planner.cc), applied
// to the s-component only
constexpr float kVelocitySingularityEps = 0.1f;
constexpr double kWeightProximity = 1.0;

constexpr double kOutputDt = 0.25;

// One sample of the solved spline, already converted to world-frame
// (x,y,heading,speed) -- kept separate from the final TrajectoryPoint so
// consecutive samples can be finite-differenced into
// acceleration/heading-rate/curvature afterward (same idea as
// estimateVelAccel above, applied to the OUTPUT samples this time).
struct BezierSample
{
	double t;
	float x, y, heading, speed;
};

// solved spline into actual waypoints. Heading/speed use the standard
// Frenet->Cartesian formula (Werling et al.): heading = yaw_ref(s) +
// atan2(dDot, sDot*(1-kappa_r*d)), speed = the resulting velocity
// vector's own magnitude.
BezierSample sampleBezierSpline(const BezierSpline<kBezierOrder, kBezierDim> & spline,
    const mpl::interpolation::SplineInterpolationPoints2d & refLane, double t)
{
	mt::Vecf<2> pos, vel;
	spline.evaluate(t, 0, &pos);
	spline.evaluate(t, 1, &vel);
	const float s = pos[0];
	const float d = pos[1];
	const float sDot = vel[0];
	const float dDot = vel[1];

	const double sForLane = std::clamp(static_cast<double>(s), refLane.getAccumulatedLength(0),
	    refLane.getAccumulatedLength(refLane.getSize() - 1));
	const double yawRef = refLane.getSplineInterpolatedYaw(0, sForLane);
	const double kappaRef = refLane.getSplineInterpolatedCurvature(0, sForLane);
	// Floored the same way velStart/velEnd are for the QP's boundary
	// conditions (kVelocitySingularityEps) -- without this, a near-zero
	// sample (mid-trajectory, not just at the boundary) makes atan2(dDot,
	// sDotEff) noisy/undefined instead of a real heading. Using the same
	// floored value for speed below keeps the two internally consistent.
	const double sDotEff =
	    std::max(sDot * (1.0 - kappaRef * d), static_cast<double>(kVelocitySingularityEps));
	const double heading = yawRef + std::atan2(dDot, sDotEff);
	const double speed = std::hypot(sDotEff, dDot);

	const auto point = ssc_planner::toCartesian(refLane, s, d, 0.0);
	return BezierSample{t, static_cast<float>(point.x), static_cast<float>(point.y),
	    static_cast<float>(heading), static_cast<float>(speed)};
}

// Samples the whole solved spline at kOutputDt  
//Werling, Ziegler, Kammel & Thrun's 2010 ICRA paper "Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame"
project_utils_msgs::msg::Trajectory buildTrajectoryFromSpline(
    const BezierSpline<kBezierOrder, kBezierDim> & spline,
    const mpl::interpolation::SplineInterpolationPoints2d & refLane, const rclcpp::Time & stamp)
{
	project_utils_msgs::msg::Trajectory traj;
	traj.header.stamp = stamp;
	traj.header.frame_id = "map";

	const double tBegin = spline.begin();
	const double tEnd = spline.end();
	if (tEnd <= tBegin) {
		return traj;
	}

	std::vector<BezierSample> samples;
	for (double t = tBegin; t < tEnd; t += kOutputDt) {
		samples.push_back(sampleBezierSpline(spline, refLane, t));
	}
	samples.push_back(sampleBezierSpline(spline, refLane, tEnd));

	traj.points.reserve(samples.size());
	for (std::size_t i = 0; i < samples.size(); ++i) {
		project_utils_msgs::msg::TrajectoryPoint pt;
		pt.time_from_start = rclcpp::Duration::from_seconds(samples[i].t - tBegin);
		pt.pose.position.x = samples[i].x;
		pt.pose.position.y = samples[i].y;
		pt.pose.orientation = mpl::geometry_utils::createQuaternionFromYaw(samples[i].heading);
		pt.longitudinal_velocity_mps = samples[i].speed;
		pt.lateral_velocity_mps = 0.0f;

		if (i + 1 < samples.size()) {
			const double dtOut = samples[i + 1].t - samples[i].t;
			const double dSpeed = samples[i + 1].speed - samples[i].speed;
			const double dHeading = mpl::geometry_utils::shortestAngularDistanceNormalized(
			    static_cast<double>(samples[i].heading), static_cast<double>(samples[i + 1].heading));
			const double headingRate = dtOut > 1e-6 ? dHeading / dtOut : 0.0;
			pt.acceleration_mps2 = static_cast<float>(dtOut > 1e-6 ? dSpeed / dtOut : 0.0);
			pt.heading_rate_rps = static_cast<float>(headingRate);
			pt.track_kappa_radpm =
			    headingRate / std::max(static_cast<double>(samples[i].speed),
			                      static_cast<double>(kVelocitySingularityEps));
		} else if (!traj.points.empty()) {
			// Last sample: repeat the previous point's rates rather than
			// snapping to zero right at the final waypoint.
			pt.acceleration_mps2 = traj.points.back().acceleration_mps2;
			pt.heading_rate_rps = traj.points.back().heading_rate_rps;
			pt.track_kappa_radpm = traj.points.back().track_kappa_radpm;
		}
		traj.points.push_back(pt);
	}
	return traj;
}
}  // namespace

void SscPlanner::computeBezierTrajectory(ssc_planner::ErrorType corridorStatus)
{
	if (corridorStatus == ssc_planner::kWrongStatus) {
		mLogger.error("[SscPlanner] skipping Bezier solve -- corridor generation failed");
		return;
	}
	mLogger.info("[SscPlanner] computing bezier trajectory from corridor");

	const auto finalCorridor = mSscMap->finalCorridorVec();
	const auto ifCorridorValid = mSscMap->ifCorridorValid();
	// One fixed ego route -> one corridor 
	if (finalCorridor.empty() || finalCorridor[0].empty() ||
	    (!ifCorridorValid.empty() && ifCorridorValid[0] == 0)) {
		mLogger.error("[SscPlanner] skipping Bezier solve -- empty/invalid corridor");
		return;
	}

	const auto & egoTraj = mWorldSnapshot.egoFrenetSnapShot;
	if (egoTraj.size() < 3) {
		mLogger.error(
		    "[SscPlanner] skipping Bezier solve -- egoFrenetSnapShot has < 3 points (%zu)",
		    egoTraj.size());
		return;
	}

	// ~ boundary conditions, finite-differenced from the seed trajectory
	mt::Vecf<2> velStart, accStart, velEnd, accEnd;
	estimateVelAccel(egoTraj[0], egoTraj[1], egoTraj[2], &velStart, &accStart);
	estimateVelAccel(
	    egoTraj[egoTraj.size() - 3], egoTraj[egoTraj.size() - 2], egoTraj.back(), &velEnd, &accEnd);
	velStart[0] = std::max(velStart[0], kVelocitySingularityEps);
	velEnd[0] = std::max(velEnd[0], kVelocitySingularityEps);

	const mt::Vecf<2> posStart(egoTraj.front().frenetState.s, egoTraj.front().frenetState.d);
	const float sEndClamped =
	    std::min(egoTraj.back().frenetState.s, finalCorridor[0].back().pUb[0]);
	const mt::Vecf<2> posEnd(sEndClamped, egoTraj.back().frenetState.d);

	const mt::vec_E<mt::Vecf<2>> startConstraints{posStart, velStart, accStart};
	const mt::vec_E<mt::Vecf<2>> endConstraints{posEnd, velEnd};

	// ~ reference-tracking samples: the whole seed trajectory
	std::vector<double> refStamps;
	mt::vec_E<mt::Vecf<2>> refPoints;
	refStamps.reserve(egoTraj.size());
	refPoints.reserve(egoTraj.size());
	for (const auto & fv : egoTraj) {
		refStamps.push_back(static_cast<double>(fv.frenetState.t));
		refPoints.push_back(mt::Vecf<2>(fv.frenetState.s, fv.frenetState.d));
	}

	BezierSpline<kBezierOrder, kBezierDim> spline;
	const auto solveStatus = SolveBezierSpline(finalCorridor[0], startConstraints, endConstraints,
	    refStamps, refPoints, kWeightProximity, &spline);
	if (solveStatus != ssc_planner::kSuccess) {
		mLogger.error("[SscPlanner] Bezier QP solve failed");
		return;
	}

	mBezierSpline = spline;
	mBezierSplineValid = true;
	mLogger.info("[SscPlanner] Bezier trajectory solved -- %d segments", spline.num_segments());

	const auto trajectory = buildTrajectoryFromSpline(mBezierSpline, mRefLane, mNode->now());
	mBezierTrajectoryPub->publish(trajectory);
	mLogger.info(
	    "[SscPlanner] published Bezier trajectory -- %zu waypoints", trajectory.points.size());

	const auto trajectoryMarker =
	    ssc_planner::buildBezierTrajectoryMarker(trajectory, "map", mNode->now());
	mBezierTrajectoryMarkerPub->publish(trajectoryMarker);
	mLogger.info("[SscPlanner] computed bezier trajectory,published marker");

}

PLUGINLIB_EXPORT_CLASS(SscPlanner, PlannerBase)
