/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#include "min_curv_planner/min_curv_planner.hpp"

#include "project_utils/resampling_utils.hpp"
#include "min_curv_planner/min_curv_optimizer.hpp"
#include "min_curv_planner/track_io.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <filesystem>
#include <stdexcept>

//////////////////////////////////////////////////////////////////////////

void MinCurvPlanner::configure(mpl::rclcpp_utils::Logger & logger, const YAML::Node & agentsConfig,
    const std::string & /*graphFile*/, const std::string & plannerConfigFile, const UniqueId & id,
    const rclcpp::Node::SharedPtr & node)
{
	mLogger = logger;
	mNode = node;
	mAgentNum = static_cast<int>(id.value());

	try {
		// Vehicle parameters: this agent's agents.yaml entry, the same one
		// the simulator uses.
		const std::string agentKey = "agent_" + std::to_string(mAgentNum);
		const YAML::Node agentConfig = agentsConfig[agentKey];
		if (!agentConfig) {
			throw std::runtime_error("no '" + agentKey + "' entry in agents config");
		}

		// Planner tuning: "min_curv_planner" block of the planner config.
		const YAML::Node racelineConfig =
		    YAML::LoadFile(plannerConfigFile)["/**"]["ros__parameters"]["min_curv_planner"];
		if (!racelineConfig) {
			throw std::runtime_error(
			    "no '/**/ros__parameters/min_curv_planner' block in " + plannerConfigFile);
		}
		mParams = min_curv_planner::loadRacelineParams(agentConfig, racelineConfig);
		if (racelineConfig["frame_id"]) {
			mFrameId = racelineConfig["frame_id"].as<std::string>();
		}

		// Centerline: a relative path is resolved against the planner
		// config's own directory, so the scenario folder can move freely.
		std::filesystem::path centerlineFile =
		    racelineConfig["centerline_file"].as<std::string>();
		if (centerlineFile.is_relative()) {
			centerlineFile = std::filesystem::path(plannerConfigFile).parent_path() / centerlineFile;
		}
		const std::string centerlineKey = racelineConfig["centerline_key"]
		    ? racelineConfig["centerline_key"].as<std::string>()
		    : std::string("centerline_waypoints");
		mCenterline =
		    min_curv_planner::loadTrackFromWaypointsJson(centerlineFile.string(), centerlineKey);
		mLogger.info("MinCurvPlanner: loaded %d centerline points from %s", mCenterline.size(),
		    centerlineFile.c_str());
	} catch (const std::exception & e) {
		mLogger.error("MinCurvPlanner: configure failed: %s", e.what());
		throw;
	}

	mLogger.info(
	    "MinCurvPlanner: curvature limit %.3f rad/m, lateral accel limit %.2f m/s^2, "
	    "optimization width %.2f m, max speed %.2f m/s",
	    mParams.curvatureLimit(), mParams.lateralAccelLimit(), mParams.optimizationWidth(),
	    mParams.vehicle.maxSpeed);

	// Latched: computed once, and controllers/RViz may connect later.
	const auto latched = rclcpp::QoS(1).transient_local();
	mTrajectoryPub =
	    mNode->create_publisher<project_utils_msgs::msg::Trajectory>("~/trajectory", latched);
	mWaypointsPub =
	    mNode->create_publisher<project_utils_msgs::msg::WpntArray>("~/waypoints", latched);
	mMarkersPub =
	    mNode->create_publisher<visualization_msgs::msg::MarkerArray>("~/markers", latched);
}

//////////////////////////////////////////////////////////////////////////

PlannerInputs MinCurvPlanner::requiredInputs() const
{
	PlannerInputs inputs;
	inputs.referencePath = false;
	inputs.referenceTrajectory = false;
	inputs.odometry = false;
	return inputs;
}

//////////////////////////////////////////////////////////////////////////

void MinCurvPlanner::computeTrajectory(const InputData & /*inputData*/)
{
	if (mComputed) {
		return;  // static track, outputs are latched
	}
	min_curv_planner::Raceline raceline;
	try {
		raceline = min_curv_planner::computeRaceline(mCenterline, mParams);
	} catch (const std::exception & e) {
		mLogger.error("MinCurvPlanner: computeRaceline failed: %s", e.what());
		return;
	}
	if (!raceline.iqpConverged) {
		mLogger.warn("MinCurvPlanner: IQP stopped after %d iterations without reaching the "
		             "curvature-error tolerance (last error %.4f rad/m)",
		    raceline.iqpIterations, raceline.iqpCurvErrorMax);
	}
	const double maxKappa = raceline.kappa.cwiseAbs().maxCoeff();
	if (maxKappa > mParams.curvatureLimit()) {
		mLogger.warn("MinCurvPlanner: max |kappa| %.3f rad/m exceeds the curvature limit %.3f",
		    maxKappa, mParams.curvatureLimit());
	}
	publish(raceline);
	mComputed = true;
	mLogger.info("MinCurvPlanner: published raceline -- %d points, length %.2f m, estimated lap "
	             "time %.2f s, max speed %.2f m/s, max |kappa| %.3f rad/m, IQP %d iterations",
	    raceline.size(), raceline.length, raceline.lapTime, raceline.vx.maxCoeff(), maxKappa,
	    raceline.iqpIterations);
}

//////////////////////////////////////////////////////////////////////////

void MinCurvPlanner::publish(const min_curv_planner::Raceline & raceline)
{
	std_msgs::msg::Header header;
	header.frame_id = mFrameId;
	header.stamp = mNode->now();

	project_utils_msgs::msg::WpntArray waypoints;
	waypoints.header = header;
	waypoints.wpnts.resize(raceline.size());
	for (int i = 0; i < raceline.size(); ++i) {
		auto & w = waypoints.wpnts[i];
		w.id = i;
		w.s_m = raceline.s(i);
		w.d_m = 0.0;
		w.x_m = raceline.x(i);
		w.y_m = raceline.y(i);
		w.d_right = raceline.wRight(i);
		w.d_left = raceline.wLeft(i);
		w.psi_rad = raceline.psi(i);
		w.kappa_radpm = raceline.kappa(i);
		w.vx_mps = raceline.vx(i);
		w.ax_mps2 = raceline.ax(i);
	}
	mpl::extra_utils::convertPathWptsToTrajectory(waypoints, mTrajectory, header);

	// One closed LINE_STRIP, blue (slow) to red (fast).
	visualization_msgs::msg::Marker line;
	line.header = header;
	line.ns = "raceline";
	line.id = 0;
	line.type = visualization_msgs::msg::Marker::LINE_STRIP;
	line.action = visualization_msgs::msg::Marker::ADD;
	line.pose.orientation.w = 1.0;
	line.scale.x = 0.05;
	const double vMin = raceline.vx.minCoeff();
	const double vRange = std::max(raceline.vx.maxCoeff() - vMin, 1e-6);
	for (int k = 0; k <= raceline.size(); ++k) {
		const int i = k % raceline.size();
		geometry_msgs::msg::Point p;
		p.x = raceline.x(i);
		p.y = raceline.y(i);
		line.points.push_back(p);
		std_msgs::msg::ColorRGBA c;
		const double t = (raceline.vx(i) - vMin) / vRange;
		c.r = static_cast<float>(t);
		c.g = 0.0f;
		c.b = static_cast<float>(1.0 - t);
		c.a = 1.0f;
		line.colors.push_back(c);
	}
	visualization_msgs::msg::MarkerArray markers;
	markers.markers.push_back(line);

	mTrajectoryPub->publish(mTrajectory);
	mWaypointsPub->publish(waypoints);
	mMarkersPub->publish(markers);
}

//////////////////////////////////////////////////////////////////////////

PLUGINLIB_EXPORT_CLASS(MinCurvPlanner, PlannerBase)
