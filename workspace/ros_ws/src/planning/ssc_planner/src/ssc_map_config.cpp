/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#include "ssc_planner/ssc_map_config.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>

namespace ssc_planner
{

SscMapConfig loadSscMapConfig(const std::string & sscConfigFilePath)
{
	SscMapConfig config;


	const YAML::Node root = YAML::LoadFile(sscConfigFilePath)["/**"]["ros__parameters"];
	const YAML::Node sscMap = root["ssc_map"];

	const YAML::Node resolution = sscMap["map_resolution"];
	config.mapReslX = resolution["x"].as<double>();
	config.mapReslY = resolution["y"].as<double>();
	config.mapReslZ = resolution["z"].as<double>();

	// Lateral bound (in d dimension): one lane's width, not the full road. d is measured
	// relative to mRefLane -- ego's own already-curving seed path -- so the
	// turn's sweep is already captured by s moving along that curve; d is
	// only ever a small perpendicular offset from wherever the path is
	// currently pointing.
	const double laneWidth = sscMap["lane_width"].as<double>();
	config.mapSizeY = static_cast<int>(std::ceil(laneWidth / config.mapReslY));

	// Longitudinal span: generously cover a full arm-to-arm crossing.
	// s dimension hopefully enough length for one shot planning
	const double armLength = sscMap["arm_length"].as<double>();
	config.mapSizeX = static_cast<int>(std::ceil((2.0 * armLength + laneWidth) / config.mapReslX));

	// Planning horizon in time axis: no time-horizon field in this config-- placeholder.
	const double horizonSec = sscMap["planning_horizon"].as<double>();;
	config.mapSizeZ = static_cast<int>(std::ceil(horizonSec / config.mapReslZ));

	config.egoInflation.simple = sscMap["if_ego_inflation_simple"].as<bool>();
	config.egoInflation.minkowski = sscMap["if_ego_inflation_minkowski"].as<bool>();

	config.sBackLen = sscMap["s_back_len"].as<double>();

	const YAML::Node dynBounds = sscMap["dynamic_bounds"];
	config.dynBounds.maxLonVel = dynBounds["max_lon_vel"].as<double>();
	config.dynBounds.minLonVel = dynBounds["min_lon_vel"].as<double>();
	config.dynBounds.maxLonAcc = dynBounds["max_lon_acc"].as<double>();
	config.dynBounds.maxLonDec = dynBounds["max_lon_dec"].as<double>();
	config.dynBounds.maxLatVel = dynBounds["max_lat_vel"].as<double>();
	config.dynBounds.maxLatAcc = dynBounds["max_lat_acc"].as<double>();

	config.maxGridsAlongTime = sscMap["max_grids_along_time"].as<int>();

	const YAML::Node inflSteps = sscMap["inflate_steps"];
	config.inflSteps.xP = inflSteps["x_pos"].as<int>();
	config.inflSteps.xN = inflSteps["x_neg"].as<int>();
	config.inflSteps.yP = inflSteps["y_pos"].as<int>();
	config.inflSteps.yN = inflSteps["y_neg"].as<int>();
	config.inflSteps.zP = inflSteps["z_pos"].as<int>();
	config.inflSteps.zN = inflSteps["z_neg"].as<int>();

	return config;
}

}  // namespace ssc_planner
