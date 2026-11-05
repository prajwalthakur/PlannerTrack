/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#pragma once

#include <yaml-cpp/yaml.h>

/** \file
 * \brief Parameters for the raceline pipeline, and the one place that reads
 * them from YAML (used by the ROS plugin, the Python bindings and tests).
 */

namespace min_curv_planner
{

/// Physical vehicle values, read from the agent's entry in agents.yaml.
struct VehicleParams
{
	double mu{1.0};  ///< tire-road friction coefficient [-]
	double wheelbase{0.34};  ///< [m]
	double width{0.31};  ///< [m]
	double mass{3.47};  ///< [kg]
	double maxSteer{0.4189};  ///< [rad]
	double maxAccel{5.0};  ///< drive limit [m/s^2], > 0
	double maxDecel{8.0};  ///< braking limit [m/s^2], > 0
	double maxSpeed{10.0};  ///< [m/s]
	double maxSteerRate{3.2};  ///< [rad/s], bound for the MPC (not used by the raceline)
	double maxJerk{50.0};  ///< [m/s^3], bound for the MPC (not used by the raceline)
	double gravity{9.81};  ///< [m/s^2]
};

struct PreprocessingParams
{
	bool enableSmoothing{true};
	double stepsizePrep{0.1};  ///< linear resampling before smoothing [m]
	double stepsizeReg{0.3};  ///< optimization grid spacing [m]
	double smoothingResidual{10.0};  ///< sum of squared residuals, same meaning as TUM s_reg [m^2]
};

enum class Objective {
	MinCurvature,  ///< minimize sum(kappa^2) (corrected objective)
	TphCompat,  ///< reproduce TUM opt_min_curv (2x linear term), parity tests only
};

struct OptimizationParams
{
	Objective objective{Objective::MinCurvature};
	int iqpItersMin{3};
	int iqpItersMax{20};
	double iqpCurvErrorAllowed{0.01};  ///< [rad/m]
};

struct VelocityProfileParams
{
	double dynModelExp{1.0};  ///< friction-ellipse exponent, range [1, 2]
	double dragCoeff{0.0};  ///< [kg/m], drag decel = dragCoeff * v^2 / mass
};

struct RacelineParams
{
	VehicleParams vehicle;
	double frictionMargin{0.85};  ///< ay_max = frictionMargin * mu * g
	double safetyMargin{0.10};  ///< per side, added to the vehicle width [m]
	PreprocessingParams preprocessing;
	OptimizationParams optimization;
	VelocityProfileParams velocityProfile;
	double stepsizeInterp{0.2};  ///< output spacing [m]

	/// Curvature limit from the steering limit: tan(maxSteer) / wheelbase [rad/m].
	double curvatureLimit() const;
	/// Lateral acceleration limit: frictionMargin * mu * g [m/s^2].
	double lateralAccelLimit() const;
	/// Width the optimizer keeps clear: vehicle width + 2 * safetyMargin [m].
	double optimizationWidth() const;
};

/**
 * \brief Build parameters from YAML.
 * \param agentConfig one agent's entry of agents.yaml ("agent_1": dynamics_params,
 *        geometry_params, vehicle_limits).
 * \param racelineConfig the "min_curv_planner" node of the planner config.
 * Missing keys keep the defaults above.
 */
RacelineParams loadRacelineParams(const YAML::Node & agentConfig, const YAML::Node & racelineConfig);

}  // namespace min_curv_planner
