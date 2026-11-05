/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#include "min_curv_planner/params.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace min_curv_planner
{

namespace
{
/// Overwrite value with node[key] if present; keeps the default otherwise.
template <typename T>
void readIfPresent(const YAML::Node & node, const char * key, T & value)
{
	if (node && node[key]) {
		value = node[key].as<T>();
	}
}

void requirePositive(double value, const char * name)
{
	if (!(value > 0.0)) {
		throw std::invalid_argument(std::string("raceline params: ") + name + " must be > 0");
	}
}
}  // namespace

//////////////////////////////////////////////////////////////////////////

double RacelineParams::curvatureLimit() const
{
	return std::tan(vehicle.maxSteer) / vehicle.wheelbase;
}

double RacelineParams::lateralAccelLimit() const
{
	return frictionMargin * vehicle.mu * vehicle.gravity;
}

double RacelineParams::optimizationWidth() const
{
	return vehicle.width + 2.0 * safetyMargin;
}

//////////////////////////////////////////////////////////////////////////

RacelineParams loadRacelineParams(const YAML::Node & agentConfig, const YAML::Node & racelineConfig)
{
	RacelineParams params;

	// Vehicle: physical values come from the same agents.yaml entry the
	// simulator uses, so the reference and the plant cannot drift apart.
	VehicleParams & veh = params.vehicle;
	const YAML::Node dyn = agentConfig["dynamics_params"];
	readIfPresent(dyn, "mu", veh.mu);
	readIfPresent(dyn, "vehWheelBase", veh.wheelbase);
	readIfPresent(dyn, "mass", veh.mass);
	readIfPresent(agentConfig["geometry_params"], "width", veh.width);
	const YAML::Node limits = agentConfig["vehicle_limits"];
	readIfPresent(limits, "max_steer", veh.maxSteer);
	readIfPresent(limits, "max_accel", veh.maxAccel);
	readIfPresent(limits, "max_decel", veh.maxDecel);
	readIfPresent(limits, "max_speed", veh.maxSpeed);
	readIfPresent(limits, "max_steer_rate", veh.maxSteerRate);
	readIfPresent(limits, "max_jerk", veh.maxJerk);

	// Planner tuning.
	readIfPresent(racelineConfig, "friction_margin", params.frictionMargin);
	readIfPresent(racelineConfig, "safety_margin", params.safetyMargin);

	const YAML::Node prep = racelineConfig["preprocessing"];
	readIfPresent(prep, "enable_smoothing", params.preprocessing.enableSmoothing);
	readIfPresent(prep, "stepsize_prep", params.preprocessing.stepsizePrep);
	readIfPresent(prep, "stepsize_reg", params.preprocessing.stepsizeReg);
	readIfPresent(prep, "smoothing_residual", params.preprocessing.smoothingResidual);

	const YAML::Node opt = racelineConfig["optimization"];
	std::string objective = "min_curvature";
	readIfPresent(opt, "objective", objective);
	if (objective == "min_curvature") {
		params.optimization.objective = Objective::MinCurvature;
	} else if (objective == "tph_compat") {
		params.optimization.objective = Objective::TphCompat;
	} else {
		throw std::invalid_argument(
		    "raceline params: optimization.objective must be min_curvature or tph_compat, got " +
		    objective);
	}
	readIfPresent(opt, "iqp_iters_min", params.optimization.iqpItersMin);
	readIfPresent(opt, "iqp_iters_max", params.optimization.iqpItersMax);
	readIfPresent(opt, "iqp_curv_error_allowed", params.optimization.iqpCurvErrorAllowed);

	const YAML::Node vel = racelineConfig["velocity_profile"];
	readIfPresent(vel, "dyn_model_exp", params.velocityProfile.dynModelExp);
	readIfPresent(vel, "drag_coeff", params.velocityProfile.dragCoeff);

	readIfPresent(racelineConfig["output"], "stepsize_interp", params.stepsizeInterp);

	requirePositive(veh.mu, "mu");
	requirePositive(veh.wheelbase, "vehWheelBase");
	requirePositive(veh.width, "width");
	requirePositive(veh.mass, "mass");
	requirePositive(veh.maxSteer, "max_steer");
	requirePositive(veh.maxAccel, "max_accel");
	requirePositive(veh.maxDecel, "max_decel");
	requirePositive(veh.maxSpeed, "max_speed");
	requirePositive(veh.maxSteerRate, "max_steer_rate");
	requirePositive(veh.maxJerk, "max_jerk");
	requirePositive(params.frictionMargin, "friction_margin");
	requirePositive(params.preprocessing.stepsizePrep, "stepsize_prep");
	requirePositive(params.preprocessing.stepsizeReg, "stepsize_reg");
	requirePositive(params.stepsizeInterp, "stepsize_interp");
	if (params.optimization.iqpItersMax < params.optimization.iqpItersMin) {
		throw std::invalid_argument("raceline params: iqp_iters_max < iqp_iters_min");
	}
	return params;
}

}  // namespace min_curv_planner
