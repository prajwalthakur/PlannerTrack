/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */

#include "min_curv_planner/min_curv_optimizer.hpp"
#include "min_curv_planner/params.hpp"
#include "min_curv_planner/track_io.hpp"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <string>

/** \file
 * \brief Minimum-curvature raceline (min_curv_planner core) in
 * \c plannertrack_py._core. Python wrapper: plannertrack_py/raceline.py.
 */

namespace py = pybind11;

namespace plannertrack_py
{

namespace
{
using min_curv_planner::Raceline;
using min_curv_planner::Track;

// (N, 4) array [x, y, w_right, w_left] -> Track.
Track trackFromArray(const Eigen::MatrixXd & a)
{
	if (a.cols() != 4) {
		throw std::invalid_argument(
		    "track must be an (N, 4) array [x, y, w_right, w_left], got " +
		    std::to_string(a.cols()) + " columns");
	}
	Track t;
	t.xy = a.leftCols(2);
	t.wRight = a.col(2);
	t.wLeft = a.col(3);
	return t;
}

Eigen::MatrixXd trackToArray(const Track & t)
{
	Eigen::MatrixXd a(t.size(), 4);
	a << t.xy, t.wRight, t.wLeft;
	return a;
}

py::dict racelineToDict(const Raceline & r)
{
	py::dict d;
	d["s"] = r.s;
	d["x"] = r.x;
	d["y"] = r.y;
	d["psi"] = r.psi;
	d["kappa"] = r.kappa;
	d["vx"] = r.vx;
	d["ax"] = r.ax;
	d["w_right"] = r.wRight;
	d["w_left"] = r.wLeft;
	d["length"] = r.length;
	d["lap_time"] = r.lapTime;
	d["iqp_iterations"] = r.iqpIterations;
	d["iqp_converged"] = r.iqpConverged;
	d["iqp_curv_error_max"] = r.iqpCurvErrorMax;
	return d;
}

// Parameters exactly as the ROS plugin builds them: one agent's node of
// agents.yaml plus the "min_curv_planner" block of the planner config.
min_curv_planner::RacelineParams paramsFromYaml(
    const std::string & agentYaml, const std::string & plannerBlockYaml)
{
	return min_curv_planner::loadRacelineParams(
	    YAML::Load(agentYaml), YAML::Load(plannerBlockYaml));
}
}  // namespace

void bindRaceline(py::module_ & m)
{
	m.def(
	    "load_track_json",
	    [](const std::string & path, const std::string & key) {
		    return trackToArray(min_curv_planner::loadTrackFromWaypointsJson(path, key));
	    },
	    py::arg("path"), py::arg("key") = "centerline_waypoints",
	    "Closed track from a global_waypoints.json as an (N, 4) array "
	    "[x, y, w_right, w_left] (repeated closing point dropped).");

	m.def(
	    "compute_raceline",
	    [](const Eigen::MatrixXd & centerline, const std::string & agentYaml,
	        const std::string & plannerYaml) {
		    const Track track = trackFromArray(centerline);
		    const auto params = paramsFromYaml(agentYaml, plannerYaml);
		    Raceline r;
		    {
			    py::gil_scoped_release release;
			    r = min_curv_planner::computeRaceline(track, params);
		    }
		    return racelineToDict(r);
	    },
	    py::arg("centerline"), py::arg("agent_yaml"), py::arg("planner_yaml"),
	    "Full pipeline (preprocessing, minimum-curvature IQP, speed profile) on an (N, 4) "
	    "centerline. agent_yaml: one agent's node of agents.yaml; planner_yaml: the "
	    "'min_curv_planner' block of the planner config. Returns a dict of arrays.");

	m.def(
	    "raceline_limits",
	    [](const std::string & agentYaml, const std::string & plannerYaml) {
		    const auto p = paramsFromYaml(agentYaml, plannerYaml);
		    py::dict d;
		    d["curvature_limit"] = p.curvatureLimit();
		    d["ay_max"] = p.lateralAccelLimit();
		    d["max_decel"] = p.vehicle.maxDecel;
		    d["max_accel"] = p.vehicle.maxAccel;
		    d["v_max"] = p.vehicle.maxSpeed;
		    d["max_steer"] = p.vehicle.maxSteer;
		    d["max_steer_rate"] = p.vehicle.maxSteerRate;
		    d["max_jerk"] = p.vehicle.maxJerk;
		    d["dyn_model_exp"] = p.velocityProfile.dynModelExp;
		    d["optimization_width"] = p.optimizationWidth();
		    d["vehicle_width"] = p.vehicle.width;
		    return d;
	    },
	    py::arg("agent_yaml"), py::arg("planner_yaml"),
	    "Limits the optimizer uses (curvature, lateral/longitudinal acceleration, speed, "
	    "widths), computed by the same C++ code; for validating a reference.");

	m.def(
	    "optimize_raceline",
	    [](const Eigen::MatrixXd & reference, const std::string & agentYaml,
	        const std::string & plannerYaml) {
		    const Track track = trackFromArray(reference);
		    const auto params = paramsFromYaml(agentYaml, plannerYaml);
		    Raceline r;
		    {
			    py::gil_scoped_release release;
			    r = min_curv_planner::optimizeRaceline(track, params);
		    }
		    return racelineToDict(r);
	    },
	    py::arg("reference"), py::arg("agent_yaml"), py::arg("planner_yaml"),
	    "Like compute_raceline but skips preprocessing (reference line already prepared).");
}

}  // namespace plannertrack_py
