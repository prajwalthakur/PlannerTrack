/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#pragma once

#include "min_curv_planner/params.hpp"

#include <Eigen/Dense>

/** \file
 * \brief Speed profile on a fixed closed path (paper §3.5, tph
 * calc_vel_profile / calc_ax_profile / calc_t_profile), with constant
 * acceleration limits instead of a speed-dependent ggv table.
 *
 * Forward-backward solver: start from the cornering limit
 * v = sqrt(ay_max / |kappa|) capped at vMax, raise speed only as far as the
 * remaining longitudinal grip allows going forward (accelerating), then the
 * same backward (braking). Grip left for a_x at lateral load a_y:
 *   a_x,avail = maxDecel * (1 - (a_y / ayTires)^p)^(1/p),   p = dynModelExp
 * (p = 1: diamond, p = 2: friction ellipse). maxDecel is the tires'
 * longitudinal grip, so it bounds braking and, through this budget,
 * acceleration as well; acceleration is additionally capped by the drive
 * limit maxAccel. Drag a = -dragCoeff v^2 / mass. The closed loop is handled by
 * running each pass over the lap twice.
 *
 * Differences from tph:
 *  - tph's backward pass pairs reversed points with the segment length of the
 *    next segment (it reverses the per-segment array as if it were per point).
 *    Here each step uses its own segment's length.
 *  - Closed loop: after the backward pass tph keeps the second of the two
 *    laps, whose end is never braked for the corner at the start of the next
 *    lap: a speed jump at start/finish (-13 m/s^2 on the racing track with an
 *    8 m/s^2 limit). Here the first lap is kept.
 */

namespace min_curv_planner
{

struct VelocityLimits
{
	double vMax{10.0};  ///< [m/s]
	/// Longitudinal tire grip [m/s^2]: limits braking, and through the shared
	/// grip budget also acceleration (before the maxAccel cap). Should not
	/// exceed ayTires (both are the tires' friction).
	double maxDecel{8.0};
	double ayTires{8.0};  ///< lateral tire limit [m/s^2]
	double maxAccel{5.0};  ///< drive (motor) limit, accelerating only [m/s^2]
	double dynModelExp{1.0};  ///< friction-ellipse exponent p, in [1, 2]
	double dragCoeff{0.0};  ///< [kg/m]
	double mass{1.0};  ///< [kg]
};

/// Limits from the pipeline parameters: maxDecel and maxAccel from
/// vehicle_limits (agents.yaml), ayTires = frictionMargin * mu * g.
VelocityLimits velocityLimitsFromParams(const RacelineParams & params);

/**
 * \brief Speed at each point of a closed path.
 * \param kappa curvature at the points [rad/m].
 * \param elLengths distance from point k to k+1, last entry closes the loop [m].
 */
Eigen::VectorXd computeVelocityProfile(
    const Eigen::VectorXd & kappa, const Eigen::VectorXd & elLengths, const VelocityLimits & limits);

/// a_x,k = (v_{k+1}^2 - v_k^2) / (2 l_k), closed (paper eq. 27).
Eigen::VectorXd computeAccelProfile(const Eigen::VectorXd & vx, const Eigen::VectorXd & elLengths);

/// Time at each point, N+1 entries: t_0 = 0 ... t_N = lap time [s].
Eigen::VectorXd computeTimeProfile(
    const Eigen::VectorXd & vx, const Eigen::VectorXd & ax, const Eigen::VectorXd & elLengths);

}  // namespace min_curv_planner
