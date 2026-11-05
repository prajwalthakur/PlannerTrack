/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#pragma once

#include "min_curv_planner/params.hpp"
#include "min_curv_planner/types.hpp"

/** \file
 * \brief Entry point of the raceline pipeline: a light C++ port of TUM
 * global_racetrajectory_optimization, opt_type = mincurv_iqp (Heilmeier et
 * al., VSD 2020), with the objective's factor-2 error corrected.
 *
 * Stages (derivations in project_docs/learning_based_controls/):
 *  1. track_preprocessing -- linear resample, smoothing, widths, resample
 *  2. closed_spline       -- C2 closed cubic spline, normals, S1/S2 maps
 *  3. min_curvature       -- QP on lateral offsets + IQP loop
 *  4. final raceline      -- spline through the result, resampled at
 *                            stepsizeInterp, analytic heading/curvature
 *  5. velocity_profile    -- forward/backward speed profile, a_x, lap time
 */

namespace min_curv_planner
{

// struct SplineBasis
// {
// 	Eigen::MatrixXd splineCoeff_x;
// 	Eigen::MatrixXd splineCoeff_xdot;
// 	Eigen::MatrixXd splineCoeff_xddot;
// }

/**
 * \brief Compute the raceline and speed profile for a closed track
 * (stages 1-5).
 * \param centerline closed centerline with track widths.
 * \throws std::invalid_argument for degenerate input (< 3 points, bad widths).
 * \throws std::runtime_error if a stage fails (infeasible QP, line leaves the
 *         track, invalid output).
 */
Raceline computeRaceline(const Track & centerline, const RacelineParams & params);

/**
 * \brief Stages 2-5 on an already prepared reference line (skips
 * preprocessing). Used by computeRaceline() and for parity tests with an
 * externally prepared reference.
 */
Raceline optimizeRaceline(const Track & reference, const RacelineParams & params);

}  // namespace min_curv_planner
