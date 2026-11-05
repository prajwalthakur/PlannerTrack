/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#pragma once

#include "min_curv_planner/params.hpp"
#include "min_curv_planner/types.hpp"

#include <Eigen/Dense>

/** \file
 * \brief Reference-line preparation (paper §3.4.2, Fig. 6; tph prep_track /
 * spline_approximation). A noisy centerline makes the QP produce a noisy
 * line, so it is cleaned first:
 *  (a) linear resampling to stepsizePrep,
 *  (b) smoothing,
 *  (c) width correction for the sideways displacement of the smoothing,
 *  (d) spline resampling to the optimization step stepsizeReg.
 *
 * (b) differs from tph: tph uses a FITPACK smoothing B-spline (scipy splprep)
 * with smoothing factor s_reg = sum of squared residuals. Here a closed
 * Whittaker smoother (penalized second differences, the discrete smoothing
 * spline) is used, with its weight chosen by bisection so that the sum of
 * squared residuals equals the same s_reg. Same meaning of the parameter,
 * not bit-identical output.
 */

namespace min_curv_planner
{

/// (a) Linear resampling of a closed track at ~stepsize (tph interp_track).
Track interpolateTrackLinear(const Track & track, double stepsize);

/**
 * \brief (b) Closed Whittaker smoother: z = argmin |z - xy|^2 + lambda |D z|^2,
 * D = cyclic second difference, lambda chosen so that |z - xy|_F^2 equals
 * residualTarget (within 0.1 %). residualTarget <= 0 returns xy unchanged.
 */
Eigen::MatrixX2d smoothClosedCurve(const Eigen::MatrixX2d & xy, double residualTarget);

/**
 * \brief Full preparation (a)-(d). With enableSmoothing = false the
 * centerline is returned unchanged (use when it is already smooth and evenly
 * spaced, or to feed an externally prepared reference line).
 * \throws std::runtime_error if smoothing moves the line outside the track.
 */
Track prepareReferenceTrack(const Track & centerline, const PreprocessingParams & params);

}  // namespace min_curv_planner
