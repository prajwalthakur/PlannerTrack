/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#pragma once

#include <Eigen/Dense>

/** \file
 * \brief Closed C2 cubic spline through a loop of points (paper eq. 4-7,
 * TUM tph calc_splines).
 *
 * Segment i joins point i to point (i+1) mod N with a local parameter
 * t in [0, 1]:  x_i(t) = a_i + b_i t + c_i t^2 + d_i t^3  (same for y).
 *
 * Steps :
 *  1. segment lengths l_i                                        
 *  2. scaling factors sigma_i = l_i / l_{(i+1) mod N} (for circular track)          
 *  3. assemble the 4N x 4N system A z = b                        
 *  4. solve for the coefficients (x and y share A)               
 *  5. values, normals, heading, curvature at the points          
 *  6. derivative maps S1, S2 for the QP (eq. 16-17)              
 */

namespace min_curv_planner
{

struct ClosedSpline
{
	/// l_i: chord length of segment i, point i -> point (i+1) mod N [m].
	Eigen::VectorXd segLen;
	/// sigma_i: joint scaling between segment i and segment (i+1) mod N.
	/// x_i'(1) = sigma_i x_{i+1}'(0) and x_i''(1) = sigma_i^2 x_{i+1}''(0),
	/// which makes the curve smooth in arc length. All ones without
	/// distance scaling (smooth in t instead).
	Eigen::VectorXd sigma;
	/// Row i: (a_i, b_i, c_i, d_i) of segment i, local t in [0, 1].
	Eigen::MatrixX4d coeffX;
	Eigen::MatrixX4d coeffY;

	/// Unit normal at point i, pointing to the RIGHT of the direction of
	/// travel: (y', -x') / |(x', y')|. Raceline point = p_i + alpha_i n_i (eq. 2).
	Eigen::MatrixX2d normals;
	/// Heading at point i, psi = 0 along +x, in (-pi, pi] [rad].
	Eigen::VectorXd psi;
	/// Curvature at point i (eq. 9), positive = left turn [rad/m].
	Eigen::VectorXd kappa;

	/// S1 (N x N): x' at the points = S1 * x, y' = S1 * y.
	/// Column j = change of x' at every point when point j moves by 1.
	/// Used for the IQP stopping test (x' of the moved points).
	Eigen::MatrixXd firstDerivMap;
	/// S2 (N x N): x'' at the points = S2 * x, y'' = S2 * y. The paper's T_c
	/// (eq. 16-17): x'' = S2 x_ref + S2 diag(n_x) alpha. Exact for any point
	/// set with the same spacing (sigma); see
	/// project_docs/learning_based_controls/min_curv_derivative_maps.md.
	Eigen::MatrixXd secondDerivMap;

	int size() const { return static_cast<int>(segLen.size()); }
};

/**
 * \brief Build the closed spline through xy (N x 2, first point not repeated).
 * \param distScaling scale derivative continuity by the segment-length ratio
 *        (tph use_dist_scaling). tph uses true when preparing the reference
 *        line and false inside the IQP loop.
 * \throws std::invalid_argument for fewer than 3 points or a zero-length segment.
 * \throws std::runtime_error if the system is singular or a tangent at a point is ~0.
 */
ClosedSpline buildClosedSpline(const Eigen::MatrixX2d & xy, bool distScaling);

/**
 * \brief Arc length of each segment, measured along a polyline through
 * samplesPerSegment points of the cubic (tph calc_spline_lengths; 15 points
 * as in tph, accurate to ~1e-4 relative for our segment lengths).
 */
Eigen::VectorXd computeSegmentArcLengths(const ClosedSpline & spline, int samplesPerSegment = 15);

/// Points sampled along a closed spline at (approximately) equal arc length.
struct SplineSamples
{
	Eigen::MatrixX2d xy;  ///< sampled points (closing point not repeated)
	Eigen::VectorXi segment;  ///< segment index of each sample
	Eigen::VectorXd t;  ///< local parameter of each sample in its segment, [0, 1)
	Eigen::VectorXd s;  ///< arc length of each sample from point 0 [m]
	/// Distance from sample k to sample k+1; the last entry closes the loop [m].
	Eigen::VectorXd elLengths;
	double length{0.0};  ///< total arc length of the closed spline [m]

	int size() const { return static_cast<int>(xy.rows()); }
};

/**
 * \brief Resample the closed spline at about `stepsize` (tph interp_splines
 * with stepsize_approx, incl_last_point = False). Uses ceil(L / stepsize)
 * equal arc-length intervals; within a segment t is taken proportional to
 * arc length (tph's approximation).
 */
SplineSamples sampleClosedSpline(const ClosedSpline & spline, double stepsize);

/**
 * \brief Heading (psi = 0 along +x) and curvature at the samples, from the
 * analytic derivatives of their segments (tph calc_head_curv_an).
 */
void computeHeadingCurvature(
    const ClosedSpline & spline, const SplineSamples & samples, Eigen::VectorXd & psi,
    Eigen::VectorXd & kappa);

/**
 * \brief Linear interpolation of per-point values (e.g. track widths) at the
 * samples, along each segment's t (tph interp_track_widths).
 */
Eigen::VectorXd interpolateAtSamples(
    const Eigen::VectorXd & valuesAtPoints, const SplineSamples & samples);

}  // namespace min_curv_planner
