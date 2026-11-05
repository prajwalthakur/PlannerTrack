/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#pragma once

#include "min_curv_planner/closed_spline.hpp"
#include "min_curv_planner/params.hpp"
#include "min_curv_planner/types.hpp"

#include <Eigen/Dense>

/** \file
 * \brief Minimum-curvature QP (paper eq. 2-3, 8-26). Derivation:
 * project_docs/learning_based_controls/min_curv_qp.md.
 *
 * Each reference point moves along its right-pointing normal by alpha_i [m].
 * With the first derivatives frozen at the reference line, the curvature is
 * linear in alpha:  kappa ~ kappaRef + E * alpha.
 *
 * Steps:
 *  Q1. bounds on alpha from track and vehicle width (eq. 3)           
 *  Q2. frozen first derivatives -> weights q_x, q_y (eq. 21-22)        
 *  Q3. kappaRef and E (eq. 23-25)                                      
 *  Q4. objective H, f (eq. 18-19, corrected)                           
 *  Q5. constraints G alpha <= h (bounds + curvature limit, eq. 20, 26) 
 *  Q6. solve with eigen-quadprog                                       
 *  Q7. linearization error for the IQP loop (p. 17)                    
 *  IQP loop around Q1-Q7 (p. 17, tph iqp_handler)                     
 */

namespace min_curv_planner
{

/// The linearized problem around one reference line (Q1-Q3).
struct MinCurvLinearization
{
	/// Q1: alpha_min,i <= alpha_i <= alpha_max,i [m], positive = right.
	Eigen::VectorXd alphaMin;
	Eigen::VectorXd alphaMax;
	/// Q3: curvature of the reference line (alpha = 0) [rad/m]. Equals
	/// ClosedSpline::kappa: the frozen derivatives are the reference's own.
	Eigen::VectorXd kappaRef;
	/// Q3: E (N x N) [1/m^2]: column j = change of curvature at every point
	/// when point j moves 1 m along its normal. kappa ~ kappaRef + E * alpha.
	Eigen::MatrixXd E;
};

/**
 * \brief Q1-Q3: bounds, reference curvature and curvature sensitivity E.
 * \param reference the reference line; reference.xy must be the points the
 *        spline was built from.
 * \param spline buildClosedSpline(reference.xy, ...).
 * \param vehicleWidth width the line must keep clear, incl. safety margin [m].
 * \throws std::invalid_argument on size mismatch or if the track is narrower
 *         than vehicleWidth at some point.
 */
MinCurvLinearization buildMinCurvLinearization(
    const Track & reference, const ClosedSpline & spline, double vehicleWidth);

/// Result of one QP solve (Q4-Q7).
struct MinCurvQpResult
{
	/// Offsets along the normals [m], positive = right.
	Eigen::VectorXd alpha;
	/// Q7: max |kappa_actual - kappa_assumed| at the moved points, where
	/// kappa_assumed = kappaRef + E alpha (frozen x', y') and kappa_actual
	/// uses x', y' of the moved points [rad/m]. IQP stopping criterion.
	double curvErrorMax{0.0};
	/// True if H needed a small diagonal term to be positive definite.
	bool regularized{false};
};

/**
 * \brief Q4-Q7: solve the QP around one reference line.
 * \param reference, spline the reference line and its spline (as for
 *        buildMinCurvLinearization).
 * \param lin buildMinCurvLinearization(reference, spline, ...).
 * \param kappaBound curvature limit [rad/m], |kappaRef + E alpha| <= kappaBound.
 * \param objective MinCurvature (H = 2 E^T E, f = 2 E^T kappaRef) or TphCompat
 *        (H = E^T E, f = 2 E^T kappaRef: paper eq. 19 / tph, factor-2 error).
 * \throws std::runtime_error if the QP is infeasible or cannot be solved.
 */
MinCurvQpResult solveMinCurvQp(const Track & reference, const ClosedSpline & spline,
    const MinCurvLinearization & lin, double kappaBound, Objective objective);

/// Result of the iterated QP (IQP).
struct MinCurvIqpResult
{
	/// Reference line the final alpha refers to (moves every iteration).
	Track reference;
	/// Its spline; the raceline is reference.xy + normals * alpha.
	ClosedSpline spline;
	Eigen::VectorXd alpha;
	int iterations{0};
	/// Q7 error of the last QP [rad/m].
	double curvErrorMax{0.0};
	/// False if iqpItersMax was reached before the error tolerance.
	bool converged{false};
};

/**
 * \brief Iterated QP (paper p. 17, tph iqp_handler).
 *
 * Solve the QP; in iterations 1 .. itersMin-1 scale alpha by
 * iteration / itersMin; stop once iteration >= itersMin and the Q7 error is
 * <= iqpCurvErrorAllowed. Otherwise the moved points become the new reference
 * line: spline through them (no distance scaling), resampled at
 * stepsizeReg, widths shifted by alpha and interpolated, and the QP is solved
 * again. iqpItersMax caps the loop (tph has no cap).
 *
 * \param reference reference line (e.g. the preprocessed centerline).
 */
MinCurvIqpResult runMinCurvIqp(const Track & reference, const RacelineParams & params);

}  // namespace min_curv_planner
