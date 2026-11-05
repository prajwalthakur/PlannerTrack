/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#pragma once

#include <Eigen/Dense>

/** \file
 * \brief Track and raceline containers shared by every min_curv_planner
 * stage. Conventions:
 * - Closed tracks are stored *without* repeating the first point at the end.
 * - Heading psi = 0 along +x.
 * - Widths are distances from the line to the right/left track bound, along
 *   the right-pointing normal.
 */

namespace min_curv_planner
{

/// Closed reference line with track widths, e.g. the centerline.
struct Track
{
	Eigen::MatrixX2d xy;  ///< N x 2 points [m]
	Eigen::VectorXd wRight;  ///< distance to right bound [m]
	Eigen::VectorXd wLeft;  ///< distance to left bound [m]

	int size() const { return static_cast<int>(xy.rows()); }
};

/// Optimized closed raceline with speed profile, sampled along arc length.
struct Raceline
{
	Eigen::VectorXd s;  ///< arc length from the start point [m]
	Eigen::VectorXd x;  ///< [m]
	Eigen::VectorXd y;  ///< [m]
	Eigen::VectorXd psi;  ///< heading, 0 along +x [rad]
	Eigen::VectorXd kappa;  ///< curvature, positive = left turn [rad/m]
	Eigen::VectorXd vx;  ///< longitudinal speed [m/s]
	Eigen::VectorXd vy; ///< lateral speed [m/s]
	Eigen::VectorXd ax;  ///< longitudinal acceleration [m/s^2]
	Eigen::VectorXd wRight;  ///< distance to right bound [m]
	Eigen::VectorXd wLeft;  ///< distance to left bound [m]

	double length{0.0};  ///< closed-loop length [m]
	double lapTime{0.0};  ///< estimated lap time [s]

	// Diagnostics of the minimum-curvature IQP.
	int iqpIterations{0};
	bool iqpConverged{false};
	double iqpCurvErrorMax{0.0};  ///< linearization error of the last QP [rad/m]

	int size() const { return static_cast<int>(s.size()); }
};

}  // namespace min_curv_planner
