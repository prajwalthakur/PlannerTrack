/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#include "min_curv_planner/min_curvature.hpp"

#include <eigen-quadprog/QuadProg.h>

#include <algorithm>
#include <stdexcept>
#include <string>

namespace min_curv_planner
{

MinCurvLinearization buildMinCurvLinearization(
    const Track & reference, const ClosedSpline & spline, double vehicleWidth)
{
	const int n = reference.size();
	if (spline.size() != n || reference.wRight.size() != n || reference.wLeft.size() != n) {
		throw std::invalid_argument(
		    "buildMinCurvLinearization: reference and spline sizes do not match");
	}
	if (!(vehicleWidth >= 0.0)) {
		throw std::invalid_argument("buildMinCurvLinearization: vehicleWidth must be >= 0");
	}
	MinCurvLinearization lin;

	// Q1: bounds (eq. 3). Right bound at p + w_right n, left bound at
	// p - w_left n; the car's centre keeps half a car width from each.
	lin.alphaMin = -(reference.wLeft.array() - 0.5 * vehicleWidth).matrix();
	lin.alphaMax = (reference.wRight.array() - 0.5 * vehicleWidth).matrix();
	for (int i = 0; i < n; ++i) 
	{
		if (lin.alphaMin(i) > lin.alphaMax(i)) 
		{
			throw std::invalid_argument("buildMinCurvLinearization: track narrower than the vehicle "
			                            "width at point " +
			    std::to_string(i));
		}
	}

	// Q2: freeze x', y' at the reference line (spline parameter t, not time).
	// kappa_i ~ q_y,i * y''_i - q_x,i * x''_i with
	// q_x = y' / den, q_y = x' / den, den = (x'^2 + y'^2)^(3/2)  (eq. 21-22).
	const Eigen::ArrayXd xd = spline.coeffX.col(1).array();
	const Eigen::ArrayXd yd = spline.coeffY.col(1).array();
	const Eigen::ArrayXd den = (xd.square() + yd.square()).pow(1.5);
	const Eigen::VectorXd qx = (yd / den).matrix();
	const Eigen::VectorXd qy = (xd / den).matrix();

	// Q3: insert x'' = S2 x_ref + S2 diag(n_x) alpha (and y) into Q2:
	//   kappaRef = q_y o (S2 y_ref) - q_x o (S2 x_ref)                  (eq. 23)
	//   E        = diag(q_y) S2 diag(n_y) - diag(q_x) S2 diag(n_x)       (eq. 24-25)
	// Element-wise E_ij = S2_ij * (q_y,i n_y,j - q_x,i n_x,j).
	const Eigen::MatrixXd & S2 = spline.secondDerivMap;
	const Eigen::VectorXd nx = spline.normals.col(0);
	const Eigen::VectorXd ny = spline.normals.col(1);
	lin.kappaRef = (qy.array() * (S2 * reference.xy.col(1)).array() -
	    qx.array() * (S2 * reference.xy.col(0)).array())
	                   .matrix();
	lin.E = (S2.array() * (qy * ny.transpose() - qx * nx.transpose()).array()).matrix();
	return lin;
}

//////////////////////////////////////////////////////////////////////////

MinCurvQpResult solveMinCurvQp(const Track & reference, const ClosedSpline & spline,
    const MinCurvLinearization & lin, double kappaBound, Objective objective)
{
	const int n = reference.size();
	if (lin.E.rows() != n || lin.E.cols() != n || spline.size() != n) {
		throw std::invalid_argument("solveMinCurvQp: sizes do not match");
	}
	if (!(kappaBound > 0.0)) {
		throw std::invalid_argument("solveMinCurvQp: kappaBound must be > 0");
	}
	const Eigen::MatrixXd & E = lin.E;
	MinCurvQpResult result;

	// Q4: sum kappa^2 = |kappaRef + E alpha|^2
	//               = alpha^T (E^T E) alpha + 2 kappaRef^T E alpha + const.
	// In the solver's form 1/2 a^T H a + f^T a: H = 2 E^T E, f = 2 E^T kappaRef.
	// TphCompat keeps H = E^T E (paper eq. 19 / tph): linear term 2x too strong.
	Eigen::MatrixXd H = E.transpose() * E;
	if (objective == Objective::MinCurvature) {
		H *= 2.0;
	}
	H = 0.5 * (H + H.transpose());  // exact symmetry for the Cholesky factorization
	const Eigen::VectorXd f = 2.0 * E.transpose() * lin.kappaRef;

	// Q5: G alpha <= h, four blocks of n rows:
	//   alpha <= alphaMax, -alpha <= -alphaMin,
	//   E alpha <= kappaBound - kappaRef, -E alpha <= kappaBound + kappaRef.
	Eigen::MatrixXd G(4 * n, n);
	G << Eigen::MatrixXd::Identity(n, n), -Eigen::MatrixXd::Identity(n, n), E, -E;
	Eigen::VectorXd h(4 * n);
	h << lin.alphaMax, -lin.alphaMin, (kappaBound - lin.kappaRef.array()).matrix(),
	    (kappaBound + lin.kappaRef.array()).matrix();

	// Q6: Goldfarb-Idnani (same algorithm as Python quadprog). It factors H
	// with Cholesky, so H must be strictly positive definite; E^T E is only
	// guaranteed semi-definite. On failure code 2 retry with a tiny diagonal.
	const Eigen::MatrixXd Aeq(0, n);
	const Eigen::VectorXd beq(0);
	Eigen::QuadProgDense qp(n, 0, 4 * n);
	bool solved = qp.solve(H, f, Aeq, beq, G, h);
	if (!solved && qp.fail() == 2) {
		const double eps = 1e-10 * std::max(H.diagonal().mean(), 1e-12);
		H.diagonal().array() += eps;
		solved = qp.solve(H, f, Aeq, beq, G, h);
		result.regularized = true;
	}
	if (!solved) {
		throw std::runtime_error(qp.fail() == 1
		        ? "solveMinCurvQp: QP infeasible (track bounds and curvature limit cannot "
		          "both be met)"
		        : "solveMinCurvQp: QP solver failed, code " + std::to_string(qp.fail()));
	}
	result.alpha = qp.result();

	// Q7: linearization error. kappa_assumed uses the frozen x', y' (what the
	// QP assumed); kappa_actual uses x', y' of the moved points via S1.
	const Eigen::VectorXd xs =
	    reference.xy.col(0) + spline.normals.col(0).cwiseProduct(result.alpha);
	const Eigen::VectorXd ys =
	    reference.xy.col(1) + spline.normals.col(1).cwiseProduct(result.alpha);
	const Eigen::ArrayXd xd = (spline.firstDerivMap * xs).array();
	const Eigen::ArrayXd yd = (spline.firstDerivMap * ys).array();
	const Eigen::ArrayXd xdd = (spline.secondDerivMap * xs).array();
	const Eigen::ArrayXd ydd = (spline.secondDerivMap * ys).array();
	const Eigen::ArrayXd kappaActual = (xd * ydd - yd * xdd) / (xd.square() + yd.square()).pow(1.5);
	const Eigen::ArrayXd kappaAssumed = (lin.kappaRef + E * result.alpha).array();
	result.curvErrorMax = (kappaActual - kappaAssumed).abs().maxCoeff();
	return result;
}

//////////////////////////////////////////////////////////////////////////

MinCurvIqpResult runMinCurvIqp(const Track & reference, const RacelineParams & params)
{
	const OptimizationParams & opt = params.optimization;
	const double vehicleWidth = params.optimizationWidth();
	const double kappaBound = params.curvatureLimit();

	MinCurvIqpResult out;
	out.reference = reference;
	// First spline with distance scaling (tph prep_track); inside the loop
	// the points are equally spaced and tph turns it off.
	out.spline = buildClosedSpline(reference.xy, true);

	for (int iter = 1;; ++iter) {
		const MinCurvLinearization lin =
		    buildMinCurvLinearization(out.reference, out.spline, vehicleWidth);
		const MinCurvQpResult qp =
		    solveMinCurvQp(out.reference, out.spline, lin, kappaBound, opt.objective);
		out.alpha = qp.alpha;
		out.curvErrorMax = qp.curvErrorMax;
		out.iterations = iter;

		// Damp the first steps while the linearization is still poor.
		if (iter < opt.iqpItersMin) {
			out.alpha *= static_cast<double>(iter) / opt.iqpItersMin;
		}
		if (iter >= opt.iqpItersMin && qp.curvErrorMax <= opt.iqpCurvErrorAllowed) {
			out.converged = true;
			break;
		}
		if (iter >= opt.iqpItersMax) {
			break;
		}

		// New reference line: the moved points, splined and resampled.
		Eigen::MatrixX2d moved = out.reference.xy;
		moved.col(0) += out.spline.normals.col(0).cwiseProduct(out.alpha);
		moved.col(1) += out.spline.normals.col(1).cwiseProduct(out.alpha);
		const ClosedSpline movedSpline = buildClosedSpline(moved, false);
		const SplineSamples samples =
		    sampleClosedSpline(movedSpline, params.preprocessing.stepsizeReg);
		// Distances to the bounds measured from the moved points.
		const Eigen::VectorXd wRight = out.reference.wRight - out.alpha;
		const Eigen::VectorXd wLeft = out.reference.wLeft + out.alpha;

		Track next;
		next.xy = samples.xy;
		next.wRight = interpolateAtSamples(wRight, samples);
		next.wLeft = interpolateAtSamples(wLeft, samples);
		out.reference = next;
		out.spline = buildClosedSpline(out.reference.xy, false);
	}
	return out;
}

}  // namespace min_curv_planner
