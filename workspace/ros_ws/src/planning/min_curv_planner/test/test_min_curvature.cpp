/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#include "min_curv_planner/min_curvature.hpp"

#include "tph_calc_splines_reference.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

using min_curv_planner::buildClosedSpline;
using min_curv_planner::buildMinCurvLinearization;
using min_curv_planner::ClosedSpline;
using min_curv_planner::MinCurvLinearization;
using min_curv_planner::Track;

namespace
{
// Irregular, unevenly spaced loop shared with the tph reference data,
// with widths that differ per point and side.
Track irregularTrack()
{
	const int n = tph_reference::kRefN;
	Track track;
	track.xy.resize(n, 2);
	track.wRight.resize(n);
	track.wLeft.resize(n);
	for (int i = 0; i < n; ++i) {
		track.xy(i, 0) = tph_reference::kRefXY[i][0];
		track.xy(i, 1) = tph_reference::kRefXY[i][1];
		track.wRight(i) = 1.0 + 0.1 * i;
		track.wLeft(i) = 0.8;
	}
	return track;
}
}  // namespace

// Q1: bounds follow eq. 3.
TEST(MinCurvLinearization, BoundsFromWidthsAndVehicle)
{
	const Track track = irregularTrack();
	const double width = 0.5;
	const MinCurvLinearization lin =
	    buildMinCurvLinearization(track, buildClosedSpline(track.xy, true), width);
	for (int i = 0; i < track.size(); ++i) {
		EXPECT_DOUBLE_EQ(lin.alphaMin(i), -(0.8 - 0.25)) << "i=" << i;
		EXPECT_DOUBLE_EQ(lin.alphaMax(i), 1.0 + 0.1 * i - 0.25) << "i=" << i;
	}
}

TEST(MinCurvLinearization, RejectsTrackNarrowerThanVehicle)
{
	Track track = irregularTrack();
	track.wRight(3) = 0.1;
	track.wLeft(3) = 0.1;  // 0.2 m wide at point 3, vehicle 0.5 m
	EXPECT_THROW(buildMinCurvLinearization(track, buildClosedSpline(track.xy, true), 0.5),
	    std::invalid_argument);
}

// Q2/Q3 consistency: the frozen derivatives are the reference's own, so
// kappaRef is exactly the spline curvature of step 5.
TEST(MinCurvLinearization, KappaRefEqualsSplineCurvature)
{
	const Track track = irregularTrack();
	for (const bool distScaling : {true, false}) {
		const ClosedSpline spline = buildClosedSpline(track.xy, distScaling);
		const MinCurvLinearization lin = buildMinCurvLinearization(track, spline, 0.5);
		EXPECT_LT((lin.kappaRef - spline.kappa).cwiseAbs().maxCoeff(), 1e-10) << distScaling;
	}
}

// Q3: kappaRef + E alpha is the curvature of the moved points with x', y'
// frozen at the reference line. Checked against an independent computation:
// rebuild the spline through the moved points (exact x'', y'' without
// distance scaling) and apply the frozen-derivative formula directly.
TEST(MinCurvLinearization, MatchesFrozenDerivativeCurvature)
{
	const Track track = irregularTrack();
	const int n = track.size();
	const ClosedSpline ref = buildClosedSpline(track.xy, false);
	const MinCurvLinearization lin = buildMinCurvLinearization(track, ref, 0.5);

	Eigen::VectorXd alpha(n);
	for (int i = 0; i < n; ++i) {
		alpha(i) = 0.3 * std::sin(1.7 * i + 0.4);
	}
	Eigen::MatrixX2d moved = track.xy;
	moved.col(0) += ref.normals.col(0).cwiseProduct(alpha);
	moved.col(1) += ref.normals.col(1).cwiseProduct(alpha);
	const ClosedSpline rebuilt = buildClosedSpline(moved, false);

	const Eigen::ArrayXd xd = ref.coeffX.col(1).array();  // frozen
	const Eigen::ArrayXd yd = ref.coeffY.col(1).array();
	const Eigen::ArrayXd xdd = 2.0 * rebuilt.coeffX.col(2).array();  // moved points
	const Eigen::ArrayXd ydd = 2.0 * rebuilt.coeffY.col(2).array();
	const Eigen::VectorXd kappaFrozen =
	    ((xd * ydd - yd * xdd) / (xd.square() + yd.square()).pow(1.5)).matrix();

	EXPECT_LT((lin.kappaRef + lin.E * alpha - kappaFrozen).cwiseAbs().maxCoeff(), 1e-10);
}

// Q3, the sign check from min_curv_qp.md 3.3: E_ii = -S2_ii / |t_i|^2 > 0,
// i.e. pushing a point to the right makes the path turn left there.
TEST(MinCurvLinearization, DiagonalSignPushRightTurnsLeft)
{
	const Track track = irregularTrack();
	const ClosedSpline spline = buildClosedSpline(track.xy, true);
	const MinCurvLinearization lin = buildMinCurvLinearization(track, spline, 0.5);
	for (int i = 0; i < track.size(); ++i) {
		const double t2 = spline.coeffX(i, 1) * spline.coeffX(i, 1) +
		    spline.coeffY(i, 1) * spline.coeffY(i, 1);
		EXPECT_NEAR(lin.E(i, i), -spline.secondDerivMap(i, i) / t2, 1e-12) << "i=" << i;
		EXPECT_GT(lin.E(i, i), 0.0) << "i=" << i;
	}
}

// Parity with the matrices of tph 0.76 opt_min_curv (E_kappa, k_kappa_ref).
TEST(MinCurvLinearization, MatchesTphOptMinCurvMatrices)
{
	const Track track = irregularTrack();
	const MinCurvLinearization lin =
	    buildMinCurvLinearization(track, buildClosedSpline(track.xy, true), 0.5);
	const int n = tph_reference::kRefN;
	for (int i = 0; i < n; ++i) {
		EXPECT_NEAR(lin.kappaRef(i), tph_reference::kRefKappaRefScaled[i], 1e-10) << "i=" << i;
		for (int j = 0; j < n; ++j) {
			EXPECT_NEAR(lin.E(i, j), tph_reference::kRefEScaled[i][j], 1e-10)
			    << "i=" << i << " j=" << j;
		}
	}
}

namespace
{
// Counter-clockwise ellipse with a wave: curvature varies, so the QP has work
// to do. Constant widths.
Track wavyLoop(int n, double width)
{
	Track track;
	track.xy.resize(n, 2);
	for (int i = 0; i < n; ++i) {
		const double t = 2.0 * M_PI * i / n;
		track.xy(i, 0) = 20.0 * std::cos(t) + 1.5 * std::cos(5.0 * t);
		track.xy(i, 1) = 10.0 * std::sin(t) + 1.5 * std::sin(5.0 * t);
	}
	track.wRight = Eigen::VectorXd::Constant(n, width);
	track.wLeft = Eigen::VectorXd::Constant(n, width);
	return track;
}

min_curv_planner::MinCurvQpResult solve(const Track & track, double vehWidth, double kappaBound,
    min_curv_planner::Objective objective, MinCurvLinearization * linOut = nullptr)
{
	const ClosedSpline spline = buildClosedSpline(track.xy, true);
	const MinCurvLinearization lin = buildMinCurvLinearization(track, spline, vehWidth);
	if (linOut != nullptr) {
		*linOut = lin;
	}
	return min_curv_planner::solveMinCurvQp(track, spline, lin, kappaBound, objective);
}
}  // namespace

// Q4/Q6, the factor-2 check from min_curv_qp.md 4.3. Very wide track, no
// active constraints: the corrected objective removes the linearized
// curvature, the tph objective mirrors it (kappaRef + E alpha = -kappaRef).
TEST(MinCurvQp, CorrectObjectiveRemovesCurvatureTphMirrorsIt)
{
	const Track track = wavyLoop(120, 50.0);
	MinCurvLinearization lin;
	const auto correct =
	    solve(track, 0.0, 100.0, min_curv_planner::Objective::MinCurvature, &lin);
	const auto tph = solve(track, 0.0, 100.0, min_curv_planner::Objective::TphCompat);

	const double refNorm = lin.kappaRef.norm();
	const double correctResidual = (lin.kappaRef + lin.E * correct.alpha).norm();
	const Eigen::VectorXd tphKappa = lin.kappaRef + lin.E * tph.alpha;
	EXPECT_LT(correctResidual, 1e-6 * refNorm);
	EXPECT_LT((tphKappa + lin.kappaRef).norm(), 1e-6 * refNorm);  // = -kappaRef
	EXPECT_NEAR(tph.alpha.norm() / correct.alpha.norm(), 2.0, 1e-6);
}

// Q5/Q6: the solution respects the track bounds, and some are active on a
// narrow track (otherwise the test would prove nothing).
TEST(MinCurvQp, RespectsTrackBounds)
{
	const Track track = wavyLoop(120, 1.0);
	MinCurvLinearization lin;
	const auto qp = solve(track, 0.5, 100.0, min_curv_planner::Objective::MinCurvature, &lin);
	int active = 0;
	for (int i = 0; i < track.size(); ++i) {
		EXPECT_GE(qp.alpha(i), lin.alphaMin(i) - 1e-9) << "i=" << i;
		EXPECT_LE(qp.alpha(i), lin.alphaMax(i) + 1e-9) << "i=" << i;
		active += (qp.alpha(i) > lin.alphaMax(i) - 1e-6 || qp.alpha(i) < lin.alphaMin(i) + 1e-6);
	}
	EXPECT_GT(active, 0);
}

// Q5/Q6: with a curvature limit below the track's curvature, the linearized
// curvature of the solution stays within it.
TEST(MinCurvQp, EnforcesCurvatureLimit)
{
	const Track track = wavyLoop(120, 3.0);
	MinCurvLinearization lin;
	const auto free = solve(track, 0.5, 100.0, min_curv_planner::Objective::MinCurvature, &lin);
	const double freeMax = (lin.kappaRef + lin.E * free.alpha).cwiseAbs().maxCoeff();
	const double bound = 0.8 * freeMax;  // tighter than the unconstrained optimum
	const auto limited = solve(track, 0.5, bound, min_curv_planner::Objective::MinCurvature);
	const double limitedMax = (lin.kappaRef + lin.E * limited.alpha).cwiseAbs().maxCoeff();
	EXPECT_LE(limitedMax, bound + 1e-9);
	EXPECT_GT(freeMax, bound);
}

// Q7: the linearization error is zero for alpha = 0 and grows with the step.
TEST(MinCurvQp, LinearizationErrorIsReportedAndPositive)
{
	const Track track = wavyLoop(120, 3.0);
	const auto qp = solve(track, 0.5, 100.0, min_curv_planner::Objective::MinCurvature);
	EXPECT_TRUE(std::isfinite(qp.curvErrorMax));
	EXPECT_GT(qp.curvErrorMax, 0.0);
	EXPECT_LT(qp.curvErrorMax, 1.0);
}
