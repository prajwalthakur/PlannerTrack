/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#include "min_curv_planner/closed_spline.hpp"

#include "tph_calc_splines_reference.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

using min_curv_planner::buildClosedSpline;
using min_curv_planner::ClosedSpline;

namespace
{
// 3-4-5 triangle: segments 0->1 = 3, 1->2 = 4, closing 2->0 = 5.
Eigen::MatrixX2d triangle()
{
	Eigen::MatrixX2d xy(3, 2);
	xy << 0.0, 0.0, 3.0, 0.0, 3.0, 4.0;
	return xy;
}

// Irregular, unevenly spaced loop shared with the tph reference data.
Eigen::MatrixX2d irregularLoop()
{
	Eigen::MatrixX2d xy(tph_reference::kRefN, 2);
	for (int i = 0; i < tph_reference::kRefN; ++i) {
		xy(i, 0) = tph_reference::kRefXY[i][0];
		xy(i, 1) = tph_reference::kRefXY[i][1];
	}
	return xy;
}

// Circle of radius r, n evenly spaced points, counter-clockwise by default.
// Point i is at angle +-2*pi*i/n.
Eigen::MatrixX2d circle(double r, int n, bool clockwise = false)
{
	Eigen::MatrixX2d xy(n, 2);
	for (int i = 0; i < n; ++i) {
		const double a = (clockwise ? -1.0 : 1.0) * 2.0 * M_PI * i / n;
		xy(i, 0) = r * std::cos(a);
		xy(i, 1) = r * std::sin(a);
	}
	return xy;
}

Eigen::MatrixX4d toMatrix(const double (&ref)[tph_reference::kRefN][4])
{
	Eigen::MatrixX4d m(tph_reference::kRefN, 4);
	for (int i = 0; i < tph_reference::kRefN; ++i) {
		for (int k = 0; k < 4; ++k) {
			m(i, k) = ref[i][k];
		}
	}
	return m;
}

Eigen::MatrixX2d toMatrix2(const double (&ref)[tph_reference::kRefN][2])
{
	Eigen::MatrixX2d m(tph_reference::kRefN, 2);
	for (int i = 0; i < tph_reference::kRefN; ++i) {
		m(i, 0) = ref[i][0];
		m(i, 1) = ref[i][1];
	}
	return m;
}

double wrapAngle(double a)
{
	return std::atan2(std::sin(a), std::cos(a));
}
}  // namespace

TEST(ClosedSpline, SegmentLengthsIncludeClosingSegment)
{
	const auto spline = buildClosedSpline(triangle(), true);
	ASSERT_EQ(spline.size(), 3);
	EXPECT_DOUBLE_EQ(spline.segLen(0), 3.0);
	EXPECT_DOUBLE_EQ(spline.segLen(1), 4.0);
	EXPECT_DOUBLE_EQ(spline.segLen(2), 5.0);
}

TEST(ClosedSpline, SigmaIsLengthRatioWithWrap)
{
	const auto spline = buildClosedSpline(triangle(), true);
	EXPECT_DOUBLE_EQ(spline.sigma(0), 3.0 / 4.0);
	EXPECT_DOUBLE_EQ(spline.sigma(1), 4.0 / 5.0);
	EXPECT_DOUBLE_EQ(spline.sigma(2), 5.0 / 3.0);  // last joint wraps to segment 0
}

TEST(ClosedSpline, SigmaIsOneWithoutDistScaling)
{
	const auto spline = buildClosedSpline(triangle(), false);
	EXPECT_TRUE(spline.sigma.isOnes());
}

TEST(ClosedSpline, RejectsDegenerateInput)
{
	Eigen::MatrixX2d two(2, 2);
	two << 0.0, 0.0, 1.0, 0.0;
	EXPECT_THROW(buildClosedSpline(two, true), std::invalid_argument);

	Eigen::MatrixX2d duplicate(3, 2);
	duplicate << 0.0, 0.0, 0.0, 0.0, 1.0, 1.0;
	EXPECT_THROW(buildClosedSpline(duplicate, true), std::invalid_argument);
}

// Step 4 checks: position rows and the row-major Map order.
TEST(ClosedSpline, PassesThroughEveryPoint)
{
	const Eigen::MatrixX2d xy = irregularLoop();
	const int n = static_cast<int>(xy.rows());
	for (const bool distScaling : {true, false}) {
		const ClosedSpline s = buildClosedSpline(xy, distScaling);
		ASSERT_EQ(s.coeffX.rows(), n);
		for (int i = 0; i < n; ++i) {
			const int j = (i + 1) % n;
			// t = 0: a_i is point i;  t = 1: a+b+c+d is point i+1 (wraps)
			EXPECT_NEAR(s.coeffX(i, 0), xy(i, 0), 1e-12) << "scaling=" << distScaling << " i=" << i;
			EXPECT_NEAR(s.coeffY(i, 0), xy(i, 1), 1e-12) << "scaling=" << distScaling << " i=" << i;
			EXPECT_NEAR(s.coeffX.row(i).sum(), xy(j, 0), 1e-12) << "scaling=" << distScaling << " i=" << i;
			EXPECT_NEAR(s.coeffY.row(i).sum(), xy(j, 1), 1e-12) << "scaling=" << distScaling << " i=" << i;
		}
	}
}

// Step 3/4 checks: slope and curvature joints, including the wrap N-1 -> 0.
TEST(ClosedSpline, JointsAreSmoothIncludingWrap)
{
	const Eigen::MatrixX2d xy = irregularLoop();
	const int n = static_cast<int>(xy.rows());
	for (const bool distScaling : {true, false}) {
		const ClosedSpline s = buildClosedSpline(xy, distScaling);
		for (const Eigen::MatrixX4d * c : {&s.coeffX, &s.coeffY}) {
			for (int i = 0; i < n; ++i) {
				const int j = (i + 1) % n;
				const double sig = s.sigma(i);
				// x_i'(1) = sigma_i x_j'(0)
				const double slopeEnd = (*c)(i, 1) + 2.0 * (*c)(i, 2) + 3.0 * (*c)(i, 3);
				EXPECT_NEAR(slopeEnd, sig * (*c)(j, 1), 1e-10)
				    << "scaling=" << distScaling << " joint " << i << "->" << j;
				// x_i''(1) = sigma_i^2 x_j''(0)
				const double accEnd = 2.0 * (*c)(i, 2) + 6.0 * (*c)(i, 3);
				EXPECT_NEAR(accEnd, sig * sig * 2.0 * (*c)(j, 2), 1e-10)
				    << "scaling=" << distScaling << " joint " << i << "->" << j;
			}
		}
	}
}

// Geometry end to end on a counter-clockwise circle (kappa = +1/R):
// - every knot sees the same curvature, including the start/finish point
//   (an open natural spline would force 0 there) -- the closed-loop property;
// - the cubic-spline error is second order: about h^2 / (12 R^2) relative,
//   so halving the spacing cuts it by ~4.
TEST(ClosedSpline, CircleCurvatureIsUniformAndConverges)
{
	const double r = 5.0;
	for (const bool distScaling : {true, false}) {
		double maxErr[2] = {0.0, 0.0};
		const int sizes[2] = {40, 80};
		for (int k = 0; k < 2; ++k) {
			const int n = sizes[k];
			const ClosedSpline s = buildClosedSpline(circle(r, n), distScaling);
			const double kappa0 = s.kappa(0);
			for (int i = 0; i < n; ++i) {
				const double kappa = s.kappa(i);
				EXPECT_NEAR(kappa, kappa0, 1e-10) << "scaling=" << distScaling << " n=" << n << " i=" << i;
				maxErr[k] = std::max(maxErr[k], std::abs(kappa - 1.0 / r));
			}
			const double h = 2.0 * r * std::sin(M_PI / n);  // chord length
			EXPECT_LT(maxErr[k], 1.5 * h * h / (12.0 * r * r) / r) << "scaling=" << distScaling << " n=" << n;
		}
		EXPECT_GT(maxErr[0] / maxErr[1], 3.5) << "scaling=" << distScaling;  // ~4 for second order
	}
}

// Parity with TUM trajectory_planning_helpers 0.76 calc_splines.
TEST(ClosedSpline, MatchesTphCalcSplines)
{
	const Eigen::MatrixX2d xy = irregularLoop();

	const ClosedSpline scaled = buildClosedSpline(xy, true);
	EXPECT_LT((scaled.coeffX - toMatrix(tph_reference::kRefCoeffXScaled)).cwiseAbs().maxCoeff(), 1e-10);
	EXPECT_LT((scaled.coeffY - toMatrix(tph_reference::kRefCoeffYScaled)).cwiseAbs().maxCoeff(), 1e-10);

	const ClosedSpline uniform = buildClosedSpline(xy, false);
	EXPECT_LT(
	    (uniform.coeffX - toMatrix(tph_reference::kRefCoeffXUniform)).cwiseAbs().maxCoeff(), 1e-10);
	EXPECT_LT(
	    (uniform.coeffY - toMatrix(tph_reference::kRefCoeffYUniform)).cwiseAbs().maxCoeff(), 1e-10);
}

// Step 5: normals have unit length and point to the RIGHT of travel:
// outward on a counter-clockwise circle, inward on a clockwise one.
TEST(ClosedSpline, NormalsPointRightOfTravel)
{
	const double r = 5.0;
	const int n = 40;
	for (const bool clockwise : {false, true}) {
		const Eigen::MatrixX2d xy = circle(r, n, clockwise);
		const ClosedSpline s = buildClosedSpline(xy, true);
		const double outward = clockwise ? -1.0 : 1.0;
		for (int i = 0; i < n; ++i) {
			EXPECT_NEAR(s.normals.row(i).norm(), 1.0, 1e-12) << "cw=" << clockwise << " i=" << i;
			const Eigen::RowVector2d expected = outward * xy.row(i) / r;
			EXPECT_LT((s.normals.row(i) - expected).norm(), 1e-9) << "cw=" << clockwise << " i=" << i;
		}
	}
}

// Step 5: heading follows the tangent (psi = 0 along +x) and curvature has
// the left-turn-positive sign.
TEST(ClosedSpline, HeadingAndCurvatureSigns)
{
	const double r = 5.0;
	const int n = 40;
	for (const bool clockwise : {false, true}) {
		const ClosedSpline s = buildClosedSpline(circle(r, n, clockwise), true);
		const double dir = clockwise ? -1.0 : 1.0;
		for (int i = 0; i < n; ++i) {
			// tangent of a circle at angle a: a + pi/2 (ccw) or a - pi/2 (cw)
			const double a = dir * 2.0 * M_PI * i / n;
			EXPECT_NEAR(wrapAngle(s.psi(i) - (a + dir * M_PI / 2.0)), 0.0, 1e-9)
			    << "cw=" << clockwise << " i=" << i;
			// normal is the heading rotated by -90 deg: (sin psi, -cos psi)
			EXPECT_NEAR(s.normals(i, 0), std::sin(s.psi(i)), 1e-12);
			EXPECT_NEAR(s.normals(i, 1), -std::cos(s.psi(i)), 1e-12);
			// left turn (ccw) positive, right turn (cw) negative, ~1/R
			EXPECT_NEAR(s.kappa(i), dir / r, 2e-3) << "cw=" << clockwise << " i=" << i;
		}
	}
}

// Parity with tph calc_splines normals (4th return value).
TEST(ClosedSpline, NormalsMatchTph)
{
	const Eigen::MatrixX2d xy = irregularLoop();
	const ClosedSpline scaled = buildClosedSpline(xy, true);
	EXPECT_LT((scaled.normals - toMatrix2(tph_reference::kRefNormalsScaled)).cwiseAbs().maxCoeff(),
	    1e-10);
	const ClosedSpline uniform = buildClosedSpline(xy, false);
	EXPECT_LT(
	    (uniform.normals - toMatrix2(tph_reference::kRefNormalsUniform)).cwiseAbs().maxCoeff(),
	    1e-10);
}

// Step 6: S1 x and S2 x reproduce the step-4 coefficients (x' = b, x'' = 2c).
TEST(ClosedSpline, DerivativeMapsReproduceCoefficients)
{
	const Eigen::MatrixX2d xy = irregularLoop();
	for (const bool distScaling : {true, false}) {
		const ClosedSpline s = buildClosedSpline(xy, distScaling);
		const Eigen::MatrixXd & S1 = s.firstDerivMap;
		const Eigen::MatrixXd & S2 = s.secondDerivMap;
		EXPECT_LT((S1 * xy.col(0) - s.coeffX.col(1)).cwiseAbs().maxCoeff(), 1e-10) << distScaling;
		EXPECT_LT((S1 * xy.col(1) - s.coeffY.col(1)).cwiseAbs().maxCoeff(), 1e-10) << distScaling;
		EXPECT_LT((S2 * xy.col(0) - 2.0 * s.coeffX.col(2)).cwiseAbs().maxCoeff(), 1e-10) << distScaling;
		EXPECT_LT((S2 * xy.col(1) - 2.0 * s.coeffY.col(2)).cwiseAbs().maxCoeff(), 1e-10) << distScaling;
	}
}

// Step 6: shifting the whole track by a constant changes no derivative,
// so every row of S1 and S2 sums to zero.
TEST(ClosedSpline, DerivativeMapRowsSumToZero)
{
	for (const bool distScaling : {true, false}) {
		const ClosedSpline s = buildClosedSpline(irregularLoop(), distScaling);
		EXPECT_LT(s.firstDerivMap.rowwise().sum().cwiseAbs().maxCoeff(), 1e-12) << distScaling;
		EXPECT_LT(s.secondDerivMap.rowwise().sum().cwiseAbs().maxCoeff(), 1e-12) << distScaling;
	}
}

// Step 6, the property the QP relies on (paper eq. 17): moving the points by
// alpha along their normals gives x'' = S2 x_ref + S2 diag(n_x) alpha. Without
// distance scaling A does not depend on the spacing, so this is exact.
TEST(ClosedSpline, DerivativeMapsAreExactInAlphaWithoutScaling)
{
	const Eigen::MatrixX2d xy = irregularLoop();
	const int n = static_cast<int>(xy.rows());
	const ClosedSpline ref = buildClosedSpline(xy, false);

	Eigen::VectorXd alpha(n);
	for (int i = 0; i < n; ++i) {
		alpha(i) = 0.3 * std::sin(1.7 * i + 0.4);  // arbitrary offsets in [-0.3, 0.3] m
	}
	Eigen::MatrixX2d moved = xy;
	moved.col(0) += ref.normals.col(0).cwiseProduct(alpha);
	moved.col(1) += ref.normals.col(1).cwiseProduct(alpha);
	const ClosedSpline rebuilt = buildClosedSpline(moved, false);

	const Eigen::MatrixXd & S1 = ref.firstDerivMap;
	const Eigen::MatrixXd & S2 = ref.secondDerivMap;
	const Eigen::VectorXd xdd = S2 * xy.col(0) + S2 * ref.normals.col(0).asDiagonal() * alpha;
	const Eigen::VectorXd ydd = S2 * xy.col(1) + S2 * ref.normals.col(1).asDiagonal() * alpha;
	EXPECT_LT((xdd - 2.0 * rebuilt.coeffX.col(2)).cwiseAbs().maxCoeff(), 1e-10);
	EXPECT_LT((ydd - 2.0 * rebuilt.coeffY.col(2)).cwiseAbs().maxCoeff(), 1e-10);
	const Eigen::VectorXd xd = S1 * xy.col(0) + S1 * ref.normals.col(0).asDiagonal() * alpha;
	EXPECT_LT((xd - rebuilt.coeffX.col(1)).cwiseAbs().maxCoeff(), 1e-10);
}

// Step 6: the 8-point example in min_curv_derivative_maps.md (sigma = 1, so
// S1/S2 depend only on N, not on where the points are).
TEST(ClosedSpline, DerivativeMapsMatchDocExample)
{
	const ClosedSpline s = buildClosedSpline(circle(5.0, 8), false);
	const double s2Row0[8] = {-4.393, 2.786, -0.750, 0.214, -0.107, 0.214, -0.750, 2.786};
	const double s1Row0[8] = {0.000, 0.804, -0.214, 0.054, 0.000, -0.054, 0.214, -0.804};
	for (int j = 0; j < 8; ++j) {
		EXPECT_NEAR(s.secondDerivMap(0, j), s2Row0[j], 1e-3) << "j=" << j;
		EXPECT_NEAR(s.firstDerivMap(0, j), s1Row0[j], 1e-3) << "j=" << j;
	}
}
