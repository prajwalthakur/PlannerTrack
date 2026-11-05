/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
// Stage tests (sampling, speed profile, preprocessing) and parity with the
// tph fixtures in test/data (see test/data/README.md).
#include "min_curv_planner/closed_spline.hpp"
#include "min_curv_planner/min_curv_optimizer.hpp"
#include "min_curv_planner/min_curvature.hpp"
#include "min_curv_planner/track_io.hpp"
#include "min_curv_planner/track_preprocessing.hpp"
#include "min_curv_planner/velocity_profile.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace min_curv_planner;

namespace
{
// Numeric CSV (comma separated, '#' lines are comments). Returns rows.
std::vector<std::vector<double>> readCsv(const std::string & file, std::string * comment = nullptr)
{
	std::ifstream in(std::string(TEST_DATA_DIR) + "/" + file);
	if (!in) {
		throw std::runtime_error("cannot open " + file);
	}
	std::vector<std::vector<double>> rows;
	std::string line;
	while (std::getline(in, line)) {
		if (line.empty()) {
			continue;
		}
		if (line[0] == '#') {
			if (comment != nullptr) {
				*comment += line;
			}
			continue;
		}
		std::vector<double> row;
		std::stringstream ss(line);
		std::string cell;
		while (std::getline(ss, cell, ',')) {
			row.push_back(std::stod(cell));
		}
		rows.push_back(row);
	}
	return rows;
}

double valueAfter(const std::string & text, const std::string & key)
{
	const auto pos = text.find(key);
	if (pos == std::string::npos) {
		throw std::runtime_error("key not found: " + key);
	}
	return std::stod(text.substr(pos + key.size()));
}

Track tphReference()
{
	const auto rows = readCsv("tph_reftrack.csv");
	Track t;
	const int n = static_cast<int>(rows.size());
	t.xy.resize(n, 2);
	t.wRight.resize(n);
	t.wLeft.resize(n);
	for (int i = 0; i < n; ++i) {
		t.xy(i, 0) = rows[i][0];
		t.xy(i, 1) = rows[i][1];
		t.wRight(i) = rows[i][2];
		t.wLeft(i) = rows[i][3];
	}
	return t;
}

// Parameters the fixture was generated with (test/data/README.md).
RacelineParams fixtureParams()
{
	RacelineParams p;
	p.vehicle.mu = 1.0489;
	p.vehicle.wheelbase = 0.34;
	p.vehicle.width = 0.31;
	p.vehicle.mass = 3.47;
	p.vehicle.maxSteer = 0.4189;
	p.vehicle.maxAccel = 5.0;
	p.vehicle.maxDecel = 8.0;
	p.vehicle.maxSpeed = 10.0;
	p.frictionMargin = 0.85;
	p.safetyMargin = 0.10;
	p.preprocessing.stepsizePrep = 0.1;
	p.preprocessing.stepsizeReg = 0.3;
	p.preprocessing.smoothingResidual = 10.0;
	p.optimization.objective = Objective::MinCurvature;
	p.optimization.iqpItersMin = 3;
	p.optimization.iqpItersMax = 20;
	p.optimization.iqpCurvErrorAllowed = 0.01;
	p.stepsizeInterp = 0.2;
	return p;
}

double wrapAngle(double a)
{
	return std::atan2(std::sin(a), std::cos(a));
}

Eigen::MatrixX2d circlePoints(double r, int n)
{
	Eigen::MatrixX2d xy(n, 2);
	for (int i = 0; i < n; ++i) {
		xy(i, 0) = r * std::cos(2.0 * M_PI * i / n);
		xy(i, 1) = r * std::sin(2.0 * M_PI * i / n);
	}
	return xy;
}
}  // namespace

// ---------------------------------------------------------------- sampling

TEST(SplineSampling, CircleLengthAndEqualSpacing)
{
	const double r = 5.0;
	const ClosedSpline spline = buildClosedSpline(circlePoints(r, 60), true);
	const SplineSamples s = sampleClosedSpline(spline, 0.2);
	EXPECT_NEAR(s.length, 2.0 * M_PI * r, 2e-3);  // 15-point polyline, as tph
	EXPECT_EQ(s.size(), static_cast<int>(std::ceil(s.length / 0.2)));
	EXPECT_NEAR(s.elLengths.sum(), s.length, 1e-9);
	EXPECT_LT((s.elLengths.array() - s.elLengths.mean()).abs().maxCoeff(), 1e-9);
	for (int k = 0; k < s.size(); ++k) {
		EXPECT_NEAR(s.xy.row(k).norm(), r, 1e-3) << "k=" << k;  // on the circle
	}
	Eigen::VectorXd psi;
	Eigen::VectorXd kappa;
	computeHeadingCurvature(spline, s, psi, kappa);
	for (int k = 0; k < s.size(); ++k) {
		const double tangent = std::atan2(s.xy(k, 1), s.xy(k, 0)) + M_PI / 2.0;
		EXPECT_NEAR(wrapAngle(psi(k) - tangent), 0.0, 1e-3) << "k=" << k;
		EXPECT_NEAR(kappa(k), 1.0 / r, 5e-3) << "k=" << k;
	}
}

TEST(SplineSampling, InterpolatesLinearlyInT)
{
	const ClosedSpline spline = buildClosedSpline(circlePoints(5.0, 8), true);
	const SplineSamples s = sampleClosedSpline(spline, 0.3);
	Eigen::VectorXd values(8);
	values << 0, 1, 2, 3, 4, 5, 6, 7;
	const Eigen::VectorXd v = interpolateAtSamples(values, s);
	for (int k = 0; k < s.size(); ++k) {
		const int j = s.segment(k);
		const double expected = (1.0 - s.t(k)) * values(j) + s.t(k) * values((j + 1) % 8);
		EXPECT_DOUBLE_EQ(v(k), expected);
	}
}

// ----------------------------------------------------------- speed profile

TEST(VelocityProfile, ConstantCurvatureGivesCorneringSpeed)
{
	VelocityLimits lim;
	lim.ayTires = 8.0;
	lim.vMax = 20.0;
	const Eigen::VectorXd kappa = Eigen::VectorXd::Constant(50, 0.5);
	const Eigen::VectorXd el = Eigen::VectorXd::Constant(50, 0.2);
	const Eigen::VectorXd v = computeVelocityProfile(kappa, el, lim);
	EXPECT_LT((v.array() - std::sqrt(8.0 / 0.5)).abs().maxCoeff(), 1e-9);
}

// Straight with one tight corner: acceleration within the machine limit,
// braking within the combined tire limit (p = 1: |a_x|/ax + |a_y|/ay <= 1),
// and the corner is slower than the straight.
TEST(VelocityProfile, RespectsAccelerationLimits)
{
	VelocityLimits lim;
	lim.vMax = 10.0;
	lim.maxDecel = 8.0;
	lim.ayTires = 8.0;
	lim.maxAccel = 5.0;
	const int n = 300;
	Eigen::VectorXd kappa = Eigen::VectorXd::Zero(n);
	kappa.segment(150, 20).setConstant(1.0);
	const Eigen::VectorXd el = Eigen::VectorXd::Constant(n, 0.2);
	const Eigen::VectorXd v = computeVelocityProfile(kappa, el, lim);
	const Eigen::VectorXd ax = computeAccelProfile(v, el);
	EXPECT_NEAR(v(160), std::sqrt(8.0), 1e-6);
	EXPECT_NEAR(v(0), lim.vMax, 1e-6);
	for (int k = 0; k < n; ++k) {
		const double ay = v(k) * v(k) * std::abs(kappa(k));
		EXPECT_LE(ay, lim.ayTires + 1e-6) << "k=" << k;
		EXPECT_LE(ax(k), lim.maxAccel + 0.1) << "k=" << k;  // finite-step discretization
		if (ax(k) < 0.0) {
			EXPECT_LE(-ax(k) / lim.maxDecel + ay / lim.ayTires, 1.0 + 0.05) << "k=" << k;
		}
	}
	const Eigen::VectorXd t = computeTimeProfile(v, ax, el);
	EXPECT_GT(t(n), el.sum() / lim.vMax);  // slower than flat out
}

// ----------------------------------------------------------- preprocessing

TEST(Preprocessing, SmoothingHitsResidualTarget)
{
	Eigen::MatrixX2d xy = circlePoints(5.0, 300);
	for (int i = 0; i < 300; ++i) {
		xy(i, 0) += 0.05 * std::sin(37.0 * i);  // deterministic noise
		xy(i, 1) += 0.05 * std::cos(53.0 * i);
	}
	const double target = 0.5;
	const Eigen::MatrixX2d z = smoothClosedCurve(xy, target);
	EXPECT_NEAR((z - xy).squaredNorm(), target, 1e-3 * target);
	EXPECT_TRUE(smoothClosedCurve(xy, 0.0).isApprox(xy));
}

// Our Whittaker preprocessing on the scenario centerline vs tph's splprep
// result. Not identical by design (track_preprocessing.hpp): same residual
// target, but the Whittaker smoother cuts corners slightly more. What matters
// is the final raceline, which is bounded by the track: compare that too.
TEST(Preprocessing, ScenarioReferenceAndRacelineCloseToTph)
{
	const Track center = loadTrackFromWaypointsJson(RACING_WAYPOINTS_JSON);
	const Track ours = prepareReferenceTrack(center, fixtureParams().preprocessing);
	const Track tph = tphReference();
	double maxDist = 0.0;
	for (int k = 0; k < ours.size(); ++k) {
		const double d = (tph.xy.rowwise() - ours.xy.row(k)).rowwise().norm().minCoeff();
		maxDist = std::max(maxDist, d);
	}
	std::string comment;
	const auto tphLine = readCsv("tph_raceline.csv", &comment);
	const Raceline line = optimizeRaceline(ours, fixtureParams());
	const double tphLap = valueAfter(comment, "lap_time_s = ");
	const double tphLength = valueAfter(comment, "length_m = ");
	std::cout << "[ measured ] preprocessing: " << ours.size() << " vs " << tph.size()
	          << " points, max distance to tph reference " << maxDist
	          << " m; resulting raceline length " << line.length << " vs " << tphLength
	          << " m, lap " << line.lapTime << " vs " << tphLap << " s\n";
	EXPECT_NEAR(ours.size(), tph.size(), 0.05 * tph.size());
	EXPECT_LT(maxDist, 0.3);
	EXPECT_TRUE((ours.wRight.array() > 0.0).all() && (ours.wLeft.array() > 0.0).all());
	EXPECT_NEAR(line.length, tphLength, 0.005 * tphLength);
	EXPECT_NEAR(line.lapTime, tphLap, 0.01 * tphLap);
}

// -------------------------------------------------------------- parity

// One QP with the original tph objective on tph's reference line.
TEST(TphParity, SingleQpMatchesOptMinCurv)
{
	const Track ref = tphReference();
	std::string comment;
	const auto rows = readCsv("tph_qp_alpha.csv", &comment);
	const RacelineParams p = fixtureParams();
	const ClosedSpline spline = buildClosedSpline(ref.xy, true);
	const MinCurvLinearization lin = buildMinCurvLinearization(ref, spline, p.optimizationWidth());
	const MinCurvQpResult qp =
	    solveMinCurvQp(ref, spline, lin, p.curvatureLimit(), Objective::TphCompat);
	ASSERT_EQ(qp.alpha.size(), static_cast<int>(rows.size()));
	double maxDiff = 0.0;
	for (int i = 0; i < qp.alpha.size(); ++i) {
		maxDiff = std::max(maxDiff, std::abs(qp.alpha(i) - rows[i][0]));
	}
	const double tphErr = valueAfter(comment, "curv_error_max = ");
	std::cout << "[ measured ] single QP: max |alpha - tph| " << maxDiff << " m, curv error "
	          << qp.curvErrorMax << " vs tph " << tphErr << "\n";
	EXPECT_LT(maxDiff, 1e-6);
	EXPECT_NEAR(qp.curvErrorMax, tphErr, 1e-6 * tphErr);
}

// Full stages 2-5 on tph's reference line vs tph (corrected objective).
TEST(TphParity, PipelineMatchesTphRaceline)
{
	std::string comment;
	const auto rows = readCsv("tph_raceline.csv", &comment);
	const Raceline line = optimizeRaceline(tphReference(), fixtureParams());

	EXPECT_EQ(line.iqpIterations, static_cast<int>(valueAfter(comment, "iqp_iterations = ")));
	EXPECT_TRUE(line.iqpConverged);
	ASSERT_EQ(line.size(), static_cast<int>(rows.size()));
	double dXY = 0.0, dPsi = 0.0, dKappa = 0.0, dV = 0.0, dW = 0.0;
	for (int k = 0; k < line.size(); ++k) {
		const auto & r = rows[k];
		dXY = std::max(dXY, std::hypot(line.x(k) - r[1], line.y(k) - r[2]));
		dPsi = std::max(dPsi, std::abs(wrapAngle(line.psi(k) - r[3])));
		dKappa = std::max(dKappa, std::abs(line.kappa(k) - r[4]));
		dV = std::max(dV, std::abs(line.vx(k) - r[5]));
		dW = std::max({dW, std::abs(line.wRight(k) - r[7]), std::abs(line.wLeft(k) - r[8])});
	}
	const double tphLap = valueAfter(comment, "lap_time_s = ");
	const double tphLength = valueAfter(comment, "length_m = ");
	std::cout << "[ measured ] pipeline vs tph: xy " << dXY << " m, psi " << dPsi << " rad, kappa "
	          << dKappa << " rad/m, widths " << dW << " m, vx " << dV << " m/s, lap "
	          << line.lapTime << " vs " << tphLap << " s, length " << line.length << " vs "
	          << tphLength << " m\n";
	EXPECT_LT(dXY, 1e-6);
	EXPECT_LT(dPsi, 1e-6);
	EXPECT_LT(dKappa, 1e-5);
	EXPECT_LT(dW, 1e-6);
	EXPECT_NEAR(line.length, tphLength, 1e-6);
	// Speed: identical to tph except in the braking zone before start/finish,
	// where tph misses the braking for the first corner (seam fix,
	// velocity_profile.hpp). There ours is lower, never higher.
	int firstDiff = line.size();
	for (int k = 0; k < line.size(); ++k) {
		EXPECT_LE(line.vx(k), rows[k][5] + 1e-9) << "k=" << k;
		if (std::abs(line.vx(k) - rows[k][5]) > 1e-6 && firstDiff == line.size()) {
			firstDiff = k;
		}
	}
	for (int k = firstDiff; k < line.size(); ++k) {  // one contiguous zone up to the seam
		EXPECT_GT(rows[k][5] - line.vx(k), 1e-6) << "k=" << k;
	}
	std::cout << "[ measured ] speed differs from tph only for the last " << line.size() - firstDiff
	          << " of " << line.size() << " points (braking for start/finish), max " << dV
	          << " m/s\n";
	EXPECT_LT(line.size() - firstDiff, line.size() / 4);
	EXPECT_GT(line.lapTime, tphLap);
}

// End to end on the scenario centerline (our preprocessing).
TEST(ComputeRaceline, ScenarioTrackEndToEnd)
{
	const RacelineParams p = fixtureParams();
	const Raceline line = computeRaceline(loadTrackFromWaypointsJson(RACING_WAYPOINTS_JSON), p);
	std::cout << "[ measured ] scenario: " << line.size() << " points, length " << line.length
	          << " m, lap " << line.lapTime << " s, IQP " << line.iqpIterations
	          << " iterations (converged " << line.iqpConverged << "), max |kappa| "
	          << line.kappa.cwiseAbs().maxCoeff() << " rad/m\n";
	EXPECT_TRUE(line.iqpConverged);
	EXPECT_LE(line.vx.maxCoeff(), p.vehicle.maxSpeed + 1e-9);
	EXPECT_TRUE((line.wRight.array() >= 0.5 * p.optimizationWidth() - 0.05).all());
	EXPECT_TRUE((line.wLeft.array() >= 0.5 * p.optimizationWidth() - 0.05).all());
	EXPECT_LE(line.kappa.cwiseAbs().maxCoeff(), 1.1 * p.curvatureLimit());
}

namespace
{
// Longitudinal acceleration within the limits at every step of a closed
// profile, using the grip budget at the end of the step that has more of it
// (the forward/backward passes evaluate it at either end).
double worstAccelViolation(const Eigen::VectorXd & v, const Eigen::VectorXd & kappa,
    const Eigen::VectorXd & el, const VelocityLimits & lim)
{
	const int n = static_cast<int>(v.size());
	const Eigen::VectorXd ax = computeAccelProfile(v, el);
	double worst = 0.0;
	for (int k = 0; k < n; ++k) {
		const int j = (k + 1) % n;
		double budget = 0.0;
		for (const int i : {k, j}) {
			const double ay = v(i) * v(i) * std::abs(kappa(i));
			const double r = std::max(0.0, 1.0 - std::pow(ay / lim.ayTires, lim.dynModelExp));
			budget = std::max(budget, lim.maxDecel * std::pow(r, 1.0 / lim.dynModelExp));
		}
		const double limit = ax(k) >= 0.0 ? std::min(budget, lim.maxAccel) : budget;
		worst = std::max(worst, std::abs(ax(k)) - limit);
	}
	return worst;
}
}  // namespace

// Regression: a corner right at start/finish. Before the fix the end of the
// lap stayed at vMax and the speed dropped instantly at point 0.
TEST(VelocityProfile, BrakesForCornerAtStartFinish)
{
	VelocityLimits lim;
	lim.vMax = 10.0;
	lim.maxDecel = 8.0;
	lim.ayTires = 8.0;
	lim.maxAccel = 5.0;
	const int n = 300;
	Eigen::VectorXd kappa = Eigen::VectorXd::Zero(n);
	kappa.head(20).setConstant(1.0);  // corner at the very start of the lap
	const Eigen::VectorXd el = Eigen::VectorXd::Constant(n, 0.2);
	const Eigen::VectorXd v = computeVelocityProfile(kappa, el, lim);
	EXPECT_LT(v(n - 1), lim.vMax - 1.0);  // braking before the seam
	EXPECT_LT(worstAccelViolation(v, kappa, el, lim), 0.5);
}

// steps.md 3.4(6): a_x within the limits on the real scenario raceline.
TEST(VelocityProfile, ScenarioRacelineAccelWithinLimits)
{
	const RacelineParams p = fixtureParams();
	const Raceline line = computeRaceline(loadTrackFromWaypointsJson(RACING_WAYPOINTS_JSON), p);
	Eigen::VectorXd el(line.size());
	el.head(line.size() - 1) = line.s.tail(line.size() - 1) - line.s.head(line.size() - 1);
	el(line.size() - 1) = line.length - line.s(line.size() - 1);
	const double worst = worstAccelViolation(line.vx, line.kappa, el, velocityLimitsFromParams(p));
	std::cout << "[ measured ] scenario: worst a_x beyond the limit " << worst << " m/s^2, a_x in ["
	          << line.ax.minCoeff() << ", " << line.ax.maxCoeff() << "]\n";
	EXPECT_LT(worst, 0.5);
}
