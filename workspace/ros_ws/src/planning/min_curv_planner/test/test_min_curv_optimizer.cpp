/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#include "min_curv_planner/params.hpp"
#include "min_curv_planner/min_curv_optimizer.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

using namespace min_curv_planner;

namespace
{
// Counter-clockwise circle of radius r with constant widths.
Track makeCircle(double r, int n, double width)
{
	Track track;
	track.xy.resize(n, 2);
	for (int i = 0; i < n; ++i) {
		const double t = 2.0 * M_PI * i / n;
		track.xy(i, 0) = r * std::cos(t);
		track.xy(i, 1) = r * std::sin(t);
	}
	track.wRight = Eigen::VectorXd::Constant(n, width);
	track.wLeft = Eigen::VectorXd::Constant(n, width);
	return track;
}
}  // namespace

TEST(Params, DefaultsAndDerivedValues)
{
	const RacelineParams params = loadRacelineParams(YAML::Node(), YAML::Node());
	EXPECT_NEAR(params.curvatureLimit(), std::tan(0.4189) / 0.34, 1e-12);
	EXPECT_NEAR(params.lateralAccelLimit(), 0.85 * 1.0 * 9.81, 1e-12);
	EXPECT_NEAR(params.optimizationWidth(), 0.31 + 0.2, 1e-12);
}

TEST(Params, ReadsAgentAndPlannerYaml)
{
	const YAML::Node agent = YAML::Load(
	    "{dynamics_params: {mu: 1.0489, vehWheelBase: 0.34, mass: 3.47},"
	    " geometry_params: {width: 0.31},"
	    " vehicle_limits: {max_steer: 0.4, max_accel: 4.0, max_decel: 7.0, max_speed: 9.0,"
	    " max_steer_rate: 3.0, max_jerk: 40.0}}");
	const YAML::Node raceline = YAML::Load(
	    "{friction_margin: 0.9, optimization: {objective: tph_compat, iqp_iters_max: 30}}");
	const RacelineParams params = loadRacelineParams(agent, raceline);
	EXPECT_DOUBLE_EQ(params.vehicle.mu, 1.0489);
	EXPECT_DOUBLE_EQ(params.vehicle.maxSpeed, 9.0);
	EXPECT_DOUBLE_EQ(params.vehicle.maxSteerRate, 3.0);
	EXPECT_DOUBLE_EQ(params.vehicle.maxJerk, 40.0);
	EXPECT_DOUBLE_EQ(params.frictionMargin, 0.9);
	EXPECT_EQ(params.optimization.objective, Objective::TphCompat);
	EXPECT_EQ(params.optimization.iqpItersMax, 30);
}

TEST(Params, RejectsUnknownObjective)
{
	const YAML::Node raceline = YAML::Load("{optimization: {objective: fastest}}");
	EXPECT_THROW(loadRacelineParams(YAML::Node(), raceline), std::invalid_argument);
}

// On a circle every point is equivalent, so the optimized line is again a
// circle: constant curvature and speed, inside the track, and a consistent
// lap time. Smoothing off so the reference is the exact circle.
TEST(ComputeRaceline, CircleGivesConstantCurvatureAndSpeed)
{
	const double r = 5.0;
	RacelineParams params = loadRacelineParams(YAML::Node(), YAML::Node());
	params.preprocessing.enableSmoothing = false;
	const Raceline line = computeRaceline(makeCircle(r, 200, 1.0), params);

	ASSERT_GT(line.size(), 10);
	EXPECT_TRUE(line.iqpConverged);
	const double k0 = line.kappa(0);
	const double v0 = line.vx(0);
	for (int i = 0; i < line.size(); ++i) {
		EXPECT_NEAR(line.kappa(i), k0, 1e-3 * k0) << "i=" << i;  // resampling ripple
		EXPECT_NEAR(line.vx(i), v0, 1e-3) << "i=" << i;
		EXPECT_GE(line.wRight(i), 0.5 * params.optimizationWidth() - 1e-6) << "i=" << i;
		EXPECT_GE(line.wLeft(i), 0.5 * params.optimizationWidth() - 1e-6) << "i=" << i;
	}
	EXPECT_NEAR(v0, std::min(params.vehicle.maxSpeed, std::sqrt(params.lateralAccelLimit() / k0)), 1e-3);
	EXPECT_NEAR(line.lapTime, line.length / v0, 1e-3);
}

// Pins down a property of the paper's linearization (min_curv_qp.md): with
// x', y' frozen, an outward offset looks like MORE curvature (the formula
// misses that the path gets longer), so on a circle the QP moves the line to
// the INNER bound, although the outer bound has the smaller true curvature.
// tph 0.76 does the same (alpha = -0.745 on this circle). If the
// linearization is ever replaced by the full Jacobian, this test flips.
TEST(ComputeRaceline, PaperLinearizationMovesCircleInward)
{
	const double r = 5.0;
	RacelineParams params = loadRacelineParams(YAML::Node(), YAML::Node());
	params.preprocessing.enableSmoothing = false;
	const Raceline line = computeRaceline(makeCircle(r, 200, 1.0), params);
	const double innerRadius = r - (1.0 - 0.5 * params.optimizationWidth());
	EXPECT_NEAR(1.0 / line.kappa(0), innerRadius, 5e-3);  // spline discretization
}

TEST(ComputeRaceline, RejectsDegenerateTrack)
{
	Track track = makeCircle(5.0, 2, 1.0);
	const RacelineParams params;
	EXPECT_THROW(computeRaceline(track, params), std::invalid_argument);
}
