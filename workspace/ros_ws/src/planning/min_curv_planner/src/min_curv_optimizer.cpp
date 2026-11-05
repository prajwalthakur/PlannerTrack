/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#include "min_curv_planner/min_curv_optimizer.hpp"

#include "min_curv_planner/closed_spline.hpp"
#include "min_curv_planner/min_curvature.hpp"
#include "min_curv_planner/track_preprocessing.hpp"
#include "min_curv_planner/velocity_profile.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace min_curv_planner
{

namespace
{

//////////////////////////////////////////////////////////////////////////

double wrapAngle(double a)
{
	return std::atan2(std::sin(a), std::cos(a));
}

//////////////////////////////////////////////////////////////////////////

void validateTrack(const Track & track)
{
	const int n = track.size();
	if (n < 3) {
		throw std::invalid_argument("computeRaceline: track needs at least 3 points");
	}
	if (track.wRight.size() != n || track.wLeft.size() != n) {
		throw std::invalid_argument("computeRaceline: width arrays do not match the points");
	}
	if (!track.xy.allFinite() || !track.wRight.allFinite() || !track.wLeft.allFinite()) {
		throw std::invalid_argument("computeRaceline: track contains non-finite values");
	}
	if ((track.wRight.array() < 0.0).any() || (track.wLeft.array() < 0.0).any()) {
		throw std::invalid_argument("computeRaceline: negative track width");
	}
}

//////////////////////////////////////////////////////////////////////////

// Output checks before the line is handed to a controller: finite values and
// no heading jump between samples (a cusp or loop shows up as a jump of
// ~pi). The curvature limit is only enforced on the linearized curvature, so
// it is reported (diagnostics) rather than checked here.
void validateRaceline(const Raceline & line, double kappaBound)
{
	if (!line.x.allFinite() || !line.y.allFinite() || !line.psi.allFinite() ||
	    !line.kappa.allFinite() || !line.vx.allFinite() || !line.ax.allFinite()) {
		throw std::runtime_error("optimizeRaceline: raceline contains non-finite values");
	}
	const int n = line.size();
	for (int k = 0; k < n; ++k) {
		const int j = (k + 1) % n;
		const double ds = (j == 0) ? line.length - line.s(k) : line.s(j) - line.s(k);
		const double dpsi = std::abs(wrapAngle(line.psi(j) - line.psi(k)));
		if (dpsi > std::max(M_PI / 4.0, 3.0 * kappaBound * ds)) {
			throw std::runtime_error(
			    "optimizeRaceline: heading jump (cusp or loop) at s = " + std::to_string(line.s(k)));
		}
	}
}
}  // namespace

//////////////////////////////////////////////////////////////////////////

Raceline optimizeRaceline(const Track & reference, const RacelineParams & params)
{
	validateTrack(reference);

	// Stages 2-3: spline + iterated minimum-curvature QP.
	const MinCurvIqpResult iqp = runMinCurvIqp(reference, params);

	// Stage 4: final raceline (tph create_raceline + calc_head_curv_an).
	Eigen::MatrixX2d points = iqp.reference.xy;
	points.col(0) += iqp.spline.normals.col(0).cwiseProduct(iqp.alpha);
	points.col(1) += iqp.spline.normals.col(1).cwiseProduct(iqp.alpha);
	const ClosedSpline spline = buildClosedSpline(points, false);
	const SplineSamples samples = sampleClosedSpline(spline, params.stepsizeInterp);

	Raceline out;
	out.s = samples.s;
	out.x = samples.xy.col(0);
	out.y = samples.xy.col(1);
	computeHeadingCurvature(spline, samples, out.psi, out.kappa);
	out.wRight = interpolateAtSamples(iqp.reference.wRight - iqp.alpha, samples);
	out.wLeft = interpolateAtSamples(iqp.reference.wLeft + iqp.alpha, samples);
	out.length = samples.length;

	// Stage 5: speed profile on the fixed path.
	const VelocityLimits limits = velocityLimitsFromParams(params);
	out.vx = computeVelocityProfile(out.kappa, samples.elLengths, limits);
	out.vy = Eigen::VectorXd::Zero(out.size());  // point-mass reference
	out.ax = computeAccelProfile(out.vx, samples.elLengths);
	out.lapTime = computeTimeProfile(out.vx, out.ax, samples.elLengths)(out.size());

	out.iqpIterations = iqp.iterations;
	out.iqpConverged = iqp.converged;
	out.iqpCurvErrorMax = iqp.curvErrorMax;
	validateRaceline(out, params.curvatureLimit());
	return out;
}

//////////////////////////////////////////////////////////////////////////

Raceline computeRaceline(const Track & centerline, const RacelineParams & params)
{
	validateTrack(centerline);
	return optimizeRaceline(prepareReferenceTrack(centerline, params.preprocessing), params);
}

}  // namespace min_curv_planner
