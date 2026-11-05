/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#include "min_curv_planner/velocity_profile.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <stdexcept>

namespace min_curv_planner
{

namespace
{
enum class Pass { AccelForward, DecelBackward };

// tph calc_ax_poss with constant limits.
double possibleAccel(double v, double radius, const VelocityLimits & lim, Pass pass)
{
	const double ayUsed = v * v / radius;  // radius may be +inf (straight)
	const double radicand = 1.0 - std::pow(ayUsed / lim.ayTires, lim.dynModelExp);
	const double maxDecel =
	    radicand > 0.0 ? lim.maxDecel * std::pow(radicand, 1.0 / lim.dynModelExp) : 0.0;
	const double drag = -v * v * lim.dragCoeff / lim.mass;  // <= 0
	if (pass == Pass::AccelForward) {
		return std::min(maxDecel, lim.maxAccel) + drag;
	}
	return maxDecel - drag;  // braking: drag helps
}

// tph __solver_fb_acc_profile on arrays that are already in pass order:
// segLen(k) joins point k and k+1 in that order.
void accelPass(Eigen::VectorXd & vx, const Eigen::VectorXd & radii, const Eigen::VectorXd & segLen,
    const VelocityLimits & lim, Pass pass)
{
	const int n = static_cast<int>(vx.size());
	// Start of each run of increasing speed.
	std::deque<int> starts;
	int last = -2;
	for (int i = 0; i + 1 < n; ++i) {
		if (vx(i + 1) > vx(i)) {
			if (i - last > 1) {
				starts.push_back(i);
			}
			last = i;
		}
	}
	while (!starts.empty()) {
		int i = starts.front();
		starts.pop_front();
		while (i < n - 1) {
			const double ax = possibleAccel(vx(i), radii(i), lim, pass);
			double vNext = std::sqrt(std::max(0.0, vx(i) * vx(i) + 2.0 * ax * segLen(i)));
			if (pass == Pass::DecelBackward) {
				// re-evaluate the grip at the next point once (tph does the same)
				const double axNext = possibleAccel(vNext, radii(i + 1), lim, pass);
				const double vTmp =
				    std::sqrt(std::max(0.0, vx(i) * vx(i) + 2.0 * axNext * segLen(i)));
				vNext = std::min(vNext, vTmp);
			}
			if (vNext < vx(i + 1)) {
				vx(i + 1) = vNext;
			}
			++i;
			if (vNext > lim.vMax || (!starts.empty() && i >= starts.front())) {
				break;
			}
		}
	}
}
}  // namespace

//////////////////////////////////////////////////////////////////////////

VelocityLimits velocityLimitsFromParams(const RacelineParams & params)
{
	VelocityLimits lim;
	lim.vMax = params.vehicle.maxSpeed;
	lim.maxDecel = params.vehicle.maxDecel;
	lim.ayTires = params.lateralAccelLimit();
	lim.maxAccel = params.vehicle.maxAccel;
	lim.dynModelExp = params.velocityProfile.dynModelExp;
	lim.dragCoeff = params.velocityProfile.dragCoeff;
	lim.mass = params.vehicle.mass;
	return lim;
}

//////////////////////////////////////////////////////////////////////////

Eigen::VectorXd computeVelocityProfile(
    const Eigen::VectorXd & kappa, const Eigen::VectorXd & elLengths, const VelocityLimits & limits)
{
	const int n = static_cast<int>(kappa.size());
	if (n < 2 || elLengths.size() != n) {
		throw std::invalid_argument("computeVelocityProfile: kappa and elLengths must have equal size >= 2");
	}
	if (!kappa.allFinite() || !elLengths.allFinite() || (elLengths.array() <= 0.0).any()) {
		throw std::invalid_argument("computeVelocityProfile: non-finite curvature or bad lengths");
	}
	if (!(limits.vMax > 0.0 && limits.maxDecel > 0.0 && limits.ayTires > 0.0 &&
	        limits.maxAccel > 0.0 && limits.mass > 0.0)) {
		throw std::invalid_argument("computeVelocityProfile: limits must be positive");
	}

	// Cornering limit, capped at vMax (kappa = 0 -> radius inf -> vMax).
	const double inf = std::numeric_limits<double>::infinity();
	Eigen::VectorXd radii(n);
	Eigen::VectorXd vx(n);
	for (int i = 0; i < n; ++i) {
		radii(i) = kappa(i) != 0.0 ? 1.0 / std::abs(kappa(i)) : inf;
		vx(i) = std::min(limits.vMax, std::sqrt(limits.ayTires * radii(i)));
	}

	// Closed loop: run each pass over two laps and keep the second lap, so
	// the start/finish point sees the speed coming from the previous lap.
	Eigen::VectorXd vx2(2 * n);
	Eigen::VectorXd radii2(2 * n);
	Eigen::VectorXd seg2(2 * n);
	vx2 << vx, vx;
	radii2 << radii, radii;
	seg2 << elLengths, elLengths;

	// Forward pass (acceleration).
	accelPass(vx2, radii2, seg2, limits, Pass::AccelForward);
	vx2.head(n) = vx2.tail(n);

	// Backward pass (braking), on reversed arrays. Reversed point k is
	// original point 2n-1-k; the step k -> k+1 runs along original segment
	// 2n-2-k (tph uses 2n-1-k here, see the file comment).
	Eigen::VectorXd vxRev = vx2.reverse();
	const Eigen::VectorXd radiiRev = radii2.reverse();
	Eigen::VectorXd segRev(2 * n);
	for (int k = 0; k < 2 * n; ++k) {
		segRev(k) = seg2((2 * n - 2 - k + 2 * n) % (2 * n));
	}
	accelPass(vxRev, radiiRev, segRev, limits, Pass::DecelBackward);
	vx2 = vxRev.reverse();
	// Keep the FIRST lap: braking propagates backward, so the end of lap 1
	// is constrained by the corner at the start of lap 2. The end of lap 2
	// would need a lap 3 and is not braked for the next corner (tph keeps
	// lap 2 here, which gives a speed jump at start/finish).
	return vx2.head(n);
}

//////////////////////////////////////////////////////////////////////////

Eigen::VectorXd computeAccelProfile(const Eigen::VectorXd & vx, const Eigen::VectorXd & elLengths)
{
	const int n = static_cast<int>(vx.size());
	if (elLengths.size() != n) {
		throw std::invalid_argument("computeAccelProfile: sizes do not match");
	}
	Eigen::VectorXd ax(n);
	for (int k = 0; k < n; ++k) {
		const double v1 = vx((k + 1) % n);
		ax(k) = (v1 * v1 - vx(k) * vx(k)) / (2.0 * elLengths(k));
	}
	return ax;
}

//////////////////////////////////////////////////////////////////////////

Eigen::VectorXd computeTimeProfile(
    const Eigen::VectorXd & vx, const Eigen::VectorXd & ax, const Eigen::VectorXd & elLengths)
{
	const int n = static_cast<int>(vx.size());
	if (ax.size() != n || elLengths.size() != n) {
		throw std::invalid_argument("computeTimeProfile: sizes do not match");
	}
	Eigen::VectorXd t(n + 1);
	t(0) = 0.0;
	for (int k = 0; k < n; ++k) {
		double dt;
		if (ax(k) != 0.0) {
			// constant acceleration over the step: l = v dt + a dt^2 / 2
			dt = (-vx(k) + std::sqrt(std::max(0.0, vx(k) * vx(k) + 2.0 * ax(k) * elLengths(k)))) /
			    ax(k);
		} else {
			if (!(vx(k) > 0.0)) {
				throw std::runtime_error("computeTimeProfile: zero speed with zero acceleration");
			}
			dt = elLengths(k) / vx(k);
		}
		t(k + 1) = t(k) + dt;
	}
	return t;
}

}  // namespace min_curv_planner
