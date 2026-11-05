/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#include "min_curv_planner/track_preprocessing.hpp"

#include "min_curv_planner/closed_spline.hpp"

#include <Eigen/Sparse>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace min_curv_planner
{

Track interpolateTrackLinear(const Track & track, double stepsize)
{
	const int n = track.size();
	if (n < 3 || !(stepsize > 0.0)) {
		throw std::invalid_argument("interpolateTrackLinear: need >= 3 points and stepsize > 0");
	}
	// Cumulative chord length along the closed polyline (point n = point 0).
	std::vector<double> cum(n + 1, 0.0);
	for (int i = 0; i < n; ++i) {
		cum[i + 1] = cum[i] + (track.xy.row((i + 1) % n) - track.xy.row(i)).norm();
	}
	const double total = cum[n];
	// tph: linspace(0, L, ceil(L / step) + 1), last point (== first) dropped.
	const int m = static_cast<int>(std::ceil(total / stepsize));
	Track out;
	out.xy.resize(m, 2);
	out.wRight.resize(m);
	out.wLeft.resize(m);
	for (int k = 0; k < m; ++k) {
		const double s = total * k / m;
		int i = static_cast<int>(std::upper_bound(cum.begin(), cum.end(), s) - cum.begin()) - 1;
		i = std::clamp(i, 0, n - 1);
		const double len = cum[i + 1] - cum[i];
		const double r = len > 0.0 ? (s - cum[i]) / len : 0.0;
		const int j = (i + 1) % n;
		out.xy.row(k) = (1.0 - r) * track.xy.row(i) + r * track.xy.row(j);
		out.wRight(k) = (1.0 - r) * track.wRight(i) + r * track.wRight(j);
		out.wLeft(k) = (1.0 - r) * track.wLeft(i) + r * track.wLeft(j);
	}
	return out;
}

//////////////////////////////////////////////////////////////////////////

Eigen::MatrixX2d smoothClosedCurve(const Eigen::MatrixX2d & xy, double residualTarget)
{
	const int n = static_cast<int>(xy.rows());
	if (residualTarget <= 0.0) {
		return xy;
	}
	if (n < 3) {
		throw std::invalid_argument("smoothClosedCurve: need at least 3 points");
	}
	// D: cyclic second difference, row i = (1, -2, 1) at i-1, i, i+1.
	std::vector<Eigen::Triplet<double>> entries;
	entries.reserve(3 * n);
	for (int i = 0; i < n; ++i) {
		entries.emplace_back(i, (i - 1 + n) % n, 1.0);
		entries.emplace_back(i, i, -2.0);
		entries.emplace_back(i, (i + 1) % n, 1.0);
	}
	Eigen::SparseMatrix<double> D(n, n);
	D.setFromTriplets(entries.begin(), entries.end());
	const Eigen::SparseMatrix<double> DtD = Eigen::SparseMatrix<double>(D.transpose()) * D;
	Eigen::SparseMatrix<double> I(n, n);
	I.setIdentity();

	// Residual grows monotonically with lambda: bisect on log10(lambda).
	Eigen::MatrixX2d z;
	auto residualFor = [&](double logLambda) {
		const Eigen::SparseMatrix<double> M = I + std::pow(10.0, logLambda) * DtD;
		Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> ldlt(M);
		if (ldlt.info() != Eigen::Success) {
			throw std::runtime_error("smoothClosedCurve: factorization failed");
		}
		z = ldlt.solve(xy);
		return (z - xy).squaredNorm();
	};
	double lo = -8.0;
	double hi = 12.0;
	if (residualFor(hi) <= residualTarget) {
		return z;  // even maximal smoothing stays within the target
	}
	for (int it = 0; it < 200; ++it) {
		const double mid = 0.5 * (lo + hi);
		const double res = residualFor(mid);
		if (std::abs(res - residualTarget) <= 1e-3 * residualTarget) {
			return z;
		}
		(res < residualTarget ? lo : hi) = mid;
	}
	residualFor(lo);
	return z;
}

//////////////////////////////////////////////////////////////////////////

namespace
{
// (c) For each new point, find the closest segment of the fine (unsmoothed)
// polyline that runs in the same direction, and correct its widths by the
// signed sideways offset: right width shrinks by the offset to the right.
void correctWidths(const Track & fine, const Eigen::MatrixX2d & xy, const Eigen::VectorXd & psi,
    Eigen::VectorXd & wRight, Eigen::VectorXd & wLeft)
{
	const int nf = fine.size();
	const int m = static_cast<int>(xy.rows());
	wRight.resize(m);
	wLeft.resize(m);
	for (int k = 0; k < m; ++k) {
		const Eigen::RowVector2d p = xy.row(k);
		const Eigen::RowVector2d u(std::cos(psi(k)), std::sin(psi(k)));
		int best = -1;
		double bestTau = 0.0;
		double bestDist = std::numeric_limits<double>::infinity();
		for (const bool requireSameDirection : {true, false}) {
			for (int i = 0; i < nf; ++i) {
				const Eigen::RowVector2d a = fine.xy.row(i);
				const Eigen::RowVector2d d = fine.xy.row((i + 1) % nf) - a;
				if (requireSameDirection && d.dot(u) <= 0.0) {
					continue;  // other leg of a hairpin
				}
				const double tau = std::clamp((p - a).dot(d) / d.squaredNorm(), 0.0, 1.0);
				const double dist = (p - (a + tau * d)).squaredNorm();
				if (dist < bestDist) {
					bestDist = dist;
					best = i;
					bestTau = tau;
				}
			}
			if (best >= 0) {
				break;
			}
		}
		const int j = (best + 1) % nf;
		const Eigen::RowVector2d a = fine.xy.row(best);
		const Eigen::RowVector2d d = fine.xy.row(j) - a;
		const Eigen::RowVector2d foot = a + bestTau * d;
		const Eigen::RowVector2d rightNormal = Eigen::RowVector2d(d(1), -d(0)) / d.norm();
		const double offsetRight = (p - foot).dot(rightNormal);
		wRight(k) = (1.0 - bestTau) * fine.wRight(best) + bestTau * fine.wRight(j) - offsetRight;
		wLeft(k) = (1.0 - bestTau) * fine.wLeft(best) + bestTau * fine.wLeft(j) + offsetRight;
	}
}
}  // namespace

//////////////////////////////////////////////////////////////////////////

Track prepareReferenceTrack(const Track & centerline, const PreprocessingParams & params)
{
	if (!params.enableSmoothing) {
		return centerline;
	}
	// (a) linear resampling, (b) smoothing.
	const Track fine = interpolateTrackLinear(centerline, params.stepsizePrep);
	const Eigen::MatrixX2d smoothed = smoothClosedCurve(fine.xy, params.smoothingResidual);
	// (d) spline through the smoothed points, resampled at the optimization step.
	const ClosedSpline spline = buildClosedSpline(smoothed, true);
	const SplineSamples samples = sampleClosedSpline(spline, params.stepsizeReg);
	Eigen::VectorXd psi;
	Eigen::VectorXd kappa;
	computeHeadingCurvature(spline, samples, psi, kappa);

	Track out;
	out.xy = samples.xy;
	// (c) widths w.r.t. the unsmoothed track.
	correctWidths(fine, out.xy, psi, out.wRight, out.wLeft);
	for (int k = 0; k < out.size(); ++k) {
		if (out.wRight(k) <= 0.0 || out.wLeft(k) <= 0.0) {
			throw std::runtime_error("prepareReferenceTrack: smoothing moved the line outside the "
			                         "track at point " +
			    std::to_string(k) + "; reduce smoothing_residual");
		}
	}
	return out;
}

}  // namespace min_curv_planner
