/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#include "min_curv_planner/closed_spline.hpp"

#include <Eigen/Sparse>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace min_curv_planner
{

//////////////////////////////////////////////////////////////////////////

ClosedSpline buildClosedSpline(const Eigen::MatrixX2d & xy, bool distScaling)
{
	const int n = static_cast<int>(xy.rows());
	if (n < 3) {
		throw std::invalid_argument("buildClosedSpline: needs at least 3 points");
	}
	const auto x = xy.col(0);
	const auto y = xy.col(1);
	ClosedSpline spline;

	// Step 1: segment lengths. Segment i joins point i to point i+1;
	// the last one closes the loop back to point 0.
	Eigen::VectorXd & segLen = spline.segLen;
	segLen.resize(n);
	segLen.head(n - 1) = ((x.tail(n - 1) - x.head(n - 1)).array().square() +
	    (y.tail(n - 1) - y.head(n - 1)).array().square())
	                         .sqrt()
	                         .matrix();
	segLen(n - 1) = std::hypot(x(0) - x(n - 1), y(0) - y(n - 1));
	if ((segLen.array() <= 0.0).any()) {
		throw std::invalid_argument("buildClosedSpline: duplicate consecutive points");
	}

	// joint scaling sigma_i = l_i / l_{i+1};
	Eigen::VectorXd & sigma = spline.sigma;
	sigma.resize(n);
	if (distScaling) {
		sigma.head(n - 1) = segLen.head(n - 1).array() / segLen.tail(n - 1).array();
		sigma(n - 1) = segLen(n - 1) / segLen(0);
	} else {
		sigma.setOnes();
	}

	// Ax = b
	// with speed and acceleration constraints
	using Triplet = Eigen::Triplet<double>;  // (r,c,value)
	std::vector<Triplet> entries;
	entries.reserve(12 * n);
	for (int i = 0; i < n; ++i) {
		const int j = (i + 1) % n;  // (i+1)%n segment (wraps)
		const int r = 4 * i;  // ith seg row;
		const int ci = 4 * i;  // ith seg first colmn;
		const int cj = 4 * j;  // i+1 seg first colmn;
		const double s = sigma(i);

		// each row from r to r+4 corresponds to a constraint
		// first constrait
		// ie. X(0) =  a_i = x_i
		// a_i = x_i;
		entries.emplace_back(r, ci, 1.0);
		// X(1) = x_j//next seg point
		// X(1) = a_i + b_i + c_i + d_i = x_j
		for (int k = 0; k < 4; ++k) {
			entries.emplace_back(r + 1, ci + k, 1.0);
		}
		// speed constraints
		// x_i_dot(1) = sigma_i * x_j_dot(0)
		// b_i + 2c_i + 3d_i = sigma_i * b_j
		entries.emplace_back(r + 2, ci + 1, 1.0);
		entries.emplace_back(r + 2, ci + 2, 2.0);
		entries.emplace_back(r + 2, ci + 3, 3.0);
		entries.emplace_back(r + 2, cj + 1, -s);  // - sigma_i b_j = 0

		// acceleration constraint
		// x_i_ddot(1) = sigma_i^2 * x_j_ddot(0)
		// 2ci + 6di  = 2cj*s(i)*s(i)
		entries.emplace_back(r + 3, ci + 2, 2.0);  // 2c_i + 6d_i
		entries.emplace_back(r + 3, ci + 3, 6.0);
		entries.emplace_back(r + 3, cj + 2, -2.0 * s * s);  //   - 2 sigma_i^2 c_j = 0
	}
	Eigen::SparseMatrix<double> A(4 * n, 4 * n);
	A.setFromTriplets(entries.begin(), entries.end());
	Eigen::VectorXd bx = Eigen::VectorXd::Zero(4 * n);
	Eigen::VectorXd by = Eigen::VectorXd::Zero(4 * n);
	for (int i = 0; i < n; ++i) {
		const int j = (i + 1) % n;
		bx(4 * i) = x(i);
		by(4 * i) = y(i);
		bx(4 * i + 1) = x(j);
		by(4 * i + 1) = y(j);
	}
	// Step 4: solve A z = b.
	// LU factorizaion
	// cannot do chelosky because of assymmetry matrix
	Eigen::SparseLU<Eigen::SparseMatrix<double>> lu;
	lu.compute(A);
	if (lu.info() != Eigen::Success) {
		throw std::runtime_error("buildClosedSpline: spline system is singular");
	}
	const Eigen::VectorXd zx = lu.solve(bx);
	const Eigen::VectorXd zy = lu.solve(by);

	// z is laid out [a0 b0 c0 d0 | a1 b1 c1 d1 | ...], i.e. an N x 4 table in
	// ROW-major order. Map it with that order, then copy into the struct.
	using RowMajorX4 = Eigen::Matrix<double, Eigen::Dynamic, 4, Eigen::RowMajor>;
	spline.coeffX = Eigen::Map<const RowMajorX4>(zx.data(), n, 4);
	spline.coeffY = Eigen::Map<const RowMajorX4>(zy.data(), n, 4);

	// Step 5: values at the points (t = 0 of each segment): x' = b, x'' = 2c.
	// Derivatives are w.r.t. the spline parameter t, not time.
	// Tangent at each point = first derivative at t = 0 of its segment.
	Eigen::MatrixX2d tangent(n, 2);
	tangent.col(0) = spline.coeffX.col(1);
	tangent.col(1) = spline.coeffY.col(1);
	// Guard the divisions below (normalize, kappa): a (near-)zero tangent at a
	// point makes the normal and the curvature undefined -- the curve's
	// parameter "stops" there. This only checks the points, not inside segments.
	const Eigen::ArrayXd tangentNorm = tangent.rowwise().norm().array();
	if ((tangentNorm < 1e-6 * spline.segLen.array()).any()) {
		throw std::runtime_error("buildClosedSpline: zero tangent at a spline point");
	}

	// Right-pointing unit normal: rotate the tangent by -90 deg, normalize.
	spline.normals.resize(n, 2);
	spline.normals.col(0) = tangent.col(1);
	spline.normals.col(1) = -tangent.col(0);
	spline.normals.rowwise().normalize();

	// Heading: direction of the tangent, psi = 0 along +x, in (-pi, pi].
	spline.psi = tangent.col(1).binaryExpr(
	    tangent.col(0), [](double dy, double dx) { return std::atan2(dy, dx); });

	// Curvature (paper eq. 9), positive = left turn:
	// kappa = (x'y'' - y'x'') / (x'^2 + y'^2)^(3/2)
	const auto xd = tangent.col(0).array();
	const auto yd = tangent.col(1).array();
	const Eigen::ArrayXd xdd = 2.0 * spline.coeffX.col(2).array();
	const Eigen::ArrayXd ydd = 2.0 * spline.coeffY.col(2).array();
	spline.kappa = ((xd * ydd - yd * xdd) / (xd.square() + yd.square()).pow(1.5)).matrix();

	// Step 6: derivative maps (project_docs/learning_based_controls/
	// min_curv_derivative_maps.md). b = R x with R(4j, j) = R(4(j-1)+1, j) = 1,
	// so z = A^-1 R x, and
	//   S1 = rows 4i+1 of A^-1 R    (x'  = b_i)
	//   S2 = 2 * rows 4i+2 of A^-1 R (x'' = 2 c_i).
	// Column j of R is the right-hand side for "move only point j by 1", so
	// column j of S1/S2 comes from one solve. Solving column by column keeps
	// memory at one 4N vector instead of the dense 4N x N matrix A^-1 R.
	spline.firstDerivMap.resize(n, n);
	spline.secondDerivMap.resize(n, n);
	Eigen::VectorXd rhs = Eigen::VectorXd::Zero(4 * n);
	for (int j = 0; j < n; ++j) {
		const int prev = (j - 1 + n) % n;
		rhs(4 * j) = 1.0;  // segment j starts at point j
		rhs(4 * prev + 1) = 1.0;  // segment j-1 ends at point j
		const Eigen::VectorXd z = lu.solve(rhs);
		rhs(4 * j) = 0.0;
		rhs(4 * prev + 1) = 0.0;
		// b_i sits at 4i+1, c_i at 4i+2: read them with stride 4.
		using Stride4 = Eigen::Map<const Eigen::VectorXd, 0, Eigen::InnerStride<4>>;
		spline.firstDerivMap.col(j) = Stride4(z.data() + 1, n);
		spline.secondDerivMap.col(j) = 2.0 * Stride4(z.data() + 2, n);
	}
	return spline;
}

//////////////////////////////////////////////////////////////////////////

Eigen::VectorXd computeSegmentArcLengths(const ClosedSpline & spline, int samplesPerSegment)
{
	if (samplesPerSegment < 2) {
		throw std::invalid_argument("computeSegmentArcLengths: need at least 2 samples");
	}
	const int n = spline.size();
	// Monomials [1, t, t^2, t^3] at evenly spaced t in [0, 1].
	Eigen::MatrixX4d basis(samplesPerSegment, 4);
	for (int k = 0; k < samplesPerSegment; ++k) {
		const double t = static_cast<double>(k) / (samplesPerSegment - 1);
		basis.row(k) << 1.0, t, t * t, t * t * t;
	}
	Eigen::VectorXd lengths(n);
	for (int i = 0; i < n; ++i) {
		const Eigen::VectorXd px = basis * spline.coeffX.row(i).transpose();
		const Eigen::VectorXd py = basis * spline.coeffY.row(i).transpose();
		const int m = samplesPerSegment;
		lengths(i) = ((px.tail(m - 1) - px.head(m - 1)).array().square() +
		    (py.tail(m - 1) - py.head(m - 1)).array().square())
		                 .sqrt()
		                 .sum();
	}
	return lengths;
}

//////////////////////////////////////////////////////////////////////////

SplineSamples sampleClosedSpline(const ClosedSpline & spline, double stepsize)
{
	if (!(stepsize > 0.0)) {
		throw std::invalid_argument("sampleClosedSpline: stepsize must be > 0");
	}
	const Eigen::VectorXd segLen = computeSegmentArcLengths(spline);
	const int n = spline.size();
	std::vector<double> cum(n);  // cumulative arc length at the END of each segment
	double total = 0.0;
	for (int i = 0; i < n; ++i) {
		total += segLen(i);
		cum[i] = total;
	}

	SplineSamples out;
	out.length = total;
	// tph: linspace(0, L, ceil(L / step) + 1), last point (== first) dropped.
	const int m = static_cast<int>(std::ceil(total / stepsize));
	out.xy.resize(m, 2);
	out.segment.resize(m);
	out.t.resize(m);
	out.s.resize(m);
	for (int k = 0; k < m; ++k) {
		const double s = total * k / m;
		// first segment whose end lies beyond s (tph: argmax(s < dists_cum))
		const int j = std::min(
		    static_cast<int>(std::upper_bound(cum.begin(), cum.end(), s) - cum.begin()), n - 1);
		const double segStart = (j > 0) ? cum[j - 1] : 0.0;
		const double t = (s - segStart) / segLen(j);
		const Eigen::Vector4d basis(1.0, t, t * t, t * t * t);
		out.xy(k, 0) = spline.coeffX.row(j).dot(basis);
		out.xy(k, 1) = spline.coeffY.row(j).dot(basis);
		out.segment(k) = j;
		out.t(k) = t;
		out.s(k) = s;
	}
	out.elLengths.resize(m);
	out.elLengths.head(m - 1) = out.s.tail(m - 1) - out.s.head(m - 1);
	out.elLengths(m - 1) = total - out.s(m - 1);
	return out;
}

//////////////////////////////////////////////////////////////////////////

void computeHeadingCurvature(const ClosedSpline & spline, const SplineSamples & samples,
    Eigen::VectorXd & psi, Eigen::VectorXd & kappa)
{
	const int m = samples.size();
	psi.resize(m);
	kappa.resize(m);
	for (int k = 0; k < m; ++k) {
		const int j = samples.segment(k);
		const double t = samples.t(k);
		const Eigen::Vector4d d1(0.0, 1.0, 2.0 * t, 3.0 * t * t);  // d/dt (spline parameter)
		const Eigen::Vector4d d2(0.0, 0.0, 2.0, 6.0 * t);
		const double xd = spline.coeffX.row(j).dot(d1);
		const double yd = spline.coeffY.row(j).dot(d1);
		const double xdd = spline.coeffX.row(j).dot(d2);
		const double ydd = spline.coeffY.row(j).dot(d2);
		psi(k) = std::atan2(yd, xd);
		kappa(k) = (xd * ydd - yd * xdd) / std::pow(xd * xd + yd * yd, 1.5);
	}
}

//////////////////////////////////////////////////////////////////////////

Eigen::VectorXd interpolateAtSamples(
    const Eigen::VectorXd & valuesAtPoints, const SplineSamples & samples)
{
	const int n = static_cast<int>(valuesAtPoints.size());
	Eigen::VectorXd out(samples.size());
	for (int k = 0; k < samples.size(); ++k) {
		const int j = samples.segment(k);
		if (j >= n) {
			throw std::invalid_argument("interpolateAtSamples: sample outside the value range");
		}
		const double t = samples.t(k);
		out(k) = (1.0 - t) * valuesAtPoints(j) + t * valuesAtPoints((j + 1) % n);
	}
	return out;
}

}  // namespace min_curv_planner
