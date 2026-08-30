/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */


#include "ssc_planner/bezier_traj_gen/bezier_curve.hpp"

#include "mpl_qp_interface/osqp_interface.hpp"

#undef VERBOSE
#include "yaml-cpp/yaml.h"

#include <fstream>
#include <iostream>
#include <random>
#include <vector>

//////////////////////////////////////////////////////////////////////////

long long fac(int n)
{
	if (n == 0) return 1;
	if (n == 1) return 1;
	if (n == 2) return 2;
	if (n == 3) return 6;
	if (n == 4) return 24;
	if (n == 5) return 120;

	long long ans = 1;
	for (int i = 1; i <= n; i++) ans *= i;
	return ans;
}

//////////////////////////////////////////////////////////////////////////

long long nchoosek(int n, int k)
{
	return fac(n) / fac(k) / fac(n - k);
}

// Hardcoded jerk-cost Hessian for a degree-5 Bezier curve, derivative
// degree 3 -- H(j,k) = integral_0^1 B_j'''(tau)*B_k'''(tau) dtau, pure
// tau-domain, duration-independent. Not computed generically (see
// kBezierOrder's doc comment); verified against direct derivation in
// project_docs/bezier_curve_code_explaination.md.
namespace
{
Eigen::Matrix<double, kBezierNumCtrl, kBezierNumCtrl> jerkHessianOrder5()
{
	Eigen::Matrix<double, kBezierNumCtrl, kBezierNumCtrl> hessian;
	hessian << 720.0, -1800.0, 1200.0, 0.0, 0.0, -120.0, -1800.0, 4800.0, -3600.0, 0.0, 600.0, 0.0,
	    1200.0, -3600.0, 3600.0, -1200.0, 0.0, 0.0, 0.0, 0.0, -1200.0, 3600.0, -3600.0, 1200.0, 0.0,
	    600.0, 0.0, -3600.0, 4800.0, -1800.0, -120.0, 0.0, 0.0, 1200.0, -1800.0, 720.0;
	return hessian;
}
}  // namespace

//////////////////////////////////////////////////////////////////////////

ssc_planner::ErrorType GetBezierSplineUsingCorridor(
    const mt::vec_E<ssc_planner::SpatioTemporalSemanticCubeNd<kBezierDim>> & cubes,
    const mt::vec_E<mt::Vecf<kBezierDim>> & start_constraints,
    const mt::vec_E<mt::Vecf<kBezierDim>> & end_constraints,
    const std::vector<double> & ref_stamps, const mt::vec_E<mt::Vecf<kBezierDim>> & ref_points,
    const double & weight_proximity, BezierQpProblem * problem)
{
	if (cubes.empty()) {
		return ssc_planner::kWrongStatus;
	}

	const int numSegments = static_cast<int>(cubes.size());
	const int numOrder = kBezierNumCtrl;
	const int nDim = kBezierDim;
	const int nDervOrder = 3;  // jerk
	const int totalNumVals = nDim * numSegments * numOrder;

	// ~ Stage I: stack objective ------------------------------------------

	Eigen::SparseMatrix<double, Eigen::RowMajor> Q(totalNumVals, totalNumVals);
	Q.reserve(Eigen::VectorXi::Constant(totalNumVals, numOrder));
	{
		const auto hessian = jerkHessianOrder5();
		for (int n = 0; n < numSegments; ++n) {
			const double duration = cubes[n].tUb - cubes[n].tLb;
			for (int d = 0; d < nDim; ++d) {
				for (int j = 0; j < numOrder; ++j) {
					for (int k = 0; k < numOrder; ++k) {
						const int idx = d * numSegments * numOrder + n * numOrder + j;
						const int idy = d * numSegments * numOrder + n * numOrder + k;
						const double val = hessian(j, k) / std::pow(duration, 2 * nDervOrder - 3);
						Q.insert(idx, idy) = val;
					}
				}
			}
		}
	}

	// path points hug cost -- only position difference is considered
	Eigen::VectorXd c = Eigen::VectorXd::Zero(totalNumVals);
	Eigen::SparseMatrix<double, Eigen::RowMajor> P(totalNumVals, totalNumVals);
	P.reserve(Eigen::VectorXi::Constant(totalNumVals, numOrder));
	if (!ref_stamps.empty()) {
		const int numRefSamples = static_cast<int>(ref_stamps.size());
		for (int i = 0; i < numRefSamples; i++) {
			if (ref_stamps[i] < cubes[0].tLb || ref_stamps[i] > cubes[numSegments - 1].tUb) {
				continue;
			}
			int n = 0;
			for (; n < numSegments; n++) {
				if (cubes[n].tUb > ref_stamps[i]) {
					break;
				}
			}
			n = std::min(numSegments - 1, n);
			const double s = cubes[n].tUb - cubes[n].tLb;  // THIS segment's duration
			const double t = ref_stamps[i] - cubes[n].tLb;  // time elapsed since segment started
			for (int d = 0; d < nDim; d++) {
				for (int j = 0; j < numOrder; j++) {
					const int idx = d * numSegments * numOrder + n * numOrder + j;
					c[idx] += -2 * ref_points[i][d] * s * nchoosek(kBezierOrder, j) *
					    std::pow(t / s, j) * std::pow(1 - t / s, kBezierOrder - j);
					for (int k = 0; k < numOrder; k++) {
						const int idy = d * numSegments * numOrder + n * numOrder + k;
						P.coeffRef(idx, idy) += s * s * nchoosek(kBezierOrder, j) *
						    nchoosek(kBezierOrder, k) * std::pow(t / s, j + k) *
						    std::pow(1 - t / s, 2 * kBezierOrder - j - k);
					}
				}
			}
		}
	}
	P = P * weight_proximity;
	c = c * weight_proximity;
	Q = 2 * (Q + P);  // 0.5 * x' * Q * x -- OSQP/eigen-quadprog convention

	// ~ Stage II: stack equality constraints -------------------------------

	// continuity up to acceleration (C2), NOT jerk: a degree-5 segment (6
	// control points) only has 6 DOF, and C2 continuity at both ends of an
	// interior segment already uses all 6 -- see
	// project_docs/bezier_curve_code_explaination.md.
	const int numContinuity = 3;
	const int numConnections = numSegments - 1;
	const int numContinuityConstraints = nDim * numConnections * numContinuity;
	const int numStartEqConstraints = static_cast<int>(start_constraints.size()) * nDim;
	const int numEndEqConstraints = static_cast<int>(end_constraints.size()) * nDim;
	const int totalNumEqConstraints =
	    numContinuityConstraints + numStartEqConstraints + numEndEqConstraints;

	Eigen::SparseMatrix<double, Eigen::RowMajor> A(totalNumEqConstraints, totalNumVals);
	A.reserve(Eigen::VectorXi::Constant(totalNumEqConstraints, 2 * numOrder));
	Eigen::VectorXd b = Eigen::VectorXd::Zero(totalNumEqConstraints);

	{
		// ~ continuity constraints
		for (int n = 0; n < numConnections; n++) {
			const double durationL = cubes[n].tUb - cubes[n].tLb;
			const double durationR = cubes[n + 1].tUb - cubes[n + 1].tLb;
			for (int cIdx = 0; cIdx < numContinuity; cIdx++) {
				const double scaleL = std::pow(durationL, 1 - cIdx);
				const double scaleR = std::pow(durationR, 1 - cIdx);
				for (int d = 0; d < nDim; d++) {
					const int idx = d * numConnections * numContinuity + n * numContinuity + cIdx;
					if (cIdx == 0) {
						// ~ position end / begin
						A.insert(idx, d * numSegments * numOrder + n * numOrder + kBezierOrder) = scaleL;
						A.insert(idx, d * numSegments * numOrder + (n + 1) * numOrder + 0) = -scaleR;
					} else if (cIdx == 1) {
						// ~ velocity end / begin
						A.insert(idx, d * numSegments * numOrder + n * numOrder + kBezierOrder - 1) =
						    -scaleL;
						A.insert(idx, d * numSegments * numOrder + n * numOrder + kBezierOrder) = scaleL;
						A.insert(idx, d * numSegments * numOrder + (n + 1) * numOrder + 0) = scaleR;
						A.insert(idx, d * numSegments * numOrder + (n + 1) * numOrder + 1) = -scaleR;
					} else if (cIdx == 2) {
						// ~ acceleration end / begin
						A.insert(idx, d * numSegments * numOrder + n * numOrder + kBezierOrder - 2) =
						    scaleL;
						A.insert(idx, d * numSegments * numOrder + n * numOrder + kBezierOrder - 1) =
						    -2.0 * scaleL;
						A.insert(idx, d * numSegments * numOrder + n * numOrder + kBezierOrder) = scaleL;
						A.insert(idx, d * numSegments * numOrder + (n + 1) * numOrder + 0) = -scaleR;
						A.insert(idx, d * numSegments * numOrder + (n + 1) * numOrder + 1) = 2.0 * scaleR;
						A.insert(idx, d * numSegments * numOrder + (n + 1) * numOrder + 2) = -scaleR;
					}
				}
			}
		}
		// ~ start state constraints (position, velocity, acceleration)
		{
			const int numOrderConstraintStart = static_cast<int>(start_constraints.size());
			const double duration = cubes[0].tUb - cubes[0].tLb;
			const int n = 0;
			for (int j = 0; j < numOrderConstraintStart; j++) {
				const double scale = std::pow(duration, 1 - j);
				for (int d = 0; d < nDim; d++) {
					const int idx = numContinuityConstraints + d * numOrderConstraintStart + j;
					if (j == 0) {
						A.insert(idx, d * numSegments * numOrder + n * numOrder + 0) = scale;
					} else if (j == 1) {
						A.insert(idx, d * numSegments * numOrder + n * numOrder + 0) =
						    -1.0 * kBezierOrder * scale;
						A.insert(idx, d * numSegments * numOrder + n * numOrder + 1) =
						    1.0 * kBezierOrder * scale;
					} else if (j == 2) {
						A.insert(idx, d * numSegments * numOrder + n * numOrder + 0) =
						    1.0 * kBezierOrder * (kBezierOrder - 1) * scale;
						A.insert(idx, d * numSegments * numOrder + n * numOrder + 1) =
						    -2.0 * kBezierOrder * (kBezierOrder - 1) * scale;
						A.insert(idx, d * numSegments * numOrder + n * numOrder + 2) =
						    1.0 * kBezierOrder * (kBezierOrder - 1) * scale;
					}
					b[idx] = start_constraints[j][d];
				}
			}
		}
		// ~ end state constraints (position, velocity only -- acceleration
		// deliberately left free at the end)
		{
			const int numOrderConstraintEnd = static_cast<int>(end_constraints.size());
			const double duration = cubes[numSegments - 1].tUb - cubes[numSegments - 1].tLb;
			const int n = numSegments - 1;
			int accuEqConsIdx = numContinuityConstraints + numStartEqConstraints;
			for (int j = 0; j < numOrderConstraintEnd; j++) {
				const double scale = std::pow(duration, 1 - j);
				for (int d = 0; d < nDim; d++) {
					const int idx = accuEqConsIdx++;
					if (j == 0) {
						A.insert(idx, d * numSegments * numOrder + n * numOrder + kBezierOrder) = scale;
					} else if (j == 1) {
						A.insert(idx, d * numSegments * numOrder + n * numOrder + kBezierOrder - 1) =
						    -1.0 * kBezierOrder * scale;
						A.insert(idx, d * numSegments * numOrder + n * numOrder + kBezierOrder) =
						    1.0 * kBezierOrder * scale;
					}
					b[idx] = end_constraints[j][d];
				}
			}
		}
	}

	// ~ Stage III: stack inequality constraints (corridor bounds) ---------

	int totalNumIneq = 0;
	for (int i = 0; i < numSegments; i++) {
		totalNumIneq += static_cast<int>(cubes[i].pUb.size()) * numOrder;
		totalNumIneq += static_cast<int>(cubes[i].vUb.size()) * (numOrder - 1);
		totalNumIneq += static_cast<int>(cubes[i].aUb.size()) * (numOrder - 2);
	}
	Eigen::VectorXd lbd = Eigen::VectorXd::Zero(totalNumIneq);
	Eigen::VectorXd ubd = Eigen::VectorXd::Zero(totalNumIneq);
	Eigen::SparseMatrix<double, Eigen::RowMajor> C(totalNumIneq, totalNumVals);
	C.reserve(Eigen::VectorXi::Constant(totalNumIneq, 3));
	{
		int accuNumIneq = 0;
		for (int n = 0; n < numSegments; n++) {
			const double duration = cubes[n].tUb - cubes[n].tLb;
			for (int d = 0; d < nDim; d++) {
				// ~ enforce position bounds -- one row per control point
				{
					const double scale = std::pow(duration, 1 - 0);
					for (int j = 0; j < numOrder; j++) {
						const int idx = accuNumIneq++;
						C.insert(idx, d * numSegments * numOrder + n * numOrder + j) = scale;
						lbd[idx] = cubes[n].pLb[d];
						ubd[idx] = cubes[n].pUb[d];
					}
				}
				// ~ enforce velocity bounds -- one row per hodograph control point
				{
					const double scale = std::pow(duration, 1 - 1);
					for (int j = 0; j < numOrder - 1; j++) {
						const int idx = accuNumIneq++;
						C.insert(idx, d * numSegments * numOrder + n * numOrder + j) = -kBezierOrder * scale;
						C.insert(idx, d * numSegments * numOrder + n * numOrder + (j + 1)) =
						    kBezierOrder * scale;
						lbd[idx] = cubes[n].vLb[d];
						ubd[idx] = cubes[n].vUb[d];
					}
				}
				// ~ enforce acceleration bounds -- one row per second-derivative
				// curve's own control point
				{
					const double scale = std::pow(duration, 1 - 2);
					for (int j = 0; j < numOrder - 2; j++) {
						const int idx = accuNumIneq++;
						C.insert(idx, d * numSegments * numOrder + n * numOrder + j) =
						    kBezierOrder * (kBezierOrder - 1) * scale;
						C.insert(idx, d * numSegments * numOrder + n * numOrder + (j + 1)) =
						    -2.0 * kBezierOrder * (kBezierOrder - 1) * scale;
						C.insert(idx, d * numSegments * numOrder + n * numOrder + (j + 2)) =
						    kBezierOrder * (kBezierOrder - 1) * scale;
						lbd[idx] = cubes[n].aLb[d];
						ubd[idx] = cubes[n].aUb[d];
					}
				}
			}
		}
	}

	problem->Q = std::move(Q);
	problem->c = std::move(c);
	problem->A = std::move(A);
	problem->b = std::move(b);
	problem->C = std::move(C);
	problem->lbd = std::move(lbd);
	problem->ubd = std::move(ubd);
	problem->numSegments = numSegments;
	problem->numOrder = numOrder;
	problem->nDim = nDim;

	return ssc_planner::kSuccess;
}

//////////////////////////////////////////////////////////////////////////

namespace
{

// 1e-4 (sub-millimeter) turned out to be far tighter than this problem
// could reach in a reasonable iteration count -- live-tested: with the
// (now-fixed, see ssc_planner.yaml's own comment) short ~0.2s cube
// durations, OSQP's primal residual was still steadily shrinking at
// iteration 20000, nowhere near 1e-4 ("maximum iterations reached", not
// infeasible, confirmed via verbose output). Even 1e-2 alone proved
// borderline across runs (same live pipeline, sometimes converged,
// sometimes didn't) once real corridor data varied slightly -- so this is
// deliberately paired with max_grids_along_time's bump (real conditioning
// fix) rather than relied on alone. 5e-2 (5cm-scale) is still far tighter
// than anything physically meaningful here.
constexpr double kOsqpEpsAbs = 5e-2;
constexpr double kOsqpEpsRel = 5e-2;
// Headroom above mpl_qp_interface's own OSQP_MAX_ITERATION (20000) --
// cheap even if never needed (~23us/iteration measured live, so 50000 is
// still ~1.1s worst case for a one-shot, non-realtime solve).
constexpr int kOsqpMaxIteration = 50000;

// ~ Stage IV: solve. Stacks BezierQpProblem's Ax=b / lbd<=Cx<=ubd into
// OSQP's single l<=Ax<=u form (concatenate [A;C], equality rows get
// l=u=b) and solves via mpl_qp_interface::OSQPInterface -- solver choice
// is an implementation detail of SolveBezierSpline, not exposed in the
// header, callers only ever see the resulting BezierSpline.
bool solveBezierQp(const BezierQpProblem & problem, std::vector<double> * solution)
{
	const int totalVars = problem.nDim * problem.numSegments * problem.numOrder;
	const int numEq = static_cast<int>(problem.A.rows());
	const int numIneq = static_cast<int>(problem.C.rows());

	Eigen::MatrixXd Astack(numEq + numIneq, totalVars);
	Astack.topRows(numEq) = problem.A.toDense();
	Astack.bottomRows(numIneq) = problem.C.toDense();

	std::vector<double> l(numEq + numIneq);
	std::vector<double> u(numEq + numIneq);
	for (int i = 0; i < numEq; ++i) {
		l[i] = problem.b[i];
		u[i] = problem.b[i];
	}
	for (int i = 0; i < numIneq; ++i) {
		l[numEq + i] = problem.lbd[i];
		u[numEq + i] = problem.ubd[i];
	}

	const std::vector<double> q(problem.c.data(), problem.c.data() + problem.c.size());
	const Eigen::MatrixXd Pdense = problem.Q.toDense();

	mpl::qp_interface::OSQPInterface osqp(/*enable_warm_start=*/false, kOsqpMaxIteration,
	    kOsqpEpsAbs, kOsqpEpsRel, /*polish=*/true,
	    /*verbose=*/false);
	*solution = osqp.QPInterface::optimize(Pdense, Astack, q, l, u);
	if (!osqp.isSolved()) {
		// NOTE: deliberately not calling osqp.getStatus()/getIterationNumber()
		// here -- both are broken in mpl_qp_interface's vendored
		// OSQPInterface: getStatus() is a hardcoded stub that always
		// returns "OSQP_SOLVED" regardless of the real outcome, and
		// getIterationNumber() dereferences work_->info with no null
		// check, which segfaults because optimize() already reset work_
		// to null by this point (see osqp_interface.cpp:363). isSolved()
		// itself is safe -- it reads the separately-cached
		// latest_work_info_, not work_.
		std::cerr << "[Bezier] OSQP failed to solve -- vars: " << totalVars
		          << ", eq rows: " << numEq << ", ineq rows: " << numIneq << std::endl;
		return false;
	}
	return true;
}
}  // namespace

//////////////////////////////////////////////////////////////////////////

ssc_planner::ErrorType SolveBezierSpline(
    const mt::vec_E<ssc_planner::SpatioTemporalSemanticCubeNd<kBezierDim>> & cubes,
    const mt::vec_E<mt::Vecf<kBezierDim>> & start_constraints,
    const mt::vec_E<mt::Vecf<kBezierDim>> & end_constraints,
    const std::vector<double> & ref_stamps, const mt::vec_E<mt::Vecf<kBezierDim>> & ref_points,
    const double & weight_proximity, BezierSpline<kBezierOrder, kBezierDim> * bezier_spline)
{
	BezierQpProblem problem;
	const auto buildStatus = GetBezierSplineUsingCorridor(
	    cubes, start_constraints, end_constraints, ref_stamps, ref_points, weight_proximity,
	    &problem);
	if (buildStatus != ssc_planner::kSuccess) {
		return buildStatus;
	}

	std::vector<double> solution;
	if (!solveBezierQp(problem, &solution)) {
		// Verbose failure dump (mirrors EPSILON's own ssc_planner.cc
		// approach) -- the most common real cause is the start/end
		// boundary conditions (finite-differenced from the seed
		// trajectory) falling outside the first/last cube's own v/a
		// bounds, which makes the QP a direct contradiction (an equality
		// constraint and an inequality constraint on the same quantity
		// that can't both hold).
		std::cerr << "[Bezier] solve failed -- numSegments=" << cubes.size() << "\n";
		std::cerr << "  start pos=(" << start_constraints[0][0] << "," << start_constraints[0][1]
		          << ") vel=(" << start_constraints[1][0] << "," << start_constraints[1][1]
		          << ") acc=(" << start_constraints[2][0] << "," << start_constraints[2][1] << ")\n";
		std::cerr << "  end   pos=(" << end_constraints[0][0] << "," << end_constraints[0][1]
		          << ") vel=(" << end_constraints[1][0] << "," << end_constraints[1][1] << ")\n";
		if (!cubes.empty()) {
			const auto & c0 = cubes.front();
			std::cerr << "  cube[0]  p=[" << c0.pLb[0] << "," << c0.pUb[0] << "]x[" << c0.pLb[1]
			          << "," << c0.pUb[1] << "] v=[" << c0.vLb[0] << "," << c0.vUb[0] << "]x["
			          << c0.vLb[1] << "," << c0.vUb[1] << "] a=[" << c0.aLb[0] << "," << c0.aUb[0]
			          << "]x[" << c0.aLb[1] << "," << c0.aUb[1] << "] t=[" << c0.tLb << "," << c0.tUb
			          << "]\n";
			const auto & cN = cubes.back();
			std::cerr << "  cube[N]  p=[" << cN.pLb[0] << "," << cN.pUb[0] << "]x[" << cN.pLb[1]
			          << "," << cN.pUb[1] << "] v=[" << cN.vLb[0] << "," << cN.vUb[0] << "]x["
			          << cN.vLb[1] << "," << cN.vUb[1] << "] a=[" << cN.aLb[0] << "," << cN.aUb[0]
			          << "]x[" << cN.aLb[1] << "," << cN.aUb[1] << "] t=[" << cN.tLb << "," << cN.tUb
			          << "]\n";
		}
		return ssc_planner::kWrongStatus;
	}

	// ~ Stage V: read the flat solution vector back into per-segment
	// control points -- idx(d,n,j) matches GetBezierSplineUsingCorridor's
	// own flattening scheme exactly.
	std::vector<double> vecDomain;
	vecDomain.reserve(problem.numSegments + 1);
	vecDomain.push_back(cubes.front().tLb);
	for (int n = 0; n < problem.numSegments; n++) {
		vecDomain.push_back(cubes[n].tUb);
	}
	bezier_spline->set_vec_domain(vecDomain);
	for (int n = 0; n < problem.numSegments; n++) {
		for (int j = 0; j < problem.numOrder; j++) {
			mt::Vecf<kBezierDim> coeff;
			for (int d = 0; d < problem.nDim; d++) {
				coeff[d] = static_cast<float>(
				    solution[d * problem.numSegments * problem.numOrder + n * problem.numOrder + j]);
			}
			bezier_spline->set_coeff(n, j, coeff);
		}
	}
	return ssc_planner::kSuccess;
}

//////////////////////////////////////////////////////////////////////////
