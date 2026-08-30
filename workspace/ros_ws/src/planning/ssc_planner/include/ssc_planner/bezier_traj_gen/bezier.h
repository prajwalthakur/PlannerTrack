/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */

#pragma once

#include "project_utils/types.hpp"
#include "ssc_planner/ssc_grid.hpp"

#include <eigen3/Eigen/Dense>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <vector>

//////////////////////////////////////////////////////////////////////////
template <int N_DEG>
class BezierUtils;

/**
 * @brief Bezier spline class -- one piecewise curve made of
 * num_segments() Bezier segments, each degree N_DEG, dimension N_DIM.
 * The j-th segment's evaluation is given by
 * B_j(t) = duration_j * sum_{i=0}^{N_DEG} c_j^i * b_{N_DEG}^i((t-T_j)/duration_j)
 * (duration_j is the time-scaling factor of the j-th segment -- see
 * project_docs/bezier_curve_code_explaination.md's Stage I section for why
 * the scale is applied this way; this matches
 * GetBezierSplineUsingCorridor's own convention in bezier_curve.cpp).
 */
template <int N_DEG, int N_DIM>
class BezierSpline
{
  public:
	BezierSpline() {}

	/**
	 * @brief Set the vector domain: real-time breakpoints between segments,
	 * size num_segments()+1.
	 */
	void set_vec_domain(const std::vector<double> & vec_domain)
	{
		assert(vec_domain.size() > 1);
		vec_domain_ = vec_domain;
		ctrl_pts_.resize(vec_domain.size() - 1);
		for (int j = 0; j < static_cast<int>(ctrl_pts_.size()); j++) {
			ctrl_pts_[j].setZero();
		}
	}

	void set_coeff(const int segment_idx, const int ctrl_pt_index, const mt::Vecf<N_DIM> & coeff)
	{
		ctrl_pts_[segment_idx].row(ctrl_pt_index) = coeff;
	}

	void set_ctrl_pts(const mt::vec_E<Eigen::Matrix<float, N_DEG + 1, N_DIM>> & pts)
	{
		ctrl_pts_ = pts;
	}

	/** @brief Number of segments in the spline */
	int num_segments() const { return static_cast<int>(ctrl_pts_.size()); }

	/** @brief Real-time breakpoints between segments */
	std::vector<double> vec_domain() const { return vec_domain_; }

	mt::vec_E<Eigen::Matrix<float, N_DEG + 1, N_DIM>> ctrl_pts() const { return ctrl_pts_; }

	/** @brief Start of the parameterization (real time) */
	double begin() const
	{
		if (vec_domain_.empty()) return 0.0;
		return vec_domain_.front();
	}

	/** @brief End of the parameterization (real time) */
	double end() const
	{
		if (vec_domain_.empty()) return 0.0;
		return vec_domain_.back();
	}

	/**
	 * @brief Evaluate the spline (or a derivative of it) at real time s.
	 * @param s real time, should be within vec_domain()'s range
	 * @param d derivative order to evaluate (0 = position)
	 * @param ret output value
	 * @note s below vec_domain()'s start is rejected (kIllegalInput); s
	 * past the end is clamped to the last segment's tau=1 by
	 * GetBezierBasis's own internal [0,1] clamp.
	 */
	ssc_planner::ErrorType evaluate(const double s, const int d, mt::Vecf<N_DIM> * ret) const
	{
		const int numPts = static_cast<int>(vec_domain_.size());
		if (numPts < 1) return ssc_planner::kIllegalInput;
		if (s < vec_domain_[0]) return ssc_planner::kIllegalInput;

		const auto it = std::lower_bound(vec_domain_.begin(), vec_domain_.end(), s);
		const int idx =
		    std::min(std::max(static_cast<int>(it - vec_domain_.begin()) - 1, 0), numPts - 2);
		const double duration = vec_domain_[idx + 1] - vec_domain_[idx];
		const double normalizedS = (s - vec_domain_[idx]) / duration;

		const Eigen::Matrix<double, N_DEG + 1, 1> basis =
		    BezierUtils<N_DEG>::GetBezierBasis(d, normalizedS);
		const Eigen::Matrix<float, N_DEG + 1, 1> basisF = basis.template cast<float>();
		const float scale = static_cast<float>(std::pow(duration, 1 - d));
		*ret = (scale * basisF.transpose() * ctrl_pts_[idx]).transpose();
		return ssc_planner::kSuccess;
	}

	void print() const
	{
		printf("Bezier control points.\n");
		for (int j = 0; j < static_cast<int>(ctrl_pts_.size()); j++) {
			printf("segment %d -->.\n", j);
			std::cout << ctrl_pts_[j] << std::endl;
		}
	}

  private:
	mt::vec_E<Eigen::Matrix<float, N_DEG + 1, N_DIM>> ctrl_pts_;
	std::vector<double> vec_domain_;
};

//////////////////////////////////////////////////////////////////////////

/**
 * @brief Bezier basis / Hessian utilities. Hardcoded per-degree (only
 * N_DEG=5 implemented, matching this project's decision not to generalize
 * Bezier order for the first pass -- see
 * project_docs/bezier_curve_code_explaination.md).
 */
template <int N_DEG>
class BezierUtils
{
  public:
	/**
	 * @brief Jerk-cost Hessian for the non-scaled (tau-domain) Bezier
	 * basis -- H(j,k) = integral_0^1 B_j'''(tau)*B_k'''(tau) dtau. Same
	 * literal values as bezier_curve.cpp's own jerkHessianOrder5() (which
	 * has its own copy rather than calling this -- see that file's
	 * comment); kept here too so BezierUtils<5> is independently usable.
	 */
	static Eigen::Matrix<double, N_DEG + 1, N_DEG + 1> GetBezierHessianMat(int derivative_degree)
	{
		Eigen::Matrix<double, N_DEG + 1, N_DEG + 1> hessian;
		switch (N_DEG) {
			case 5: {
				if (derivative_degree == 3) {
					// jerk hessian
					hessian << 720.0, -1800.0, 1200.0, 0.0, 0.0, -120.0, -1800.0, 4800.0, -3600.0,
					    0.0, 600.0, 0.0, 1200.0, -3600.0, 3600.0, -1200.0, 0.0, 0.0, 0.0, 0.0,
					    -1200.0, 3600.0, -3600.0, 1200.0, 0.0, 600.0, 0.0, -3600.0, 4800.0,
					    -1800.0, -120.0, 0.0, 0.0, 1200.0, -1800.0, 720.0;
				} else {
					assert(
					    false &&
					    "GetBezierHessianMat: only derivative_degree=3 implemented for N_DEG=5");
				}
				break;
			}
			default:
				assert(false && "GetBezierHessianMat: only N_DEG=5 implemented");
		}
		return hessian;
	}

	/**
	 * @brief Bernstein basis (or its derivative) for the non-scaled control
	 * points [c_0, c_1, ...], degree N_DEG, evaluated at normalized
	 * parameter t in [0,1].
	 */
	static Eigen::Matrix<double, N_DEG + 1, 1> GetBezierBasis(int derivative_degree, double t)
	{
		t = std::max(std::min(1.0, t), 0.0);
		Eigen::Matrix<double, N_DEG + 1, 1> basis;
		switch (N_DEG) {
			case 5:
				if (derivative_degree == 0) {
					basis << -pow(t - 1, 5), 5 * t * pow(t - 1, 4), -10 * pow(t, 2) * pow(t - 1, 3),
					    10 * pow(t, 3) * pow(t - 1, 2), -5 * pow(t, 4) * (t - 1), pow(t, 5);
				} else if (derivative_degree == 1) {
					basis << -5 * pow(t - 1, 4), 20 * t * pow(t - 1, 3) + 5 * pow(t - 1, 4),
					    -20 * t * pow(t - 1, 3) - 30 * pow(t, 2) * pow(t - 1, 2),
					    10 * pow(t, 3) * (2 * t - 2) + 30 * pow(t, 2) * pow(t - 1, 2),
					    -20 * pow(t, 3) * (t - 1) - 5 * pow(t, 4), 5 * pow(t, 4);
				} else if (derivative_degree == 2) {
					basis << -20 * pow(t - 1, 3), 60 * t * pow(t - 1, 2) + 40 * pow(t - 1, 3),
					    -120 * t * pow(t - 1, 2) - 20 * pow(t - 1, 3) - 30 * t * t * (2 * t - 2),
					    60 * t * pow(t - 1, 2) + 60 * t * t * (2 * t - 2) + 20 * t * t * t,
					    -60 * t * t * (t - 1) - 40 * t * t * t, 20 * t * t * t;
				} else if (derivative_degree == 3) {
					basis << -60 * pow(t - 1, 2), 60 * t * (2 * t - 2) + 180 * pow(t - 1, 2),
					    -180 * t * (2 * t - 2) - 180 * pow(t - 1, 2) - 60 * t * t,
					    180 * t * (2 * t - 2) + 60 * pow(t - 1, 2) + 180 * t * t,
					    -120 * t * (t - 1) - 180 * t * t, 60 * t * t;
				} else {
					assert(false && "GetBezierBasis: derivative_degree must be 0..3 for N_DEG=5");
				}
				break;
			default:
				printf("N_DEG %d, derivative_degree %d.\n", N_DEG, derivative_degree);
				assert(false && "GetBezierBasis: only N_DEG=5 implemented");
				break;
		}
		return basis;
	}
};
