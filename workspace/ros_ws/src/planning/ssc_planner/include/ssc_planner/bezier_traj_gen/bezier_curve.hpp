/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#pragma once
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include <vector>
#include "yaml-cpp/yaml.h"
#include "project_utils/types.hpp"
#include "ssc_planner/bezier_traj_gen/bezier.h"
#include "ssc_planner/ssc_map.hpp"
typedef Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::ColMajor>> MapMatrixCol;
typedef Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> MatrixXdRow;
using AXXf = Eigen::ArrayXXf;
using AXf = Eigen::ArrayXf;       // column vector
     

// Bezier order/dimension hardcoded for now -- see project decision not to
// generalize order for the first pass (project_docs/bezier_curve_code_explaination.md).
constexpr int kBezierOrder = 5;
constexpr int kBezierNumCtrl = kBezierOrder + 1;  // 6 control points per segment
constexpr int kBezierDim = 2;                     // s, d

// The finalized QP problem (Stages I-III) for one corridor's piecewise
// Bezier fit. Stage IV (solve) and Stage V (readout into per-segment
// control points) are deliberately not part of this struct yet -- TODO,
// see GetBezierSplineUsingCorridor's own comment.
struct BezierQpProblem
{
	Eigen::SparseMatrix<double, Eigen::RowMajor> Q;  // quadratic cost, 0.5*x'Qx
	Eigen::VectorXd c;                                // linear cost, c'x
	Eigen::SparseMatrix<double, Eigen::RowMajor> A;  // equality constraints, Ax=b
	Eigen::VectorXd b;
	Eigen::SparseMatrix<double, Eigen::RowMajor> C;  // inequality constraints, lbd<=Cx<=ubd
	Eigen::VectorXd lbd, ubd;
	int numSegments{0};
	int numOrder{0};
	int nDim{0};
};


ssc_planner::ErrorType GetBezierSplineUsingCorridor(
    const mt::vec_E<ssc_planner::SpatioTemporalSemanticCubeNd<kBezierDim>> & cubes,
    const mt::vec_E<mt::Vecf<kBezierDim>> & start_constraints,
    const mt::vec_E<mt::Vecf<kBezierDim>> & end_constraints,
    const std::vector<double> & ref_stamps, const mt::vec_E<mt::Vecf<kBezierDim>> & ref_points,
    const double & weight_proximity, BezierQpProblem * problem);


ssc_planner::ErrorType SolveBezierSpline(
    const mt::vec_E<ssc_planner::SpatioTemporalSemanticCubeNd<kBezierDim>> & cubes,
    const mt::vec_E<mt::Vecf<kBezierDim>> & start_constraints,
    const mt::vec_E<mt::Vecf<kBezierDim>> & end_constraints,
    const std::vector<double> & ref_stamps, const mt::vec_E<mt::Vecf<kBezierDim>> & ref_points,
    const double & weight_proximity, BezierSpline<kBezierOrder, kBezierDim> * bezier_spline);

