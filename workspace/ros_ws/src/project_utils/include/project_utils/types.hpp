/**
 * @file types.hpp
 * @author Prajwal Thakur 
 * @brief  header file containing definations for typedefs.
 * */
#pragma once
#include <Eigen/Dense>
#include <memory>
namespace project_utils::types
{

    // Scalar type (easy to change later)
    using Scalar = float;

    // Dynamic matrices
    using MatrixXX  = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
    using MatrixXXr = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

    using ArrayXX   = Eigen::Array<Scalar, Eigen::Dynamic, Eigen::Dynamic>; // Column Major
    using ArrayXXr  = Eigen::Array<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    using ArrayX =   Eigen::Array<Scalar, Eigen::Dynamic, 1>;
    using ArrayX3 = Eigen::Array<float, Eigen::Dynamic, 3>;
    using ArrayX2 = Eigen::Array<float, Eigen::Dynamic, 2>;
    // Vectors
    using VectorX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
    using RowVectorX = Eigen::Matrix<Scalar, 1, Eigen::Dynamic>;

    // Fixed size vectors (very common in robotics)
    using Vector2 = Eigen::Matrix<Scalar, 2, 1>;
    using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
    using Vector4 = Eigen::Matrix<Scalar, 4, 1>;
    using Vector6 = Eigen::Matrix<Scalar, 6, 1>;

    // Fixed matrices
    using Matrix2 = Eigen::Matrix<Scalar, 2, 2>;
    using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;
    using Matrix4 = Eigen::Matrix<Scalar, 4, 4>;


    using InputVector =  Eigen::VectorXd ;
    using StateVector = Eigen::VectorXd ;


    using  MapArrayXfRow = Eigen::Map<Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> ;

    using MapMatrixfRow =  Eigen::Map<Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>> ;

    using MatrixXfRow =  Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> ;
}

namespace mt = project_utils::types;



