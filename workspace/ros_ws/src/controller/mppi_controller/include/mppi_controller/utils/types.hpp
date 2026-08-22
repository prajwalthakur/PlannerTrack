#pragma once
#include <Eigen/Dense>

/**
 * \file
 * \brief Eigen type aliases (`mppi_mt::...`) used throughout `mppi_controller`
 * for batched rollout arrays -- e.g. `ArrayXX` is the `[batch_size x
 * time_steps]` shape used by \ref controller::mppi_controller::models::State
 * "State"/\ref controller::mppi_controller::models::Trajectories "Trajectories".
 */
namespace controller::mppi_controller::math_types
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




}

namespace mppi_mt = controller::mppi_controller::math_types;


