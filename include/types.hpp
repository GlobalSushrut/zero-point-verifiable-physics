#pragma once

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <functional>
#include <cmath>
#include <complex>

namespace physics {

// Define basic types using Eigen
using Scalar = double;
using Complex = std::complex<Scalar>;
using Vector2 = Eigen::Vector2d;
using Vector3 = Eigen::Vector3d;
using VectorX = Eigen::VectorXd;
using Matrix2 = Eigen::Matrix2d;
using Matrix3 = Eigen::Matrix3d;
using MatrixX = Eigen::MatrixXd;

// Complex vectors and matrices
using ComplexVector = Eigen::VectorXcd;
using ComplexMatrix = Eigen::MatrixXcd;

// Small epsilon value for numerical stability
constexpr Scalar kEpsilon = 1e-10;

// Dimension type
enum class Dimensions : int {
    D1 = 1,
    D2 = 2,
    D3 = 3
};

// Convert Dimensions enum to int
inline int dimensionToInt(Dimensions dim) {
    return static_cast<int>(dim);
}

// Function type for field calculations
using FieldFunction = std::function<Scalar(const VectorX&)>;
using VectorFieldFunction = std::function<VectorX(const VectorX&)>;

} // namespace physics
