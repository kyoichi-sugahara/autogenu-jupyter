#ifndef CGMRES__MATRIXFREE_GMRES_HPP_
#define CGMRES__MATRIXFREE_GMRES_HPP_

#include <iostream>
#include <cmath>
#include <limits>
#include <stdexcept>


#include "cgmres/detail/macros.hpp"


namespace cgmres {
namespace detail {

template <typename LinearProblem, int _kmax>
class MatrixFreeGMRES {
public:
  static constexpr int dim = LinearProblem::dim;
  static constexpr int kmax = std::min(dim, _kmax);

  MatrixFreeGMRES()
    : hessenberg_mat_(Eigen::Matrix<double, kmax+1, kmax+1>::Zero()), 
      basis_mat_(Eigen::Matrix<double, dim, kmax+1>::Zero()), 
      b_vec_(Eigen::Matrix<double, dim, 1>::Zero()), 
      givens_c_vec_(Eigen::Matrix<double, kmax+1, 1>::Zero()), 
      givens_s_vec_(Eigen::Matrix<double, kmax+1, 1>::Zero()), 
      g_vec_(Eigen::Matrix<double, kmax+1, 1>::Zero()) {
    static_assert(dim > 0);
    static_assert(kmax > 0);
    static_assert(dim >= kmax);
  }

  ~MatrixFreeGMRES() = default;

  template <typename... LinearProblemArgs>
  int solve(LinearProblem& linear_problem, 
            LinearProblemArgs... linear_problem_args, 
            Eigen::Matrix<double, dim, 1>& linear_problem_solution) {
    // 1. Initialization: Initialize vectors for Givens rotation with zeros
    givens_c_vec_.setZero();
    givens_s_vec_.setZero();
    g_vec_.setZero();
    // 2. Generate initial basis for Krylov subspace
    linear_problem.eval_b(linear_problem_args..., linear_problem_solution, b_vec_);
    g_vec_.coeffRef(0) = b_vec_.template lpNorm<2>();
    basis_mat_.col(0) = b_vec_ / g_vec_.coeff(0);
    // 3. Main iteration loop
    // k: Dimension of Krylov subspace in the current iteration
    int k = 0;
    for (; k<kmax; ++k) {
      // a. Generate new basis vector: Calculate A * v_k
      linear_problem.eval_Ax(linear_problem_args..., basis_mat_.col(k), 
                            basis_mat_.col(k+1));
      // b. Arnoldi process (Gram-Schmidt orthogonalization)
      for (int j=0; j<=k; ++j) {
        // Update Hessenberg matrix element
        hessenberg_mat_.coeffRef(k, j) = basis_mat_.col(k+1).dot(basis_mat_.col(j));
        // Orthogonalize new vector against existing basis vectors
        basis_mat_.col(k+1).noalias() -= hessenberg_mat_.coeff(k, j) * basis_mat_.col(j);
      }
      // c. Normalize new basis vector
      hessenberg_mat_.coeffRef(k, k+1) = basis_mat_.col(k+1).template lpNorm<2>();
      if (std::abs(hessenberg_mat_.coeff(k, k+1)) < std::numeric_limits<double>::epsilon()) {
        // If norm is very small, end the loop
        // Numerical stability: When the norm of the new basis vector is very small,
        // it means it's numerically close to zero. This suggests that the Krylov
        // subspace cannot be expanded further.
        // Detection of linear dependence: If the norm is very small, it's likely
        // that the newly generated vector is linearly dependent on the existing
        // basis vectors. This suggests that either the algorithm has converged
        // or the problem is not well-conditioned.
        break;
      }
      else {
        // Normalize the vector
        basis_mat_.col(k+1).array() /= hessenberg_mat_.coeff(k, k+1);
      }
      // d. QR decomposition using Givens rotation
      // This process efficiently solves the least squares problem that arises in each GMRES iteration
      for (int j=0; j<k; ++j) {
        // Apply Givens rotation to the new row of Hessenberg matrix
        givensRotation(hessenberg_mat_.row(k), j);
      }
      // Calculate rotation parameters (c, s)
      const double nu = std::sqrt(hessenberg_mat_.coeff(k, k)*hessenberg_mat_.coeff(k, k)
                                  +hessenberg_mat_.coeff(k, k+1)*hessenberg_mat_.coeff(k, k+1));
      if (nu) {
        givens_c_vec_.coeffRef(k) = hessenberg_mat_.coeff(k, k) / nu;
        givens_s_vec_.coeffRef(k) = - hessenberg_mat_.coeff(k, k+1) / nu;
        // Update Hessenberg matrix
        hessenberg_mat_.coeffRef(k, k) = givens_c_vec_.coeff(k) * hessenberg_mat_.coeff(k, k) 
                                          - givens_s_vec_.coeff(k) * hessenberg_mat_.coeff(k, k+1);
        hessenberg_mat_.coeffRef(k, k+1) = 0.0;
        // Update right-hand side vector g_vec_
        givensRotation(g_vec_, k);
      }
      else {
        throw std::runtime_error("Lose orthogonality of the basis of the Krylov subspace");
      }
    }
    // 4. Calculate the solution
    // a. Solve the transformed upper triangular system by back substitution
    for (int i=k-1; i>=0; --i) {
      double tmp = g_vec_.coeff(i);
      for (int j=i+1; j<k; ++j) {
        tmp -= hessenberg_mat_.coeff(j, i) * givens_c_vec_.coeff(j);
      }
      givens_c_vec_.coeffRef(i) = tmp / hessenberg_mat_.coeff(i, i);
    }
    // b. Calculate the final solution
    for (int i=0; i<dim; ++i) {
      double tmp = 0.0;
      for (int j=0; j<k; ++j) { 
        tmp += basis_mat_.coeff(i, j) * givens_c_vec_.coeff(j);
      }
      linear_problem_solution.coeffRef(i) += tmp;
    }
    // 5. Return the number of iterations
    return k;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  Eigen::Matrix<double, kmax+1, kmax+1> hessenberg_mat_;
  Eigen::Matrix<double, dim, kmax+1> basis_mat_;
  Eigen::Matrix<double, dim, 1> b_vec_;
  Eigen::Matrix<double, kmax+1, 1> givens_c_vec_, givens_s_vec_, g_vec_;

  template <typename VectorType>
  inline void givensRotation(const Eigen::MatrixBase<VectorType>& column_vec, 
                             const int i_column) const {
    const double tmp1 = givens_c_vec_.coeff(i_column) * column_vec.coeff(i_column) 
                        - givens_s_vec_.coeff(i_column) * column_vec.coeff(i_column+1);
    const double tmp2 = givens_s_vec_.coeff(i_column) * column_vec.coeff(i_column) 
                        + givens_c_vec_.coeff(i_column) * column_vec.coeff(i_column+1);
    CGMRES_EIGEN_CONST_CAST(VectorType, column_vec).coeffRef(i_column) = tmp1;
    CGMRES_EIGEN_CONST_CAST(VectorType, column_vec).coeffRef(i_column+1) = tmp2;
  }

};

} // namespace detail
} // namespace cgmres

#endif // CGMRES__MATRIXFREE_GMRES_HPP_