#ifndef CGMRES__CONTINUATION_GMRES_HPP_
#define CGMRES__CONTINUATION_GMRES_HPP_

#include <stdexcept>


#include "cgmres/detail/macros.hpp"


namespace cgmres {
namespace detail {

template <class NLP>
class ContinuationGMRES {
public:
  static constexpr int nx = NLP::nx;
  static constexpr int nu = NLP::nu;
  static constexpr int nc = NLP::nc;
  static constexpr int dim = NLP::dim;

  ContinuationGMRES(const NLP& nlp, const double finite_difference_epsilon, 
                    const double zeta) 
    : nlp_(nlp),
      finite_difference_epsilon_(finite_difference_epsilon),
      zeta_(zeta),
      updated_solution_(Eigen::Matrix<double, dim, 1>::Zero()), 
      fonc_(Eigen::Matrix<double, dim, 1>::Zero()), 
      fonc_1_(Eigen::Matrix<double, dim, 1>::Zero()),
      fonc_2_(Eigen::Matrix<double, dim, 1>::Zero()),
      x_1_(Eigen::Matrix<double, nx, 1>::Zero()),
      dx_(Eigen::Matrix<double, nx, 1>::Zero()) {
    if (finite_difference_epsilon <= 0.0) {
      throw std::invalid_argument("[ContinuationGMRES]: 'finite_difference_epsilon' must be positive!");
    }
    if (zeta <= 0.0) {
      throw std::invalid_argument("[ContinuationGMRES]: 'zeta' must be positive!");
    }
  }

  ContinuationGMRES() = default;

  ~ContinuationGMRES() = default;

  double optError() const {
    return fonc_.template lpNorm<2>();
  }

  Eigen::Matrix<double, dim, 1> optErrorArray() const {
  return fonc_.cwiseAbs();
  }

  template <typename VectorType>
  void eval_fonc(const double t, const Eigen::MatrixBase<VectorType>& x, const Eigen::Matrix<double, dim, 1>& solution) {
    nlp_.eval_fonc_hu(t, x, solution, fonc_);
  }

  template <typename VectorType1, typename VectorType2, typename VectorType3, typename VectorType4>
  void eval_b(const double t, const Eigen::MatrixBase<VectorType1>& x,
              const Eigen::MatrixBase<VectorType2>& solution,
              const Eigen::MatrixBase<VectorType3>& solution_update,
              const Eigen::MatrixBase<VectorType4>& b_vec) {
    assert(solution.size() == dim);
    assert(solution_update.size() == dim);
    assert(b_vec.size() == dim);

    const double t1 = t + finite_difference_epsilon_;
    nlp_.ocp().eval_f(t, 0, x.derived().data(), solution.derived().data(), dx_.data());
    x_1_ = x + finite_difference_epsilon_ * dx_;
    updated_solution_ = solution + finite_difference_epsilon_ * solution_update;

    nlp_.eval_fonc_hu(t, x, solution, fonc_);
    nlp_.eval_fonc_hu(t1, x_1_, solution, fonc_1_);
    nlp_.eval_fonc_hu(t1, x_1_, updated_solution_, fonc_2_);

    CGMRES_EIGEN_CONST_CAST(VectorType4, b_vec) = (1/finite_difference_epsilon_ - zeta_) * fonc_
                                                    - fonc_2_ / finite_difference_epsilon_;
  }

  template <typename VectorType1, typename VectorType2, typename VectorType3, typename VectorType4>
  void eval_Ax(const double t, const Eigen::MatrixBase<VectorType1>& x,
               const Eigen::MatrixBase<VectorType2>& solution, 
               const Eigen::MatrixBase<VectorType3>& solution_update, 
               const Eigen::MatrixBase<VectorType4>& ax_vec) {
    assert(solution.size() == dim);
    assert(solution_update.size() == dim);
    assert(ax_vec.size() == dim);
    const double t1 = t + finite_difference_epsilon_;
    updated_solution_ = solution + finite_difference_epsilon_ * solution_update;
    nlp_.eval_fonc_hu(t1, x_1_, updated_solution_, fonc_2_);
    CGMRES_EIGEN_CONST_CAST(VectorType4, ax_vec) = (fonc_2_ - fonc_1_) / finite_difference_epsilon_;
  }

  void retrieve_dummy(Eigen::Matrix<double, dim, 1>& solution, const double min_dummy) {
    fonc_1_.setZero();
    nlp_.retrieve_dummy(solution, fonc_1_, min_dummy);
  }

  void retrieve_mu(Eigen::Matrix<double, dim, 1>& solution) {
    fonc_1_.setZero();
    nlp_.retrieve_mu(solution, fonc_1_);
  }

  decltype(auto) x() const { return nlp_.x(); }

  decltype(auto) lmd() const { return nlp_.lmd(); }

  const NLP& get_nlp() const { return nlp_; }

  void synchronize_ocp() { nlp_.synchronize_ocp(); }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  NLP nlp_;
  double finite_difference_epsilon_, zeta_; 
  Eigen::Matrix<double, dim, 1> updated_solution_, fonc_, fonc_1_, fonc_2_;
  Eigen::Matrix<double, nx, 1> x_1_, dx_;
};

} // namespace detail
} // namespace cgmres

#endif // CGMRES__CONTINUATION_GMRES_HPP_