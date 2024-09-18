#ifndef CGMRES__CONTROL_INPUT_BOUNDS_SHOOTING_HPP_
#define CGMRES__CONTROL_INPUT_BOUNDS_SHOOTING_HPP_

#include <array>


#include "cgmres/detail/macros.hpp"
#include "cgmres/detail/control_input_bounds.hpp"

namespace cgmres {
namespace detail {
namespace ubounds {

template <typename OCP, int N>
void eval_fonc_hu(const OCP& ocp, const Eigen::Matrix<double, OCP::nuc*N, 1>& solution,
                  const std::array<Eigen::Matrix<double, OCP::nub, 1>, N>& dummy, 
                  const std::array<Eigen::Matrix<double, OCP::nub, 1>, N>& mu,
                  Eigen::Matrix<double, OCP::nuc*N, 1>& fonc_hu) {
  if constexpr (OCP::nub > 0) {
    constexpr int nuc = OCP::nuc;
    for (size_t i=0; i<N; ++i) {
      eval_hu(ocp, solution.template segment<nuc>(nuc*i), dummy[i], mu[i],
              fonc_hu.template segment<nuc>(nuc*i));
    }
  }
}

template <typename OCP, int N>
void eval_fonc_hdummy(const OCP& ocp, const Eigen::Matrix<double, OCP::nuc*N, 1>& solution,
                      const std::array<Eigen::Matrix<double, OCP::nub, 1>, N>& dummy, 
                      const std::array<Eigen::Matrix<double, OCP::nub, 1>, N>& mu,
                      std::array<Eigen::Matrix<double, OCP::nub, 1>, N>& fonc_hdummy) {
  if constexpr (OCP::nub > 0) {
    constexpr int nuc = OCP::nuc;
    for (size_t i=0; i<N; ++i) {
      eval_hdummy(ocp, solution.template segment<nuc>(nuc*i), dummy[i], mu[i],
                  fonc_hdummy[i]);
    }
  }
}

template <typename OCP, int N>
void eval_fonc_hmu(const OCP& ocp, const Eigen::Matrix<double, OCP::nuc*N, 1>& solution,
                   const std::array<Eigen::Matrix<double, OCP::nub, 1>, N>& dummy, 
                   const std::array<Eigen::Matrix<double, OCP::nub, 1>, N>& mu,
                   std::array<Eigen::Matrix<double, OCP::nub, 1>, N>& fonc_hmu) {
  if constexpr (OCP::nub > 0) {
    constexpr int nuc = OCP::nuc;
    for (size_t i=0; i<N; ++i) {
      eval_hmu(ocp, solution.template segment<nuc>(nuc*i), dummy[i], mu[i],
               fonc_hmu[i]);
    }
  }
}

template <typename OCP, int N>
void multiply_hdummy_inv(const std::array<Eigen::Matrix<double, OCP::nub, 1>, N>& dummy, 
                         const std::array<Eigen::Matrix<double, OCP::nub, 1>, N>& mu,
                         const std::array<Eigen::Matrix<double, OCP::nub, 1>, N>& fonc_hdummy,
                         const std::array<Eigen::Matrix<double, OCP::nub, 1>, N>& fonc_hmu,
                         std::array<Eigen::Matrix<double, OCP::nub, 1>, N>& fonc_hdummy_inv) {
  if constexpr (OCP::nub > 0) {
    for (size_t i=0; i<N; ++i) {
      multiply_hdummy_inv(dummy[i], mu[i], fonc_hdummy[i], fonc_hmu[i],
                          fonc_hdummy_inv[i]);
    }
  }
}

template <typename OCP, int N>
void multiply_hmu_inv(const std::array<Eigen::Matrix<double, OCP::nub, 1>, N>& dummy, 
                      const std::array<Eigen::Matrix<double, OCP::nub, 1>, N>& mu,
                      const std::array<Eigen::Matrix<double, OCP::nub, 1>, N>& fonc_hdummy,
                      const std::array<Eigen::Matrix<double, OCP::nub, 1>, N>& fonc_hmu,
                      const std::array<Eigen::Matrix<double, OCP::nub, 1>, N>& fonc_hdummy_inv,
                      std::array<Eigen::Matrix<double, OCP::nub, 1>, N>& fonc_hmu_inv) {
  if constexpr (OCP::nub > 0) {
    for (size_t i=0; i<N; ++i) {
      multiply_hmu_inv(dummy[i], mu[i], fonc_hdummy[i], fonc_hmu[i],
                       fonc_hdummy_inv[i], fonc_hmu_inv[i]);
    }
  }
}

template <typename OCP, int N>
void retrieve_dummy_update(const OCP& ocp,
                          const Eigen::Matrix<double, OCP::nuc*N, 1>& solution,
                          const std::array<Eigen::Matrix<double, OCP::nub, 1>, N>& dummy, 
                          const std::array<Eigen::Matrix<double, OCP::nub, 1>, N>& mu,
                          const Eigen::Matrix<double, OCP::nuc*N, 1>& solution_update,
                          std::array<Eigen::Matrix<double, OCP::nub, 1>, N>& dummy_update) {
  if constexpr (OCP::nub > 0) {
    constexpr int nuc = OCP::nuc;
    for (size_t i=0; i<N; ++i) {
      retrieve_dummy_update(ocp, solution.template segment<nuc>(nuc*i), dummy[i], mu[i], 
                           solution_update.template segment<nuc>(nuc*i), dummy_update[i]);
    }
  } 
}

template <typename OCP, int N>
void retrieve_mu_update(const OCP& ocp,
                       const Eigen::Matrix<double, OCP::nuc*N, 1>& solution,
                       const std::array<Eigen::Matrix<double, OCP::nub, 1>, N>& dummy, 
                       const std::array<Eigen::Matrix<double, OCP::nub, 1>, N>& mu,
                       const Eigen::Matrix<double, OCP::nuc*N, 1>& solution_update,
                       std::array<Eigen::Matrix<double, OCP::nub, 1>, N>& mu_update) {
  if constexpr (OCP::nub > 0) {
    constexpr int nuc = OCP::nuc;
    for (size_t i=0; i<N; ++i) {
      retrieve_mu_update(ocp, solution.template segment<nuc>(nuc*i), dummy[i], mu[i], 
                        solution_update.template segment<nuc>(nuc*i), mu_update[i]);
    }
  } 
}

template <typename OCP, int N>
void clip_dummy(std::array<Eigen::Matrix<double, OCP::nub, 1>, N>& dummy, const double min) {
  if constexpr (OCP::nub > 0) {
    for (size_t i=0; i<N; ++i) {
      clip_dummy(dummy[i], min);
    }
  } 
}

} // namespace ubounds
} // namespace detail
} // namespace cgmres

#endif // CGMRES__CONTROL_INPUT_BOUNDS_SHOOTING_HPP_