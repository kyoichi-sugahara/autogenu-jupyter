// This file was automatically generated by autogenu-jupyter (https://github.com/ohtsukalab/autogenu-jupyter). 
// The autogenu-jupyter copyright holders make no ownership claim of its contents. 

#ifndef CGMRES__OCP_GENERATED_CODE_HPP_ 
#define CGMRES__OCP_GENERATED_CODE_HPP_ 
 
#define _USE_MATH_DEFINES

#include <cmath>
#include <array>
#include <iostream>

#include "cgmres/types.hpp"
#include "cgmres/detail/macros.hpp"

namespace cgmres {

/// 
/// @class OCP_generated_code
/// @brief Definition of the optimal control problem (OCP) of generated_code.
/// 
class OCP_generated_code { 
public:
  ///
  /// @brief Dimension of the state. 
  ///
  static constexpr int nx = 4;
 
  ///
  /// @brief Dimension of the control input. 
  ///
  static constexpr int nu = 1;
 
  ///
  /// @brief Dimension of the equality constraints. 
  ///
  static constexpr int nc = 0;
 
  ///
  /// @brief Dimension of the Fischer-Burmeister function (already counded in nc). 
  ///
  static constexpr int nh = 0;
 
  ///
  /// @brief Dimension of the concatenation of the control input and equality constraints. 
  ///
  static constexpr int nuc = nu + nc;

  ///
  /// @brief Dimension of the bound constraints on the control input. 
  ///
  static constexpr int nub = 0;

  double m_c = 2;
  double m_p = 0.2;
  double l = 0.5;
  double g = 9.80665;

  std::array<double, 4> q = {2.5, 10, 0.01, 0.01};
  std::array<double, 4> q_terminal = {2.5, 10, 0.01, 0.01};
  std::array<double, 4> x_ref = {0, 0, 1, 0};
  std::array<double, 1> r = {1};

  void disp(std::ostream& os) const {
    os << "OCP_generated_code:" << std::endl;
    os << "  nx:  " << nx << std::endl;
    os << "  nu:  " << nu << std::endl;
    os << "  nc:  " << nc << std::endl;
    os << "  nh:  " << nh << std::endl;
    os << "  nuc: " << nuc << std::endl;
    os << "  nub: " << nub << std::endl;
    os << std::endl;
    os << "  m_c: " << m_c << std::endl;
    os << "  m_p: " << m_p << std::endl;
    os << "  l: " << l << std::endl;
    os << "  g: " << g << std::endl;
    os << std::endl;
    Eigen::IOFormat fmt(4, 0, ", ", "", "[", "]");
    Eigen::IOFormat intfmt(1, 0, ", ", "", "[", "]");
    os << "  q: " << Map<const VectorX>(q.data(), q.size()).transpose().format(fmt) << std::endl;
    os << "  q_terminal: " << Map<const VectorX>(q_terminal.data(), q_terminal.size()).transpose().format(fmt) << std::endl;
    os << "  x_ref: " << Map<const VectorX>(x_ref.data(), x_ref.size()).transpose().format(fmt) << std::endl;
    os << "  r: " << Map<const VectorX>(r.data(), r.size()).transpose().format(fmt) << std::endl;
  }

  friend std::ostream& operator<<(std::ostream& os, const OCP_generated_code& ocp) { 
    ocp.disp(os);
    return os;
  }


  ///
  /// @brief Synchrozies the internal parameters of this OCP with the external references.
  /// This method is called at the beginning of each MPC update.
  ///
  void synchronize() {
  }

  ///
  /// @brief Computes the state equation dx = f(t, x, u).
  /// @param[in] t Time.
  /// @param[in] x State.
  /// @param[in] u Control input.
  /// @param[out] dx Evaluated value of the state equation.
  /// @remark This method is intended to be used inside of the cgmres solvers and does not check size of each argument. 
  /// Use the overloaded method if you call this outside of the cgmres solvers. 
  ///
  void eval_f(const double t, const double* x, const double* u, 
              double* dx) const {
    const double x0 = sin(x[1]);
    const double x1 = 1.0/(m_c + m_p*pow(x0, 2));
    const double x2 = cos(x[1]);
    const double x3 = l*pow(x[1], 2);
    const double x4 = m_p*x0;
    dx[0] = x[2];
    dx[1] = x[3];
    dx[2] = x1*(u[0] + x4*(g*x2 + x3));
    dx[3] = x1*(-g*x0*(m_c + m_p) - u[0]*x2 - x2*x3*x4)/l;
 
  }

  ///
  /// @brief Computes the partial derivative of terminal cost with respect to state, 
  /// i.e., phix = dphi/dx(t, x).
  /// @param[in] t Time.
  /// @param[in] x State.
  /// @param[out] phix Evaluated value of the partial derivative of terminal cost.
  /// @remark This method is intended to be used inside of the cgmres solvers and does not check size of each argument. 
  /// Use the overloaded method if you call this outside of the cgmres solvers. 
  ///
  void eval_phix(const double t, const double* x, double* phix) const {
    phix[0] = (1.0/2.0)*q_terminal[0]*(2*x[0] - 2*x_ref[0]);
    phix[1] = (1.0/2.0)*q_terminal[1]*(2*x[1] - 2*x_ref[1]);
    phix[2] = (1.0/2.0)*q_terminal[2]*(2*x[2] - 2*x_ref[2]);
    phix[3] = (1.0/2.0)*q_terminal[3]*(2*x[3] - 2*x_ref[3]);
 
  }

  ///
  /// @brief Computes the partial derivative of the Hamiltonian with respect to state, 
  /// i.e., hx = dH/dx(t, x, u, lmd).
  /// @param[in] t Time.
  /// @param[in] x State.
  /// @param[in] u Concatenatin of the control input and Lagrange multiplier with respect to the equality constraints. 
  /// @param[in] lmd Costate. 
  /// @param[out] hx Evaluated value of the partial derivative of the Hamiltonian.
  /// @remark This method is intended to be used inside of the cgmres solvers and does not check size of each argument. 
  /// Use the overloaded method if you call this outside of the cgmres solvers. 
  ///
  void eval_hx(const double t, const double* x, const double* u, 
               const double* lmd, double* hx) const {
    const double x0 = 2*x[1];
    const double x1 = sin(x[1]);
    const double x2 = cos(x[1]);
    const double x3 = g*x2;
    const double x4 = l*pow(x[1], 2);
    const double x5 = m_p*(x3 + x4);
    const double x6 = m_p*pow(x1, 2);
    const double x7 = m_c + x6;
    const double x8 = m_p*x1;
    const double x9 = x2*x8;
    const double x10 = 2*x9/pow(x7, 2);
    const double x11 = 1.0/x7;
    const double x12 = l*x0;
    const double x13 = g*x1;
    const double x14 = m_c + m_p;
    const double x15 = lmd[3]/l;
    hx[0] = (1.0/2.0)*q[0]*(2*x[0] - 2*x_ref[0]);
    hx[1] = -lmd[2]*x10*(u[0] + x1*x5) + lmd[2]*x11*(x2*x5 + x8*(x12 - x13)) + (1.0/2.0)*q[1]*(x0 - 2*x_ref[1]) - x10*x15*(-u[0]*x2 - x13*x14 - x4*x9) + x11*x15*(-m_p*pow(x2, 2)*x4 + u[0]*x1 - x12*x9 - x14*x3 + x4*x6);
    hx[2] = lmd[0] + (1.0/2.0)*q[2]*(2*x[2] - 2*x_ref[2]);
    hx[3] = lmd[1] + (1.0/2.0)*q[3]*(2*x[3] - 2*x_ref[3]);
 
  }

  ///
  /// @brief Computes the partial derivative of the Hamiltonian with respect to control input and the equality constraints, 
  /// i.e., hu = dH/du(t, x, u, lmd).
  /// @param[in] t Time.
  /// @param[in] x State.
  /// @param[in] u Concatenatin of the control input and Lagrange multiplier with respect to the equality constraints. 
  /// @param[in] lmd Costate. 
  /// @param[out] hu Evaluated value of the partial derivative of the Hamiltonian.
  /// @remark This method is intended to be used inside of the cgmres solvers and does not check size of each argument. 
  /// Use the overloaded method if you call this outside of the cgmres solvers. 
  ///
  void eval_hu(const double t, const double* x, const double* u, 
               const double* lmd, double* hu) const {
    const double x0 = 1.0/(m_c + m_p*pow(sin(x[1]), 2));
    hu[0] = lmd[2]*x0 + r[0]*u[0] - lmd[3]*x0*cos(x[1])/l;
 
  }

  ///
  /// @brief Computes the state equation dx = f(t, x, u).
  /// @param[in] t Time.
  /// @param[in] x State. Size must be nx.
  /// @param[in] u Control input. Size must be nu.
  /// @param[out] dx Evaluated value of the state equation. Size must be nx.
  ///
  template <typename VectorType1, typename VectorType2, typename VectorType3>
  void eval_f(const double t, const MatrixBase<VectorType1>& x, 
              const MatrixBase<VectorType2>& u, 
              const MatrixBase<VectorType3>& dx) const {
    if (x.size() != nx) {
      throw std::invalid_argument("[OCP]: x.size() must be " + std::to_string(nx));
    }
    if (u.size() != nu) {
      throw std::invalid_argument("[OCP]: u.size() must be " + std::to_string(nu));
    }
    if (dx.size() != nx) {
      throw std::invalid_argument("[OCP]: dx.size() must be " + std::to_string(nx));
    }
    eval_f(t, x.derived().data(), u.derived().data(), CGMRES_EIGEN_CONST_CAST(VectorType3, dx).data());
  }

  ///
  /// @brief Computes the partial derivative of terminal cost with respect to state, 
  /// i.e., phix = dphi/dx(t, x).
  /// @param[in] t Time.
  /// @param[in] x State. Size must be nx.
  /// @param[out] phix Evaluated value of the partial derivative of terminal cost. Size must be nx.
  ///
  template <typename VectorType1, typename VectorType2>
  void eval_phix(const double t, const MatrixBase<VectorType1>& x, 
                 const MatrixBase<VectorType2>& phix) const {
    if (x.size() != nx) {
      throw std::invalid_argument("[OCP]: x.size() must be " + std::to_string(nx));
    }
    if (phix.size() != nx) {
      throw std::invalid_argument("[OCP]: phix.size() must be " + std::to_string(nx));
    }
    eval_phix(t, x.derived().data(), CGMRES_EIGEN_CONST_CAST(VectorType2, phix).data());
  }

  ///
  /// @brief Computes the partial derivative of the Hamiltonian with respect to the state, 
  /// i.e., hx = dH/dx(t, x, u, lmd).
  /// @param[in] t Time.
  /// @param[in] x State. Size must be nx.
  /// @param[in] uc Concatenatin of the control input and Lagrange multiplier with respect to the equality constraints. Size must be nuc. 
  /// @param[in] lmd Costate.  Size must be nx.
  /// @param[out] hx Evaluated value of the partial derivative of the Hamiltonian. Size must be nx.
  ///
  template <typename VectorType1, typename VectorType2, typename VectorType3, typename VectorType4>
  void eval_hx(const double t, const MatrixBase<VectorType1>& x, 
               const MatrixBase<VectorType2>& uc, 
               const MatrixBase<VectorType3>& lmd, 
               const MatrixBase<VectorType4>& hx) const {
    if (x.size() != nx) {
      throw std::invalid_argument("[OCP]: x.size() must be " + std::to_string(nx));
    }
    if (uc.size() != nuc) {
      throw std::invalid_argument("[OCP]: uc.size() must be " + std::to_string(nuc));
    }
    if (lmd.size() != nx) {
      throw std::invalid_argument("[OCP]: lmd.size() must be " + std::to_string(nx));
    }
    if (hx.size() != nuc) {
      throw std::invalid_argument("[OCP]: hx.size() must be " + std::to_string(nx));
    }
    eval_hx(t, x.derived().data(), uc.derived().data(), lmd.derived().data(), CGMRES_EIGEN_CONST_CAST(VectorType4, hx).data());
  }

  ///
  /// @brief Computes the partial derivative of the Hamiltonian with respect to control input and the equality constraints, 
  /// i.e., hu = dH/du(t, x, u, lmd).
  /// @param[in] t Time.
  /// @param[in] x State. Size must be nx.
  /// @param[in] uc Concatenatin of the control input and Lagrange multiplier with respect to the equality constraints. Size must be nuc. 
  /// @param[in] lmd Costate. Size must be nx. 
  /// @param[out] hu Evaluated value of the partial derivative of the Hamiltonian. Size must be nuc.
  ///
  template <typename VectorType1, typename VectorType2, typename VectorType3, typename VectorType4>
  void eval_hu(const double t, const MatrixBase<VectorType1>& x, 
               const MatrixBase<VectorType2>& uc, 
               const MatrixBase<VectorType3>& lmd, 
               const MatrixBase<VectorType4>& hu) const {
    if (x.size() != nx) {
      throw std::invalid_argument("[OCP]: x.size() must be " + std::to_string(nx));
    }
    if (uc.size() != nuc) {
      throw std::invalid_argument("[OCP]: uc.size() must be " + std::to_string(nuc));
    }
    if (lmd.size() != nx) {
      throw std::invalid_argument("[OCP]: lmd.size() must be " + std::to_string(nx));
    }
    if (hu.size() != nuc) {
      throw std::invalid_argument("[OCP]: hu.size() must be " + std::to_string(nuc));
    }
    eval_hu(t, x.derived().data(), uc.derived().data(), lmd.derived().data(), CGMRES_EIGEN_CONST_CAST(VectorType4, hu).data());
  }

};

} // namespace cgmres

#endif // CGMRES_OCP_HPP_
