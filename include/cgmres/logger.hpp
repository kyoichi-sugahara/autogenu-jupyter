#ifndef CGMRES__LOGGER_HPP_
#define CGMRES__LOGGER_HPP_

#include <fstream>
#include <string>

#include "cgmres/types.hpp"
#include "cgmres/timer.hpp"


namespace cgmres {

///
/// @class Logger
/// @brief Logger for MPC. 
///
class Logger {
public:
  ///
  /// @brief Constructor.
  /// @param[in] log_name Name of the log.
  ///
  explicit Logger(const std::string& log_name)
    : log_name_(log_name),
       t_log_(log_name+ "_t.log"), 
       x_log_(log_name+ "_x.log"), 
       u_log_(log_name + "_u.log"), 
       opterr_log_(log_name + "_opterr.log"),
       diff_norm_log_(log_name + "_norm_diff.log"),
       relative_standard_deviation_log_(log_name + "_relative_standard_deviation.log") {
  }

  ///
  /// @brief Destructor.
  ///
  ~Logger() {
    t_log_.close();
    x_log_.close();
    u_log_.close();
    opterr_log_.close();
    diff_norm_log_.close();
    relative_standard_deviation_log_.close();
  }

  ///
  /// @brief Save data.
  /// @param[in] t Time.
  /// @param[in] x State.
  /// @param[in] u Control input.
  /// @param[in] opterr Optimality error.
  ///
  template <typename StateVectorType, typename ControlInputVectorType>
  void save(const Scalar t, const MatrixBase<StateVectorType>& x, 
            const MatrixBase<ControlInputVectorType >& u,
            const double opterr, const double norm_diff, const double relative_standard_deviation) {
    t_log_ << t << '\n';
    x_log_ << x.transpose() << '\n';
    u_log_ << u.transpose() << '\n';
    opterr_log_ << opterr << '\n';
    diff_norm_log_ << norm_diff << '\n';
    relative_standard_deviation_log_ << relative_standard_deviation << '\n';
  }

  ///
  /// @brief Save the timing profile.
  /// @param[in] timing_profile Timing profile.
  ///
  void save(const TimingProfile& timing_profile) const {
    std::ofstream timing_log(log_name_ + "_timing_profile.log");
    timing_log << timing_profile;
    timing_log.close();
  }

private:
  std::string log_name_;
  std::ofstream t_log_, x_log_, u_log_, opterr_log_, diff_norm_log_, relative_standard_deviation_log_;
};

} // namespace cgmres

#endif // CGMRES__LOGGER_HPP_