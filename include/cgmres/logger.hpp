#ifndef CGMRES__LOGGER_HPP_
#define CGMRES__LOGGER_HPP_

#include <sys/stat.h>
#include <fstream>
#include <string>
#include <sstream>
#include <unistd.h>
#include <sys/time.h>

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
explicit Logger(const std::string& log_dir) {
    // Check if the specified directory exists
    struct stat info;
    if (stat(log_dir.c_str(), &info) != 0 || !(info.st_mode & S_IFDIR)) {
        // Directory does not exist
        std::stringstream ss;
        ss << "Error: The specified directory does not exist: " << log_dir;
        throw std::runtime_error(ss.str());
    }

    // Get current process ID
    pid_t pid = getpid();

    // Get current timestamp
    struct timeval tv;
    gettimeofday(&tv, NULL);
    int64_t timestamp = static_cast<int64_t>(tv.tv_sec) * 1000000 + tv.tv_usec;

    // Create number string
    std::stringstream ss;
    ss << "_" << pid << "_" << timestamp;
    std::string number_string = ss.str();

    // Create directory name
    std::string directory = log_dir + "cgmres_debug" + number_string;

    // Create directory
    mkdir(directory.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    // Create log file names
    std::string t_log_name = directory + "/t.log";
    std::string x_log_name = directory + "/x.log";
    std::string u_log_name = directory + "/u.log";
    std::string uopt_log_name = directory + "/uopt.log";
    std::string initial_solution_log_name = directory + "/initial_solution.log";
    std::string updated_solution_log_name = directory + "/updated_solution.log";
    std::string opterr_log_name = directory + "/opterr.log";
    std::string iter_log_name = directory + "/iter.log";

    // Open log files
    t_log_.open(t_log_name);
    x_log_.open(x_log_name);
    u_log_.open(u_log_name);
    uopt_log_.open(uopt_log_name);
    initial_solution_log_.open(initial_solution_log_name);
    updated_solution_log_.open(updated_solution_log_name);
    opterr_log_.open(opterr_log_name);
    iter_log_.open(iter_log_name);
}

  ///
  /// @brief Destructor.
  ///
  ~Logger() {
    t_log_.close();
    x_log_.close();
    u_log_.close();
    uopt_log_.close();
    initial_solution_log_.close();
    updated_solution_log_.close();
    opterr_log_.close();
    iter_log_.close();
  }

  ///
  /// @brief Save datas.
  /// @param[in] t Time.
  /// @param[in] x State.
  /// @param[in] u Control input.
  /// @param[in] opterr Optimality error.
  ///
  template <typename StateVectorType, typename ControlInputVectorType, std::size_t N>
  void save(
    const Scalar t, const MatrixBase<StateVectorType> & x,
    const MatrixBase<ControlInputVectorType> & u,
    const std::array<ControlInputVectorType, N> & uopt,
    const std::array<ControlInputVectorType, N> & initial_solution,
    const std::array<ControlInputVectorType, N> & updated_solution,
    const double opterr, const int iter)
  {
    t_log_ << t << '\n';
    x_log_ << x.transpose() << '\n';
    u_log_ << u.transpose() << '\n';
    for (auto &matrix : uopt) {
      for (int i = 0; i < matrix.rows(); i++) {
        for (int j = 0; j < matrix.cols(); j++) {
          uopt_log_ << matrix(i, j) << " ";
        }
      }
    }
    uopt_log_ << '\n';
    for (auto &matrix : initial_solution) {
      for (int i = 0; i < matrix.rows(); i++) {
        for (int j = 0; j < matrix.cols(); j++) {
          initial_solution_log_ << matrix(i, j) << " ";
        }
      }
    }
    initial_solution_log_ << '\n';
    for (auto &matrix : updated_solution) {
      for (int i = 0; i < matrix.rows(); i++) {
        for (int j = 0; j < matrix.cols(); j++) {
          updated_solution_log_ << matrix(i, j) << " ";
        }
      }
    }
    updated_solution_log_ << '\n';
    opterr_log_ << opterr << '\n';
    iter_log_ << iter << '\n';
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
  std::ofstream t_log_, x_log_, u_log_, uopt_log_, initial_solution_log_, updated_solution_log_, opterr_log_, iter_log_;
};

} // namespace cgmres

#endif // CGMRES__LOGGER_HPP_