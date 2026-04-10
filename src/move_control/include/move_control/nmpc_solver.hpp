#pragma once

#include <Eigen/Dense>

#include <memory>

class NmpcSolver
{
public:
  static constexpr int kStateDim = 4;
  static constexpr int kInputDim = 2;

  using StateVector = Eigen::Matrix<double, kStateDim, 1>;
  using InputVector = Eigen::Matrix<double, kInputDim, 1>;

  NmpcSolver();
  ~NmpcSolver();

  NmpcSolver(const NmpcSolver &) = delete;
  NmpcSolver & operator=(const NmpcSolver &) = delete;

  bool initialize();
  int solve(const StateVector & current_state, const StateVector & reference_state, InputVector & control);
  bool is_initialized() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
