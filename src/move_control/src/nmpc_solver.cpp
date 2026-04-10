#include "move_control/nmpc_solver.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <memory>

extern "C"
{
#include "acados_c/external_function_interface.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_solver_balance_car_full_state.h"
}

namespace
{
constexpr int kHorizon = BALANCE_CAR_FULL_STATE_N;
constexpr int kNy = BALANCE_CAR_FULL_STATE_NY;
constexpr int kNyN = BALANCE_CAR_FULL_STATE_NYN;
constexpr int kNbx0 = BALANCE_CAR_FULL_STATE_NBX0;
}  // namespace

struct NmpcSolver::Impl
{
  balance_car_full_state_solver_capsule * capsule{nullptr};
  ocp_nlp_config * nlp_config{nullptr};
  ocp_nlp_dims * nlp_dims{nullptr};
  ocp_nlp_in * nlp_in{nullptr};
  ocp_nlp_out * nlp_out{nullptr};
  ocp_nlp_solver * nlp_solver{nullptr};
  bool initialized{false};
};

NmpcSolver::NmpcSolver()
: impl_(std::make_unique<Impl>())
{
}

NmpcSolver::~NmpcSolver()
{
  if (!impl_ || !impl_->capsule) {
    return;
  }

  balance_car_full_state_acados_free(impl_->capsule);
  balance_car_full_state_acados_free_capsule(impl_->capsule);
}

bool NmpcSolver::initialize()
{
  if (impl_->initialized) {
    return true;
  }

  impl_->capsule = balance_car_full_state_acados_create_capsule();
  if (impl_->capsule == nullptr) {
    return false;
  }

  const int status = balance_car_full_state_acados_create(impl_->capsule);
  if (status != 0) {
    return false;
  }

  impl_->nlp_config = balance_car_full_state_acados_get_nlp_config(impl_->capsule);
  impl_->nlp_dims = balance_car_full_state_acados_get_nlp_dims(impl_->capsule);
  impl_->nlp_in = balance_car_full_state_acados_get_nlp_in(impl_->capsule);
  impl_->nlp_out = balance_car_full_state_acados_get_nlp_out(impl_->capsule);
  impl_->nlp_solver = balance_car_full_state_acados_get_nlp_solver(impl_->capsule);
  impl_->initialized = true;
  return true;
}

int NmpcSolver::solve(
  const StateVector & current_state,
  const StateVector & reference_state,
  InputVector & control)
{
  // Provide a valid warm start for all shooting nodes before SQP_RTI linearizes
  // the model. The reduced planar model lives in minimal coordinates, so copying
  // the measured state is a consistent initialization for every stage.
  std::array<double, kStateDim> x_guess{};
  std::copy_n(current_state.data(), kStateDim, x_guess.data());

  std::array<double, kInputDim> u_guess{};
  std::fill(u_guess.begin(), u_guess.end(), 0.0);

  for (int stage = 0; stage < kHorizon; ++stage) {
    ocp_nlp_out_set(impl_->nlp_config, impl_->nlp_dims, impl_->nlp_out, impl_->nlp_in, stage, "x", x_guess.data());
    ocp_nlp_out_set(impl_->nlp_config, impl_->nlp_dims, impl_->nlp_out, impl_->nlp_in, stage, "u", u_guess.data());
  }
  ocp_nlp_out_set(
    impl_->nlp_config, impl_->nlp_dims, impl_->nlp_out, impl_->nlp_in, kHorizon, "x", x_guess.data());

  std::array<int, kNbx0> idxbx0{};
  for (int i = 0; i < kNbx0; ++i) {
    idxbx0[i] = i;
  }

  std::array<double, kNbx0> lbx0{};
  std::array<double, kNbx0> ubx0{};
  std::copy_n(current_state.data(), kNbx0, lbx0.data());
  std::copy_n(current_state.data(), kNbx0, ubx0.data());

  ocp_nlp_constraints_model_set(
    impl_->nlp_config, impl_->nlp_dims, impl_->nlp_in, impl_->nlp_out, 0, "idxbx", idxbx0.data());
  ocp_nlp_constraints_model_set(
    impl_->nlp_config, impl_->nlp_dims, impl_->nlp_in, impl_->nlp_out, 0, "lbx", lbx0.data());
  ocp_nlp_constraints_model_set(
    impl_->nlp_config, impl_->nlp_dims, impl_->nlp_in, impl_->nlp_out, 0, "ubx", ubx0.data());

  std::array<double, kNy> yref{};
  std::array<double, kNyN> yref_n{};
  std::fill(yref.begin(), yref.end(), 0.0);
  std::fill(yref_n.begin(), yref_n.end(), 0.0);

  std::copy_n(reference_state.data(), kStateDim, yref.data());
  std::copy_n(reference_state.data(), kStateDim, yref_n.data());

  for (int stage = 0; stage < kHorizon; ++stage) {
    ocp_nlp_cost_model_set(impl_->nlp_config, impl_->nlp_dims, impl_->nlp_in, stage, "yref", yref.data());
  }
  ocp_nlp_cost_model_set(impl_->nlp_config, impl_->nlp_dims, impl_->nlp_in, kHorizon, "yref", yref_n.data());

  const int status = balance_car_full_state_acados_solve(impl_->capsule);
  if (status != 0) {
    return status;
  }

  std::array<double, kInputDim> u0{};
  ocp_nlp_out_get(impl_->nlp_config, impl_->nlp_dims, impl_->nlp_out, 0, "u", u0.data());
  for (int i = 0; i < kInputDim; ++i) {
    control(i) = u0[i];
  }

  return 0;
}

bool NmpcSolver::is_initialized() const
{
  return impl_ && impl_->initialized;
}
