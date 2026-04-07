from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import casadi as ca
import pinocchio as pin
import pinocchio.casadi as cpin
from acados_template import AcadosModel


DEFAULT_URDF_PATH = Path(__file__).resolve().parents[2] / "car" / "model" / "car.urdf"
EXPECTED_JOINT_NAMES = ("universe", "floating_base", "left_wheel_joint", "right_wheel_joint")


@dataclass(frozen=True)
class CarModelInfo:
    urdf_path: Path
    model_name: str
    nq: int
    nv: int
    nx: int
    nu: int
    floating_base_q_slice: slice
    floating_base_v_slice: slice
    left_wheel_q_slice: slice
    right_wheel_q_slice: slice
    left_wheel_v_index: int
    right_wheel_v_index: int


def _as_path(path_like: str | Path) -> Path:
    return Path(path_like).expanduser().resolve()


def build_numeric_model(urdf_path: str | Path = DEFAULT_URDF_PATH) -> pin.Model:
    urdf_path = _as_path(urdf_path)
    model = pin.buildModelFromUrdf(str(urdf_path))

    joint_names = tuple(model.names)
    if joint_names != EXPECTED_JOINT_NAMES:
        raise ValueError(
            "Unexpected URDF joint layout. "
            f"Expected {EXPECTED_JOINT_NAMES}, got {joint_names}."
        )

    return model


def build_symbolic_model(urdf_path: str | Path = DEFAULT_URDF_PATH) -> cpin.Model:
    numeric_model = build_numeric_model(urdf_path)
    symbolic_model = cpin.Model(numeric_model)
    symbolic_model.name = f"{numeric_model.name}_full_state"
    return symbolic_model


def describe_model(urdf_path: str | Path = DEFAULT_URDF_PATH) -> CarModelInfo:
    model = build_numeric_model(urdf_path)

    floating_base_q_slice = slice(model.idx_qs[1], model.idx_qs[1] + model.nqs[1])
    floating_base_v_slice = slice(model.idx_vs[1], model.idx_vs[1] + model.nvs[1])
    left_wheel_q_slice = slice(model.idx_qs[2], model.idx_qs[2] + model.nqs[2])
    right_wheel_q_slice = slice(model.idx_qs[3], model.idx_qs[3] + model.nqs[3])

    return CarModelInfo(
        urdf_path=_as_path(urdf_path),
        model_name=f"{model.name}_full_state",
        nq=model.nq,
        nv=model.nv,
        nx=model.nq + model.nv,
        nu=2,
        floating_base_q_slice=floating_base_q_slice,
        floating_base_v_slice=floating_base_v_slice,
        left_wheel_q_slice=left_wheel_q_slice,
        right_wheel_q_slice=right_wheel_q_slice,
        left_wheel_v_index=model.idx_vs[2],
        right_wheel_v_index=model.idx_vs[3],
    )


def default_state(urdf_path: str | Path = DEFAULT_URDF_PATH) -> ca.DM:
    numeric_model = build_numeric_model(urdf_path)
    q0 = pin.neutral(numeric_model)
    v0 = ca.DM.zeros(numeric_model.nv, 1)
    return ca.vertcat(ca.DM(q0), v0)


def default_input() -> ca.DM:
    return ca.DM.zeros(2, 1)


def export_car_ode_model(urdf_path: str | Path = DEFAULT_URDF_PATH) -> AcadosModel:
    model_info = describe_model(urdf_path)
    pin_model = build_symbolic_model(urdf_path)
    pin_data = pin_model.createData()

    q = ca.SX.sym("q", model_info.nq)
    v = ca.SX.sym("v", model_info.nv)
    x = ca.vertcat(q, v)

    tau_left = ca.SX.sym("tau_left")
    tau_right = ca.SX.sym("tau_right")
    u = ca.vertcat(tau_left, tau_right)

    xdot = ca.SX.sym("xdot", model_info.nx)

    tau = ca.SX.zeros(model_info.nv, 1)
    tau[model_info.left_wheel_v_index] = tau_left
    tau[model_info.right_wheel_v_index] = tau_right

    qdd = cpin.aba(pin_model, pin_data, q, v, tau)

    # Map tangent velocity to the configuration derivative for the free-flyer
    # quaternion and the continuous wheel joints represented in Pinocchio's q space.
    delta = ca.SX.sym("delta", model_info.nv)
    q_integrated = cpin.integrate(pin_model, q, delta)
    qdot_map = ca.substitute(ca.jacobian(q_integrated, delta), delta, ca.SX.zeros(model_info.nv, 1))
    qdot = qdot_map @ v

    f_expl = ca.vertcat(qdot, qdd)
    f_impl = xdot - f_expl

    acados_model = AcadosModel()
    acados_model.name = model_info.model_name
    acados_model.x = x
    acados_model.xdot = xdot
    acados_model.u = u
    acados_model.p = ca.SX.zeros(0, 1)
    acados_model.f_expl_expr = f_expl
    acados_model.f_impl_expr = f_impl

    return acados_model


def export_car_full_state_ode_model(urdf_path: str | Path = DEFAULT_URDF_PATH) -> AcadosModel:
    return export_car_ode_model(urdf_path)


if __name__ == "__main__":
    info = describe_model(DEFAULT_URDF_PATH)
    print(f"URDF: {info.urdf_path}")
    print(f"model name: {info.model_name}")
    print(f"nq={info.nq}, nv={info.nv}, nx={info.nx}, nu={info.nu}")
    print(f"default state shape: {default_state().shape}")
