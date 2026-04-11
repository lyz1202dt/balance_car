from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

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
    reduced_nq: int
    reduced_nv: int
    reduced_nx: int
    wheel_radius: float
    floating_base_q_slice: slice
    floating_base_v_slice: slice
    floating_base_position_slice: slice
    floating_base_quaternion_slice: slice
    floating_base_linear_v_slice: slice
    floating_base_angular_v_slice: slice
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


def _parse_xyz(node: ET.Element | None) -> tuple[float, float, float]:
    if node is None:
        return 0.0, 0.0, 0.0

    xyz_text = node.attrib.get("xyz", "0 0 0")
    xyz = tuple(float(value) for value in xyz_text.split())
    if len(xyz) != 3:
        raise ValueError(f"Expected xyz triplet, got {xyz_text!r}.")
    return xyz


def _infer_wheel_radius(urdf_path: Path) -> float:
    root = ET.parse(urdf_path).getroot()
    wheel_joint_origins = []
    for joint_name in ("left_wheel_joint", "right_wheel_joint"):
        joint = root.find(f"./joint[@name='{joint_name}']")
        if joint is None:
            raise ValueError(f"Joint {joint_name} not found in {urdf_path}.")
        wheel_joint_origins.append(_parse_xyz(joint.find("origin")))

    axle_height = sum(origin[2] for origin in wheel_joint_origins) / len(wheel_joint_origins)
    wheel_radius = abs(axle_height)
    if wheel_radius <= 0.0:
        raise ValueError("Wheel radius inferred from wheel joint origins must be positive.")
    return wheel_radius


def describe_model(urdf_path: str | Path = DEFAULT_URDF_PATH) -> CarModelInfo:
    urdf_path = _as_path(urdf_path)
    model = build_numeric_model(urdf_path)

    floating_base_q_slice = slice(model.idx_qs[1], model.idx_qs[1] + model.nqs[1])
    floating_base_v_slice = slice(model.idx_vs[1], model.idx_vs[1] + model.nvs[1])
    floating_base_position_slice = slice(floating_base_q_slice.start, floating_base_q_slice.start + 3)
    floating_base_quaternion_slice = slice(floating_base_q_slice.start + 3, floating_base_q_slice.stop)
    floating_base_linear_v_slice = slice(floating_base_v_slice.start, floating_base_v_slice.start + 3)
    floating_base_angular_v_slice = slice(floating_base_v_slice.start + 3, floating_base_v_slice.stop)
    left_wheel_q_slice = slice(model.idx_qs[2], model.idx_qs[2] + model.nqs[2])
    right_wheel_q_slice = slice(model.idx_qs[3], model.idx_qs[3] + model.nqs[3])

    return CarModelInfo(
        urdf_path=urdf_path,
        model_name=f"{model.name}_full_state",
        nq=model.nq,
        nv=model.nv,
        nx=model.nq + model.nv,
        nu=2,
        reduced_nq=2,
        reduced_nv=2,
        reduced_nx=4,
        wheel_radius=_infer_wheel_radius(urdf_path),
        floating_base_q_slice=floating_base_q_slice,
        floating_base_v_slice=floating_base_v_slice,
        floating_base_position_slice=floating_base_position_slice,
        floating_base_quaternion_slice=floating_base_quaternion_slice,
        floating_base_linear_v_slice=floating_base_linear_v_slice,
        floating_base_angular_v_slice=floating_base_angular_v_slice,
        left_wheel_q_slice=left_wheel_q_slice,
        right_wheel_q_slice=right_wheel_q_slice,
        left_wheel_v_index=model.idx_vs[2],
        right_wheel_v_index=model.idx_vs[3],
    )


def default_state(urdf_path: str | Path = DEFAULT_URDF_PATH) -> ca.DM:
    _ = urdf_path
    return ca.DM.zeros(4, 1)


def default_input() -> ca.DM:
    return ca.DM.zeros(2, 1)


def _pitch_quaternion_xyzw(pitch: ca.SX) -> ca.SX:
    half_pitch = 0.5 * pitch
    return ca.vertcat(0.0, ca.sin(half_pitch), 0.0, ca.cos(half_pitch))


def _continuous_joint_configuration(angle: ca.SX) -> ca.SX:
    return ca.vertcat(ca.cos(angle), ca.sin(angle))


def _quaternion_to_rpy_xyz(quaternion_xyzw: ca.SX) -> tuple[ca.SX, ca.SX, ca.SX]:
    qx = quaternion_xyzw[0]
    qy = quaternion_xyzw[1]
    qz = quaternion_xyzw[2]
    qw = quaternion_xyzw[3]

    sin_roll = 2.0 * (qw * qx + qy * qz)
    cos_roll = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = ca.atan2(sin_roll, cos_roll)

    sin_pitch = 2.0 * (qw * qy - qz * qx)
    sin_pitch = ca.fmax(-1.0, ca.fmin(1.0, sin_pitch))
    pitch = ca.asin(sin_pitch)

    sin_yaw = 2.0 * (qw * qz + qx * qy)
    cos_yaw = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = ca.atan2(sin_yaw, cos_yaw)

    return roll, pitch, yaw


def _rotation_matrix_from_quaternion_xyzw(quaternion_xyzw: ca.SX) -> ca.SX:
    qx = quaternion_xyzw[0]
    qy = quaternion_xyzw[1]
    qz = quaternion_xyzw[2]
    qw = quaternion_xyzw[3]

    return ca.vertcat(
        ca.horzcat(
            1.0 - 2.0 * (qy * qy + qz * qz),
            2.0 * (qx * qy - qz * qw),
            2.0 * (qx * qz + qy * qw),
        ),
        ca.horzcat(
            2.0 * (qx * qy + qz * qw),
            1.0 - 2.0 * (qx * qx + qz * qz),
            2.0 * (qy * qz - qx * qw),
        ),
        ca.horzcat(
            2.0 * (qx * qz - qy * qw),
            2.0 * (qy * qz + qx * qw),
            1.0 - 2.0 * (qx * qx + qy * qy),
        ),
    )


def _build_configuration_rate_map(pin_model: cpin.Model, q: ca.SX, nv: int) -> ca.SX:
    delta = ca.SX.sym("delta", nv)
    q_integrated = cpin.integrate(pin_model, q, delta)
    return ca.substitute(ca.jacobian(q_integrated, delta), delta, ca.SX.zeros(nv, 1))


def _embed_reduced_configuration(xi: ca.SX, model_info: CarModelInfo) -> ca.SX:
    forward_position = xi[0]
    pitch = xi[1]
    wheel_angle = forward_position / model_info.wheel_radius

    base_position = ca.vertcat(forward_position, 0.0, 0.0)
    base_quaternion = _pitch_quaternion_xyzw(pitch)
    wheel_configuration = _continuous_joint_configuration(wheel_angle)

    return ca.vertcat(base_position, base_quaternion, wheel_configuration, wheel_configuration)


def _build_constraint_null_space(pitch: ca.SX, model_info: CarModelInfo) -> ca.SX:
    wheel_rate_scale = 1.0 / model_info.wheel_radius
    forward_motion_basis = ca.vertcat(
        ca.cos(pitch),
        0.0,
        ca.sin(pitch),
        0.0,
        0.0,
        0.0,
        wheel_rate_scale,
        wheel_rate_scale,
    )
    pitch_motion_basis = ca.vertcat(
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
    )
    return ca.horzcat(forward_motion_basis, pitch_motion_basis)


def _embed_reduced_velocity(xi: ca.SX, eta: ca.SX, model_info: CarModelInfo) -> tuple[ca.SX, ca.SX]:
    null_space_basis = _build_constraint_null_space(xi[1], model_info)
    return null_space_basis @ eta, null_space_basis


def _build_constraint_projection_terms(
    pin_model: cpin.Model,
    q: ca.SX,
    v: ca.SX,
    model_info: CarModelInfo,
) -> tuple[ca.SX, ca.SX, ca.SX, ca.SX]:
    q_symbol = ca.SX.sym("q_full", q.shape[0])
    v_symbol = ca.SX.sym("v_full", v.shape[0])
    qdot_symbol = _build_configuration_rate_map(pin_model, q_symbol, model_info.nv) @ v_symbol

    base_position = q_symbol[model_info.floating_base_position_slice]
    base_quaternion = q_symbol[model_info.floating_base_quaternion_slice]
    roll, _pitch, yaw = _quaternion_to_rpy_xyz(base_quaternion)

    qdot_translation = qdot_symbol[model_info.floating_base_position_slice]
    left_wheel_rate = v_symbol[model_info.left_wheel_v_index]
    right_wheel_rate = v_symbol[model_info.right_wheel_v_index]

    holonomic_residual = ca.vertcat(
        base_position[1],
        base_position[2],
        roll,
        yaw,
    )
    holonomic_velocity = ca.jacobian(holonomic_residual, q_symbol) @ qdot_symbol

    rolling_velocity = qdot_translation[0] - 0.5 * model_info.wheel_radius * (left_wheel_rate + right_wheel_rate)
    wheel_sync_velocity = left_wheel_rate - right_wheel_rate

    constraint_velocity = ca.vertcat(
        holonomic_velocity,
        rolling_velocity,
        wheel_sync_velocity,
    )
    constraint_jacobian = ca.jacobian(constraint_velocity, v_symbol)
    constraint_bias = ca.jacobian(constraint_velocity, q_symbol) @ qdot_symbol

    substitution_source = ca.vertcat(q_symbol, v_symbol)
    substitution_target = ca.vertcat(q, v)
    return (
        ca.substitute(holonomic_residual, substitution_source, substitution_target),
        ca.substitute(constraint_velocity, substitution_source, substitution_target),
        ca.substitute(constraint_jacobian, substitution_source, substitution_target),
        ca.substitute(constraint_bias, substitution_source, substitution_target),
    )


def build_motion_space_expression(
    x: ca.SX,
    u: ca.SX | None = None,
    urdf_path: str | Path = DEFAULT_URDF_PATH,
) -> dict[str, ca.SX]:
    model_info = describe_model(urdf_path)
    pin_model = build_symbolic_model(urdf_path)

    xi = x[: model_info.reduced_nq]
    eta = x[model_info.reduced_nq :]
    q = _embed_reduced_configuration(xi, model_info)
    v, null_space_basis = _embed_reduced_velocity(xi, eta, model_info)
    qdot_map = _build_configuration_rate_map(pin_model, q, model_info.nv)
    qdot = qdot_map @ v

    base_position = q[model_info.floating_base_position_slice]
    base_quaternion = q[model_info.floating_base_quaternion_slice]
    roll, pitch, yaw = _quaternion_to_rpy_xyz(base_quaternion)
    world_from_body_rotation = _rotation_matrix_from_quaternion_xyzw(base_quaternion)
    base_linear_velocity_world = world_from_body_rotation @ v[model_info.floating_base_linear_v_slice]

    pitch_rate = eta[1]
    holonomic_residual, constraint_velocity, constraint_jacobian, _constraint_bias = _build_constraint_projection_terms(
        pin_model, q, v, model_info
    )

    tracking_output = ca.vertcat(
        base_position[0],
        pitch,
        base_linear_velocity_world[0],
        pitch_rate,
    )
    soft_constraint_residual = ca.vertcat(holonomic_residual, constraint_velocity)
    tracking_output_augmented = ca.vertcat(tracking_output, soft_constraint_residual)
    tracking_output_terminal_augmented = ca.vertcat(tracking_output, holonomic_residual)
    tracking_output_with_input = tracking_output if u is None else ca.vertcat(tracking_output, u)
    restricted_motion_residual = ca.vertcat(holonomic_residual, constraint_velocity)

    return {
        "full_configuration": q,
        "full_velocity": v,
        "full_configuration_rate": qdot,
        "constraint_null_space_basis": null_space_basis,
        "constraint_jacobian": constraint_jacobian,
        "forward_position": base_position[0],
        "pitch": pitch,
        "forward_velocity": base_linear_velocity_world[0],
        "pitch_rate": pitch_rate,
        "roll": roll,
        "yaw": yaw,
        "tracking_output": tracking_output,
        "tracking_output_augmented": tracking_output_augmented,
        "tracking_output_terminal_augmented": tracking_output_terminal_augmented,
        "tracking_output_with_input": tracking_output_with_input,
        "restricted_motion_residual": restricted_motion_residual,
    }


def default_motion_reference(urdf_path: str | Path = DEFAULT_URDF_PATH) -> ca.DM:
    x0 = default_state(urdf_path)
    x_symbol = ca.SX.sym("x", x0.numel())
    motion_expr = build_motion_space_expression(x_symbol, urdf_path=urdf_path)["tracking_output"]
    motion_fun = ca.Function("car_motion_reference", [x_symbol], [motion_expr])
    return ca.DM(motion_fun(x0))


def export_car_ode_model(urdf_path: str | Path = DEFAULT_URDF_PATH) -> AcadosModel:
    model_info = describe_model(urdf_path)
    pin_model = build_symbolic_model(urdf_path)
    pin_data = pin_model.createData()

    xi = ca.SX.sym("xi", model_info.reduced_nq)
    eta = ca.SX.sym("eta", model_info.reduced_nv)
    x = ca.vertcat(xi, eta)

    tau_left = ca.SX.sym("tau_left")
    tau_right = ca.SX.sym("tau_right")
    u = ca.vertcat(tau_left, tau_right)

    xdot = ca.SX.sym("xdot", model_info.reduced_nx)

    q = _embed_reduced_configuration(xi, model_info)
    v, null_space_basis = _embed_reduced_velocity(xi, eta, model_info)

    tau = ca.SX.zeros(model_info.nv, 1)
    tau[model_info.left_wheel_v_index] = tau_left
    tau[model_info.right_wheel_v_index] = tau_right

    qdot_map = _build_configuration_rate_map(pin_model, q, model_info.nv)
    qdot = qdot_map @ v

    mass_matrix = cpin.crba(pin_model, pin_data, q)
    mass_matrix = 0.5 * (mass_matrix + mass_matrix.T)
    nonlinear_effects = cpin.nonLinearEffects(pin_model, pin_data, q, v)

    _holonomic_residual, _constraint_velocity, _constraint_jacobian, _constraint_bias = _build_constraint_projection_terms(
        pin_model, q, v, model_info
    )
    null_space_rate_map = ca.jacobian(ca.reshape(null_space_basis, -1, 1), xi)
    null_space_rate_map = ca.reshape(
        null_space_rate_map @ eta,
        model_info.nv,
        model_info.reduced_nv,
    )
    velocity_bias = null_space_rate_map @ eta

    reduced_mass_matrix = null_space_basis.T @ mass_matrix @ null_space_basis
    reduced_generalized_force = null_space_basis.T @ (tau - nonlinear_effects - mass_matrix @ velocity_bias)
    eta_dot = ca.solve(reduced_mass_matrix, reduced_generalized_force)

    f_expl = ca.vertcat(eta, eta_dot)
    f_impl = xdot - f_expl

    motion_space_expr = build_motion_space_expression(x, u=u, urdf_path=urdf_path)

    acados_model = AcadosModel()
    acados_model.name = model_info.model_name
    acados_model.x = x
    acados_model.xdot = xdot
    acados_model.u = u
    acados_model.p = ca.SX.zeros(0, 1)
    acados_model.f_expl_expr = f_expl
    acados_model.f_impl_expr = f_impl
    acados_model.cost_y_expr = motion_space_expr["tracking_output_with_input"]
    acados_model.cost_y_expr_e = motion_space_expr["tracking_output"]
    acados_model.con_h_expr = ca.SX.zeros(0, 1)
    acados_model.con_h_expr_e = ca.SX.zeros(0, 1)

    return acados_model


def export_car_full_state_ode_model(urdf_path: str | Path = DEFAULT_URDF_PATH) -> AcadosModel:
    return export_car_ode_model(urdf_path)


if __name__ == "__main__":
    info = describe_model(DEFAULT_URDF_PATH)
    print(f"URDF: {info.urdf_path}")
    print(f"model name: {info.model_name}")
    print(
        "full dims: "
        f"nq={info.nq}, nv={info.nv}, nx={info.nx}; "
        f"reduced dims: nq={info.reduced_nq}, nv={info.reduced_nv}, nx={info.reduced_nx}; "
        f"nu={info.nu}, wheel_radius={info.wheel_radius}"
    )
    print(f"default state shape: {default_state().shape}")
