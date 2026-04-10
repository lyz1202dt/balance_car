from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

import casadi as ca
import pinocchio as pin
from acados_template import AcadosModel


DEFAULT_URDF_PATH = Path(__file__).resolve().parents[2] / "car" / "model" / "car.urdf"
EXPECTED_JOINT_NAMES = ("universe", "floating_base", "left_wheel_joint", "right_wheel_joint")
GRAVITY = 9.81


@dataclass(frozen=True)
class CarModelInfo:
    urdf_path: Path
    model_name: str
    nx: int
    nu: int
    body_mass: float
    wheel_mass_total: float
    wheel_radius: float
    com_height_above_axle: float
    body_pitch_inertia_about_axle: float
    wheel_spin_inertia: float
    cart_effective_mass: float
    gravity: float


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


def _parse_xyz(node: ET.Element | None) -> tuple[float, float, float]:
    if node is None:
        return 0.0, 0.0, 0.0

    xyz_text = node.attrib.get("xyz", "0 0 0")
    xyz = tuple(float(value) for value in xyz_text.split())
    if len(xyz) != 3:
        raise ValueError(f"Expected xyz triplet, got {xyz_text!r}.")
    return xyz


def _build_planar_parameter_map(urdf_path: Path) -> dict[str, float | str]:
    root = ET.parse(urdf_path).getroot()

    link_inertials: dict[str, dict[str, float | tuple[float, float, float]]] = {}
    for link in root.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue

        inertia = inertial.find("inertia")
        if inertia is None:
            raise ValueError(f"Link {link.attrib['name']} is missing an inertia block.")

        link_inertials[link.attrib["name"]] = {
            "mass": float(inertial.find("mass").attrib["value"]),
            "origin": _parse_xyz(inertial.find("origin")),
            "ixx": float(inertia.attrib["ixx"]),
            "iyy": float(inertia.attrib["iyy"]),
            "izz": float(inertia.attrib["izz"]),
        }

    wheel_joint_origins: list[tuple[float, float, float]] = []
    for joint_name in ("left_wheel_joint", "right_wheel_joint"):
        joint = root.find(f"./joint[@name='{joint_name}']")
        if joint is None:
            raise ValueError(f"Joint {joint_name} not found in {urdf_path}.")
        wheel_joint_origins.append(_parse_xyz(joint.find("origin")))

    body = link_inertials["base_link"]
    left_wheel = link_inertials["left_wheel"]
    right_wheel = link_inertials["right_wheel"]

    axle_height = sum(origin[2] for origin in wheel_joint_origins) / len(wheel_joint_origins)
    wheel_radius = abs(axle_height)
    body_com_height_above_axle = body["origin"][2] - axle_height

    if wheel_radius <= 0.0:
        raise ValueError("Wheel radius inferred from wheel joint origin must be positive.")
    if body_com_height_above_axle <= 0.0:
        raise ValueError("Body COM must be above the wheel axle for the planar balance model.")

    body_mass = float(body["mass"])
    wheel_mass_total = float(left_wheel["mass"]) + float(right_wheel["mass"])

    # The wheel inertial frames in the URDF are not rotated with the visuals.
    # Using the minimum principal inertia is a robust way to recover the spin inertia.
    wheel_spin_inertia = min(
        float(left_wheel["ixx"]),
        float(left_wheel["iyy"]),
        float(left_wheel["izz"]),
        float(right_wheel["ixx"]),
        float(right_wheel["iyy"]),
        float(right_wheel["izz"]),
    )
    body_pitch_inertia_about_axle = float(body["iyy"]) + body_mass * body_com_height_above_axle**2
    cart_effective_mass = wheel_mass_total + 2.0 * wheel_spin_inertia / wheel_radius**2

    return {
        "model_name": root.attrib.get("name", "balance_car"),
        "body_mass": body_mass,
        "wheel_mass_total": wheel_mass_total,
        "wheel_radius": wheel_radius,
        "com_height_above_axle": body_com_height_above_axle,
        "body_pitch_inertia_about_axle": body_pitch_inertia_about_axle,
        "wheel_spin_inertia": wheel_spin_inertia,
        "cart_effective_mass": cart_effective_mass,
    }


def describe_model(urdf_path: str | Path = DEFAULT_URDF_PATH) -> CarModelInfo:
    urdf_path = _as_path(urdf_path)
    numeric_model = build_numeric_model(urdf_path)
    params = _build_planar_parameter_map(urdf_path)

    return CarModelInfo(
        urdf_path=urdf_path,
        model_name=f"{params['model_name']}_full_state" if numeric_model.name == params["model_name"] else f"{numeric_model.name}_full_state",
        nx=4,
        nu=2,
        body_mass=float(params["body_mass"]),
        wheel_mass_total=float(params["wheel_mass_total"]),
        wheel_radius=float(params["wheel_radius"]),
        com_height_above_axle=float(params["com_height_above_axle"]),
        body_pitch_inertia_about_axle=float(params["body_pitch_inertia_about_axle"]),
        wheel_spin_inertia=float(params["wheel_spin_inertia"]),
        cart_effective_mass=float(params["cart_effective_mass"]),
        gravity=GRAVITY,
    )


def default_state(urdf_path: str | Path = DEFAULT_URDF_PATH) -> ca.DM:
    describe_model(urdf_path)
    return ca.DM.zeros(4, 1)


def default_input() -> ca.DM:
    return ca.DM.zeros(2, 1)


def build_motion_space_expression(x: ca.SX, u: ca.SX | None = None) -> dict[str, ca.SX]:
    forward_position = x[0]
    pitch = x[1]
    forward_velocity = x[2]
    pitch_rate = x[3]

    tracking_output = x
    tracking_output_with_input = tracking_output if u is None else ca.vertcat(tracking_output, u)
    symmetric_torque_residual = (
        ca.SX.zeros(0, 1) if u is None else ca.vertcat(u[0] - u[1])
    )

    return {
        "forward_position": forward_position,
        "pitch": pitch,
        "forward_velocity": forward_velocity,
        "pitch_rate": pitch_rate,
        "tracking_output": tracking_output,
        "tracking_output_with_input": tracking_output_with_input,
        "restricted_motion_residual": symmetric_torque_residual,
    }


def default_motion_reference(urdf_path: str | Path = DEFAULT_URDF_PATH) -> ca.DM:
    describe_model(urdf_path)
    return ca.DM.zeros(4, 1)


def export_car_ode_model(urdf_path: str | Path = DEFAULT_URDF_PATH) -> AcadosModel:
    model_info = describe_model(urdf_path)

    x = ca.SX.sym("x", model_info.nx)
    u = ca.SX.sym("u", model_info.nu)
    xdot = ca.SX.sym("xdot", model_info.nx)

    forward_position = x[0]
    pitch = x[1]
    forward_velocity = x[2]
    pitch_rate = x[3]

    tau_left = u[0]
    tau_right = u[1]
    traction_force = (tau_left + tau_right) / model_info.wheel_radius

    total_mass = model_info.cart_effective_mass + model_info.body_mass
    coupling = model_info.body_mass * model_info.com_height_above_axle * ca.cos(pitch)
    body_inertia = model_info.body_pitch_inertia_about_axle

    rhs_forward = traction_force + model_info.body_mass * model_info.com_height_above_axle * ca.sin(pitch) * pitch_rate**2
    rhs_pitch = model_info.body_mass * model_info.gravity * model_info.com_height_above_axle * ca.sin(pitch)
    determinant = total_mass * body_inertia - coupling**2

    forward_acceleration = (body_inertia * rhs_forward - coupling * rhs_pitch) / determinant
    pitch_acceleration = (total_mass * rhs_pitch - coupling * rhs_forward) / determinant

    f_expl = ca.vertcat(
        forward_velocity,
        pitch_rate,
        forward_acceleration,
        pitch_acceleration,
    )
    f_impl = xdot - f_expl

    motion_space_expr = build_motion_space_expression(x, u=u)

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
    acados_model.con_h_expr = motion_space_expr["restricted_motion_residual"]
    acados_model.con_h_expr_e = ca.SX.zeros(0, 1)

    return acados_model


def export_car_full_state_ode_model(urdf_path: str | Path = DEFAULT_URDF_PATH) -> AcadosModel:
    return export_car_ode_model(urdf_path)


if __name__ == "__main__":
    info = describe_model(DEFAULT_URDF_PATH)
    print(f"URDF: {info.urdf_path}")
    print(f"model name: {info.model_name}")
    print(f"nx={info.nx}, nu={info.nu}")
    print(
        "planar params: "
        f"body_mass={info.body_mass}, wheel_mass_total={info.wheel_mass_total}, "
        f"wheel_radius={info.wheel_radius}, com_height_above_axle={info.com_height_above_axle}, "
        f"body_pitch_inertia_about_axle={info.body_pitch_inertia_about_axle}"
    )
