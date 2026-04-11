from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

import numpy as np
import scipy.linalg


DEFAULT_EXPORT_DIR = Path(__file__).resolve().parent / "c_generated_code"
DEFAULT_JSON_PATH = Path(__file__).resolve().parent / "acados_ocp_car_full_state.json"
DEFAULT_ACADOS_SOURCE_DIR = Path(__file__).resolve().parents[2] / "acados"


if "ACADOS_SOURCE_DIR" not in os.environ and DEFAULT_ACADOS_SOURCE_DIR.exists():
    os.environ["ACADOS_SOURCE_DIR"] = str(DEFAULT_ACADOS_SOURCE_DIR.resolve())

from acados_template import AcadosOcp, AcadosOcpSolver

try:
    from .car_nmpc_model import (
        DEFAULT_URDF_PATH,
        default_input,
        default_motion_reference,
        default_state,
        export_car_ode_model,
    )
except ImportError:
    from car_nmpc_model import (
        DEFAULT_URDF_PATH,
        default_input,
        default_motion_reference,
        default_state,
        export_car_ode_model,
    )


def _build_motion_weight() -> np.ndarray:
    return np.diag([
        40.0,    # forward position
        1500.0,  # pitch
        30.0,    # forward velocity
        500.0,   # pitch rate
    ])


def _build_terminal_motion_weight() -> np.ndarray:
    return np.diag([
        80.0,    # forward position
        2500.0,  # pitch
        60.0,    # forward velocity
        800.0,   # pitch rate
    ])


def build_ocp(
    urdf_path: str | Path = DEFAULT_URDF_PATH,
    horizon_steps: int = 40,
    horizon_time: float = 1.0,
    export_directory: str | Path = DEFAULT_EXPORT_DIR,
) -> AcadosOcp:
    model = export_car_ode_model(urdf_path)
    ocp = AcadosOcp()
    ocp.model = model

    ocp.solver_options.N_horizon = horizon_steps
    ocp.solver_options.tf = horizon_time

    ocp.code_gen_opts.code_export_directory = str(Path(export_directory).resolve())

    Q = _build_motion_weight()
    Q_e = _build_terminal_motion_weight()
    R = np.diag([0.5, 0.5])
    ocp.cost.W = scipy.linalg.block_diag(Q, R)
    ocp.cost.W_e = Q_e

    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.cost.cost_type_e = "NONLINEAR_LS"

    motion_ref = np.asarray(default_motion_reference(urdf_path)).reshape(-1)
    u_ref = np.asarray(default_input()).reshape(-1)
    ocp.cost.yref = np.concatenate([motion_ref, u_ref])
    ocp.cost.yref_e = motion_ref

    ocp.constraints.x0 = np.asarray(default_state(urdf_path)).reshape(-1)
    ocp.constraints.lbu = np.array([-5.0, -5.0])
    ocp.constraints.ubu = np.array([5.0, 5.0])
    ocp.constraints.idxbu = np.array([0, 1], dtype=np.int64)
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.print_level = 0

    return ocp


def generate_solver(
    urdf_path: str | Path = DEFAULT_URDF_PATH,
    horizon_steps: int = 40,
    horizon_time: float = 1.0,
    export_directory: str | Path = DEFAULT_EXPORT_DIR,
    json_path: str | Path = DEFAULT_JSON_PATH,
) -> tuple[AcadosOcp, AcadosOcpSolver]:
    export_directory = Path(export_directory).resolve()
    export_directory.mkdir(parents=True, exist_ok=True)
    json_path = Path(json_path).resolve()

    ocp = build_ocp(
        urdf_path=urdf_path,
        horizon_steps=horizon_steps,
        horizon_time=horizon_time,
        export_directory=export_directory,
    )

    solver = AcadosOcpSolver(ocp, json_file=str(json_path))
    return ocp, solver


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate acados C code for the full-state balance car NMPC model.")
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF_PATH, help="Path to the full-state URDF model.")
    parser.add_argument(
        "--export-dir",
        type=Path,
        default=DEFAULT_EXPORT_DIR,
        help="Directory where acados C code will be generated.",
    )
    parser.add_argument(
        "--json",
        type=Path,
        default=DEFAULT_JSON_PATH,
        help="Path to the generated acados JSON configuration.",
    )
    parser.add_argument("--N", type=int, default=40, help="Prediction horizon steps.")
    parser.add_argument("--Tf", type=float, default=1.0, help="Prediction horizon time in seconds.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    ocp, _solver = generate_solver(
        urdf_path=args.urdf,
        horizon_steps=args.N,
        horizon_time=args.Tf,
        export_directory=args.export_dir,
        json_path=args.json,
    )

    summary = {
        "model_name": ocp.model.name,
        "nx": int(ocp.model.x.size()[0]),
        "nu": int(ocp.model.u.size()[0]),
        "N": int(ocp.solver_options.N_horizon),
        "Tf": float(ocp.solver_options.tf),
        "code_export_directory": ocp.code_gen_opts.code_export_directory,
        "json_file": str(Path(args.json).resolve()),
    }
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
