from casadi import *
from pathlib import Path
import os


def skew(v):
    S = MX.zeros(3, 3)
    S[0, 1] = -v[2]
    S[0, 2] = v[1]
    S[1, 0] = v[2]
    S[1, 2] = -v[0]
    S[2, 0] = -v[1]
    S[2, 1] = v[0]
    return S


def main():
    # Scan-model parameters reused across all beams in one scan.
    theta_min = MX.sym("theta_min")
    delta_theta = MX.sym("delta_theta")

    # Intrinsic parameters / systematic beam model.
    b_rho = MX.sym("b_rho")
    s_rho = MX.sym("s_rho")
    b_theta = MX.sym("b_theta")

    # Extrinsics from lidar frame into target frame.
    c = MX.sym("c", 3, 1)
    R = MX.sym("R", 3, 3)

    # Fixed uncertainty terms.
    Sigma_c = MX.sym("Sigma_c", 3, 3)
    Sigma_phi = MX.sym("Sigma_phi", 3, 3)
    sigma_b_rho = MX.sym("sigma_b_rho")
    sigma_s_rho = MX.sym("sigma_s_rho")
    sigma_b_theta = MX.sym("sigma_b_theta")

    Rt = R.T
    sigma_b_rho_sq = sigma_b_rho * sigma_b_rho
    sigma_s_rho_sq = sigma_s_rho * sigma_s_rho
    sigma_b_theta_sq = sigma_b_theta * sigma_b_theta

    outer = Function(
        "per_lidar_precompute",
        [
            theta_min,
            delta_theta,
            b_rho,
            s_rho,
            b_theta,
            c,
            R,
            Sigma_c,
            Sigma_phi,
            sigma_b_rho,
            sigma_s_rho,
            sigma_b_theta,
        ],
        [
            theta_min,
            delta_theta,
            b_rho,
            s_rho,
            b_theta,
            c,
            Rt,
            Sigma_c,
            Sigma_phi,
            sigma_b_rho_sq,
            sigma_s_rho_sq,
            sigma_b_theta_sq,
        ],
        [
            "theta_min",
            "delta_theta",
            "b_rho",
            "s_rho",
            "b_theta",
            "c",
            "R",
            "Sigma_c",
            "Sigma_phi",
            "sigma_b_rho",
            "sigma_s_rho",
            "sigma_b_theta",
        ],
        [
            "theta_min_out",
            "delta_theta_out",
            "b_rho_out",
            "s_rho_out",
            "b_theta_out",
            "c_out",
            "Rt_out",
            "Sigma_c_out",
            "Sigma_phi_out",
            "sigma_b_rho_sq_out",
            "sigma_s_rho_sq_out",
            "sigma_b_theta_sq_out",
        ],
    )

    # Outer-loop outputs fed into the per-beam stage.
    theta_min_in = MX.sym("theta_min_in")
    delta_theta_in = MX.sym("delta_theta_in")
    b_rho_in = MX.sym("b_rho_in")
    s_rho_in = MX.sym("s_rho_in")
    b_theta_in = MX.sym("b_theta_in")
    c_in = MX.sym("c_in", 3, 1)
    Rt_in = MX.sym("Rt_in", 3, 3)
    Sigma_c_in = MX.sym("Sigma_c_in", 3, 3)
    Sigma_phi_in = MX.sym("Sigma_phi_in", 3, 3)
    sigma_b_rho_sq_in = MX.sym("sigma_b_rho_sq_in")
    sigma_s_rho_sq_in = MX.sym("sigma_s_rho_sq_in")
    sigma_b_theta_sq_in = MX.sym("sigma_b_theta_sq_in")

    # Per-beam inputs.
    beam_index = MX.sym("beam_index")
    rho_i = MX.sym("rho_i")
    sigma_rho_i = MX.sym("sigma_rho_i")

    theta_i = theta_min_in + beam_index * delta_theta_in + b_theta_in
    q_i = s_rho_in * rho_i + b_rho_in

    a_i = vertcat(
        cos(theta_i),
        sin(theta_i),
        0,
    )

    pL_i = q_i * a_i
    X_i = c_in + Rt_in @ pL_i

    # Jacobians from the notebook model.
    J_c_i = jacobian(X_i, c_in)
    J_phi_i = Rt_in @ skew(pL_i)
    J_b_rho_i = jacobian(X_i, b_rho_in)
    J_s_rho_i = jacobian(X_i, s_rho_in)
    J_b_theta_i = jacobian(X_i, b_theta_in)
    J_rho_i = jacobian(X_i, rho_i)

    Sigma_X_i = (
        J_c_i @ Sigma_c_in @ J_c_i.T
        + J_phi_i @ Sigma_phi_in @ J_phi_i.T
        + sigma_b_rho_sq_in * (J_b_rho_i @ J_b_rho_i.T)
        + sigma_s_rho_sq_in * (J_s_rho_i @ J_s_rho_i.T)
        + sigma_b_theta_sq_in * (J_b_theta_i @ J_b_theta_i.T)
        + (sigma_rho_i * sigma_rho_i) * (J_rho_i @ J_rho_i.T)
    )

    inner = Function(
        "per_range_compute",
        [
            theta_min_in,
            delta_theta_in,
            b_rho_in,
            s_rho_in,
            b_theta_in,
            c_in,
            Rt_in,
            Sigma_c_in,
            Sigma_phi_in,
            sigma_b_rho_sq_in,
            sigma_s_rho_sq_in,
            sigma_b_theta_sq_in,
            beam_index,
            rho_i,
            sigma_rho_i,
        ],
        [
            X_i,
            Sigma_X_i,
        ],
        [
            "theta_min",
            "delta_theta",
            "b_rho",
            "s_rho",
            "b_theta",
            "c",
            "Rt",
            "Sigma_c",
            "Sigma_phi",
            "sigma_b_rho_sq",
            "sigma_s_rho_sq",
            "sigma_b_theta_sq",
            "beam_index",
            "rho_i",
            "sigma_rho_i",
        ],
        [
            "X_i",
            "Sigma_X_i",
        ],
    )

    output_dir = Path(__file__).resolve().parents[1] / "generated" / "c" / "casadi"
    output_dir.mkdir(parents=True, exist_ok=True)
    generated_c_name = "lidar_point_cloud_staged.c"

    old_cwd = os.getcwd()
    os.chdir(output_dir)
    try:
        cg = CodeGenerator(generated_c_name, {"with_header": True})
        cg.add(outer)
        cg.add(inner)
        cg.generate()
    finally:
        os.chdir(old_cwd)

    print(outer)
    print(inner)
    print("Generated code in:", output_dir)
    print("Generated source:", output_dir / generated_c_name)
    print("Generated header:", output_dir / generated_c_name.replace(".c", ".h"))


if __name__ == "__main__":
    main()
