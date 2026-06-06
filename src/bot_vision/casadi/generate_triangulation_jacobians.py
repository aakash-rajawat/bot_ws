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


def norm3(v):
    return sqrt(dot(v, v))


def make_K(fx, fy, cx, cy):
    K = MX.zeros(3, 3)
    K[0, 0] = fx
    K[1, 1] = fy
    K[0, 2] = cx
    K[1, 2] = cy
    K[2, 2] = 1
    return K


def make_Kinv_derivatives(fx, fy, cx, cy):
    dKinv_dfx = MX.zeros(3, 3)
    dKinv_dfx[0, 0] = -1 / (fx * fx)
    dKinv_dfx[0, 2] = cx / (fx * fx)

    dKinv_dfy = MX.zeros(3, 3)
    dKinv_dfy[1, 1] = -1 / (fy * fy)
    dKinv_dfy[1, 2] = cy / (fy * fy)

    dKinv_dcx = MX.zeros(3, 3)
    dKinv_dcx[0, 2] = -1 / fx

    dKinv_dcy = MX.zeros(3, 3)
    dKinv_dcy[1, 2] = -1 / fy

    return dKinv_dfx, dKinv_dfy, dKinv_dcx, dKinv_dcy


def main():
    # Camera j intrinsics.
    fx = MX.sym("fx")
    fy = MX.sym("fy")
    cx = MX.sym("cx")
    cy = MX.sym("cy")

    # Reference camera j' intrinsics.
    fxp = MX.sym("fxp")
    fyp = MX.sym("fyp")
    cxp = MX.sym("cxp")
    cyp = MX.sym("cyp")

    # Camera geometry.
    R = MX.sym("R", 3, 3)
    C = MX.sym("C", 3, 1)
    Cp = MX.sym("Cp", 3, 1)

    # Camera-level uncertainty terms.
    Sigma_C = MX.sym("Sigma_C", 3, 3)
    Sigma_theta = MX.sym("Sigma_theta", 3, 3)
    Sigma_kappa = MX.sym("Sigma_kappa", 4, 4)

    K = make_K(fx, fy, cx, cy)
    Kp = make_K(fxp, fyp, cxp, cyp)

    Kinv = inv(K)
    Kpinv = inv(Kp)
    dKinv_dfx, dKinv_dfy, dKinv_dcx, dKinv_dcy = make_Kinv_derivatives(fx, fy, cx, cy)

    # Camera-pair quantity reused for every keypoint pair.
    b = R @ (C - Cp)

    outer = Function(
        "triangulation_outer_precompute",
        [R, C, Cp, fx, fy, cx, cy, fxp, fyp, cxp, cyp, Sigma_C, Sigma_theta, Sigma_kappa],
        [
            Kinv,
            Kpinv,
            R,
            b,
            dKinv_dfx,
            dKinv_dfy,
            dKinv_dcx,
            dKinv_dcy,
            Sigma_C,
            Sigma_theta,
            Sigma_kappa,
        ],
        [
            "R", "C", "Cp",
            "fx", "fy", "cx", "cy",
            "fxp", "fyp", "cxp", "cyp",
            "Sigma_C", "Sigma_theta", "Sigma_kappa",
        ],
        [
            "Kinv", "Kpinv", "R_out", "b",
            "dKinv_dfx", "dKinv_dfy", "dKinv_dcx", "dKinv_dcy",
            "Sigma_C_out", "Sigma_theta_out", "Sigma_kappa_out",
        ],
    )

    # Outer-loop outputs threaded into the inner stage.
    Kinv_in = MX.sym("Kinv_in", 3, 3)
    Kpinv_in = MX.sym("Kpinv_in", 3, 3)
    R_in = MX.sym("R_in", 3, 3)
    b_in = MX.sym("b_in", 3, 1)
    dKinv_dfx_in = MX.sym("dKinv_dfx_in", 3, 3)
    dKinv_dfy_in = MX.sym("dKinv_dfy_in", 3, 3)
    dKinv_dcx_in = MX.sym("dKinv_dcx_in", 3, 3)
    dKinv_dcy_in = MX.sym("dKinv_dcy_in", 3, 3)

    Sigma_C_in = MX.sym("Sigma_C_in", 3, 3)
    Sigma_theta_in = MX.sym("Sigma_theta_in", 3, 3)
    Sigma_kappa_in = MX.sym("Sigma_kappa_in", 4, 4)

    # Per-keypoint inputs.
    x = MX.sym("x", 3, 1)
    xp = MX.sym("xp", 3, 1)
    Sigma_xdc = MX.sym("Sigma_xdc", 3, 3)

    # Rays and range surrogate.
    u = Kinv_in @ x
    up = Kpinv_in @ xp

    a = u / norm3(u)
    ap = up / norm3(up)

    rho_num = norm3(skew(b_in) @ a)
    rho_den = norm3(skew(a) @ ap)
    rho = rho_num / rho_den

    # Closed-form Jacobians used by the paper implementation.
    J_xdc = -rho * skew(a) @ Kinv_in
    J_C = -skew(u) @ R_in
    J_theta = rho * skew(u) @ skew(a)
    J_kappa = horzcat(
        -rho * skew(a) @ (dKinv_dfx_in @ x),
        -rho * skew(a) @ (dKinv_dfy_in @ x),
        -rho * skew(a) @ (dKinv_dcx_in @ x),
        -rho * skew(a) @ (dKinv_dcy_in @ x),
    )

    Sigma_res = (
        J_xdc @ Sigma_xdc @ J_xdc.T
        + J_C @ Sigma_C_in @ J_C.T
        + J_theta @ Sigma_theta_in @ J_theta.T
        + J_kappa @ Sigma_kappa_in @ J_kappa.T
    )

    inner = Function(
        "triangulation_inner_sigma_res",
        [
            Kinv_in, Kpinv_in, R_in, b_in,
            dKinv_dfx_in, dKinv_dfy_in, dKinv_dcx_in, dKinv_dcy_in,
            Sigma_C_in, Sigma_theta_in, Sigma_kappa_in,
            x, xp, Sigma_xdc,
        ],
        [a, ap, rho, J_xdc, J_C, J_theta, J_kappa, Sigma_res],
        [
            "Kinv", "Kpinv", "R", "b",
            "dKinv_dfx", "dKinv_dfy", "dKinv_dcx", "dKinv_dcy",
            "Sigma_C", "Sigma_theta", "Sigma_kappa",
            "x", "xp", "Sigma_xdc",
        ],
        [
            "a", "ap", "rho", "J_xdc", "J_C", "J_theta", "J_kappa", "Sigma_res",
        ],
    )

    output_dir = Path(__file__).resolve().parents[1] / "generated" / "c" / "casadi"
    output_dir.mkdir(parents=True, exist_ok=True)
    generated_c_name = "triangulation_sigma_res_staged.c"

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
