from casadi import *
from pathlib import Path
import os


def skew(v):
    S = MX.zeros(3, 3)
    S[0, 1] = -v[2]
    S[0, 2] =  v[1]
    S[1, 0] =  v[2]
    S[1, 2] = -v[0]
    S[2, 0] = -v[1]
    S[2, 1] =  v[0]
    return S


def main():
    fx = MX.sym("fx")
    fy = MX.sym("fy")
    cx = MX.sym("cx")
    cy = MX.sym("cy")

    K = MX.zeros(3, 3)
    K[0, 0] = fx
    K[1, 1] = fy
    K[0, 2] = cx
    K[1, 2] = cy
    K[2, 2] = 1

    x = MX.sym("x", 3, 1)      # homogeneous distortion-corrected pixel
    X = MX.sym("X", 3, 1)      # world point
    C = MX.sym("C", 3, 1)      # camera center
    R = MX.sym("R", 3, 3)      # camera rotation

    los = inv(K) @ x
    residual = skew(los) @ R @ (X - C)

    J_x = jacobian(residual, x)
    J_C = jacobian(residual, C)

    f = Function(
        "triangulationterms",
        [x, X, C, R, fx, fy, cx, cy],
        [residual, J_x, J_C],
        ["x", "X", "C", "R", "fx", "fy", "cx", "cy"],
        ["residual", "J_x", "J_C"],
    )

    output_dir = Path(__file__).resolve().parents[1] / "generated" / "c" / "casadi"
    output_dir.mkdir(parents=True, exist_ok=True)

    old_cwd = os.getcwd()
    os.chdir(output_dir)
    try:
        f.generate("triangulationterms.c", {"with_header": True})
    finally:
        os.chdir(old_cwd)

    print(f)
    print("Generated code in:", output_dir)


if __name__ == "__main__":
    main()
