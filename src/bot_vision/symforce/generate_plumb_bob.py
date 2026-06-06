import symforce
symforce.set_epsilon_to_symbol()

import symforce.symbolic as sf
from symforce.values import Values
from symforce.codegen import Codegen
from symforce.codegen.backends.cpp.cpp_config import CppConfig

from pathlib import Path


def main():
    u, v = sf.symbols("u v")
    f_x, f_y, c_x, c_y = sf.symbols("fx fy cx cy")
    k_1, k_2, p_1, p_2, k_3 = sf.symbols("k1 k2 p1 p2 k3")

    sigma_xx, sigma_xy, sigma_yx, sigma_yy = sf.symbols("sigma_xx, sigma_xy, sigma_yx, sigma_yy")

    Sigma_d = sf.Matrix([
        [sigma_xx, sigma_xy],
        [sigma_yx, sigma_yy],
    ])

    x = (u - c_x) / f_x
    y = (v - c_y) / f_y
    r2 = x**2 + y**2
    L = 1 + k_1*r2 + k_2*r2**2 + k_3*r2**3

    x_d = x*L + 2*p_1*x*y + p_2*(r2 + 2*x**2)
    y_d = y*L + p_1*(r2 + 2*y**2) + 2*p_2*x*y

    u_d = f_x * x_d + c_x
    v_d = f_y * y_d + c_y

    result = sf.Matrix([u_d, v_d])

    J = result.jacobian(sf.Matrix([u, v]))
    J_inv = J.inv()

    Sigma_ud = J_inv * Sigma_d * J_inv.T
    

    inputs = Values(
        uv=sf.Matrix([u, v]),
        intrinsics=sf.Matrix([f_x, f_y, c_x, c_y]),
        distortion=sf.Matrix([k_1, k_2, p_1, p_2, k_3]),
        sigma_d=Sigma_d
    )

    outputs = Values(
        sigma_ud=Sigma_ud,
    )

    output_dir=Path(__file__).resolve().parents[1] / "generated"

    codegen = Codegen(
        name="plumb_bob_undistorted_covariance",
        inputs=inputs,
        outputs=outputs,
        config=CppConfig(),
    )

    codegen.generate_function(
        output_dir=output_dir,
        namespace="bot_vision_generated"
    )


if __name__ == "__main__":
    main()
