import symforce
symforce.set_epsilon_to_symbol()

import symforce.symbolic as sf
from symforce.values import Values
from symforce.codegen import Codegen
from symforce.codegen.backends.cpp.cpp_config import CppConfig

from pathlib import Path


def main():
    x, y = sf.symbols("x y")

    f_x, f_y, c_x, c_y = sf.symbols("fx fy cx cy")
    K = sf.Matrix([
        [f_x, 0, c_x],
        [0, f_y, c_y],
        [0, 0, 1],
    ])
    

    

    x_dc =sf.Matrix([
        [x],
        [y],
        [1]
    ])

    a = K.inv() * x_dc
    a = a / a.norm()
