import sympy as sp
from pathlib import Path
import json


def skew(wx, wy, wz):
    return sp.Matrix([
        [0, -wz, wy],
        [wz, 0, -wx],
        [-wy, wx, 0]
    ])


def expand_small_integer_powers(expr):
    def _pow_to_mul(node):
        if isinstance(node, sp.Pow):
            base, exp = node.as_base_exp()
            if exp.is_Integer and int(exp) >= 2:
                n = int(exp)
                return sp.Mul(*([base] * n), evaluate=False)
        return node

    return expr.replace(
        lambda e: isinstance(e, sp.Pow) and e.exp.is_Integer and int(e.exp) >= 2,
        _pow_to_mul,
    )


def cxxcode_no_pow(expr):
    return sp.cxxcode(expand_small_integer_powers(expr))


# Params to optimize over
# translation increment 
tx, ty, tz = sp.symbols("tx ty tz", real=True)
t = sp.Matrix([tx, ty, tz])

# # rotation increments
# rx, ry, rz = sp.symbols("rx ry rz", real=True)
# # First order approximation of R
# R = sp.eye(3) + skew(rx, ry, rz)

qx, qy, qz, qw = sp.symbols("qx qy qz qw", real=True)

R = sp.Matrix([
    [1 - 2*(qy**2 + qz**2),     2*(qx*qy - qw*qz),     2*(qx*qz + qw*qy)],
    [    2*(qx*qy + qw*qz), 1 - 2*(qx**2 + qz**2),     2*(qy*qz - qw*qx)],
    [    2*(qx*qz - qw*qy),     2*(qy*qz + qw*qx), 1 - 2*(qx**2 + qy**2)],
])


# some point x_i in point cloud X_curr
Xx, Xy, Xz = sp.symbols("Xx Xy Xz", real=True)
X = sp.Matrix([Xx, Xy, Xz])

# some point y_j in point cloud Y_curr
Yx, Yy, Yz = sp.symbols("Yx Yy Yz", real=True)
Y = sp.Matrix([Yx, Yy, Yz])

# inverse covaraince elements of x_i
Xisxx, Xisxy, Xisxz, Xisyy, Xisyz, Xiszz = sp.symbols(
    "Xisxx Xisxy Xisxz Xisyy Xisyz Xiszz", real=True
    )
iSX = sp.Matrix([
    [Xisxx, Xisxy, Xisxz],
    [Xisxy, Xisyy, Xisyz],
    [Xisxz, Xisyz, Xiszz]
])

# inverse covaraince elements of y_j
Yisxx, Yisxy, Yisxz, Yisyy, Yisyz, Yiszz = sp.symbols(
    "Yisxx Yisxy Yisxz Yisyy Yisyz Yiszz", real=True
    )
iSY = sp.Matrix([
    [Yisxx, Yisxy, Yisxz],
    [Yisxy, Yisyy, Yisyz],
    [Yisxz, Yisyz, Yiszz]
])

# Euclidiean distance
d = (R*Y + t) - X

# C_ij_old
C_ij = sp.symbols("C_ij", real=True)

# Pairwise energy
L_ij = sp.expand(C_ij * (d.T * (iSX + R*iSY*R.T) * d)[0])

# print("L_ij =")
# sp.pprint(L_ij)
# print()
print("Number of additive terms in expanded L_ij:", len(sp.Add.make_args(L_ij)))

# poly = sp.Poly(sp.expand(L_ij / C_ij), tx, ty, tz, rx, ry, rz)
# print("Number of monomials in optimizer variables:", len(poly.monoms()))
# print("Total degree in optimizer variables:", poly.total_degree())

poly = sp.Poly(sp.expand(L_ij / C_ij), tx, ty, tz, qx, qy, qz, qw)
print("Number of monomials in optimizer variables:", len(poly.monoms()))
print("Total degree in optimizer variables:", poly.total_degree())

opt_vars = [tx, ty, tz, qx, qy, qz, qw]


# def monom_to_expr(monom, vars_):
#     expr = sp.Integer(1)
#     for var, power in zip(vars_, monom):
#         if power:
#             expr *= var**power
#     return expr

# view_path = Path("bot_dugma/m_step_opt/view.txt")

# with view_path.open("w", encoding="utf-8") as f:
#     f.write("Pairwise energy symbolic view\n")
#     f.write("============================\n\n")

#     f.write(f"Number of additive terms in expanded L_ij: {len(sp.Add.make_args(L_ij))}\n")
#     f.write(f"Number of monomials in optimizer variables: {len(poly.monoms())}\n")
#     f.write(f"Total degree in optimizer variables: {poly.total_degree()}\n\n")

#     f.write("Optimizer variable order:\n")
#     f.write(", ".join(str(v) for v in opt_vars))
#     f.write("\n\n")

#     f.write("Monomials and coefficients of L_ij / C_ij\n")
#     f.write("========================================\n\n")

#     for idx, (monom, coeff) in enumerate(zip(poly.monoms(), poly.coeffs()), start=1):
#         monom_expr = monom_to_expr(monom, opt_vars)

#         f.write(f"[{idx}]\n")
#         f.write(f"Exponent tuple : {monom}\n")
#         f.write(f"Monomial B_k   : {sp.sstr(monom_expr)}\n")
#         f.write(f"Coeff G_ijk    : {sp.sstr(coeff)}\n")
#         f.write("\n")

# print(f"Wrote symbolic view to: {view_path}")

# basis_json_path = Path("bot_dugma/m_step_opt/basis_exponents.json")

# with basis_json_path.open("w", encoding="utf-8") as f:
#     f.write("{\n")
#     f.write(f'  "variable_order": {json.dumps([str(v) for v in opt_vars])},\n')
#     f.write(f'  "basis_size": {len(poly.monoms())},\n')
#     f.write(f'  "total_degree": {poly.total_degree()},\n')
#     f.write('  "basis_exponents": [\n')

#     monoms = poly.monoms()
#     for idx, monom in enumerate(monoms):
#         comma = "," if idx < len(monoms) - 1 else ""
#         f.write(f"    {json.dumps(list(monom))}{comma}\n")

#     f.write("  ]\n")
#     f.write("}\n")

# print(f"Wrote basis JSON to: {basis_json_path}")

# g_cpp_preview_path = Path("bot_dugma/m_step_opt/g_cpp_preview.txt")

# preview_count = 3
# monoms = poly.monoms()
# coeffs = poly.coeffs()

# with g_cpp_preview_path.open("w", encoding="utf-8") as f:
#     f.write("Preview of first few G_ijk as C++ expressions\n")
#     f.write("=============================================\n\n")

#     for k in range(preview_count):
#         monom = monoms[k]
#         coeff = coeffs[k]
#         monom_expr = monom_to_expr(monom, opt_vars)

#         f.write(f"// k = {k + 1}\n")
#         f.write(f"// Exponent tuple : {monom}\n")
#         f.write(f"// B_k(u)         : {sp.sstr(monom_expr)}\n")
#         f.write(f"const float G_{k + 1} = {sp.cxxcode(coeff)};\n")
#         f.write("\n")

#     f.write(f"// k = {636}\n")
#     f.write(f"// Exponent tuple : {monoms[636]}\n")
#     f.write(f"// B_k(u)         : {sp.sstr(monom_to_expr(monoms[636], opt_vars))}\n")
#     f.write(f"const float G_{636} = {sp.cxxcode(coeffs[636])};\n")
#     f.write("\n")

# print(f"Wrote C++ G preview to: {g_cpp_preview_path}")

# all_coeffs = poly.coeffs()
# all_monoms = poly.monoms()

# repls_all, reduced_all = sp.cse(all_coeffs)

# g_all_cpp_path = Path("bot_dugma/m_step_opt/g_all_cse_cpp.txt")

# with g_all_cpp_path.open("w", encoding="utf-8") as f:
#     f.write("Full CSE-generated C++ preview for all G_k expressions\n")
#     f.write("======================================================\n\n")

#     f.write(f"Basis size: {len(all_coeffs)}\n")
#     f.write(f"Number of CSE temporaries: {len(repls_all)}\n\n")

#     f.write("Temporaries\n")
#     f.write("-----------\n\n")
#     for sym, rhs in repls_all:
#         f.write(f"const float {sym} = {sp.cxxcode(rhs)};\n")

#     f.write("\n")
#     f.write("Final coefficient assignments\n")
#     f.write("-----------------------------\n\n")

#     for idx, (monom, expr) in enumerate(zip(all_monoms, reduced_all)):
#         monom_expr = monom_to_expr(monom, opt_vars)

#         f.write(f"// k = {idx + 1}\n")
#         f.write(f"// Exponent tuple : {monom}\n")
#         f.write(f"// B_k(u)         : {sp.sstr(monom_expr)}\n")
#         f.write(f"G[{idx}] = {sp.cxxcode(expr)};\n")
#         f.write("\n")

# print(f"Wrote full CSE C++ preview to: {g_all_cpp_path}")

# all_coeffs = poly.coeffs()
# all_monoms = poly.monoms()

# repls_all, reduced_all = sp.cse(all_coeffs)

# inl_path = Path("bot_dugma/m_step_opt/g_pair_coeffs_generated.inl")

# with inl_path.open("w", encoding="utf-8") as f:
#     f.write("// Generated by derive_pairwise_energy_term.py\n")
#     f.write("// Pairwise coefficient expressions G[k] for one (i, j) pair\n")
#     f.write(f"// Basis size: {len(all_coeffs)}\n")
#     f.write(f"// CSE temporaries: {len(repls_all)}\n\n")

#     f.write("// Temporaries\n")
#     for sym, rhs in repls_all:
#         f.write(f"const float {sym} = {cxxcode_no_pow(rhs)};\n")

#     f.write("\n// Final coefficient assignments\n")
#     for idx, (monom, expr) in enumerate(zip(all_monoms, reduced_all)):
#         monom_expr = monom_to_expr(monom, opt_vars)
#         f.write(f"\n// k = {idx + 1}\n")
#         f.write(f"// Exponent tuple : {monom}\n")
#         f.write(f"// B_k(u)         : {sp.sstr(monom_expr)}\n")
#         f.write(f"G[{idx}] = {cxxcode_no_pow(expr)};\n")

# print(f"Wrote generated include to: {inl_path}")
