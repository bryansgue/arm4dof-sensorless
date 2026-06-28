"""
Genera el codigo C acados para el DQ-NMPC del brazo 4DOF (MX-28).

Estado x ∈ ℝ⁸ = [q(4), q̇(4)]
Control u ∈ ℝ⁴ = τ(4)              (interno; salida al servo = q̇*[1] -> Goal_Velocity)
Dinamica:  q̈ = M(q)⁻¹ (τ − h(q,q̇))   (M,h simbolicos validados vs MuJoCo a 1e-10)
Costo (EXTERNAL):
  e = ln_dual( dq_error(dq_ref, fk_ee_dq(q)) ) ∈ se(3)   (error de pose efector 6D)
  ℓ = eᵀ W_se3 e + q̇ᵀ W_qd q̇ + uᵀ W_u u
Tarea reducida 4D implicita: 4DOF no alcanza se(3) 6D completo; el optimizador minimiza el
subespacio alcanzable (rank(J)=4). Dinamica JOINT-SPACE (4×4 bien condicionada) esquiva la
singularidad de la inercia task-space (rank(J M⁻¹Jᵀ)=4<6).

Numerica reusada del DQ-MPCC del dron: regularize_method=PROJECT (CONVEXIFY falla pra DQ),
levenberg_marquardt=1e-2, HPIPM, GAUSS_NEWTON, SQP_RTI, ERK.

Run:  python3 generate_arm_dqnmpc_ocp.py   ->  ../c_generated_code_arm_dqnmpc/
"""
import os, shutil
import numpy as np
import casadi as ca
from casadi import MX, vertcat, diag
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel

import arm_params as P
from arm_dynamics import build_dynamics
from arm_kinematics import fk_ee_dq
from dq_math import dq_error, ln_dual

# ── Dimensiones ───────────────────────────────────────────────────────────────
NQ = 4
NX = 8            # [q(4), q̇(4)]
NU = 4            # τ(4)
T_HORIZON = 1.0   # [s]
N_HORIZON = 20    # nodo dt = 50 ms
N_PARAMS  = 8 + 6 + 4 + 4   # dq_ref(8) + W_se3(6) + W_qd(4) + W_u(4) = 22

# limites MX-28 (de los rangos de junta + specs servo)
Q_LO = np.array([-3.1416, -0.436332, -2.00713, -2.00713])
Q_HI = np.array([ 3.1416,  3.57792,  2.00713,  2.00713])
QD_MAX = 5.969                 # [rad/s] vel servo
TAU_MAX = np.array([1.4,1.4,1.4,1.4])


def build_arm_model():
    f_M, f_h = build_dynamics()

    model = AcadosModel()
    model.name = "arm4dof_dqnmpc"

    q  = MX.sym("q", NQ)
    qd = MX.sym("qd", NQ)
    x  = vertcat(q, qd)
    tau = MX.sym("tau", NU)

    p_sym = MX.sym("p", N_PARAMS)
    model.p = p_sym

    # dinamica: q̈ = M⁻¹(τ − h)   (ca.solve = mas estable que inv)
    M = f_M(q)
    h = f_h(q, qd)
    qdd = ca.solve(M, tau - h)
    f_expl = vertcat(qd, qdd)

    xdot = MX.sym("xdot", NX)
    model.x = x
    model.xdot = xdot
    model.u = tau
    model.f_expl_expr = f_expl
    model.f_impl_expr = xdot - f_expl
    return model, p_sym, q, qd, tau


def build_ocp():
    ocp = AcadosOcp()
    model, p_sym, q, qd, tau = build_arm_model()
    ocp.model = model

    script_dir = os.path.dirname(os.path.abspath(__file__))
    ocp.code_export_directory = os.path.join(script_dir, "..", "c_generated_code_arm_dqnmpc")
    ocp.solver_options.N_horizon = N_HORIZON

    # params runtime
    dq_ref = p_sym[0:8]
    W_se3  = p_sym[8:14]
    W_qd   = p_sym[14:18]
    W_u    = p_sym[18:22]

    # error de pose efector en se(3)
    dq_ee, _, _ = fk_ee_dq(q)
    e_se3 = ln_dual(dq_error(dq_ref, dq_ee))     # [φ(3); ρ(3)]

    stage = (e_se3.T @ diag(W_se3) @ e_se3
             + qd.T @ diag(W_qd) @ qd
             + tau.T @ diag(W_u) @ tau)
    term  = (e_se3.T @ diag(W_se3) @ e_se3
             + qd.T @ diag(W_qd) @ qd)

    ocp.cost.cost_type   = "EXTERNAL"
    ocp.cost.cost_type_e = "EXTERNAL"
    ocp.model.cost_expr_ext_cost   = stage
    ocp.model.cost_expr_ext_cost_e = term

    # defaults: referencia = pose home; pesos sanos
    p_def = np.zeros(N_PARAMS)
    # dq_ref default = identidad rot + pos cero (se sobreescribe en runtime)
    p_def[0] = 1.0           # qw=1
    p_def[8:14]  = np.array([2.0,2.0,2.0, 50.0,50.0,50.0])  # W_se3: [rot, trans]
    p_def[14:18] = 0.5       # W_qd (suavidad velocidad)
    p_def[18:22] = 0.05      # W_u  (esfuerzo)
    ocp.parameter_values = p_def

    # constraints de estado: q en rango + q̇ en ±QD_MAX
    ocp.constraints.idxbx = np.arange(NX)
    ocp.constraints.lbx = np.concatenate([Q_LO, -QD_MAX*np.ones(NQ)])
    ocp.constraints.ubx = np.concatenate([Q_HI,  QD_MAX*np.ones(NQ)])
    # control: τ en ±TAU_MAX
    ocp.constraints.idxbu = np.arange(NU)
    ocp.constraints.lbu = -TAU_MAX
    ocp.constraints.ubu =  TAU_MAX

    x0 = np.array([0.0, 1.0, -1.0, 0.5, 0,0,0,0])
    ocp.constraints.x0 = x0

    # solver (numerica del DQ-MPCC del dron)
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.qp_solver_cond_N = max(1, N_HORIZON // 4)
    ocp.solver_options.qp_solver_iter_max = 50
    ocp.solver_options.qp_solver_warm_start = 2
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.regularize_method = "PROJECT"      # CONVEXIFY falla pra DQ
    ocp.solver_options.levenberg_marquardt = 1e-2
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.tol = 1e-4
    ocp.solver_options.tf = T_HORIZON
    return ocp


def main():
    ocp = build_ocp()
    code_dir = ocp.code_export_directory
    if os.path.isdir(code_dir):
        shutil.rmtree(code_dir)
    json_file = os.path.join(os.path.dirname(code_dir), f"acados_ocp_{ocp.model.name}.json")
    if os.path.isfile(json_file):
        os.remove(json_file)
    print(f"Generando DQ-NMPC del brazo en {code_dir} ...")
    solver = AcadosOcpSolver(ocp, json_file=json_file)
    print("OK. Generado.")
    return solver


if __name__ == "__main__":
    main()
