"""
Baseline DESACOPLADO en NONLINEAR_LS — para el E4-BIS.

Por que existe habiendo ya generate_arm_decoupled_ocp.py: aquel es EXTERNAL, o sea
Hessiano EXACTO, mientras el DQ es NONLINEAR_LS (Gauss-Newton). Compararlos mezcla el
efecto de la METRICA con el del solver. Aca los dos son NONLINEAR_LS con los mismos
pesos, y la UNICA diferencia es el residuo:

  DQ           y = [ln_dual(dq_err); q̇; τ]        rho = J_l⁻¹ t_err   (acoplado)
  DESACOPLADO  y = [se3_decoupled(dq_err); q̇; τ]  t_err crudo         (sin acople)

Run:  python3 generate_arm_decoupled_nls_ocp.py  -> ../c_generated_code_arm_dec_nls/
"""
import os, shutil
import numpy as np
from casadi import vertcat
from acados_template import AcadosOcp, AcadosOcpSolver

from arm_kinematics import fk_ee_dq
from dq_math import dq_error, se3_error_decoupled
from generate_arm_dqnmpc_ocp import (build_arm_model, NQ, NX, NU, T_HORIZON, N_HORIZON,
                                     N_PARAMS, Q_LO, Q_HI, QD_MAX, TAU_MAX)


def build_ocp_decoupled_nls():
    ocp = AcadosOcp()
    model, p_sym, q, qd, tau = build_arm_model()
    model.name = "arm4dof_dec_nls"
    ocp.model = model

    script_dir = os.path.dirname(os.path.abspath(__file__))
    ocp.code_export_directory = os.path.join(script_dir, "..", "c_generated_code_arm_dec_nls")
    ocp.solver_options.N_horizon = N_HORIZON

    dq_ee, _, _ = fk_ee_dq(q)
    e = se3_error_decoupled(dq_error(p_sym[0:8], dq_ee))     # SIN J_l⁻¹

    ocp.model.cost_y_expr   = vertcat(e, qd, tau)
    ocp.model.cost_y_expr_e = vertcat(e, qd)
    ocp.cost.cost_type   = "NONLINEAR_LS"
    ocp.cost.cost_type_e = "NONLINEAR_LS"
    Wd_se3 = [2., 2., 2., 80., 80., 80.]; Wd_qd = [0.3]*4; Wd_u = [0.02]*4
    ocp.cost.W   = np.diag(Wd_se3 + Wd_qd + Wd_u)
    ocp.cost.W_e = np.diag(Wd_se3 + Wd_qd)
    ocp.cost.yref   = np.zeros(14)
    ocp.cost.yref_e = np.zeros(10)

    p_def = np.zeros(N_PARAMS); p_def[0] = 1.0
    ocp.parameter_values = p_def

    ocp.constraints.idxbx = np.arange(NX)
    ocp.constraints.lbx = np.concatenate([Q_LO, -QD_MAX*np.ones(NQ)])
    ocp.constraints.ubx = np.concatenate([Q_HI,  QD_MAX*np.ones(NQ)])
    ocp.constraints.idxbu = np.arange(NU)
    ocp.constraints.lbu = -TAU_MAX; ocp.constraints.ubu = TAU_MAX
    ocp.constraints.x0 = np.array([0.0, 1.0, -1.0, 0.5, 0, 0, 0, 0])

    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.qp_solver_cond_N = max(1, N_HORIZON // 4)
    ocp.solver_options.qp_solver_iter_max = 50
    ocp.solver_options.qp_solver_warm_start = 2
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.regularize_method = "PROJECT"
    ocp.solver_options.levenberg_marquardt = 1e-2
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.tol = 1e-4
    ocp.solver_options.tf = T_HORIZON
    return ocp


def main():
    ocp = build_ocp_decoupled_nls()
    cd = ocp.code_export_directory
    if os.path.isdir(cd): shutil.rmtree(cd)
    jf = os.path.join(os.path.dirname(cd), f"acados_ocp_{ocp.model.name}.json")
    if os.path.isfile(jf): os.remove(jf)
    print(f"Generando baseline DESACOPLADO-NLS en {cd} ...")
    AcadosOcpSolver(ocp, json_file=jf)
    print("OK")


if __name__ == "__main__":
    main()
