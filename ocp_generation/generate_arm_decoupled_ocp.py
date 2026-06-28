"""
E4 baseline — NMPC con costo DESACOPLADO (pos + orient separados, SIN acople J_l⁻¹).
Identico al DQ-NMPC salvo el costo: se3_error_decoupled en vez de ln_dual.
Misma dinamica, mismos params (dq_ref(8)+Wse3(6)+Wqd(4)+Wu(4)), misma interfaz.
Sirve pra E4: comparar tracking DQ vs desacoplado (empate esperado, J_l⁻¹≈I).

Run:  python3 generate_arm_decoupled_ocp.py  -> ../c_generated_code_arm_decoupled/
"""
import os, shutil
import numpy as np
import casadi as ca
from casadi import MX, vertcat, diag
from acados_template import AcadosOcp, AcadosOcpSolver

import arm_params as P
from arm_dynamics import build_dynamics
from arm_kinematics import fk_ee_dq
from dq_math import dq_error, se3_error_decoupled
from generate_arm_dqnmpc_ocp import (build_arm_model, NQ, NX, NU, T_HORIZON, N_HORIZON,
                                     N_PARAMS, Q_LO, Q_HI, QD_MAX, TAU_MAX)


def build_ocp_decoupled():
    ocp = AcadosOcp()
    model, p_sym, q, qd, tau = build_arm_model()
    model.name = "arm4dof_decoupled"      # nombre distinto (no colisiona con el DQ)
    ocp.model = model

    script_dir = os.path.dirname(os.path.abspath(__file__))
    ocp.code_export_directory = os.path.join(script_dir, "..", "c_generated_code_arm_decoupled")
    ocp.solver_options.N_horizon = N_HORIZON

    dq_ref = p_sym[0:8]; W_se3 = p_sym[8:14]; W_qd = p_sym[14:18]; W_u = p_sym[18:22]
    dq_ee, _, _ = fk_ee_dq(q)
    e = se3_error_decoupled(dq_error(dq_ref, dq_ee))    # [φ; t_err] SIN J_l⁻¹

    stage = e.T @ diag(W_se3) @ e + qd.T @ diag(W_qd) @ qd + tau.T @ diag(W_u) @ tau
    term  = e.T @ diag(W_se3) @ e + qd.T @ diag(W_qd) @ qd
    ocp.cost.cost_type="EXTERNAL"; ocp.cost.cost_type_e="EXTERNAL"
    ocp.model.cost_expr_ext_cost=stage; ocp.model.cost_expr_ext_cost_e=term

    p_def=np.zeros(N_PARAMS); p_def[0]=1.0
    p_def[8:14]=np.array([2.,2.,2.,50.,50.,50.]); p_def[14:18]=0.5; p_def[18:22]=0.05
    ocp.parameter_values=p_def

    ocp.constraints.idxbx=np.arange(NX)
    ocp.constraints.lbx=np.concatenate([Q_LO,-QD_MAX*np.ones(NQ)])
    ocp.constraints.ubx=np.concatenate([Q_HI, QD_MAX*np.ones(NQ)])
    ocp.constraints.idxbu=np.arange(NU); ocp.constraints.lbu=-TAU_MAX; ocp.constraints.ubu=TAU_MAX
    ocp.constraints.x0=np.array([0.0,1.0,-1.0,0.5,0,0,0,0])

    ocp.solver_options.qp_solver="PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.qp_solver_cond_N=max(1,N_HORIZON//4)
    ocp.solver_options.qp_solver_iter_max=50; ocp.solver_options.qp_solver_warm_start=2
    ocp.solver_options.hessian_approx="GAUSS_NEWTON"; ocp.solver_options.regularize_method="PROJECT"
    ocp.solver_options.levenberg_marquardt=1e-2; ocp.solver_options.integrator_type="ERK"
    ocp.solver_options.nlp_solver_type="SQP_RTI"; ocp.solver_options.sim_method_num_stages=4
    ocp.solver_options.tol=1e-4; ocp.solver_options.tf=T_HORIZON
    return ocp


def main():
    ocp=build_ocp_decoupled(); cd=ocp.code_export_directory
    if os.path.isdir(cd): shutil.rmtree(cd)
    jf=os.path.join(os.path.dirname(cd), f"acados_ocp_{ocp.model.name}.json")
    if os.path.isfile(jf): os.remove(jf)
    print(f"Generando NMPC DESACOPLADO en {cd} ...")
    AcadosOcpSolver(ocp, json_file=jf); print("OK")

if __name__=="__main__":
    main()
