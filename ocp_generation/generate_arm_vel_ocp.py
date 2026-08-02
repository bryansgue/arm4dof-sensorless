"""
OCP en NIVEL VELOCIDAD — dos variantes, para responder si hace falta identificar
el lazo interno del servo.

El MX-28 acepta velocidad, no par. Hay tres formulaciones posibles del OCP y este
archivo genera las dos de velocidad:

  T   (generate_arm_dqnmpc_ocp.py)  x=[q,qd], u=tau
      dinamica  M qdd + h = tau.  Se manda qd*[1] al servo.
      Es coherente con la fisica pero NO con la interfaz del actuador.

  V1  x=[q,qd], u = comando de velocidad
      dinamica  M qdd + kv qd + h = kv u        <- la planta de la ec. (5)
      Coherente con la interfaz. REQUIERE kv, o sea identificacion del servo.

  V0  x=[q],    u = comando de velocidad
      dinamica  qd = u                          <- limite cinematico kv -> inf
      Coherente con la interfaz. NO requiere NADA: sin kv, sin M, sin h.

La pregunta que decide el diseño de hardware es si V0 alcanza. Si alcanza, no hay
que identificar el servo y el controlador queda inmune a que kv cambie con la
temperatura, la carga o el firmware.

⚠️ V0 pierde las cotas de par. Se reponen como cota de velocidad y, si hace falta,
como cota sobre la fuerza estatica requerida.

Run:  python3 generate_arm_vel_ocp.py
"""
import os
import shutil
import numpy as np
import casadi as ca
from casadi import SX, vertcat
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel

from arm_dynamics import build_dynamics
from arm_kinematics import fk_ee_dq
from dq_math import dq_error, ln_dual
from generate_arm_dqnmpc_ocp import (NQ, T_HORIZON, N_HORIZON, Q_LO, Q_HI, QD_MAX,
                                     TAU_MAX as _TAU_VEC)
TAU_MAX = float(_TAU_VEC[0])

HERE = os.path.dirname(os.path.abspath(__file__))
KV_NOM = 20.0                 # ganancia nominal del lazo de velocidad del servo
N_PARAMS_V = 8                # solo dq_ref: el peso va por cost_set


def _common(ocp, name, export):
    ocp.code_export_directory = os.path.join(HERE, "..", export)
    ocp.solver_options.N_horizon = N_HORIZON
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


def build_ocp_v1(kv=KV_NOM):
    """x=[q,qd] (8), u = comando de velocidad (4). Usa la planta de la ec. (5)."""
    f_M, f_h = build_dynamics()
    m = AcadosModel(); m.name = "arm4dof_v1"
    q = SX.sym("q", NQ); qd = SX.sym("qd", NQ); u = SX.sym("u", NQ)
    p = SX.sym("p", N_PARAMS_V); m.p = p
    # M qdd + kv qd + h = kv u
    qdd = ca.solve(f_M(q), kv*(u - qd) - f_h(q, qd))
    f_expl = vertcat(qd, qdd)
    xdot = SX.sym("xdot", 2*NQ)
    m.x = vertcat(q, qd); m.u = u; m.xdot = xdot
    m.f_expl_expr = f_expl; m.f_impl_expr = xdot - f_expl

    ocp = AcadosOcp(); ocp.model = m
    dq_ee, _, _ = fk_ee_dq(q)
    e = ln_dual(dq_error(p[0:8], dq_ee))
    ocp.model.cost_y_expr = vertcat(e, qd, u)
    ocp.model.cost_y_expr_e = vertcat(e, qd)
    ocp.cost.cost_type = "NONLINEAR_LS"; ocp.cost.cost_type_e = "NONLINEAR_LS"
    W = [2., 2., 2., 80., 80., 80.] + [0.3]*4
    ocp.cost.W = np.diag(W + [0.02]*4); ocp.cost.W_e = np.diag(W)
    ocp.cost.yref = np.zeros(14); ocp.cost.yref_e = np.zeros(10)
    pdef = np.zeros(N_PARAMS_V); pdef[0] = 1.0; ocp.parameter_values = pdef

    ocp.constraints.idxbx = np.arange(2*NQ)
    ocp.constraints.lbx = np.concatenate([Q_LO, -QD_MAX*np.ones(NQ)])
    ocp.constraints.ubx = np.concatenate([Q_HI,  QD_MAX*np.ones(NQ)])
    ocp.constraints.idxbu = np.arange(NQ)
    ocp.constraints.lbu = -QD_MAX*np.ones(NQ); ocp.constraints.ubu = QD_MAX*np.ones(NQ)
    ocp.constraints.x0 = np.array([0., 1., -1., 0.5, 0, 0, 0, 0])

    # ⚠️ SIN ESTA RESTRICCION V1 NO ES UN MODELO DEL SERVO REAL.
    # Con u y qd acotados en +-QD_MAX, el termino kv(u-qd) llega a
    # 20*2*5.969 = 239 N.m, o sea 170x el par real del MX-28. El optimizador
    # explota una planta capaz de acelerar de forma absurda y el integrador
    # diverge: medido, 250/250 solves con ACADOS_NAN_DETECTED.
    # La saturacion del servo es parte del modelo, no un detalle de implementacion.
    tau_impl = kv*(u - qd)
    ocp.model.con_h_expr = tau_impl
    ocp.constraints.lh = -np.full(NQ, TAU_MAX)
    ocp.constraints.uh = np.full(NQ, TAU_MAX)
    return _common(ocp, m.name, "c_generated_code_arm_v1")


def build_ocp_v0():
    """x=[q] (4), u = comando de velocidad (4). Cinematico: qd = u.
    No usa M, ni h, ni kv: cero identificacion."""
    m = AcadosModel(); m.name = "arm4dof_v0"
    q = SX.sym("q", NQ); u = SX.sym("u", NQ)
    p = SX.sym("p", N_PARAMS_V); m.p = p
    xdot = SX.sym("xdot", NQ)
    m.x = q; m.u = u; m.xdot = xdot
    m.f_expl_expr = u; m.f_impl_expr = xdot - u

    ocp = AcadosOcp(); ocp.model = m
    dq_ee, _, _ = fk_ee_dq(q)
    e = ln_dual(dq_error(p[0:8], dq_ee))
    ocp.model.cost_y_expr = vertcat(e, u)
    ocp.model.cost_y_expr_e = e
    ocp.cost.cost_type = "NONLINEAR_LS"; ocp.cost.cost_type_e = "NONLINEAR_LS"
    Wse3 = [2., 2., 2., 80., 80., 80.]
    ocp.cost.W = np.diag(Wse3 + [0.3]*4); ocp.cost.W_e = np.diag(Wse3)
    ocp.cost.yref = np.zeros(10); ocp.cost.yref_e = np.zeros(6)
    pdef = np.zeros(N_PARAMS_V); pdef[0] = 1.0; ocp.parameter_values = pdef

    ocp.constraints.idxbx = np.arange(NQ)
    ocp.constraints.lbx = Q_LO; ocp.constraints.ubx = Q_HI
    ocp.constraints.idxbu = np.arange(NQ)
    ocp.constraints.lbu = -QD_MAX*np.ones(NQ); ocp.constraints.ubu = QD_MAX*np.ones(NQ)
    ocp.constraints.x0 = np.array([0., 1., -1., 0.5])
    return _common(ocp, m.name, "c_generated_code_arm_v0")


def _gen(ocp):
    cd = ocp.code_export_directory
    if os.path.isdir(cd): shutil.rmtree(cd)
    jf = os.path.join(os.path.dirname(cd), f"acados_ocp_{ocp.model.name}.json")
    if os.path.isfile(jf): os.remove(jf)
    print(f"generando {ocp.model.name} ...")
    AcadosOcpSolver(ocp, json_file=jf)
    print("  OK")


if __name__ == "__main__":
    _gen(build_ocp_v1())
    _gen(build_ocp_v0())
