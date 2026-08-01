"""
E4-BIS — DQ (ln_dual) vs DESACOPLADO en el regimen donde las metricas SI se separan.
Reemplaza a test_e4_compare.py, que no podia detectar una diferencia. Ver
PLAN_MECHATRONICS.md, seccion "E4 revisado".

Lo que arregla:
  1. MISMO tipo de costo (los dos NONLINEAR_LS/Gauss-Newton). El E4 viejo comparaba
     NONLINEAR_LS contra EXTERNAL (Hessiano exacto) => confound de solver.
  2. MISMOS pesos. En el E4 viejo el DQ ignoraba p[8:22] (muerto en NONLINEAR_LS) y
     corria rot2/trans80/qd0.3 contra rot3/trans120/qd0.1 del baseline.
  3. Referencia INALCANZABLE: se perturba la orientacion objetivo un angulo α, asi que
     ‖φ‖ queda grande y persistente. El E4 viejo usaba referencia de JUNTAS (alcanzable)
     => error->0 => J_l⁻¹->I => las dos metricas son identicas POR ALGEBRA.
     α inalcanzable NO es artificial: con rank(J)=4<6 el 4DOF no alcanza se(3) completo,
     asi que es la condicion NORMAL del brazo.
  4. Error inicial GRANDE y transitorio INCLUIDO (el viejo arrancaba en la referencia
     y ademas tiraba pe[:50]).

α=0 es el CONTROL POSITIVO: referencia alcanzable, empate esperado.
Las dos metricas difieren 0.38% por grado de ‖φ‖ — ver phi_range en el plan.

Run:  python3 test_e4bis.py     (requiere los dos solvers generados)
"""
import os, time
import numpy as np
from acados_template import AcadosOcpSolver
import generate_arm_dqnmpc_ocp as Gdq
import generate_arm_decoupled_nls_ocp as Gdec
from arm_dynamics import build_dynamics
from arm_kinematics import build_fk_fn

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")

fM, fh = build_dynamics(); fk = build_fk_fn()


def dyn(x, tau):
    q, qd = x[:4], x[4:]
    return np.concatenate([qd, np.linalg.solve(np.array(fM(q)), tau - np.array(fh(q, qd)).flatten())])


def rk4(x, tau, dt):
    k1 = dyn(x, tau); k2 = dyn(x + dt/2*k1, tau); k3 = dyn(x + dt/2*k2, tau); k4 = dyn(x + dt*k3, tau)
    return x + dt/6*(k1 + 2*k2 + 2*k3 + k4)


def qmul(a, b):
    w1, x1, y1, z1 = a; w2, x2, y2, z2 = b
    return np.array([w1*w2-x1*x2-y1*y2-z1*z2, w1*x2+x1*w2+y1*z2-z1*y2,
                     w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2])


def dq_from(quat, t):
    return np.concatenate([quat, 0.5*qmul(np.array([0.0, t[0], t[1], t[2]]), quat)])


sol_dq  = AcadosOcpSolver(Gdq.build_ocp(),               json_file=os.path.join(ROOT, "acados_ocp_arm4dof_dqnmpc.json"),  build=False, generate=False)
sol_dec = AcadosOcpSolver(Gdec.build_ocp_decoupled_nls(), json_file=os.path.join(ROOT, "acados_ocp_arm4dof_dec_nls.json"), build=False, generate=False)

# pesos UNICOS para los dos (via cost_set: en NONLINEAR_LS el peso NO viaja en p)
W_SE3 = [3., 3., 3., 120., 120., 120.]; W_QD = [0.1]*4; W_U = [0.02]*4
for s in (sol_dq, sol_dec):
    for k in range(Gdq.N_HORIZON):
        s.cost_set(k, "W", np.diag(W_SE3 + W_QD + W_U))
    s.cost_set(Gdq.N_HORIZON, "W", np.diag(W_SE3 + W_QD))

Q_TGT = np.array([0.6, 1.4, -0.7, 0.9])                 # objetivo alcanzable en juntas
X0    = np.array([-0.5, 0.7, -1.4, -0.3, 0, 0, 0, 0])   # arranque LEJOS -> transitorio real
AXIS  = np.array([1.0, 1.0, 0.0]); AXIS /= np.linalg.norm(AXIS)


def make_ref(alpha_deg):
    _, p_t, q_t = fk(Q_TGT)
    p_t = np.array(p_t).flatten(); q_t = np.array(q_t).flatten()
    a = np.radians(alpha_deg)
    q_pert = np.concatenate([[np.cos(a/2)], np.sin(a/2)*AXIS])
    q_ref = qmul(q_pert, q_t)
    return dq_from(q_ref, p_t), p_t, q_ref


def run(solver, dq_ref, p_ref, q_ref, T=250):
    P = np.zeros(Gdq.N_PARAMS); P[0:8] = dq_ref
    x = X0.copy(); pe = []; oe = []; ts = []; bad = 0
    for _ in range(T):
        for k in range(Gdq.N_HORIZON+1):
            solver.set(k, "p", P)
        solver.set(0, "lbx", x); solver.set(0, "ubx", x)
        t0 = time.perf_counter(); st = solver.solve(); ts.append((time.perf_counter()-t0)*1e3)
        if st != 0: bad += 1
        x = rk4(x, solver.get(0, "u"), 0.01)
        _, p, quat = fk(x[:4])
        p = np.array(p).flatten(); quat = np.array(quat).flatten()
        pe.append(np.linalg.norm(p - p_ref))
        oe.append(2*np.degrees(np.arccos(min(1.0, abs(float(quat @ q_ref))))))
    return np.array(pe), np.array(oe), np.array(ts), bad


if __name__ == "__main__":
    print("== E4-BIS: mismo costo NONLINEAR_LS, mismos pesos, referencia INALCANZABLE ==")
    print("regimen permanente (ultimos 50 de 250 pasos); arranque lejos, transitorio incluido\n")
    print(f"{'a[deg]':>7} | {'DQ pos[mm]':>11} {'DEC pos[mm]':>12} {'dpos':>8} | "
          f"{'DQ ori[deg]':>12} {'DEC ori[deg]':>13} {'dori':>8} | fallos")
    print("-"*100)
    tt = []
    for alpha in [0, 15, 30, 45, 60, 75, 90]:
        dqr, pr, qr = make_ref(alpha)
        pe1, oe1, ts1, b1 = run(sol_dq,  dqr, pr, qr)
        pe2, oe2, ts2, b2 = run(sol_dec, dqr, pr, qr)
        P1, O1 = 1000*pe1[-50:].mean(), oe1[-50:].mean()
        P2, O2 = 1000*pe2[-50:].mean(), oe2[-50:].mean()
        dp = 100*(P2-P1)/max(P2, 1e-9); do = 100*(O2-O1)/max(O2, 1e-9)
        tt += [ts1.mean(), ts2.mean()]
        print(f"{alpha:7d} | {P1:11.3f} {P2:12.3f} {dp:+7.1f}% | {O1:12.4f} {O2:13.4f} {do:+7.1f}% | {b1}/{b2}")
    print("\n(d positivo = DQ mejor. a=0 = control positivo: referencia alcanzable, empate esperado)")
    print(f"solver: {np.mean(tt):.2f} ms medio en las dos ramas")
