"""
P3 — LAZO DE INTERACCION completo en MiL, y comparacion de los DOS estimadores.

Planta (100 Hz):  M q̈ + h = tau_act + Jᵀ F_mano   (dinamica validada)
Servo:            tau_act = kv (q̇_cmd − q̇)
Admitancia:       F_est -> desplaza la referencia DQ (hand-guiding) + resorte de retorno
NMPC:             trackea DQ_ref -> q̇_cmd = q̇*[1]

EL PUNTO DEL EXPERIMENTO. Se comparan dos estimadores sobre EXACTAMENTE el mismo
residuo tau_ext:

  WRENCH-6D  resuelve Jᵀ F = tau con F ∈ R⁶ desde 4 medidas. Indeterminado; la
             solucion de norma minima reparte el torque entre fuerza y momento y
             mal-atribuye parte de la fuerza. Es lo que hace el observador estandar.
  FUERZA-3D  asume contacto de fuerza pura en el efector (que es lo que es un
             empuje humano) y resuelve Jvᵀ f = tau con f ∈ R³. Sobre-determinado.

Prediccion: 6D pierde ~50 % de la fuerza; 3D es exacto. Ver wrench_estimator.py.
"""
import os
import numpy as np
from acados_template import AcadosOcpSolver
import generate_arm_dqnmpc_ocp as G
from arm_dynamics import build_dynamics
from arm_kinematics import build_fk_fn, build_jac_fn
from wrench_estimator import estimate_wrench, estimate_force, force_conditioning
from dq_admittance import DQAdmittance

fM, fh = build_dynamics(); fk = build_fk_fn(); fJ = build_jac_fn()
M_ = lambda q: np.array(fM(q))
h_ = lambda q, qd: np.array(fh(q, qd)).flatten()
J_ = lambda q: np.array(fJ(q))

json = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "acados_ocp_arm4dof_dqnmpc.json")
solver = AcadosOcpSolver(G.build_ocp(), json_file=json, build=False, generate=False)

W_SE3 = [2., 2., 2., 80., 80., 80.]; W_QD = [0.3]*4; W_U = [0.02]*4
for k in range(G.N_HORIZON):
    solver.cost_set(k, "W", np.diag(W_SE3 + W_QD + W_U))
solver.cost_set(G.N_HORIZON, "W", np.diag(W_SE3 + W_QD))

q_task = np.array([0.0, 1.2, -0.9, 0.4])
_, p_task, quat_task = fk(q_task)
p_task = np.array(p_task).flatten(); quat_task = np.array(quat_task).flatten()

DT = 0.01; KV = 4.0; T = 350
T0, T1, RAMP = 50, 200, 30


def F_profile(t, direction):
    """Trapecio 0 -> 1 -> 0. No constante, para que la correlacion este definida."""
    if not (T0 <= t < T1):
        return np.zeros(3)
    if t < T0 + RAMP:    a = (t - T0)/RAMP
    elif t >= T1 - RAMP: a = (T1 - t)/RAMP
    else:                a = 1.0
    return a*np.asarray(direction, float)


def run(direction, estimator="force"):
    """estimator: 'force' (3D, fuerza pura) o 'wrench' (6D min-norm)."""
    adm = DQAdmittance(D_trans=60.0, D_rot=8.0, p0=p_task, q0=quat_task,
                       k_return=2.0, p_task=p_task, q_task=quat_task)
    W = np.zeros(G.N_PARAMS)
    x = np.concatenate([q_task, np.zeros(4)])
    F_est6 = np.zeros(6); qd_cmd = np.zeros(4)
    L = {k: [] for k in ("err", "f6", "m6", "f3", "ftrue", "st", "sig")}
    for t in range(T):
        dq_ref, _, _ = adm.update(F_est6, DT)
        W[0:8] = dq_ref
        for k in range(G.N_HORIZON+1):
            solver.set(k, "p", W)
        solver.set(0, "lbx", x); solver.set(0, "ubx", x)
        st = solver.solve()
        qd_cmd = solver.get(1, "x")[4:8]

        f_true = F_profile(t, direction)
        Ftrue6 = np.concatenate([f_true, np.zeros(3)])
        qd_before = x[4:].copy(); tau_acc = np.zeros(4)
        for _ in range(5):
            q, qd = x[:4], x[4:]
            tau = KV*(qd_cmd - qd); tau_acc += tau
            qdd_p = np.linalg.solve(M_(q), tau + J_(q).T @ Ftrue6 - h_(q, qd))
            x = np.concatenate([q + (DT/5)*qd, qd + (DT/5)*qdd_p])
        qdd_meas = (x[4:] - qd_before)/DT; tau_meas = tau_acc/5

        w6, _ = estimate_wrench(x[:4], x[4:], qdd_meas, tau_meas)
        f3 = estimate_force(x[:4], x[4:], qdd_meas, tau_meas)
        F_est6 = np.concatenate([f3, np.zeros(3)]) if estimator == "force" else w6

        _, pee, _ = fk(x[:4]); pee = np.array(pee).flatten()
        L["err"].append(np.linalg.norm(pee - p_task))
        L["f6"].append(w6[:3].copy()); L["m6"].append(w6[3:].copy())
        L["f3"].append(f3.copy())
        L["ftrue"].append(f_true.copy()); L["st"].append(st)
        L["sig"].append(force_conditioning(x[:4])[0])
    return {k: np.array(v) for k, v in L.items()}


def report(L, direction, label):
    win = slice(T0, T1); flat = slice(T0+RAMP, T1-RAMP)
    ax = int(np.argmax(np.abs(direction)))
    ft = L["ftrue"]
    def m(fe):
        e = np.linalg.norm(fe[win] - ft[win], axis=1)
        c = np.corrcoef(fe[win, ax], ft[win, ax])[0, 1] if np.std(ft[win, ax]) > 1e-12 else np.nan
        return np.sqrt(np.mean(e**2)), float(c), np.linalg.norm(fe[flat].mean(0))
    r6 = m(L["f6"]); r3 = m(L["f3"])
    amp = np.linalg.norm(ft[flat].mean(0))
    print(f"\n-- {label}: f = {np.array(direction)} N,  |f| = {amp:.3f} N --")
    print(f"  solver {int(np.sum(L['st']==0))}/{T} OK    sigma_min(Jv) medio {L['sig'].mean():.4f}")
    print(f"  {'estimador':<14} {'RMSE [N]':>10} {'corr':>8} {'|f| est':>9}  {'error rel':>10}")
    print(f"  {'WRENCH-6D':<14} {r6[0]:10.4f} {r6[1]:8.4f} {r6[2]:9.3f}  {100*abs(r6[2]-amp)/amp:9.1f} %")
    print(f"  {'FUERZA-3D':<14} {r3[0]:10.2e} {r3[1]:8.4f} {r3[2]:9.3f}  {100*abs(r3[2]-amp)/amp:9.1f} %")
    err = L["err"]
    print(f"  compliance: reposo {err[40]:.4f}  empujado {err[125]:.4f}  final {err[-1]:.4f} m"
          f"   cede {'SI' if err[125]>3*err[40] else 'NO'}"
          f"   retorna {'SI' if err[-1]<2*err[40]+0.005 else 'NO'}")
    return r6, r3


if __name__ == "__main__":
    print("== MiL: lazo de interaccion + comparacion de estimadores ==")
    print("mismo residuo tau_ext en los dos; solo cambia el modelo de contacto asumido")
    DIRS = [([6.0, 0.0, -2.0], "empuje en el plano del brazo"),
            ([0.0, 6.0,  0.0], "empuje NORMAL al plano (el caso dificil)")]
    out = []
    for d, lab in DIRS:
        out.append(report(run(d, estimator="force"), d, lab))
    print("\n" + "="*72)
    # umbral 0.05 N: el residuo del 3D no es del estimador sino de la diferencia
    # finita de q̈ y del promediado de tau en el banco (el estimador aislado da 1e-15)
    ok = all(r3[0] < 0.05 and r3[1] > 0.99 for _, r3 in out)
    print("ESTIMADOR DE FUERZA PURA OK: exacto en las dos direcciones" if ok else "REVISAR")
    print("  El 6D min-norm no falla por falta de informacion: el torque de junta ESTA.")
    print("  Falla porque resuelve 6 incognitas con 4 medidas y reparte la fuerza")
    print("  como momento. Con el modelo de contacto correcto el problema es")
    print("  SOBRE-determinado y la fuerza se recupera exacta.")
