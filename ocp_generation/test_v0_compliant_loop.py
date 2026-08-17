"""
EL LAZO COMPLIANT COMPLETO CON CONTROLADOR CON MODELO (T) Y SIN MODELO (V0).

⚠️ POR QUE EXISTE ESTE ARCHIVO. La Tabla `tab:v0loop` del paper era la ULTIMA sin
script propio, y no es una tabla menor: de ella sale el factor ~25 de costo que el
ABSTRACT y la CONCLUSION citan. El otro numero parecido del paper (~27) es de
`tab:formulations`, que es OTRO experimento — regulacion a una pose, no el lazo de
interaccion completo. Confundirlos ya estuvo a punto de pasar; no son el mismo.

Es el mismo defecto que la auditoria del 01/08/2026 creo `reproduce_paper_tables.py`
para eliminar, y que en `tab:regimes` termino con los numeros CAMBIADOS cuando por
fin se le escribio el script. Por eso este archivo imprime lo medido y NO contiene
los valores publicados: si no coinciden, se corrige el PAPER, nunca el script.

QUE COMPARA. El lazo entero de `test_interaction_mil.py` —planta validada, servo en
velocidad, estimador de fuerza pura, admitancia que desplaza la referencia— corrido
dos veces cambiando UNA sola cosa, el controlador:

  T   x = [q, qd] (8 estados), OCP en par sobre la dinamica completa (ec. 1).
      Usa M y h. Al servo se le manda la velocidad predicha qd*[1].
  V0  x = [q] (4 estados), OCP cinematico qd = u, el limite de la ec. 5 con
      kv -> infinito. No usa M, no usa h, no usa kv. Al servo se le manda u*[0].

El ESTIMADOR es identico en las dos ramas y conserva el modelo dinamico, que si
necesita. La pregunta del experimento es si el CONTROLADOR lo necesita.

Run:  python3 test_v0_compliant_loop.py
"""
import os
import time
import numpy as np
from acados_template import AcadosOcpSolver

import generate_arm_dqnmpc_ocp as GT
import generate_arm_vel_ocp as GV
from arm_dynamics import build_dynamics
from arm_kinematics import build_fk_fn, build_jac_fn
from wrench_estimator import estimate_force
from dq_admittance import DQAdmittance

fM, fh = build_dynamics(); fk = build_fk_fn(); fJ = build_jac_fn()
M_ = lambda q: np.array(fM(q))
h_ = lambda q, qd: np.array(fh(q, qd)).flatten()
J_ = lambda q: np.array(fJ(q))

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")

q_task = np.array([0.0, 1.2, -0.9, 0.4])
_, p_task, quat_task = fk(q_task)
p_task = np.array(p_task).flatten(); quat_task = np.array(quat_task).flatten()

# ⚠️ Constantes IDENTICAS a test_interaction_mil.py. kv = 4 es la planta de este
# experimento y del de los regimenes; test_vel_vs_torque.py corre a kv = 20. Las
# dos estan declaradas en el paper desde la ronda 5; antes no lo estaban.
DT = 0.01; KV = 4.0; T = 350
T0, T1, RAMP = 50, 200, 30


def F_profile(t, direction):
    """Trapecio 0 -> 1 -> 0, igual que en el MiL."""
    if not (T0 <= t < T1):
        return np.zeros(3)
    if t < T0 + RAMP:    a = (t - T0)/RAMP
    elif t >= T1 - RAMP: a = (T1 - t)/RAMP
    else:                a = 1.0
    return a*np.asarray(direction, float)


def load():
    """Los dos solvers, con los pesos con que se reporta cada uno."""
    sT = AcadosOcpSolver(GT.build_ocp(),
                         json_file=os.path.join(ROOT, "acados_ocp_arm4dof_dqnmpc.json"),
                         build=False, generate=False)
    W_SE3 = [2., 2., 2., 80., 80., 80.]
    for k in range(GT.N_HORIZON):
        sT.cost_set(k, "W", np.diag(W_SE3 + [0.3]*4 + [0.02]*4))
    sT.cost_set(GT.N_HORIZON, "W", np.diag(W_SE3 + [0.3]*4))

    sV0 = AcadosOcpSolver(GV.build_ocp_v0(),
                          json_file=os.path.join(ROOT, "acados_ocp_arm4dof_v0.json"),
                          build=False, generate=False)
    # ⚠️ mismo peso de comando que en test_vel_vs_torque.py: V0 no tiene estado de
    # velocidad, asi que su unico termino de regularizacion es el comando.
    for k in range(GV.N_HORIZON):
        sV0.cost_set(k, "W", np.diag(W_SE3 + [0.01]*4))
    sV0.cost_set(GV.N_HORIZON, "W", np.diag(W_SE3))
    return sT, sV0


def run(solver, kind, direction):
    """kind: 'T' (par, 8 estados) o 'V0' (cinematico, 4 estados)."""
    adm = DQAdmittance(D_trans=60.0, D_rot=8.0, p0=p_task, q0=quat_task,
                       k_return=2.0, p_task=p_task, q_task=quat_task)
    N = GT.N_HORIZON if kind == "T" else GV.N_HORIZON
    p = np.zeros(GT.N_PARAMS if kind == "T" else GV.N_PARAMS_V)
    x = np.concatenate([q_task, np.zeros(4)])
    F_est6 = np.zeros(6)
    err = []; fest = []; ftrue = []; ts = []; fails = 0

    for t in range(T):
        dq_ref, _, _ = adm.update(F_est6, DT)
        p[0:8] = dq_ref
        for k in range(N+1):
            solver.set(k, "p", p)
        x0 = x if kind == "T" else x[:4]
        solver.set(0, "lbx", x0); solver.set(0, "ubx", x0)

        t_ini = time.perf_counter()
        st = solver.solve()
        ts.append((time.perf_counter() - t_ini)*1e3)
        if st != 0:
            fails += 1
        qd_cmd = solver.get(1, "x")[4:8] if kind == "T" else solver.get(0, "u")

        # ── planta: identica en las dos ramas ──────────────────────────────
        f_true = F_profile(t, direction)
        F6 = np.concatenate([f_true, np.zeros(3)])
        qd_before = x[4:].copy(); tau_acc = np.zeros(4)
        for _ in range(5):
            q, qd = x[:4], x[4:]
            tau = KV*(qd_cmd - qd); tau_acc += tau
            qdd = np.linalg.solve(M_(q), tau + J_(q).T @ F6 - h_(q, qd))
            x = np.concatenate([q + (DT/5)*qd, qd + (DT/5)*qdd])
        qdd_meas = (x[4:] - qd_before)/DT; tau_meas = tau_acc/5

        # ── estimador: identico en las dos ramas, conserva M y h ───────────
        f3 = estimate_force(x[:4], x[4:], qdd_meas, tau_meas)
        F_est6 = np.concatenate([f3, np.zeros(3)])

        pee = np.array(fk(x[:4])[1]).flatten()
        err.append(np.linalg.norm(pee - p_task))
        fest.append(f3.copy()); ftrue.append(f_true.copy())

    err = np.array(err); fest = np.array(fest); ftrue = np.array(ftrue)
    win = slice(T0, T1); flat = slice(T0+RAMP, T1-RAMP)
    rmse = float(np.sqrt(np.mean(np.linalg.norm(fest[win] - ftrue[win], axis=1)**2)))
    return dict(rmse=rmse,
                yield_max=1e3*float(err[win].max()),
                yield_flat=1e3*float(err[flat].mean()),
                ret=1e3*float(err[-1]),
                ms=float(np.mean(ts)),
                fails=fails)


if __name__ == "__main__":
    print("== LAZO COMPLIANT COMPLETO: CONTROLADOR CON MODELO (T) vs SIN MODELO (V0) ==")
    print(f"   planta ec. 5 con kv = {KV}, estimador de fuerza pura identico en ambas\n")
    sT, sV0 = load()
    DIRS = [([6.0, 0.0, -2.0], "in plane"),
            ([0.0, 6.0,  0.0], "normal")]

    print(f"  {'push':<9} {'ctrl':<5} {'RMSE [N]':>9} {'cede max':>9} {'cede mes':>9} "
          f"{'retorna':>8} {'ms/solve':>9} {'fallos':>7}")
    R = {}
    for d, lab in DIRS:
        for kind, s in [("T", sT), ("V0", sV0)]:
            r = run(s, kind, d); R[(lab, kind)] = r
            print(f"  {lab:<9} {kind:<5} {r['rmse']:9.4f} {r['yield_max']:9.1f} "
                  f"{r['yield_flat']:9.1f} {r['ret']:8.2f} {r['ms']:9.3f} {r['fails']:7d}")

    # ⚠️ Los tres veredictos se LEEN del dato. Si alguno falla se dice, no se
    # maquilla: esta tabla sostiene una afirmacion del abstract.
    print()
    if any(r["fails"] for r in R.values()):
        print("  ⛔ HUBO FALLOS DE SOLVER: no se declara nada con estas filas.")
        raise SystemExit(1)

    ratios = [R[(l, "T")]["ms"]/R[(l, "V0")]["ms"] for _, l in DIRS]
    print(f"  razon de costo T/V0: {min(ratios):.1f} a {max(ratios):.1f}x")

    # ⚠️ La reja de la estimacion se mide contra la FUERZA APLICADA, no contra la
    # otra rama. Comparar los dos RMSE entre si es dividir dos numeros casi nulos:
    # da 70 % de diferencia relativa entre 0.008 y 0.014 N sobre un empuje de 6 N,
    # o sea entre 0.13 % y 0.22 % de error. Eso no mide la afirmacion del paper,
    # que es que la estimacion sigue siendo exacta con los dos controladores.
    rel = {k: 100*r["rmse"]/np.linalg.norm(d)
           for (d, l) in DIRS for k, r in R.items() if k[0] == l}
    print(f"  RMSE de fuerza como % del empuje aplicado: "
          f"{min(rel.values()):.2f} a {max(rel.values()):.2f} %")

    peor_ret = max(R[(l, "V0")]["ret"] - R[(l, "T")]["ret"] for _, l in DIRS)
    print(f"  retorno de V0 menos el de T, peor caso: {peor_ret:+.2f} mm"
          f"  ({'V0 retorna mas cerca' if peor_ret < 0 else 'V0 retorna mas lejos'})")

    ced = [R[(l, "V0")]["yield_max"]/R[(l, "T")]["yield_max"] for _, l in DIRS]
    print(f"  cesion de V0 relativa a la de T: {min(ced):.2f} a {max(ced):.2f}x")

    ok = min(ratios) > 10 and max(rel.values()) < 1.0 and peor_ret <= 0
    print("\n  " + ("CONTROLADOR SIN MODELO CONFIRMADO: la estimacion sigue exacta"
                    " (<1 % del empuje) con los dos, el brazo cede del mismo orden"
                    " y retorna al menos tan cerca, a un orden de magnitud menos"
                    " de costo"
                    if ok else
                    "NO SE DECLARA NADA: revisar cual de las tres rejas fallo"))
