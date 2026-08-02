"""
T vs V1 vs V0 — que formulacion del OCP conviene cuando el actuador toma VELOCIDAD,
y si hace falta identificar el lazo interno del servo.

Planta comun a las tres (la real, ec. 5 con saturacion):
    tau = sat( kv_real (u - qd), tau_max )
    M qdd + h = tau + J^T F

Controladores:
    T   x=[q,qd], u=tau   -> se manda qd*[1].  Usa M, h. No usa kv.
    V1  x=[q,qd], u=vel   -> usa M, h Y kv.    Requiere identificar el servo.
    V0  x=[q],    u=vel   -> qd = u.           No usa nada. Cero identificacion.

El experimento decisivo es el barrido de DESAJUSTE de kv: si V0 aguanta lo que V1
con kv perfecto, entonces identificar el servo no aporta y el controlador queda
inmune a que kv cambie con temperatura, carga o firmware.

Run:  python3 test_vel_vs_torque.py
"""
import os
import time
import numpy as np
from acados_template import AcadosOcpSolver

import generate_arm_dqnmpc_ocp as GT
import generate_arm_vel_ocp as GV
from arm_dynamics import build_dynamics
from arm_kinematics import build_fk_fn

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")

fM, fh = build_dynamics(); fk = build_fk_fn()
M_ = lambda q: np.array(fM(q))
h_ = lambda q, qd: np.array(fh(q, qd)).flatten()

DT = 0.01
TAU_MAX = 1.4
KV_REAL = 20.0
Q0 = np.array([-0.5, 0.7, -1.4, -0.3, 0., 0., 0., 0.])
QT = np.array([0.6, 1.4, -0.7, 0.9])


def ref_dq():
    return np.array(fk(QT)[0]).flatten()


def plant_step(x, u, kv_real=KV_REAL, sub=40):
    """Planta real: servo con ganancia kv_real y SATURACION de par."""
    ds = DT/sub
    for _ in range(sub):
        q, qd = x[:4], x[4:]
        tau = np.clip(kv_real*(u - qd), -TAU_MAX, TAU_MAX)
        qdd = np.linalg.solve(M_(q), tau - h_(q, qd))
        x = np.concatenate([q + ds*qd, qd + ds*qdd])
    return x


def run(kind, solver, N, kv_real=KV_REAL, T=250):
    p = np.zeros(GV.N_PARAMS_V if kind != "T" else GT.N_PARAMS)
    p[0:8] = ref_dq()
    x = Q0.copy(); errs = []; ts = []; bad = 0
    for _ in range(T):
        for k in range(N+1):
            solver.set(k, "p", p)
        x0 = x if kind != "V0" else x[:4]
        solver.set(0, "lbx", x0); solver.set(0, "ubx", x0)
        t0 = time.perf_counter(); st = solver.solve(); ts.append((time.perf_counter()-t0)*1e3)
        if st != 0: bad += 1
        u = solver.get(1, "x")[4:8] if kind == "T" else solver.get(0, "u")
        x = plant_step(x, u, kv_real)
        pee = np.array(fk(x[:4])[1]).flatten()
        errs.append(np.linalg.norm(pee - np.array(fk(QT)[1]).flatten()))
    return np.array(errs), np.array(ts), bad


def load():
    sT = AcadosOcpSolver(GT.build_ocp(), json_file=os.path.join(ROOT, "acados_ocp_arm4dof_dqnmpc.json"),
                         build=False, generate=False)
    W = [3., 3., 3., 120., 120., 120.] + [0.1]*4
    for k in range(GT.N_HORIZON): sT.cost_set(k, "W", np.diag(W + [0.02]*4))
    sT.cost_set(GT.N_HORIZON, "W", np.diag(W))
    sV1 = AcadosOcpSolver(GV.build_ocp_v1(), json_file=os.path.join(ROOT, "acados_ocp_arm4dof_v1.json"),
                          build=False, generate=False)
    sV0 = AcadosOcpSolver(GV.build_ocp_v0(), json_file=os.path.join(ROOT, "acados_ocp_arm4dof_v0.json"),
                          build=False, generate=False)
    return sT, sV1, sV0


if __name__ == "__main__":
    sT, sV1, sV0 = load()
    N = GT.N_HORIZON

    print("=" * 72)
    print("A) Regulacion a una pose, kv real = kv del modelo = 20.")
    print("   ⚠️ Se mide TRAS ASENTAR (1000 pasos) y con el peso del comando de V0")
    print("   ajustado, que es la condicion que reporta la Tabla IV del paper. Con")
    print("   250 pasos y W_u=0.3 los numeros son otros (T 0.754, V0 5.637): esa")
    print("   diferencia es tiempo de asentamiento, no de formulacion.")
    print("=" * 72)
    print(f"{'OCP':>4} {'estados':>8} {'usa M,h':>8} {'usa kv':>7} "
          f"{'err final [mm]':>15} {'ms/solve':>9} {'fallos':>7}")
    rows = {}
    # V0 con el peso de comando ajustado (ver seccion B del paper)
    Ws = [2., 2., 2., 80., 80., 80.]
    for k in range(GV.N_HORIZON):
        sV0.cost_set(k, "W", np.diag(Ws + [0.01]*4))
    sV0.cost_set(GV.N_HORIZON, "W", np.diag(Ws))
    for kind, s, nx, usa, usakv in [("T", sT, 8, "si", "no"),
                                    ("V1", sV1, 8, "si", "SI"),
                                    ("V0", sV0, 4, "no", "no")]:
        e, t, b = run(kind, s, N, T=1000)
        rows[kind] = (e, t, b)
        print(f"{kind:>4} {nx:>8} {usa:>8} {usakv:>7} "
              f"{1000*e[-1]:15.3f} {t.mean():9.3f} {b:7d}")

    print()
    print("=" * 72)
    print("B) DESAJUSTE de kv: el modelo cree 20, la planta real vale otra cosa.")
    print("   V0 y T no usan kv, asi que solo pueden degradarse por la planta.")
    print("=" * 72)
    print(f"{'kv real':>8} {'T [mm]':>10} {'V1 [mm]':>10} {'V0 [mm]':>10}")
    for kvr in [5., 10., 20., 40., 80.]:
        out = []
        for kind, s in [("T", sT), ("V1", sV1), ("V0", sV0)]:
            e, _, _ = run(kind, s, N, kv_real=kvr)
            out.append(1000*e[-1])
        print(f"{kvr:8.0f} {out[0]:10.3f} {out[1]:10.3f} {out[2]:10.3f}")

    print()
    print("=" * 72)
    print("C) Costo de computo")
    print("=" * 72)
    for kind in ("T", "V1", "V0"):
        t = rows[kind][1]
        print(f"  {kind:>3}: media {t.mean():6.3f} ms   max {t.max():6.3f} ms   "
              f"-> {1000/t.mean():4.0f} Hz")
