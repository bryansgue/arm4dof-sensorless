"""
LOS DOS REGIMENES DE COMPLIANCE — el script que faltaba.

La tabla `tab:regimes` del paper (desplazamiento COMANDADO por la admitancia
contra el desplazamiento REAL del efector, mas la fraccion de pasos con el
actuador saturado) no la reproducia ningun script guardado: era codigo inline.
Este es el script.

Planta con SATURACION, que es lo que distingue este experimento del MiL:

    tau = sat( kv (qd_cmd - qd), tau_max )
    M qdd + h = tau + Jv^T f_mano

Prediccion de la Sec. V: por debajo de la capacidad del actuador (mediana 5.88 N
en la peor direccion) el servo resiste, el brazo sigue la referencia que la
admitancia comanda, y la compliance es SINTETICA. Por encima, el actuador satura,
el brazo es arrastrado MAS de lo comandado, y la compliance es FISICA y gratis.

Los dos indicadores tienen que cambiar juntos: la razon real/comandado se despega
de 1 exactamente donde la fraccion saturada pasa de 0 a 100 %.

Run:  python3 test_compliance_regimes.py
"""
import os
import numpy as np
from acados_template import AcadosOcpSolver
import generate_arm_dqnmpc_ocp as G
from arm_dynamics import build_dynamics
from arm_kinematics import build_fk_fn, build_jac_fn
from wrench_estimator import estimate_force
from dq_admittance import DQAdmittance

fM, fh = build_dynamics(); fk = build_fk_fn(); fJ = build_jac_fn()
M_ = lambda q: np.array(fM(q))
h_ = lambda q, qd: np.array(fh(q, qd)).flatten()
J_ = lambda q: np.array(fJ(q))

json = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                    "acados_ocp_arm4dof_dqnmpc.json")
solver = AcadosOcpSolver(G.build_ocp(), json_file=json, build=False, generate=False)

W_SE3 = [2., 2., 2., 80., 80., 80.]; W_QD = [0.3]*4; W_U = [0.02]*4
for k in range(G.N_HORIZON):
    solver.cost_set(k, "W", np.diag(W_SE3 + W_QD + W_U))
solver.cost_set(G.N_HORIZON, "W", np.diag(W_SE3 + W_QD))

q_task = np.array([0.0, 1.2, -0.9, 0.4])
_, p_task, quat_task = fk(q_task)
p_task = np.array(p_task).flatten(); quat_task = np.array(quat_task).flatten()

# ⚠️ La meseta tiene que durar VARIAS constantes de tiempo de la admitancia, que
# vale 1/k_return = 0.5 s. Con la meseta de 0.9 s de la version anterior el valor
# medido era el FINAL DE UNA MESETA FINITA, no el estacionario: daba 7.2 mm a 1 N
# contra el equilibrio analitico |f|/(D_trans*k_return) = 8.33 mm. Ahora son 3.0 s,
# seis constantes de tiempo, y el numero SI es estacionario.
DT = 0.01; KV = 4.0
T0, RAMP, PLATEAU = 50, 30, 300
T1 = T0 + RAMP + PLATEAU + RAMP
T = T1 + 60
TAU_MAX = 1.4                      # N.m, el limite del MX-28R
DIR = np.array([0.0, 1.0, 0.0])    # normal al plano del brazo


def profile(t, mag):
    if not (T0 <= t < T1):
        return np.zeros(3)
    if t < T0 + RAMP:    a = (t - T0)/RAMP
    elif t >= T1 - RAMP: a = (T1 - t)/RAMP
    else:                a = 1.0
    return a*mag*DIR


def run(mag):
    adm = DQAdmittance(D_trans=60.0, D_rot=8.0, p0=p_task, q0=quat_task,
                       k_return=2.0, p_task=p_task, q_task=quat_task)
    W = np.zeros(G.N_PARAMS)
    x = np.concatenate([q_task, np.zeros(4)])
    F_est6 = np.zeros(6)
    cmd = []; act = []; sat = []; fails = 0
    for t in range(T):
        dq_ref, p_ref, _ = adm.update(F_est6, DT)
        W[0:8] = dq_ref
        for k in range(G.N_HORIZON+1):
            solver.set(k, "p", W)
        solver.set(0, "lbx", x); solver.set(0, "ubx", x)
        st = solver.solve()
        if st != 0:
            fails += 1
        qd_cmd = solver.get(1, "x")[4:8]

        f_true = profile(t, mag)
        F6 = np.concatenate([f_true, np.zeros(3)])
        qd_before = x[4:].copy(); tau_acc = np.zeros(4); hit = False
        for _ in range(5):
            q, qd = x[:4], x[4:]
            tau_raw = KV*(qd_cmd - qd)
            tau = np.clip(tau_raw, -TAU_MAX, TAU_MAX)
            hit |= bool(np.any(np.abs(tau_raw) > TAU_MAX))
            tau_acc += tau
            qdd = np.linalg.solve(M_(q), tau + J_(q).T @ F6 - h_(q, qd))
            x = np.concatenate([q + (DT/5)*qd, qd + (DT/5)*qdd])
        qdd_meas = (x[4:] - qd_before)/DT; tau_meas = tau_acc/5

        f3 = estimate_force(x[:4], x[4:], qdd_meas, tau_meas)
        F_est6 = np.concatenate([f3, np.zeros(3)])

        _, pee, _ = fk(x[:4]); pee = np.array(pee).flatten()
        cmd.append(np.linalg.norm(np.array(p_ref).flatten() - p_task))
        act.append(np.linalg.norm(pee - p_task))
        sat.append(hit)
    # ⚠️ ESTADO ESTACIONARIO. Promediar el tramo plano ENTERO mezcla el transitorio;
    # se mide el ultimo medio segundo de una meseta de seis constantes de tiempo, y
    # se contrasta contra el equilibrio analitico |f|/(D_trans*k_return) = |f|/120 m.
    # Sin ese contraste no hay forma de saber si el numero esta asentado.
    ss = slice(T1-RAMP-50, T1-RAMP)          # ultimo 0.5 s de una meseta de 3.0 s
    flat = slice(T0+RAMP, T1-RAMP)
    return (1e3*np.mean(np.array(cmd)[ss]),
            1e3*np.mean(np.array(act)[ss]),
            100*np.mean(np.array(sat)[flat]),
            fails)


if __name__ == "__main__":
    print("== LOS DOS REGIMENES DE COMPLIANCE ==")
    print(f"planta CON saturacion, tau_max = {TAU_MAX} N.m, empuje normal al plano\n")
    print(f"  meseta {PLATEAU*DT:.1f} s = {PLATEAU*DT*2:.0f} constantes de tiempo"
          f"   (equilibrio analitico: |f|/(D*k) = |f|*8.33 mm/N)\n")
    print(f"  {'|f| [N]':>8} {'comandado [mm]':>16} {'analitico':>10} {'real [mm]':>11} "
          f"{'razon':>7} {'saturado':>9} {'solver':>8}")
    rows = []; bad = []
    for mag in [1., 3., 5., 6., 8., 10.]:
        c, a, s, nf = run(mag)
        tag = "OK" if nf == 0 else f"{nf} FALLOS"
        if nf:
            bad.append(mag)
        else:
            rows.append((mag, c, a, s))
        print(f"  {mag:8.0f} {c:16.1f} {mag*8.33:10.1f} {a:11.1f} {a/c:7.2f} "
              f"{s:8.0f} % {tag:>8}")
    if bad:
        print(f"\n  ⚠️ FILAS INVALIDADAS por fallos del solver: {bad}")
        print("     No se declara nada con ellas.")
    # ⚠️ el umbral se LEE del dato, no se fija a mano: es la primera magnitud con
    # saturacion. Fijarlo a 6 N era suponer la conclusion.
    sats = [r for r in rows if r[3] > 50]
    if not sats:
        print("\n  NINGUNA magnitud satura: no hay dos regimenes que separar")
        raise SystemExit(1)
    thr = sats[0][0]
    lo = [r for r in rows if r[0] < thr]; hi = [r for r in rows if r[0] >= thr]
    print(f"\n  transicion observada entre {max(r[0] for r in lo):.0f} y {thr:.0f} N")
    print(f"  razon media por debajo: {np.mean([a/c for _,c,a,_ in lo]):.2f}"
          f"   por encima: {np.mean([a/c for _,c,a,_ in hi]):.2f}")
    print(f"  saturacion por debajo: {np.mean([s for *_,s in lo]):.0f} %"
          f"   por encima: {np.mean([s for *_,s in hi]):.0f} %")

    # capacidad estatica en ESTA pose y ESTA direccion, que es contra lo que hay
    # que comparar. La mediana del espacio de trabajo (5.88 N) es otra cosa.
    g = h_(q_task, np.zeros(4)); t = J_(q_task)[:3, :].T @ DIR
    cap = min((lim - g[j])/t[j] for j in range(4) if abs(t[j]) > 1e-12
              for lim in (TAU_MAX, -TAU_MAX) if (lim - g[j])/t[j] > 0)
    print(f"  capacidad estatica en esta pose y direccion: {cap:.2f} N")

    ok = (not bad
          and np.mean([s for *_, s in lo]) < 5 and np.mean([s for *_, s in hi]) > 85
          and np.mean([a/c for _, c, a, _ in hi]) > 1.5*np.mean([a/c for _, c, a, _ in lo])
          and lo[-1][0] <= cap <= thr)
    print("\n  " + ("DOS REGIMENES CONFIRMADOS: la razon salta donde satura, y la"
                    " transicion cae sobre la capacidad estatica"
                    if ok else
                    "NO SE DECLARA NADA: hubo fallos de solver o los regimenes no separan"))
