"""
Selección de poses para S1 y S2, con MuJoCo como filtro de validez FISICA.

⚠️ Por que existe. Las poses de S1 se habian elegido maximizando sensibilidad
sobre el modelo analitico, sin chequear colision. Verificado en MuJoCo: TRES de
las seis quedaban DENTRO DEL PISO (ncon 10, 10 y 54, con z_ee negativo). Y como
S2 usaba una de ellas, la pesa la sostenia el contacto con el suelo en vez de las
juntas, y la corriente no variaba con la masa. El banco analitico no podia verlo.

Criterios, distintos para cada etapa porque quieren cosas OPUESTAS:

  S1  maximiza  |f_est| falso ante un error de par  ->  quiere par de gravedad alto
  S2  maximiza  min_j |[Jv^T z]_j|                  ->  quiere que la PESA mueva
                                                        el par en las tres juntas
                                                        de cabeceo

Ambas exigen: sin contacto, efector por encima del piso con margen, y
sigma_min(Jv) > 0.02 para que el estimador este bien condicionado.

Run:  python3 select_poses.py
"""
import os
import sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "ocp_generation"))

import mujoco
import mj_arm
from arm_dynamics import build_dynamics
from arm_kinematics import build_jac_fn, build_fk_fn

fM, fh = build_dynamics(); fJ = build_jac_fn(); fk = build_fk_fn()
h_ = lambda q: np.array(fh(q, np.zeros(4))).flatten()
Jv_ = lambda q: np.array(fJ(q))[:3, :]

Q_LO = np.array([-3.1416, -0.436332, -2.00713, -2.00713])
Q_HI = np.array([3.1416, 3.57792, 2.00713, 2.00713])
Z_MARGIN = 0.06          # m sobre el piso
SIG_MIN = 0.02
LIM_MARGIN = 0.15        # rad de margen contra los topes de junta

# ⚠️ El margen contra topes NO es cosmetico. Sin el, el barrido elige el extremo
# del rango: la junta queda APOYADA contra su tope, la restriccion carga el peso y
# el actuador no ve nada. Medido: con q2 en el tope, la regresion de S2 devolvio
# kt = 89.9 contra 1.41 (6275 % de error, R2 0.66) mientras las juntas libres
# daban 0.08 % y 0.20 %.


def valid(arm, q):
    """Fisicamente alcanzable: sin contacto y con el efector despegado del piso."""
    arm.set_state(q)
    mujoco.mj_forward(arm.m, arm.d)
    if arm.d.ncon > 0:
        return False, 0.0
    _, p, _ = fk(q); p = np.array(p).flatten()
    if p[2] < Z_MARGIN:
        return False, p[2]
    return True, p[2]


def sens_s1(q, g=0.05):
    """Fuerza falsa que produce un error de ganancia g en el par."""
    return np.linalg.norm(np.linalg.lstsq(Jv_(q).T, -g*h_(q), rcond=None)[0])


def sens_s2(q):
    """Par que una carga vertical unitaria produce en la junta que MENOS ve."""
    t = Jv_(q).T @ np.array([0.0, 0.0, -1.0])
    return float(np.min(np.abs(t[1:])))


def sweep(arm, score, n=13, spread=0.7, k=6):
    cand = []
    lo, hi = Q_LO + LIM_MARGIN, Q_HI - LIM_MARGIN
    for a in np.linspace(lo[1], hi[1], n):
        for b in np.linspace(lo[2], hi[2], n):
            for c in np.linspace(lo[3], hi[3], n):
                q = np.array([0.0, a, b, c])
                if np.linalg.svd(Jv_(q), compute_uv=False)[2] < SIG_MIN:
                    continue
                ok, z = valid(arm, q)
                if not ok:
                    continue
                cand.append((score(q), z, q))
    cand.sort(key=lambda r: -r[0])
    sel = []
    for s, z, q in cand:
        if all(np.max(np.abs(q - p[2])) > spread for p in sel):
            sel.append((s, z, q))
        if len(sel) >= k:
            break
    return sel, len(cand)


if __name__ == "__main__":
    arm = mj_arm.MjArm()

    print("=" * 76)
    print("Seleccion de poses con MuJoCo como filtro (sin contacto, z_ee > "
          f"{Z_MARGIN} m)")
    print("=" * 76)

    print("\n-- S1: control negativo. Maximiza la fuerza falsa ante error de par --")
    s1, n1 = sweep(arm, sens_s1)
    print(f"   {n1} configuraciones validas de {13**3}")
    print(f"   {'|f| ante 5% kt':>15} {'z_ee':>8} {'sig_min':>9}  q")
    for s, z, q in s1:
        print(f"   {s:15.4f} {z:8.4f} "
              f"{np.linalg.svd(Jv_(q),compute_uv=False)[2]:9.4f}  {np.round(q,3)}")

    print("\n-- S2: calibracion con pesas. Maximiza el par que la carga produce --")
    s2, n2 = sweep(arm, sens_s2, k=3)
    print(f"   {'min|tau| por N':>15} {'z_ee':>8} {'sig_min':>9}  q")
    for s, z, q in s2:
        print(f"   {s:15.4f} {z:8.4f} "
              f"{np.linalg.svd(Jv_(q),compute_uv=False)[2]:9.4f}  {np.round(q,3)}")

    print("\n" + "=" * 76)
    print("POSES_S1 = [")
    for _, _, q in s1:
        print(f"    np.array([{q[0]:.2f}, {q[1]:.3f}, {q[2]:.3f}, {q[3]:.3f}]),")
    print("]")
    print(f"POSE_S2  = np.array([{s2[0][2][0]:.2f}, {s2[0][2][1]:.3f}, "
          f"{s2[0][2][2]:.3f}, {s2[0][2][3]:.3f}])")
    print("=" * 76)
