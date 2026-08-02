"""
S1 — Control negativo: SIN CONTACTO la fuerza estimada tiene que dar CERO.

Nadie toca el brazo durante esta etapa.

Por que es lo primero. Con el brazo quieto y sin contacto,
    tau_ext = M q̈ + h − tau_act  ≈  h − tau_act,
asi que un error en el modelo de gravedad, en la constante de par o en la friccion
es INDISTINGUIBLE de una fuerza externa. Si f_est no da ~0 acá, todo lo que venga
después mide basura, y con una pesa colgada no se puede separar la causa.

Un control negativo que PASA no es un descuido: es lo unico que valida el cero.

Dos partes:
  A. barrido de poses en reposo  -> sesgo de gravedad por postura
  B. barrido a velocidad constante -> friccion de Coulomb + viscosa por junta

⚠️ La friccion de reductora NO la ve la corriente del motor (esta antes de la
reductora). Lo que se identifica acá es lo que el modelo puede compensar; el resto
queda como piso irreducible, y es el numero que decide si el estimador sirve.

⚠️ QUE PUEDE Y QUE NO PUEDE DETECTAR ESTA ETAPA. Medido sobre el modelo: los pares
de gravedad de este brazo son chicos (|h| ~ 0.2-0.3 N.m), asi que un error del 5 %
en la constante de par produce apenas 0.18 N de fuerza falsa, aun en la pose mas
sensible del espacio de trabajo. Por lo tanto:

  DETECTA   modelo de gravedad equivocado (masa o centro de masa de un eslabon),
            offset de cero o signo de junta invertido, friccion sin compensar.
  NO DETECTA un error moderado de la constante de par.

La constante de par NO se puede validar sin carga: sale de S2, donde una pesa de
0.5 kg mete un par mucho mayor que el propio peso del brazo. Por eso S1 no valida
S2 y el orden importa: S1 limpia todo lo demas para que la regresion de S2 no
atribuya a kt lo que en realidad es gravedad mal modelada.

Las poses de abajo se eligieron por BARRIDO maximizando esa sensibilidad. Las
poses elegidas a ojo resultaron 3-7x menos sensibles: un control negativo poco
sensible pasa por el motivo equivocado.

PODER DE DETECCION, medido inyectando cada falla sobre el modelo validado
(peor pose del set). Es la tabla que dice que significa que S1 pase:

    falla inyectada                      |f_est| falso
    signo de par invertido                   7.38 N
    constante de par x2                      3.69 N
    offset de par 0.05 N.m en una junta      1.66 N
    stiction 0.05 N.m en todas               0.90 N
    constante de par +20 %                   0.74 N
    offset de par 0.02 N.m                   0.67 N
    masa del eslabon 4  +20 %                0.42 N
    cero de junta corrido 0.10 rad           0.33 N
    ---- criterio 0.3 N ----
    masa del eslabon 3  +20 %                0.24 N
    cero de junta corrido 0.05 rad           0.17 N
    constante de par +5 %                    0.19 N
    masa del eslabon 2  +20 %                0.03 N
    masa del eslabon 1  +20 %                0.00 N   <- ciego, y es correcto:
        el eslabon 1 gira sobre el eje vertical y la gravedad no le hace par.

⚠️ Los OFFSETS de par son lo mas detectable (0.02 N.m ya da 0.67 N) mientras que
los errores de gravedad se rechazan casi enteros. La razon es geometrica: el error
de gravedad cae en las direcciones bien condicionadas de Jv y la proyeccion por
minimos cuadrados lo absorbe, mientras que un offset no. Corolario practico: la
STICTION es el termino dominante del error de este estimador, no el ruido.

Criterio: |f_est| < 0.3 N en todo el barrido, tras compensar friccion.

Run:  python3 s1_gravity_check.py --port /dev/ttyUSB0
"""
import argparse
import os
import sys
import time
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "ocp_generation"))
import dxl_io
from arm_dynamics import build_dynamics
from arm_kinematics import build_jac_fn

fM, fh = build_dynamics(); fJ = build_jac_fn()
M_ = lambda q: np.array(fM(q))
h_ = lambda q, qd: np.array(fh(q, qd)).flatten()
Jv_ = lambda q: np.array(fJ(q))[:3, :]

# Poses elegidas por SENSIBILIDAD y VALIDADAS EN MuJoCo (select_poses.py).
# ⚠️ La tanda anterior se eligio sobre el modelo analitico sin chequear colision:
# TRES de las seis quedaban DENTRO DEL PISO (ncon 10, 10, 54; z_ee negativo). El
# banco analitico no puede detectarlo porque no tiene geometria. Estas cumplen
# ncon=0, z_ee > 0.06 m, sigma_min(Jv) > 0.02 y 0.15 rad de margen contra los
# topes de junta: sin ese margen el barrido elige el extremo del rango y la
# junta queda apoyada contra el tope, que carga el peso en vez del actuador.
POSES = [
    np.array([0.00,  3.118,  0.000, -0.310]),
    np.array([0.00,  0.333,  0.000, -0.310]),
    np.array([0.00,  0.333, -0.310,  0.619]),
    np.array([0.00,  2.190,  0.000, -0.310]),
    np.array([0.00,  3.118, -0.929,  0.310]),
    np.array([0.00,  0.333,  0.929, -0.310]),
]

# Pose para S2: quiere lo OPUESTO que S1. S1 maximiza el par de gravedad; S2
# maximiza el par que la PESA produce, o sea brazo horizontal. Ver select_poses.py.
POSE_S2 = np.array([0.00, 2.499, -1.857, -0.619])

KP_POS = 2.0          # P sobre posicion -> comando de velocidad (suave)
QD_LIM = 0.4          # rad/s, deliberadamente lento
SETTLE = 2.0          # s de espera antes de medir
NSAMP = 100


def goto(arm, q_des, timeout=8.0, tol=0.02):
    """Lleva el brazo a q_des con un P sobre posicion. El servo es fuente de
    velocidad, asi que el lazo externo de posicion lo cerramos nosotros."""
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < timeout:
        q, qd, _ = arm.read()
        e = q_des - q
        if np.max(np.abs(e)) < tol:
            arm.stop(); return True
        arm.write_velocity(np.clip(KP_POS*e, -QD_LIM, QD_LIM))
        time.sleep(0.005)
    arm.stop()
    return False


def f_est_static(q, qd, tau_act, tau_fric=None):
    """Con el brazo quieto: q̈ ~ 0, asi que tau_ext = h - tau_act."""
    t = h_(q, qd) - tau_act
    if tau_fric is not None:
        t = t - tau_fric
    return np.linalg.lstsq(Jv_(q).T, t, rcond=None)[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--ids", type=int, nargs="+", default=[1, 2, 3, 4])
    ap.add_argument("--skip-friction", action="store_true")
    args = ap.parse_args()

    print("=" * 68)
    print("S1 — control negativo.  NO TOCAR EL BRAZO durante toda la corrida.")
    print("=" * 68)
    input("Enter para habilitar par y empezar (Ctrl-C para abortar)... ")

    arm = dxl_io.DxlArm(port=args.port, baud=args.baud, ids=args.ids)
    try:
        arm.set_velocity_mode(); arm.enable(True)

        # ── B. friccion, primero: hace falta para compensar en A ─────────────
        tau_c = np.zeros(len(args.ids)); tau_b = np.zeros(len(args.ids))
        if not args.skip_friction:
            print("\n-- B. identificacion de friccion, junta por junta --")
            print(f"   {'junta':>6} {'Coulomb [N.m]':>15} {'viscosa [N.m.s]':>17} {'R2':>7}")
            goto(arm, POSES[0])
            for k in range(len(args.ids)):
                vs, ts = [], []
                for v in [-0.3, -0.2, -0.1, 0.1, 0.2, 0.3]:
                    cmd = np.zeros(len(args.ids)); cmd[k] = v
                    arm.write_velocity(cmd); time.sleep(1.0)
                    acc_v, acc_t = [], []
                    for _ in range(50):
                        q, qd, tau = arm.read()
                        # el par de friccion es lo que el servo hace de mas sobre h
                        acc_v.append(qd[k]); acc_t.append(tau[k] - h_(q, qd)[k])
                        time.sleep(0.005)
                    vs.append(np.mean(acc_v)); ts.append(np.mean(acc_t))
                arm.stop(); time.sleep(0.5)
                vs = np.array(vs); ts = np.array(ts)
                A = np.column_stack([np.sign(vs), vs])
                coef, *_ = np.linalg.lstsq(A, ts, rcond=None)
                pred = A @ coef
                r2 = 1 - np.sum((ts-pred)**2)/max(np.sum((ts-ts.mean())**2), 1e-12)
                tau_c[k], tau_b[k] = coef
                print(f"   {k+1:6d} {coef[0]:15.4f} {coef[1]:17.4f} {r2:7.3f}")
            print(f"\n   ⚠️ Coulomb medio {np.mean(np.abs(tau_c)):.4f} N.m.  Ese es el")
            print("      termino dominante del error, no el ruido del encoder.")

        # ── A. barrido de poses en reposo ────────────────────────────────────
        print("\n-- A. f_est en reposo, sin contacto --")
        print(f"   {'pose':>5} {'|f_est| crudo':>15} {'|f_est| compensado':>20} "
              f"{'sig_min(Jv)':>12}")
        raw, comp = [], []
        for p, qd_des in enumerate(POSES):
            if not goto(arm, qd_des):
                print(f"   pose {p+1}: NO alcanzada, se omite"); continue
            time.sleep(SETTLE)
            F1, F2 = [], []
            for _ in range(NSAMP):
                q, qd, tau = arm.read()
                # en reposo el signo de qd es ruido: la friccion no se compensa por signo
                F1.append(np.linalg.norm(f_est_static(q, qd, tau)))
                F2.append(np.linalg.norm(f_est_static(q, qd, tau, tau_c*np.sign(qd) + tau_b*qd)))
                time.sleep(0.005)
            q, _, _ = arm.read()
            s = np.linalg.svd(Jv_(q), compute_uv=False)[2]
            raw.append(np.mean(F1)); comp.append(np.mean(F2))
            print(f"   {p+1:5d} {np.mean(F1):15.3f} {np.mean(F2):20.3f} {s:12.4f}")
        arm.stop()

        print("\n" + "=" * 68)
        worst = max(comp) if comp else float("inf")
        print(f"peor |f_est| sin contacto = {worst:.3f} N")
        print("(referencia: un 5 % de error en la constante de par daria ~0.18 N en")
        print(" estas poses, asi que esta etapa NO valida kt. Eso es S2.)")
        if worst < 0.3:
            print("S1 APROBADO — gravedad, ceros y signos validados. Seguir con S2.")
        else:
            print("S1 RECHAZADO. No seguir: con la pesa colgada no se podra separar")
            print("la causa. Revisar, en este orden:")
            print("  1. constante de par (S2 la mide, pero la semilla puede estar muy mal)")
            print("  2. masas y centros de masa del modelo (arm_params.py)")
            print("  3. offsets de cero y signos de junta en dxl_io.py")
        print("=" * 68)
        np.savez(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "s1_result.npz"),
                 tau_coulomb=tau_c, tau_viscous=tau_b,
                 f_raw=np.array(raw), f_comp=np.array(comp))
        print("guardado -> s1_result.npz  (lo consume S2)")
    finally:
        arm.close()


if __name__ == "__main__":
    main()
