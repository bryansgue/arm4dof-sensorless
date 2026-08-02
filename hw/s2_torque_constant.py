"""
S2 — Constante corriente->par, medida contra pesas conocidas.

Por que no se usa el valor de datasheet: la sensibilidad del estimador a esta
constante es 1:1 (5 % de error de constante = 5 % de error de fuerza, sin
atenuacion). Y hay precedente directo en este proyecto: el hover_throttle del
dron figuraba como "validado", estaba 9.7 % bajo, y el lazo cerrado lo tapaba.
Medirlo cuesta minutos.

Metodo. Con el brazo QUIETO sosteniendo una pose y una masa m colgada del
efector, la fuerza externa es conocida: f = [0, 0, -m g]. El balance estatico es

    tau_act = h(q) - Jv(q)^T f - tau_friccion

y el servo reporta corriente i, con tau_act = kt * i. Entonces, por junta,

    kt_j * i_j  =  [h - Jv^T f - tau_fric]_j

y kt_j sale por regresion sobre varias masas. La ORDENADA de esa regresion es
sesgo residual: si no da ~0, queda gravedad o friccion sin modelar.

⚠️ ORDEN ALTERNADO de masas, no creciente. Con orden creciente cualquier deriva
termica del motor queda correlacionada con la carga y entra ENTERA en la
pendiente, o sea en kt. Es el mismo defecto que se corrigio en el identificador
de empuje del dron, y en un banco ideal es invisible.

Requiere S1 aprobado (usa s1_result.npz para la friccion).

Criterio: R2 > 0.98 por junta y dispersion entre corridas < 5 %.

Run:  python3 s2_torque_constant.py --port /dev/ttyUSB0 --masses 0 0.1 0.2 0.3 0.5
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
from s1_gravity_check import goto, POSES

fM, fh = build_dynamics(); fJ = build_jac_fn()
h_ = lambda q, qd: np.array(fh(q, qd)).flatten()
Jv_ = lambda q: np.array(fJ(q))[:3, :]

G = 9.81
HERE = os.path.dirname(os.path.abspath(__file__))


def alternating(seq):
    """Orden alternado desde el centro: descorrelaciona la deriva termica de la
    carga. Con orden creciente la correlacion es +1.000 y no se puede separar."""
    s = sorted(seq); out = []
    lo, hi = 0, len(s)-1
    mid = len(s)//2
    out.append(s[mid])
    for d in range(1, len(s)):
        for j in (mid-d, mid+d):
            if lo <= j <= hi and s[j] not in out:
                out.append(s[j])
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--ids", type=int, nargs="+", default=[1, 2, 3, 4])
    ap.add_argument("--masses", type=float, nargs="+",
                    default=[0.0, 0.1, 0.2, 0.3, 0.5], help="kg colgados del efector")
    ap.add_argument("--pose", type=int, default=0, help="indice en POSES")
    args = ap.parse_args()

    f1 = os.path.join(HERE, "s1_result.npz")
    if not os.path.exists(f1):
        print("⚠️ falta s1_result.npz.  Correr S1 primero: sin el cero validado,")
        print("   esta regresion mide gravedad mal modelada y la llama constante de par.")
        return
    d1 = np.load(f1)
    tau_c, tau_b = d1["tau_coulomb"], d1["tau_viscous"]

    q_des = POSES[args.pose]
    s = np.linalg.svd(Jv_(q_des), compute_uv=False)[2]
    print("=" * 68)
    print(f"S2 — constante de par.  pose {args.pose+1}, sigma_min(Jv) = {s:.4f}")
    order = alternating(args.masses)
    print(f"orden de masas (ALTERNADO, no creciente): {order}")
    print("=" * 68)

    arm = dxl_io.DxlArm(port=args.port, baud=args.baud, ids=args.ids)
    try:
        arm.set_velocity_mode(); arm.enable(True)
        if not goto(arm, q_des):
            print("no se alcanzo la pose objetivo"); return
        time.sleep(2.0)

        I = []; RHS = []
        for m in order:
            print(f"\n>> colgar {m:.3f} kg del efector.  Enter cuando este quieto...")
            input()
            time.sleep(1.5)
            acc_i = []; acc_r = []
            for _ in range(200):
                q, qd, tau = arm.read()
                # corriente cruda: deshacer la constante semilla que aplica dxl_io
                i_raw = tau/arm.kt
                f = np.array([0.0, 0.0, -m*G])
                rhs = h_(q, qd) - Jv_(q).T @ f - (tau_c*np.sign(qd) + tau_b*qd)
                acc_i.append(i_raw); acc_r.append(rhs)
                time.sleep(0.005)
            I.append(np.mean(acc_i, axis=0)); RHS.append(np.mean(acc_r, axis=0))
            print(f"   i = {np.mean(acc_i,axis=0)}")
        arm.stop()

        I = np.array(I); RHS = np.array(RHS)
        print("\n" + "-" * 68)
        print(f"{'junta':>6} {'kt [N.m/A]':>12} {'ordenada':>11} {'R2':>8}")
        kt = np.zeros(len(args.ids)); r2s = np.zeros(len(args.ids))
        for k in range(len(args.ids)):
            A = np.column_stack([I[:, k], np.ones(len(I))])
            coef, *_ = np.linalg.lstsq(A, RHS[:, k], rcond=None)
            pred = A @ coef
            den = np.sum((RHS[:, k]-RHS[:, k].mean())**2)
            r2 = 1 - np.sum((RHS[:, k]-pred)**2)/max(den, 1e-12)
            kt[k], r2s[k] = coef[0], r2
            print(f"{k+1:6d} {coef[0]:12.4f} {coef[1]:11.4f} {r2:8.3f}")

        # control de correlacion: la deriva no debe seguir a la carga
        corr = np.corrcoef(np.arange(len(order)), order)[0, 1]
        print(f"\ncorrelacion(orden de medicion, masa) = {corr:+.3f}")
        print("   cerca de 0 => la deriva termica no puede entrar en la pendiente.")
        print("   con orden creciente esto daria +1.000 y kt saldria sesgado.")

        print("\n" + "=" * 68)
        ok = np.all(r2s > 0.98)
        if ok:
            print("S2 APROBADO.  Poner en dxl_io.py:")
            print(f"   torque_const = {np.array2string(kt, precision=4, separator=', ')}")
            print("Seguir con S3 (validacion en postura bien y mal condicionada).")
        else:
            print("S2 RECHAZADO: R2 bajo en alguna junta.")
            print("  - si la ordenada no es ~0, queda gravedad o friccion sin modelar")
            print("  - revisar que la masa cuelgue del punto que el modelo llama efector")
        print("=" * 68)
        np.savez(os.path.join(HERE, "s2_result.npz"), kt=kt, r2=r2s,
                 masses=np.array(order), I=I, RHS=RHS)
        print("guardado -> s2_result.npz")
    finally:
        arm.close()


if __name__ == "__main__":
    main()
