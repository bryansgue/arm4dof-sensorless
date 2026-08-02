"""
Valida los PROCEDIMIENTOS de identificacion contra un banco donde la respuesta se
conoce. Se corre ANTES de tocar el brazo real.

La pregunta que responde: si S1 y S2 no recuperan una constante de par y una
friccion CONOCIDAS en un banco limpio, no las van a recuperar en el MX-28, y ahi
el fallo aparece con las pesas ya colgadas y sin forma de atribuirlo.

⚠️ Aprobar esto NO valida el hardware. Valida que los scripts y las regresiones
hacen lo que dicen. El banco no modela reductora, backlash, deriva termica ni
retardo del bus.

Run:  python3 validate_procedures.py
"""
import os
import sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "ocp_generation"))

import argparse
import dxl_sim, mj_arm
from dxl_sim import SimArm
from arm_dynamics import build_dynamics
from arm_kinematics import build_jac_fn
from s1_gravity_check import POSES, POSE_S2, f_est_static, goto
from s2_torque_constant import alternating

fM, fh = build_dynamics(); fJ = build_jac_fn()
h_ = lambda q, qd: np.array(fh(q, qd)).flatten()
Jv_ = lambda q: np.array(fJ(q))[:3, :]
G = 9.81


def hold(arm, q_des, iters=12000, kp=3.0):
    """Lleva el banco a q_des con el mismo P sobre posicion que usa S1."""
    for _ in range(iters):
        q, qd, _ = arm.read()
        e = q_des - q
        if np.max(np.abs(e)) < 2e-3 and np.max(np.abs(qd)) < 2e-3:
            break
        arm.write_velocity(np.clip(kp*e, -0.4, 0.4))
    for _ in range(800):                     # asentar SIN soltar el lazo
        q, qd, _ = arm.read()
        arm.write_velocity(np.clip(kp*(q_des - q), -0.4, 0.4))
    return np.max(np.abs(q_des - arm.read()[0]))


def avg(arm, n=120, q_des=None, kp=3.0):
    """Promedia manteniendo el lazo cerrado. ⚠️ Soltarlo no sirve: con un
    actuador de velocidad sin integrador, comando cero = par cero, y el brazo cae."""
    Q = []; QD = []; T = []
    for _ in range(n):
        q, qd, t = arm.read()
        if q_des is not None:
            arm.write_velocity(np.clip(kp*(q_des - q), -0.4, 0.4))
        Q.append(q); QD.append(qd); T.append(t)
    return np.mean(Q, 0), np.mean(QD, 0), np.mean(T, 0)


# ── B. friccion (procedimiento de S1) ────────────────────────────────────────
def identify_friction(arm):
    tau_c = np.zeros(4); tau_b = np.zeros(4)
    hold(arm, POSES[0])
    for k in range(4):
        vs, ts = [], []
        for v in [-0.30, -0.20, -0.10, 0.10, 0.20, 0.30]:
            cmd = np.zeros(4); cmd[k] = v
            for _ in range(1200):
                arm.write_velocity(cmd)
            q, qd, tau = avg(arm, 80)
            vs.append(qd[k]); ts.append(tau[k] - h_(q, qd)[k])
        arm.stop()
        for _ in range(600):
            arm.read()
        vs = np.array(vs); ts = np.array(ts)
        A = np.column_stack([np.sign(vs), vs])
        coef, *_ = np.linalg.lstsq(A, ts, rcond=None)
        tau_c[k], tau_b[k] = coef
    return tau_c, tau_b


# ── S2: constante de par contra pesas conocidas ──────────────────────────────
def identify_kt(arm, masses, exclude=(0,)):
    """Constante de par contra pesas conocidas, medida EN REPOSO.

    ⚠️ Tres cosas que salieron de validar el procedimiento contra un banco con la
    respuesta conocida, y que no eran obvias:

    (1) NO se resta ningun modelo de friccion. En reposo q̇~0, asi que el termino
        tau_c*sign(q̇)+tau_b*q̇ vale ~0 cualquiera sea su valor: pasarle la
        friccion de S1 no aporta y da la falsa impresion de que S2 depende de S1.
        No depende. Quitarlo elimina una dependencia circular aparente.

    (2) Medir ATRAVESANDO la pose y promediando los dos sentidos, para cancelar
        la friccion seca, NO funciona: medido, da 76-81 % de error con R2 0.10-0.35.
        El par reportado en movimiento arrastra el estado del integrador del lazo
        de velocidad del servo y terminos dinamicos que el miembro derecho no
        modela, y eso no se cancela al invertir el sentido. Reposo es mejor.

    (3) La junta de la BASE se excluye. Su eje es vertical y una pesa colgada
        aplica fuerza vertical, que no produce momento sobre un eje vertical: la
        regresion no tiene señal y devuelve pendiente cero. Su constante exige
        otra excitacion, por ejemplo tiro horizontal por polea.

    ⚠️ LIMITE DEL BANCO, no del procedimiento: en reposo la friccion ESTATICA
    real es indeterminada dentro de la banda de despegue, y este banco la modela
    como tanh(q̇/1e-3), o sea ~0 en reposo. Es decir que el banco NO puede probar
    ese caso, y en hardware hay que esperar degradacion acotada por
    ~tau_coulomb/sigma_min(Jv). Con 0.05 N.m y sigma_min mediano 0.0496 eso es
    ~1 N de incertidumbre.
    """
    q_des = POSE_S2          # pose propia de S2: brazo horizontal
    hold(arm, q_des)
    order = alternating(masses)
    I = []; RHS = []
    for m in order:
        arm.hang(m)
        hold(arm, q_des)
        q, qd, tau = avg(arm, 150, q_des)
        i_raw = tau/arm.kt                       # deshace la constante del driver
        f = np.array([0.0, 0.0, -m*G])
        rhs = h_(q, qd) - Jv_(q).T @ f           # sin termino de friccion: ver docstring
        I.append(i_raw); RHS.append(rhs)
    arm.hang(0.0)
    I = np.array(I); RHS = np.array(RHS)
    kt = np.full(4, np.nan); r2 = np.full(4, np.nan); b0 = np.full(4, np.nan)
    for k in range(4):
        if k in exclude:
            continue
        A = np.column_stack([I[:, k], np.ones(len(I))])
        coef, *_ = np.linalg.lstsq(A, RHS[:, k], rcond=None)
        pred = A @ coef
        den = np.sum((RHS[:, k] - RHS[:, k].mean())**2)
        kt[k], b0[k] = coef
        r2[k] = 1 - np.sum((RHS[:, k] - pred)**2)/max(den, 1e-12)
    return kt, b0, r2, order


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--bench", choices=["mujoco", "analytic"], default="mujoco",
                    help="mujoco = planta INDEPENDIENTE (recomendado)")
    args = ap.parse_args()
    if args.bench == "mujoco":
        Arm, T = mj_arm.MjArm, mj_arm.truth()
        BENCH = ("MuJoCo: integrador propio, damping/armature/frictionloss que el "
                 "modelo\n  del estimador NO tiene, comando de VELOCIDAD como el "
                 "servo real")
    else:
        Arm, T = SimArm, dxl_sim.truth()
        BENCH = ("analitico: comparte la dinamica con el estimador, asi que la "
                 "validacion\n  es parcialmente CIRCULAR. Solo para contraste.")
    print("=" * 74)
    print("VALIDACION DE PROCEDIMIENTOS contra banco con verdad conocida")
    print(f"banco = {BENCH}")
    print("(los scripts NO leen la verdad; arrancan con la constante semilla 1.0)")
    print("=" * 74)

    arm = Arm()
    arm.set_velocity_mode(); arm.enable(True)

    print("\n-- alcanza las poses? --")
    for i in (0, 1, 2):
        err = hold(arm, POSES[i])
        print(f"   pose {i+1}: error final {err:.2e} rad")

    print("\n-- B. identificacion de friccion (procedimiento de S1) --")
    tau_c, tau_b = identify_friction(arm)
    print(f"   {'junta':>6} {'Coulomb est':>12} {'viscosa est':>12}")
    for k in range(4):
        print(f"   {k+1:6d} {tau_c[k]:12.4f} {tau_b[k]:12.4f}")
    if "coulomb" in T:
        print(f"   verdad inyectada Coulomb = {np.round(T['coulomb'],4)}")
    else:
        print("   (en MuJoCo la friccion sale del XML: frictionloss 0.02 + "
              "damping 0.12)")

    print("\n-- A. control negativo: f_est sin contacto --")
    worst = 0.0
    for i, qd_des in enumerate(POSES):
        hold(arm, qd_des)
        q, qd, tau = avg(arm, 120, qd_des)
        f = f_est_static(q, qd, tau, tau_c*np.sign(qd) + tau_b*qd)
        worst = max(worst, np.linalg.norm(f))
        print(f"   pose {i+1}: |f_est| = {np.linalg.norm(f):7.3f} N")
    print(f"   peor = {worst:.3f} N   (criterio de S1: < 0.3 N)")
    print("   ⚠️ con la constante SEMILLA (1.0) y la verdad ~1.37, se espera que FALLE:")
    print("      es exactamente lo que S1 debe detectar antes de colgar pesas.")

    print("\n-- S2. constante de par contra pesas conocidas --")
    kt, b0, r2, order = identify_kt(arm, [0.0, 0.1, 0.2, 0.3, 0.5])
    print(f"   orden de masas (alternado): {order}")
    print(f"   {'junta':>6} {'kt estimada':>12} {'kt VERDAD':>10} {'error':>8} "
          f"{'ordenada':>10} {'R2':>8}")
    err_rel = []
    for k in range(4):
        if np.isnan(kt[k]):
            print(f"   {k+1:6d} {'excluida':>12} {T['kt'][k]:10.4f} "
                  f"{'---':>8} {'---':>10} {'---':>8}   eje vertical: sin señal")
            continue
        e = 100*abs(kt[k]-T['kt'][k])/T['kt'][k]; err_rel.append(e)
        print(f"   {k+1:6d} {kt[k]:12.4f} {T['kt'][k]:10.4f} {e:7.2f}% "
              f"{b0[k]:10.4f} {r2[k]:8.4f}")

    print("\n" + "=" * 74)
    r2v = r2[~np.isnan(r2)]
    ok = max(err_rel) < 5.0 and np.all(r2v > 0.98)
    print(f"peor error de kt = {max(err_rel):.2f} %   R2 minimo = {r2v.min():.4f}"
          f"   (3 de 4 juntas; la base necesita otra excitacion)")
    if ok:
        print("PROCEDIMIENTOS VALIDADOS: S2 recupera la constante de par inyectada.")
        print("El fallo de S1 con la semilla es el comportamiento CORRECTO.")
    else:
        print("REVISAR: la regresion no recupera la verdad en un banco limpio.")
        print("Corregir ANTES de tocar el brazo: en hardware no habria con que comparar.")
    print("=" * 74)
    arm.close()
