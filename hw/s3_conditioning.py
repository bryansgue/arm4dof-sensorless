"""
S3 — EL EXPERIMENTO QUE CAMBIA LA CATEGORIA DEL TRABAJO.

Verifica en hardware la prediccion falsable del paper:

        ||delta_f||  <=  ||delta_tau|| / sigma_min(Jv)

Son DOS partes, y miden cosas distintas. Confundirlas fue el primer diseño de
este script y estaba mal (ver abajo).

  A. LEY DE ESCALA (sin pesa).  La ley es sobre AMPLIFICACION DE RUIDO. Se mide
     la DISPERSION de f_est muestra a muestra con el brazo quieto, y se verifica
     que std(f_est) crece como 1/sigma_min. No hace falta pesa, ni modelo de
     gravedad, ni que el brazo cargue nada.

     ⚠️ ESTA PARTE NO SE PUEDE ENSAYAR EN SIMULACION. MuJoCo es determinista: en
     equilibrio el par es exactamente constante y std(tau) da 0.00000 en las
     posturas bien condicionadas. El experimento esta diseñado para medir algo
     que el simulador NO TIENE. Por eso es un experimento de hardware y no otra
     corrida de sim.

  B. EXACTITUD (con pesa conocida).  Mide el SESGO, no la dispersion. Es una
     afirmacion distinta y complementaria.

     ⚠️ SOSTENER EL BRAZO EN MODO POSICION, no en velocidad. La forma diferencial
     asume el MISMO Jv en las dos medidas, y colgar la pesa hunde el brazo. Medido
     en el banco (lazo de velocidad blando), el compromiso masa/contaminacion:

         masa      |dq|      dJ/J     senal    error
         0.05 kg   0.015 rad  2.3 %   0.098    0.13 N
         0.10 kg   0.030 rad  4.6 %   0.197    0.16 N
         0.20 kg   0.064 rad  9.6 %   0.415    0.22 N
         0.35 kg   0.120 rad 17.8 %   0.782    0.15 N

     Con lazo blando NINGUNA masa util baja del 3 %: bajar la masa reduce el
     hundimiento pero tambien la señal. La salida no es elegir la masa sino
     SOSTENER MAS RIGIDO. El modo posicion del MX-28 (kp del servo) es mucho mas
     rigido que este banco. El script reporta dq y dJ/J en cada postura y avisa
     si dJ/J supera el 3 %: si avisa, el numero de la parte B no sirve.

     ⚠️ Con un diseño anterior que promediaba tau(con)-tau(sin) sin vigilar esto,
     el ajuste de la ley daba pendiente +0.17 en vez de -1, o sea que habria
     "refutado" la Sec. IV por un error del test.

═══ POR QUE ESTE Y NO LA CAMPAÑA COMPLETA ═══
Es lo mas barato que convierte "todo es simulacion" en "la prediccion central se
verifico en hardware":

  - NO necesita lazo de control: ni NMPC, ni admitancia, ni compliance.
  - NO necesita el bus a 100 Hz. Es estatico; con 10 Hz sobra, asi que S0 deja
    de ser bloqueante.
  - La parte A NI SIQUIERA NECESITA LA PESA.

Hace falta: servos alimentados, leer Present_Current, y llevar el brazo a cuatro
posturas. La pesa solo para la parte B.

═══ CRITERIO ═══
Parte A: ajustar log(std f_est) contra log(sigma_min). Prediccion: pendiente -1.
  PASA   pendiente en [-1.4, -0.6] y R2 > 0.8
  FALLA  pendiente ~0  =>  la Seccion IV del paper esta mal y hay que reescribirla

Run:  python3 s3_conditioning.py --port /dev/ttyUSB0            # solo parte A
      python3 s3_conditioning.py --port /dev/ttyUSB0 --mass 0.2 # A y B
"""
import argparse
import os
import sys
import time
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "ocp_generation"))

import dxl_io
from arm_kinematics import build_jac_fn

fJ = build_jac_fn()
Jv_ = lambda q: np.array(fJ(q))[:3, :]
G = 9.81

# Cuatro posturas escalonadas en log(sigma_min), todas validadas en MuJoCo:
# sin contacto, z_ee > 0.08 m, 0.15 rad de margen contra topes, y con la pesa
# visible en las tres juntas de cabeceo (min|Jv^T z| > 0.07 N.m/N).
# Rango de 1/sigma_min: 9x. Cuatro puntos permiten AJUSTAR la pendiente, no solo
# comparar dos.
POSES = [
    (np.array([0.0, 2.035,  0.232,  0.000]), 0.0103),
    (np.array([0.0, 0.642,  0.464,  0.000]), 0.0208),
    (np.array([0.0, 0.874, -0.929,  1.161]), 0.0438),
    (np.array([0.0, 1.339, -0.232,  1.857]), 0.0917),
]

NSAMP = 200
SETTLE = 3.0


def measure(arm, n=NSAMP):
    Q = []; T = []
    for _ in range(n):
        q, qd, tau = arm.read()
        Q.append(q); T.append(tau)
        time.sleep(0.01)
    return np.mean(Q, 0), np.mean(T, 0), np.std(T, 0)


SIM = False


def ask(msg):
    """En hardware espera al operador; en ensayo en seco no bloquea."""
    if SIM:
        print(f"  [sim] {msg}")
        return
    input(msg)


def _place(arm, q_ref):
    """En ensayo en seco lleva el banco a la postura; en hardware no hace nada
    (el brazo lo posiciona el operador, con el par deshabilitado)."""
    if SIM:
        arm.set_state(q_ref)
        from validate_procedures import hold
        arm.enable(True); hold(arm, q_ref)


def part_A(arm):
    """Ley de escala: dispersion de f_est contra sigma_min. SIN pesa."""
    print("\n" + "=" * 74)
    print("PARTE A — ley de escala.  Sin pesa.  El brazo solo tiene que estar QUIETO.")
    print("=" * 74)
    rows = []
    for p, (q_ref, sig_model) in enumerate(POSES):
        print(f"\nPOSTURA {p+1}/4   q = {np.round(q_ref,3)}   "
              f"sigma_min esperado {sig_model:.4f}")
        ask("  Llevar el brazo a esa postura y fijarlo. Enter cuando este quieto... ")
        _place(arm, q_ref)
        Q = []; T = []
        for _ in range(NSAMP):
            q, qd, tau = arm.read()
            Q.append(q); T.append(tau)
            if not SIM: time.sleep(0.01)
        Q = np.array(Q); T = np.array(T)
        qm = Q.mean(0); Jv = Jv_(qm)
        sig = np.linalg.svd(Jv, compute_uv=False)[2]
        # estimacion muestra a muestra: su DISPERSION es lo que la ley predice
        F = np.array([np.linalg.lstsq(Jv.T, t, rcond=None)[0] for t in T])
        st = float(np.mean(np.std(T, 0))); sf = float(np.mean(np.std(F, 0)))
        rows.append((sig, st, sf))
        print(f"  sigma_min {sig:.4f}   std(tau) {st:.5f} N.m   "
              f"std(f_est) {sf:.4f} N   predicho {st/sig:.4f}")
        if st < 1e-9:
            print("  ⚠️ std(tau) = 0: la lectura no fluctua. En hardware NO deberia")
            print("     pasar; si pasa, la corriente esta saturando el cuantizador.")
    print("\n" + "-"*74)
    S = np.array([r[0] for r in rows]); D = np.array([r[2] for r in rows])
    print(f"  {'sigma_min':>10} {'std(tau)':>10} {'std(f)':>10} {'std(f)*sigma':>13}")
    for s, st, sf in rows:
        print(f"  {s:10.4f} {st:10.5f} {sf:10.4f} {sf*s:13.5f}")
    print("  la ultima columna deberia ser aproximadamente CONSTANTE")
    if np.all(D > 0):
        A = np.column_stack([np.log(S), np.ones(len(S))])
        c, *_ = np.linalg.lstsq(A, np.log(D), rcond=None)
        r2 = 1 - np.sum((np.log(D)-A@c)**2)/max(np.sum((np.log(D)-np.log(D).mean())**2), 1e-12)
        print(f"\n  pendiente = {c[0]:+.3f}   (prediccion -1)   R2 = {r2:.3f}")
        if -1.4 <= c[0] <= -0.6 and r2 > 0.8:
            print("\n  PARTE A PASA — el error escala con 1/sigma_min.")
            print("  El paper puede reportar verificacion EN HARDWARE de su prediccion")
            print("  central, y deja de ser un estudio de simulacion.")
        elif abs(c[0]) < 0.3:
            print("\n  PARTE A FALLA — el error NO depende del condicionamiento.")
            print("  La Seccion IV esta mal y hay que reescribirla. Es el resultado")
            print("  MAS informativo posible: refuta lo que el paper declara falsable.")
        else:
            print(f"\n  NO CONCLUYE: pendiente {c[0]:+.3f}, R2 {r2:.3f}")
        return S, D, c[0], r2
    print("\n  sin dispersion medible: no se puede ajustar")
    return S, D, np.nan, np.nan


def part_B(arm, mass):
    """Exactitud contra pesa conocida. Mide SESGO, no dispersion."""
    f_true = np.array([0.0, 0.0, -mass*G])
    print("\n" + "=" * 74)
    print(f"PARTE B — exactitud.  Pesa {mass:.3f} kg  ->  |f| = "
          f"{np.linalg.norm(f_true):.3f} N")
    print("⚠️ Sostener el brazo RIGIDO. Si se hunde al colgar la pesa, el")
    print("   Jacobiano cambia y el sesgo tapa el efecto que se quiere medir.")
    print("=" * 74)
    out = []
    for p, (q_ref, _) in enumerate(POSES):
        print(f"\nPOSTURA {p+1}/4   q = {np.round(q_ref,3)}")
        ask("  Llevar el brazo a esa postura y fijarlo. Enter... ")
        _place(arm, q_ref)
        M = {}
        for lab in ("SIN", "CON"):
            ask(f"  Pesa {lab} colgada. Enter cuando este quieto... ")
            if SIM:
                arm.hang(mass if lab == "CON" else 0.0); _place(arm, q_ref)
            time.sleep(SETTLE)                # re-asentar: cambia la stiction
            Q = []; T = []
            for _ in range(NSAMP):
                q, qd, tau = arm.read(); Q.append(q); T.append(tau)
            if not SIM: time.sleep(0.01)
            M[lab] = (np.mean(Q, 0), np.mean(T, 0))
        dq = np.linalg.norm(M["CON"][0] - M["SIN"][0])
        qm = 0.5*(M["SIN"][0] + M["CON"][0])
        dJ = np.linalg.norm(Jv_(M["CON"][0]) - Jv_(M["SIN"][0]))/np.linalg.norm(Jv_(qm))
        f_hat = -np.linalg.lstsq(Jv_(qm).T, M["CON"][1] - M["SIN"][1], rcond=None)[0]
        e = np.linalg.norm(f_hat - f_true)
        out.append((np.linalg.svd(Jv_(qm), compute_uv=False)[2], e, dq, dJ))
        print(f"  |f_est| {np.linalg.norm(f_hat):6.3f} N   error {e:6.3f} N   "
              f"hundimiento |dq| {dq:.4f} rad   dJ/J {100*dJ:.1f} %")
        if dJ > 0.03:
            print("  ⚠️ dJ/J > 3 %: el brazo se hundio y la forma diferencial queda")
            print("     contaminada. Sostener mas rigido o usar una pesa menor.")
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--ids", type=int, nargs="+", default=[1, 2, 3, 4])
    ap.add_argument("--mass", type=float, default=None,
                    help="kg de la pesa; omitir para correr solo la parte A")
    ap.add_argument("--sim", action="store_true",
                    help="ENSAYO EN SECO contra MuJoCo: sin hardware y sin prompts. "
                         "Verifica el flujo del script, NO produce resultados "
                         "publicables. La parte A ademas no puede validarse asi: "
                         "MuJoCo es determinista y std(tau) da 0.")
    args = ap.parse_args()

    print("=" * 74)
    print("S3 — verificacion en hardware de la ley 1/sigma_min(Jv)")
    print("=" * 74)
    print("El brazo NO se mueve solo: el par queda DESHABILITADO y lo posicionas")
    print("a mano. Solo se lee.")

    global SIM
    SIM = args.sim
    if SIM:
        import mj_arm
        print("\n⚠️ ENSAYO EN SECO contra MuJoCo. No es hardware y no produce")
        print("   resultados publicables: solo verifica que el script corre.\n")
        arm = mj_arm.MjArm(); arm.set_velocity_mode()
    else:
        arm = dxl_io.DxlArm(port=args.port, baud=args.baud, ids=args.ids)
    try:
        arm.enable(False)
        S, D, slope, r2 = part_A(arm)
        B = part_B(arm, args.mass) if args.mass else None
    finally:
        arm.close()

    np.savez(os.path.join(HERE, "s3_result.npz"), sigma=S, std_f=D,
             slope=slope, r2=r2,
             partB=np.array(B) if B else np.array([]),
             mass=args.mass if args.mass else 0.0)
    print("\nguardado -> s3_result.npz")


if __name__ == "__main__":
    main()
