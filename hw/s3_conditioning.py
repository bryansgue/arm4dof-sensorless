"""
S3 — EL EXPERIMENTO QUE CAMBIA LA CATEGORIA DEL TRABAJO.

Verifica en hardware la prediccion falsable del paper:

        ||delta_f||  <=  ||delta_tau|| / sigma_min(Jv)

Son DOS partes, y miden cosas distintas. Confundirlas fue el primer diseño de
este script y estaba mal (ver abajo).

⚠️⚠️ LAS DOS PARTES SON OBLIGATORIAS PARA EL PAPER. Correr solo la A fue el plan
    original y NO ALCANZA — lo señalo la revision del asesor (02/08/2026) y es
    correcto: un ensayo sin carga conocida valida ruido, deriva y dependencia
    postural, pero NO PUEDE validar ni la exactitud de la fuerza ni la
    mal-atribucion fuerza-momento, que es la tesis central del trabajo. La parte
    A sigue siendo la mas barata y la unica que verifica la LEY, pero sola deja
    la contribucion principal sin evidencia fisica.

  A. LEY DE ESCALA (sin carga).  La ley es sobre AMPLIFICACION DE RUIDO. Se mide
     la DISPERSION de f_est muestra a muestra con el brazo quieto, y se verifica
     que std(f_est) crece como 1/sigma_min. No hace falta carga, ni modelo de
     gravedad, ni que el brazo cargue nada.

     ⚠️ ESTA PARTE NO SE PUEDE ENSAYAR EN SIMULACION. MuJoCo es determinista: en
     equilibrio el par es exactamente constante y std(tau) da 0.00000 en las
     posturas bien condicionadas. El experimento esta diseñado para medir algo
     que el simulador NO TIENE. Por eso es un experimento de hardware y no otra
     corrida de sim.

  B. EXACTITUD Y MAL-ATRIBUCION (con carga conocida).  Mide el SESGO, no la
     dispersion, y ademas corre LOS DOS ESTIMADORES sobre el MISMO residuo. Es la
     unica parte que puede refutar la Seccion IV: si el inverso 6-D recupera la
     fuerza igual de bien que el de contacto puntual, la contribucion principal
     cae. No hace falta un sensor F/T de seis ejes.

     DOS DIRECCIONES, no una. Una sola direccion (pesa colgada) no distingue un
     estimador sesgado de uno que solo acierta en vertical, y sobre todo no
     expone el momento espurio: con f puramente vertical el reparto 6-D puede
     quedar chico por casualidad geometrica. La segunda direccion es tiro
     HORIZONTAL con una balanza de equipaje o dinamometro.

       --mass 0.2                masa colgada, f = (0,0,-mg), exacta
       --pull 2.0 --pull-axis x  tiro horizontal, |f| leido de la balanza

     ⚠️ El tiro horizontal es el que valida la junta de la base: una fuerza
     vertical no hace momento sobre un eje vertical, asi que la pesa deja esa
     junta sin excitar (ver hw/README.md).

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
Es lo mas barato que convierte "todo es simulacion" en "las predicciones centrales
se verificaron en hardware":

  - NO necesita lazo de control: ni NMPC, ni admitancia, ni compliance.
  - NO necesita el bus a 100 Hz. Es estatico; con 10 Hz sobra, asi que S0 deja
    de ser bloqueante.
  - NO necesita sensor F/T: una masa colgada y una balanza de equipaje.

Hace falta: servos alimentados, leer Present_Current, y llevar el brazo a cuatro
posturas. Una masa (~0.2 kg) y una balanza de equipaje para la parte B.

═══ CRITERIO ═══
Parte A: ajustar log(std f_est) contra log(sigma_min). Prediccion: pendiente -1.
  PASA   pendiente en [-1.4, -0.6] y R2 > 0.8
  FALLA  pendiente ~0  =>  la Seccion IV del paper esta mal y hay que reescribirla

Parte B: sobre el MISMO residuo, error del 3-D contra error del 6-D.
  PASA   el 3-D gana claramente Y el 6-D produce momento espurio no nulo
  FALLA  los dos empatan  =>  la contribucion principal no se sostiene en hardware

Run:  python3 s3_conditioning.py --port /dev/ttyUSB0 --mass 0.2 --pull 2.0
      python3 s3_conditioning.py --port /dev/ttyUSB0            # solo parte A
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
from arm_kinematics import build_jac_fn, build_fk_fn

fJ = build_jac_fn()
fk = build_fk_fn()
J_ = lambda q: np.array(fJ(q))            # 6x4
Jv_ = lambda q: np.array(fJ(q))[:3, :]    # 3x4
G = 9.81


def est_3d(q, tau):
    """Contacto puntual en punto conocido: Jv^T f = tau, SOBRE-determinado."""
    return np.linalg.lstsq(Jv_(q).T, tau, rcond=None)[0]


def est_6d(q, tau):
    """Wrench completo min-norm: J^T F = tau, INDETERMINADO (6 incognitas, 4 ec.).
    Devuelve (fuerza, momento). El momento deberia ser CERO —la carga es una
    fuerza pura— y no lo es: eso es la mal-atribucion que el paper reporta."""
    J = J_(q)
    F = J @ np.linalg.lstsq(J.T @ J, tau, rcond=None)[0]
    return F[:3], F[3:]

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

# Rejas de cuasi-estatica. La forma diferencial supone que las dos medidas son
# EQUILIBRIOS: si el brazo todavia se mueve, tau incluye inercia y la lectura de
# la balanza no corresponde al par medido.
QD_MAX = 0.02          # rad/s, velocidad maxima admitida durante la ventana
DRIFT_FRAC = 0.10      # deriva admitida entre 1a y 2a mitad, como fraccion de |dtau|


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


AXES = {"x": np.array([1.0, 0.0, 0.0]), "y": np.array([0.0, 1.0, 0.0])}


def _sample(arm):
    """Ventana de NSAMP lecturas con diagnostico de CUASI-ESTATICA.

    Devuelve tambien qd_max (el brazo tiene que estar quieto: si se mueve, tau
    lleva inercia y la lectura de la balanza no corresponde al par medido) y la
    deriva entre la primera y la segunda mitad de la ventana (stiction que se
    reacomoda, o el operador que no sostiene constante el tiro)."""
    Q = []; QD = []; T = []
    for _ in range(NSAMP):
        q, qd, tau = arm.read()
        Q.append(q); QD.append(qd); T.append(tau)
        if not SIM: time.sleep(0.01)
    Q = np.array(Q); QD = np.array(QD); T = np.array(T)
    half = len(T)//2
    return dict(q=Q.mean(0), tau=T.mean(0), tau_std=T.std(0),
                qd_max=float(np.max(np.abs(QD))),
                drift=float(np.max(np.abs(T[half:].mean(0) - T[:half].mean(0)))))


def _load_case(arm, q_ref, apply_sim):
    """Diferencial CON-SIN sobre una postura, con diagnostico completo."""
    M = {}
    for lab in ("SIN", "CON"):
        if lab == "CON":
            ask("  Aplicar la carga LENTAMENTE (cuasi-estatica) y sostenerla. "
                "Enter cuando este quieto... ")
        else:
            ask("  Sin carga. Enter cuando este quieto... ")
        if SIM:
            apply_sim(lab == "CON"); _place(arm, q_ref)
        time.sleep(SETTLE)                    # re-asentar: cambia la stiction
        M[lab] = _sample(arm)
    dtau = M["CON"]["tau"] - M["SIN"]["tau"]
    qm = 0.5*(M["SIN"]["q"] + M["CON"]["q"])
    return dict(
        qm=qm, dtau=dtau,
        dq=float(np.linalg.norm(M["CON"]["q"] - M["SIN"]["q"])),
        dJ=float(np.linalg.norm(Jv_(M["CON"]["q"]) - Jv_(M["SIN"]["q"]))
                 / np.linalg.norm(Jv_(qm))),
        # reposo: es el cero del sensor de corriente en esa postura, y su ruido
        tau_rest=M["SIN"]["tau"], tau_rest_std=M["SIN"]["tau_std"],
        qd_max=max(M["SIN"]["qd_max"], M["CON"]["qd_max"]),
        drift=max(M["SIN"]["drift"], M["CON"]["drift"]),
        drift_frac=max(M["SIN"]["drift"], M["CON"]["drift"])
        / max(float(np.linalg.norm(dtau)), 1e-9))


def part_B(arm, mass=None, pull=None, pull_axis="x", tol=0.0, repeats=3):
    """Exactitud y mal-atribucion contra carga conocida, en DOS direcciones.

    Mide SESGO, no dispersion, y corre los DOS estimadores sobre el MISMO
    residuo. Es la unica parte que puede refutar la Seccion IV en hardware.

    Registra ademas todo lo que hace falta para que el ensayo sea auditable:
    direccion y punto de aplicacion, incertidumbre del instrumento, corriente en
    reposo, repetibilidad entre ensayos, y las rejas de cuasi-estatica.
    """
    cases = []
    if mass:
        # la masa se pesa una vez y no varia: la incertidumbre es despreciable
        # frente a la de un tiro sostenido a mano
        cases.append(("colgada", np.array([0.0, 0.0, -mass*G]), mass*G*0.02,
                      lambda on, m=mass: arm.hang(m if on else 0.0)))
    if pull:
        u = AXES[pull_axis]
        # tol: incertidumbre declarada de la balanza [N]. Si no se da, 5 % del
        # valor, que es lo tipico de una balanza de equipaje barata.
        cases.append((f"tiro {pull_axis}", pull*u, tol if tol > 0 else 0.05*pull,
                      lambda on, v=pull*u: arm.push(v if on else np.zeros(3))))
    if not cases:
        return None

    print("\n" + "=" * 74)
    print("PARTE B — exactitud y mal-atribucion, con carga conocida")
    print(f"{repeats} ensayos por postura y carga (repetibilidad)")
    print("⚠️ Sostener el brazo RIGIDO. Si se hunde al aplicar la carga, el")
    print("   Jacobiano cambia y el sesgo tapa el efecto que se quiere medir.")
    print("⚠️ Aplicar la carga CUASI-ESTATICAMENTE: si el brazo se mueve, tau")
    print("   lleva inercia y la lectura del instrumento no corresponde al par.")
    print("=" * 74)

    out = []
    for name, f_true, u_f, apply_sim in cases:
        print(f"\n--- CARGA '{name}'   f_true = {np.round(f_true,3)} N   "
              f"|f| = {np.linalg.norm(f_true):.3f} ± {u_f:.3f} N ---")
        if "tiro" in name:
            print(f"   Tirar del efector con la balanza a lo largo del eje "
                  f"{pull_axis} del mundo.")
            print("   El tiro se aplica EN EL EFECTOR: es el punto de contacto que")
            print("   el estimador supone conocido. Aplicarlo en otro lado invalida")
            print("   el modelo, no el estimador.")
        for p, (q_ref, _) in enumerate(POSES):
            print(f"\nPOSTURA {p+1}/{len(POSES)}   q = {np.round(q_ref,3)}")
            ask("  Llevar el brazo a esa postura y fijarlo. Enter... ")
            _place(arm, q_ref)
            trials = []
            for r in range(repeats):
                print(f"  ensayo {r+1}/{repeats}")
                d = _load_case(arm, q_ref, apply_sim)
                # el residuo de la carga es -dtau: el par del servo la COMPENSA
                f3 = est_3d(d["qm"], -d["dtau"])
                f6, m6 = est_6d(d["qm"], -d["dtau"])
                d.update(f3=f3, f6=f6,
                         e3=float(np.linalg.norm(f3 - f_true)),
                         e6=float(np.linalg.norm(f6 - f_true)),
                         m6=float(np.linalg.norm(m6)),
                         p_contact=np.array(fk(d["qm"])[1]).flatten(),
                         sig=float(np.linalg.svd(Jv_(d["qm"]),
                                                 compute_uv=False)[2]))
                trials.append(d)
                print(f"    3-D |f| {np.linalg.norm(f3):6.3f} N  error {d['e3']:6.3f} N"
                      f"   |   6-D |f| {np.linalg.norm(f6):6.3f} N  "
                      f"error {d['e6']:6.3f} N  |M| {d['m6']:6.3f} N.m")

            F3 = np.array([t["f3"] for t in trials])
            rep = float(np.max(np.linalg.norm(F3 - F3.mean(0), axis=1)))
            agg = dict(
                carga=name, sigma=trials[0]["sig"], u_f=u_f,
                p_contact=trials[0]["p_contact"],
                dir_f=f_true/max(np.linalg.norm(f_true), 1e-9),
                e3=float(np.mean([t["e3"] for t in trials])),
                e6=float(np.mean([t["e6"] for t in trials])),
                m6=float(np.mean([t["m6"] for t in trials])),
                repet=rep,
                dq=float(np.max([t["dq"] for t in trials])),
                dJ=float(np.max([t["dJ"] for t in trials])),
                qd_max=float(np.max([t["qd_max"] for t in trials])),
                drift_frac=float(np.max([t["drift_frac"] for t in trials])),
                tau_rest=trials[0]["tau_rest"],
                tau_rest_std=trials[0]["tau_rest_std"])
            out.append(agg)
            print(f"  sigma_min {agg['sigma']:.4f}   punto de contacto "
                  f"{np.round(agg['p_contact'],3)} m")
            print(f"  3-D error medio {agg['e3']:6.3f} N      "
                  f"6-D error medio {agg['e6']:6.3f} N   "
                  f"|momento espurio| {agg['m6']:6.3f} N.m")
            print(f"  repetibilidad (dispersion de f_3D entre ensayos) "
                  f"{agg['repet']:.3f} N")
            print(f"  reposo |tau| {np.linalg.norm(agg['tau_rest']):.4f} N.m   "
                  f"ruido medio {np.mean(agg['tau_rest_std']):.5f} N.m")
            print(f"  hundimiento |dq| {agg['dq']:.4f} rad   dJ/J {100*agg['dJ']:.1f} %"
                  f"   qd_max {agg['qd_max']:.4f} rad/s   "
                  f"deriva {100*agg['drift_frac']:.1f} % de |dtau|")
            if agg["dJ"] > 0.03:
                print("  ⚠️ dJ/J > 3 %: el brazo se hundio y la forma diferencial")
                print("     queda contaminada. Sostener mas rigido o bajar la carga.")
            if agg["qd_max"] > QD_MAX:
                print(f"  ⚠️ qd_max > {QD_MAX} rad/s: NO fue cuasi-estatico. tau lleva")
                print("     inercia y no compara contra la lectura del instrumento.")
            if agg["drift_frac"] > DRIFT_FRAC:
                print(f"  ⚠️ deriva > {100*DRIFT_FRAC:.0f} % de |dtau|: la carga no se")
                print("     sostuvo constante, o la stiction se reacomodo.")

    print("\n" + "-"*74)
    E3 = np.array([r["e3"] for r in out]); E6 = np.array([r["e6"] for r in out])
    Msp = np.array([r["m6"] for r in out]); U = np.array([r["u_f"] for r in out])
    REP = np.array([r["repet"] for r in out])
    bad_J = int(np.sum([r["dJ"] > 0.03 for r in out]))
    bad_qs = int(np.sum([r["qd_max"] > QD_MAX or r["drift_frac"] > DRIFT_FRAC
                         for r in out]))
    print(f"  error medio   3-D {E3.mean():.3f} N     6-D {E6.mean():.3f} N")
    print(f"  momento espurio del 6-D:  medio {Msp.mean():.3f} N.m   "
          f"max {Msp.max():.3f} N.m   (deberia ser 0)")
    print(f"  incertidumbre del instrumento: {U.mean():.3f} N   "
          f"repetibilidad media {REP.mean():.3f} N")
    print(f"  celdas con dJ/J > 3 %: {bad_J}/{len(out)}   "
          f"no cuasi-estaticas: {bad_qs}/{len(out)}")
    # el piso de resolucion del ensayo: no se puede afirmar una diferencia
    # menor que la incertidumbre del instrumento mas la dispersion entre ensayos
    piso = float(U.mean() + REP.mean())
    print(f"  piso de resolucion (instrumento + repetibilidad): {piso:.3f} N")
    if bad_J or bad_qs:
        print(f"\n  SIN VEREDICTO — {bad_J} celdas contaminadas por hundimiento y")
        print(f"  {bad_qs} por no ser cuasi-estaticas. Los numeros de arriba mezclan")
        print("  el efecto que se quiere medir con un cambio de Jacobiano o con")
        print("  inercia. Sostener mas rigido (modo posicion), aplicar la carga mas")
        print("  despacio, y repetir. NO reportar esto en el paper.")
    elif E6.mean() - E3.mean() < piso:
        print(f"\n  NO CONCLUYE — la separacion entre estimadores "
              f"({E6.mean()-E3.mean():.3f} N) no supera el piso del ensayo")
        print(f"  ({piso:.3f} N). Hace falta una carga mayor o un instrumento mejor.")
    elif E6.mean() > 1.5*E3.mean() and Msp.mean() > 0.05:
        print("\n  PARTE B PASA — el 6-D pierde fuerza y genera momento espurio")
        print("  sobre hardware. La Seccion IV queda verificada fisicamente.")
    elif E6.mean() < 1.2*E3.mean():
        print("\n  PARTE B FALLA — los dos estimadores empatan. La contribucion")
        print("  principal no se sostiene en hardware y hay que reescribirla.")
    else:
        print("\n  NO CONCLUYE: la separacion entre estimadores es debil.")
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--ids", type=int, nargs="+", default=[1, 2, 3, 4])
    ap.add_argument("--mass", type=float, default=None,
                    help="kg de la masa colgada (direccion vertical de la parte B)")
    ap.add_argument("--pull", type=float, default=None,
                    help="N de tiro horizontal leidos de la balanza de equipaje "
                         "(segunda direccion de la parte B)")
    ap.add_argument("--pull-axis", default="x", choices=["x", "y"],
                    help="eje de mundo del tiro horizontal")
    ap.add_argument("--pull-tol", type=float, default=0.0,
                    help="incertidumbre declarada de la balanza [N]. Por defecto "
                         "5 %% del tiro, tipico de una balanza de equipaje")
    ap.add_argument("--repeats", type=int, default=3,
                    help="ensayos por postura y carga, para medir repetibilidad")
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
        print("   resultados publicables: solo verifica que el script corre.")
        print("   El banco es DETERMINISTA, asi que la repetibilidad y el ruido")
        print("   de reposo salen 0.000 por construccion. Esos dos numeros solo")
        print("   significan algo en hardware.\n")
        arm = mj_arm.MjArm(); arm.set_velocity_mode()
    else:
        arm = dxl_io.DxlArm(port=args.port, baud=args.baud, ids=args.ids)
    if not (args.mass or args.pull):
        print("\n⚠️ Sin --mass ni --pull corre SOLO la parte A. Alcanza para")
        print("   verificar la ley 1/sigma_min, pero NO valida la exactitud de la")
        print("   fuerza ni la mal-atribucion fuerza-momento: para el paper hacen")
        print("   falta las dos partes, y la B en DOS direcciones.")

    try:
        arm.enable(False)
        S, D, slope, r2 = part_A(arm)
        B = part_B(arm, args.mass, args.pull, args.pull_axis,
                   args.pull_tol, args.repeats)
    finally:
        arm.close()

    np.savez(os.path.join(HERE, "s3_result.npz"), sigma=S, std_f=D,
             slope=slope, r2=r2,
             partB=np.array(B, dtype=object) if B else np.array([]),
             mass=args.mass if args.mass else 0.0,
             pull=args.pull if args.pull else 0.0,
             pull_axis=args.pull_axis, pull_tol=args.pull_tol,
             repeats=args.repeats)
    print("\nguardado -> s3_result.npz")


if __name__ == "__main__":
    main()
