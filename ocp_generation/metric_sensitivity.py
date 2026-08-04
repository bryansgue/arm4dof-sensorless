"""
SENSIBILIDAD A LA METRICA DEL WRENCH — lo que el revisor va a atacar primero.

El inverso 6-D de norma minima resuelve

    min ‖F‖  s.a.  Jᵀ F = tau,      F = [f; m] ∈ R⁶

y ESA NORMA NO ES DIMENSIONALMENTE HOMOGENEA: f va en N y m en N·m. Sumar sus
cuadrados exige una LONGITUD. La practica estandar —pseudoinversa sin ponderar
sobre J en unidades SI— equivale a fijar esa longitud en 1 m sin decirlo.

Con la norma ponderada

    ‖F‖_lc² = ‖f‖² + ‖m‖²/lc²          W = diag(I, I/lc)

la solucion es F = W⁻² J (Jᵀ W⁻² J)⁻¹ tau —su RANGO es col(W⁻² J), no W⁻¹ col(J)—
y para una fuerza pura f el bloque de fuerza devuelto es M(lc) f, con M(lc) el
analogo ponderado de P_ff.

Este script separa lo INVARIANTE de lo que depende de la metrica:

  invariante            el conjunto recuperado exacto, que es Jv(ker Jw): las
                        fuerzas que produce un movimiento SIN rotacion (Prop. 2)
  invariante            los AUTOVECTORES de M(lc), pero NO por casualidad: con los
                        tres ejes de pitch paralelos, M = I - (1-lam) a aᵀ con a el
                        eje de pitch (Prop. 3). Para un Jacobiano generico es FALSO
  invariante            la relacion error-alineacion POR POSE, que es la identidad
                        ‖M f - f‖ = (1-lam)|cos|, exacta a cualquier lc
  DEPENDE de la metrica los AUTOVALORES: el 50.5 %, el "100 % de configuraciones
                        con lam_min<0.10", la fraccion convertida en momento, y la
                        correlacion AGRUPADA del barrido direccional (0.4475 a 0.9993)

Run:  python3 metric_sensitivity.py
"""
import numpy as np
from arm_kinematics import build_jac_fn, build_fk_fn

_fJ = build_jac_fn(); _fk = build_fk_fn()
J_ = lambda q: np.array(_fJ(q))
p_ = lambda q: np.array(_fk(q)[1]).flatten()

Q_LO = np.array([-3.1416, -0.436332, -2.00713, -2.00713])
Q_HI = np.array([ 3.1416,  3.57792,  2.00713,  2.00713])

LCS = [0.05, 0.10, 0.15, 0.20, 0.2327, 0.30, 0.50, 1.00, 2.00]
LC_STD = 1.00        # lo que la practica estandar usa sin declararlo
LC_ARM = 0.2327      # alcance medio del brazo sobre el barrido

POSES = {
    "P1": np.array([0.0,  1.2, -0.9,  0.4]),
    "P2": np.array([0.6,  1.4, -0.7,  0.9]),
    "P3": np.array([-0.5, 0.7, -1.4, -0.3]),
    "P4": np.array([0.3,  2.2, -1.6,  0.8]),
    "P5": np.array([-0.8, 0.9, -0.3, -1.2]),
    "P6": np.array([0.2,  2.8, -1.1,  0.5]),
}
Q_TASK = POSES["P1"]


def force_block(q, lc):
    """M(lc): fuerza devuelta por el inverso 6-D ponderado ante una fuerza pura.

    ⚠️ El RANGO del estimador ponderado es col(W⁻² J), NO W⁻¹ col(J). Las dos
    formas coinciden numericamente aca porque el bloque de fuerza de W es la
    identidad, pero el argumento correcto es el de W⁻², y es el que esta en la
    demostracion de la Proposicion 2.
    """
    J = J_(q)
    Wi = np.diag([1., 1., 1., lc, lc, lc])          # W⁻¹
    A = np.linalg.pinv(J.T @ Wi)                    # (Jᵀ W⁻¹)⁺
    return (Wi @ A @ J[:3, :].T)[:3, :3]


def force_block_w2(q, lc):
    """La MISMA M, por la via W⁻²: F = W⁻²J (Jᵀ W⁻² J)⁻¹ tau. Control cruzado."""
    J = J_(q)
    W2i = np.diag([1., 1., 1., lc*lc, lc*lc, lc*lc])
    return (W2i @ J @ np.linalg.pinv(J.T @ W2i @ J) @ J[:3, :].T)[:3, :3]


def plano_del_brazo(q, tol=1e-9):
    """P = Jv(ker Jw): las fuerzas que produce un movimiento SIN rotacion.
    Es el conjunto recuperado exacto, y no depende de lc (Proposicion 2)."""
    J = J_(q); Jw = J[3:, :]; Jv = J[:3, :]
    V = np.linalg.svd(Jw, full_matrices=True)[2]
    K = V[np.linalg.matrix_rank(Jw, tol):].T        # base de ker Jw
    B = Jv @ K
    U, s, _ = np.linalg.svd(B, full_matrices=False)
    r = int(np.sum(s > tol*max(1.0, s[0])))
    return U[:, :r], r


def full_estimate(q, f, lc):
    """Wrench 6-D ponderado completo ante una fuerza pura f: devuelve (f_hat, m_hat)."""
    J = J_(q)
    Wi = np.diag([1., 1., 1., lc, lc, lc])
    F = Wi @ np.linalg.pinv(J.T @ Wi) @ (J[:3, :].T @ f)
    return F[:3], F[3:]


def sweep(n=13, seed=0):
    grids = [np.linspace(Q_LO[i], Q_HI[i], n) for i in (1, 2, 3)]
    rng = np.random.default_rng(seed)
    qs, us = [], []
    for a in grids[0]:
        for b in grids[1]:
            for c in grids[2]:
                qs.append(np.array([0., a, b, c]))
                u = rng.normal(size=3); us.append(u/np.linalg.norm(u))
    return qs, us


if __name__ == "__main__":
    np.set_printoptions(precision=4, suppress=True)
    print("== SENSIBILIDAD A LA METRICA DEL WRENCH ==\n")

    qs, us = sweep()
    reach = np.array([np.linalg.norm(p_(q)) for q in qs])
    print(f"-- Escala del brazo: alcance del efector sobre {len(qs)} configuraciones")
    print(f"   media {reach.mean():.4f} m   mediana {np.median(reach):.4f} m"
          f"   max {reach.max():.4f} m\n")

    # ── A. cuanto se pierde, contra lc ────────────────────────────────────────
    print("-- A. Fuerza descartada por el inverso 6-D, contra la longitud caracteristica")
    print(f"   {'lc [m]':>8} {'media %':>9} {'mediana %':>10} {'p90 %':>8} {'max %':>8}")
    loss_by_lc = {}
    for lc in LCS:
        e = np.array([np.linalg.norm(force_block(q, lc) @ u - u) for q, u in zip(qs, us)])
        loss_by_lc[lc] = e
        tag = "  <- practica estandar" if lc == LC_STD else (
              "  <- alcance del brazo" if lc == LC_ARM else "")
        print(f"   {lc:8.4f} {100*e.mean():9.1f} {100*np.median(e):10.1f} "
              f"{100*np.percentile(e,90):8.1f} {100*e.max():8.1f}{tag}")

    # ── B. el indicador mas fragil ────────────────────────────────────────────
    print("\n-- B. lam_min(M) y la fraccion de configuraciones 'casi ciegas'")
    print(f"   {'lc [m]':>8} {'lam_min medio':>14} {'% configs lam_min<0.10':>24}")
    for lc in LCS:
        lm = np.array([np.linalg.eigvalsh(force_block(q, lc))[0] for q in qs])
        print(f"   {lc:8.4f} {lm.mean():14.3f} {100*np.mean(lm < 0.10):23.0f} %")
    print("   ⚠️ este indicador NO es invariante: no citarlo sin declarar lc")

    # ── C. lo que SI es invariante: la estructura direccional ─────────────────
    print("\n-- C. INVARIANCIA de la estructura direccional de M(lc)")
    print("   ⚠️ NO comparar autovectores uno a uno: el espectro es {lam, 1, 1} y la base")
    print("      del autoespacio degenerado es arbitraria. Se compara la direccion CIEGA")
    print("      (autovector de menor autovalor) y el autoespacio de lam=1 como SUBESPACIO.")
    ang_blind, ang_sub = [], []
    asym = 0.0
    n_deg = 0
    for q in qs[::7]:
        M1 = force_block(q, LC_STD)
        asym = max(asym, np.abs(M1 - M1.T).max())
        w1, V1 = np.linalg.eigh((M1 + M1.T)/2)
        n_deg += int(np.sum(w1 > 0.999) == 2)
        for lc in LCS:
            if lc == LC_STD:
                continue
            M = force_block(q, lc)
            w, V = np.linalg.eigh((M + M.T)/2)
            ang_blind.append(np.degrees(np.arccos(min(1.0, abs(V1[:, 0] @ V[:, 0])))))
            # angulo principal maximo entre los dos autoespacios de lam=1
            s = np.linalg.svd(V1[:, 1:].T @ V[:, 1:], compute_uv=False)
            ang_sub.append(np.degrees(np.arccos(min(1.0, s.min()))))
    ang_blind = np.array(ang_blind); ang_sub = np.array(ang_sub)
    print(f"   asimetria maxima de M: {asym:.2e}  -> M simetrica")
    print(f"   espectro {{lam, 1, 1}} en {100*n_deg/len(qs[::7]):.0f} % de las configuraciones")
    print(f"   direccion ciega: angulo contra lc=1 m   media {ang_blind.mean():.3e} deg"
          f"   max {ang_blind.max():.3e} deg")
    print(f"   autoespacio lam=1: angulo principal max  media {ang_sub.mean():.3e} deg"
          f"   max {ang_sub.max():.3e} deg")
    print("   => QUE direcciones se pierden no depende de la metrica; CUANTO, si")

    # ── D. el ejemplo del paper, contra lc ────────────────────────────────────
    print("\n-- D. El ejemplo de la Sec. IV: empuje de 6 N normal al plano en q_task")
    f = np.array([0., 6., 0.])
    tau = J_(Q_TASK)[:3, :].T @ f
    print(f"   tau = {np.round(tau,4)}   |tau| = {np.linalg.norm(tau):.4f} N.m")
    print(f"   {'lc [m]':>8} {'|f_hat| [N]':>12} {'|m_hat| [N.m]':>14} "
          f"{'autovalores de M':>28}")
    for lc in LCS:
        fh, mh = full_estimate(Q_TASK, f, lc)
        w = np.linalg.eigvalsh(force_block(Q_TASK, lc))
        print(f"   {lc:8.4f} {np.linalg.norm(fh):12.3f} {np.linalg.norm(mh):14.3f} "
              f"   {np.round(w,3)}")

    # ── E. el barrido direccional sobrevive ───────────────────────────────────
    print("\n-- E. Barrido direccional: 2000 direcciones x 6 poses, contra lc")
    print("   La correlacion POR POSE es la prediccion; la agrupada mezcla 6 pendientes")
    print("   distintas y por eso baja cuando lc separa los lam_min de las poses.")
    print(f"   {'lc [m]':>8} {'corr por pose (min)':>21} {'corr agrupada':>15} "
          f"{'mediana err %':>15}")
    for lc in LCS:
        al_all, e6_all, per_pose = [], [], []
        for q in POSES.values():
            M = force_block(q, lc)
            w, V = np.linalg.eigh((M + M.T)/2)
            blind = V[:, 0]
            rng = np.random.default_rng(0)
            U = rng.normal(size=(2000, 3)); U /= np.linalg.norm(U, axis=1, keepdims=True)
            al = np.abs(U @ blind); e6 = 100*np.linalg.norm(U @ M.T - U, axis=1)
            per_pose.append(np.corrcoef(al, e6)[0, 1])
            al_all.append(al); e6_all.append(e6)
        al = np.concatenate(al_all); e6 = np.concatenate(e6_all)
        print(f"   {lc:8.4f} {min(per_pose):+21.6f} {np.corrcoef(al, e6)[0,1]:+15.4f} "
              f"{np.median(e6):15.1f}")
    print("   => por pose el error es una funcion LINEAL EXACTA de la alineacion a")
    print("      cualquier lc: err = (1-lam_min)*|cos|. Invariante. Lo que la metrica")
    print("      fija es la PENDIENTE (1-lam_min), y con ella la correlacion agrupada.")

    # ── G. la ESTRUCTURA demostrada (Proposiciones 2 y 3) ─────────────────────
    print("\n-- G. Verificacion de la estructura que las Proposiciones 2 y 3 DEMUESTRAN")
    print("   M(lc) = I - (1-lam)*a*a^T   con a el eje de pitch, para todo lc")
    eP = []; ea = []; esym = []; ew2 = []; ok_cond = 0; ok_spec = 0; agree = 0
    for q in qs:
        J = J_(q); a = J[3:, 1]/np.linalg.norm(J[3:, 1])   # eje comun de pitch
        U, r = plano_del_brazo(q)
        cond = (r == 2)
        w = np.linalg.eigvalsh(force_block(q, LC_STD))
        spec = int(np.sum(w > 0.999)) == 2
        ok_cond += cond; ok_spec += spec; agree += (cond == spec)
        if not cond:
            continue
        for lc in LCS:
            M = force_block(q, lc)
            ew2.append(np.abs(M - force_block_w2(q, lc)).max())
            esym.append(np.abs(M - M.T).max())
            eP.append(np.abs(M @ U - U).max())              # M f = f en el plano
            Ma = M @ a
            ea.append(np.linalg.norm(Ma - (a @ Ma)*a))      # M a paralelo a a
    n = len(qs)
    print(f"   dim Jv(ker Jw) = 2      en {100*ok_cond/n:.1f} % de las configuraciones")
    print(f"   espectro {{lam, 1, 1}}     en {100*ok_spec/n:.1f} %")
    print(f"   las dos condiciones coinciden en {100*agree/n:.1f} %"
          "   <- son LA MISMA condicion")
    print(f"   max |M f - f| con f en el plano del brazo : {max(eP):.2e}")
    print(f"   max componente de M a fuera de span(a)    : {max(ea):.2e}")
    print(f"   max |M - M^T|  (simetria, ec. de proyector): {max(esym):.2e}")
    print(f"   max |M_(W-1) - M_(W-2)|  (control cruzado) : {max(ew2):.2e}")
    print("   => la invariancia NO es numerica: es una consecuencia de que los tres")
    print("      ejes de pitch sean paralelos. Para un Jacobiano generico es FALSA.")

    # ── F. el estimador de contacto puntual no tiene metrica ──────────────────
    print("\n-- F. Control: el inverso de contacto puntual no admite este problema")
    e3 = np.array([np.linalg.norm(
        np.linalg.lstsq(J_(q)[:3, :].T, J_(q)[:3, :].T @ u, rcond=None)[0] - u)
        for q, u in zip(qs, us)])
    print(f"   error maximo sobre {len(qs)} configuraciones: {e3.max():.2e}")
    print("   f ∈ R³ va entero en newtons: la norma ya es homogenea y no hay lc que elegir")
