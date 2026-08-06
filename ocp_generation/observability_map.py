"""
OBSERVABILIDAD DE LA FUERZA DE CONTACTO — el analisis que sostiene el paper.

Pregunta: en un brazo con n<6 juntas y sin sensor F/T, ¿que interacciones puede
percibir a traves de sus propios actuadores?

La respuesta depende de QUE MODELO DE CONTACTO se asuma, y esa es la tesis:

  (A) WRENCH-6D, lo que hace el observador estandar.
      Resuelve  Jᵀ F = tau  con F ∈ R⁶ y J ∈ R⁶ˣ⁴.  SEIS incognitas, CUATRO medidas.
      Indeterminado. La solucion de norma minima F = P F, con P el proyector
      ortogonal sobre col(J), reparte el torque entre fuerza y momento. Una fuerza
      pura se mal-atribuye PARCIALMENTE A UN MOMENTO.

  (B) FUERZA-3D, contacto puntual en un punto CONOCIDO (el efector).
      Resuelve  Jvᵀ f = tau  con f ∈ R³ y Jvᵀ ∈ R⁴ˣ³.  TRES incognitas, CUATRO
      medidas. SOBRE-determinado. Si rank(Jv)=3, f se recupera EXACTA.

⚠️ La conclusion importante, y contraintuitiva: la fuerza NO es inobservable en este
brazo. El torque de junta ESTA ahi. Lo que falla es el modelo de estimacion.

Run:  python3 observability_map.py   ->  tabla + ../figures/observability_map.png
"""
import os
import numpy as np
from arm_kinematics import build_jac_fn, build_fk_fn

_fJ = build_jac_fn(); _fk = build_fk_fn()

Q_LO = np.array([-3.1416, -0.436332, -2.00713, -2.00713])
Q_HI = np.array([ 3.1416,  3.57792,  2.00713,  2.00713])


# ── (A) el estimador 6D ───────────────────────────────────────────────────────
def projector(q, tol=1e-9):
    """P = proyector ortogonal sobre col(J): lo que el min-norm 6D puede devolver.
    Por SVD y no por J(JᵀJ)⁻¹Jᵀ, que explota en configuraciones singulares."""
    J = np.array(_fJ(q))
    U, s, _ = np.linalg.svd(J, full_matrices=False)
    r = int(np.sum(s > tol*max(1.0, s[0])))
    return U[:, :r] @ U[:, :r].T


def force_block(q):
    """P_ff: fraccion de una FUERZA PURA que el estimador 6D devuelve como fuerza.
    El resto no se pierde: se va al bloque de MOMENTO, mal atribuido."""
    return projector(q)[:3, :3]


# ── (B) el estimador de fuerza pura ───────────────────────────────────────────
def force_jacobian(q):
    return np.array(_fJ(q))[:3, :]            # Jv, 3x4


def force_svd(q):
    return np.linalg.svd(force_jacobian(q), compute_uv=False)


def estimate_from_tau(q, tau):
    """f = argmin ‖Jvᵀ f − tau‖  (sobre-determinado)."""
    return np.linalg.lstsq(force_jacobian(q).T, tau, rcond=None)[0]


if __name__ == "__main__":
    np.set_printoptions(precision=4, suppress=True)
    print("== OBSERVABILIDAD DE LA FUERZA DE CONTACTO — brazo 4DOF ==\n")

    POSES = {
        "q_task  (MiL interaccion)": np.array([0.0, 1.2, -0.9, 0.4]),
        "Q_TGT   (E4-bis objetivo)": np.array([0.6, 1.4, -0.7, 0.9]),
        "X0      (E4-bis arranque)": np.array([-0.5, 0.7, -1.4, -0.3]),
        "target  (test_arm_dqnmpc)": np.array([0.6, 1.4, -0.9, 0.3]),
    }

    print("-- El caso que lo muestra todo: empuje normal al plano del brazo --")
    q = POSES["q_task  (MiL interaccion)"]
    f = np.array([0., 6., 0.])
    J = np.array(_fJ(q)); tau = J[:3, :].T @ f
    P = projector(q); rec = P @ np.concatenate([f, np.zeros(3)])
    print(f"  fuerza aplicada        f = {f}")
    print(f"  torque de junta que produce  tau = Jvᵀf = {np.round(tau,4)}")
    print(f"     |tau| = {np.linalg.norm(tau):.4f} N.m   -> el empuje NO es invisible")
    print(f"  6D min-norm devuelve:  fuerza {np.round(rec[:3],4)}  momento {np.round(rec[3:],4)}")
    print(f"     reparte {np.linalg.norm(rec[:3]):.3f} N de fuerza y "
          f"{np.linalg.norm(rec[3:]):.3f} N.m de momento: MAL-ATRIBUYE")
    print(f"  fuerza-3D devuelve:    {np.round(estimate_from_tau(q, tau),4)}   EXACTO\n")

    print("-- Espectro de P_ff: cuanto de una fuerza pura devuelve el 6D como fuerza --")
    print(f"  {'configuracion':<34} {'lam_min':>8} {'lam_med':>8} {'lam_max':>8}")
    for name, qq in POSES.items():
        w = np.linalg.eigvalsh(force_block(qq))
        print(f"  {name:<34} {w[0]:8.4f} {w[1]:8.4f} {w[2]:8.4f}")

    print("\n-- Lo que de verdad limita al estimador correcto: rank y condicionamiento de Jv --")
    print(f"  {'configuracion':<34} {'rank(Jv)':>9} {'sig_min':>9} {'cond':>9}")
    for name, qq in POSES.items():
        s = force_svd(qq); r = int(np.sum(s > 1e-9*s[0]))
        print(f"  {name:<34} {r:9d} {s[2]:9.4f} {s[0]/max(s[2],1e-16):9.1f}")

    # ── barrido ───────────────────────────────────────────────────────────────
    print("\n-- Barrido del espacio de trabajo (q2,q3,q4; q1=0 sin perdida de generalidad) --")
    n = 13
    grids = [np.linspace(Q_LO[i], Q_HI[i], n) for i in (1, 2, 3)]
    rng = np.random.default_rng(0)
    rk = []; smin = []; cond = []; e6 = []; e3 = []; lmin = []
    for a in grids[0]:
        for b in grids[1]:
            for c in grids[2]:
                qq = np.array([0., a, b, c])
                s = force_svd(qq)
                rk.append(int(np.sum(s > 1e-9*s[0]))); smin.append(s[2])
                cond.append(s[0]/max(s[2], 1e-16))
                lmin.append(np.linalg.eigvalsh(force_block(qq))[0])
                u = rng.normal(size=3); u /= np.linalg.norm(u)      # fuerza unitaria
                t = np.array(_fJ(qq))[:3, :].T @ u
                e6.append(np.linalg.norm((projector(qq) @ np.concatenate([u, np.zeros(3)]))[:3] - u))
                e3.append(np.linalg.norm(estimate_from_tau(qq, t) - u))
    rk = np.array(rk); smin = np.array(smin); cond = np.array(cond)
    e6 = np.array(e6); e3 = np.array(e3); lmin = np.array(lmin)

    print(f"  {len(rk)} configuraciones")
    print(f"  rank(Jv)=3 en {100*np.mean(rk==3):.1f} %  -> la fuerza de contacto es"
          f" observable en {'TODO' if np.all(rk==3) else 'parte'} el espacio de trabajo")
    print(f"  sigma_min(Jv): media {smin.mean():.4f}  mediana {np.median(smin):.4f}"
          f"  p5 {np.percentile(smin,5):.4f}  min {smin.min():.2e}")
    print(f"  cond(Jv)     : mediana {np.median(cond):.1f}  p95 {np.percentile(cond,95):.1f}"
          f"  max {cond.max():.1e}")
    print(f"\n  error estimando una fuerza UNITARIA:")
    print(f"    WRENCH-6D : media {e6.mean():.4f}  mediana {np.median(e6):.4f}  max {e6.max():.4f}")
    print(f"    FUERZA-3D : media {e3.mean():.2e}  mediana {np.median(e3):.2e}  max {e3.max():.2e}")
    print(f"  => el 6D pierde {100*e6.mean():.0f} % de la fuerza en promedio (hasta "
          f"{100*e6.max():.0f} %); el 3D es exacto")
    print(f"  (P_ff del 6D: lam_min media {lmin.mean():.4f}, y en el {100*np.mean(lmin<0.10):.0f} %"
          f" de las configuraciones hay una direccion con lam_min<0.10)")

    # ── figura ────────────────────────────────────────────────────────────────
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        nn = 61
        a2 = np.linspace(Q_LO[1], Q_HI[1], nn); a3 = np.linspace(Q_LO[2], Q_HI[2], nn)
        Z6 = np.zeros((nn, nn)); ZS = np.zeros((nn, nn))
        for i, A in enumerate(a2):
            for j, B in enumerate(a3):
                qq = np.array([0., A, B, 0.4])
                Z6[i, j] = np.linalg.eigvalsh(force_block(qq))[0]
                ZS[i, j] = force_svd(qq)[2]
        # ⚠️ NO fijar vmax=1 en el panel (a): lam_min no pasa de 0.08 en toda la
        # rebanada, asi que con la escala 0-1 el mapa entero cae en el 8 % inferior
        # del colormap y sale NEGRO, sin estructura visible. Se escala al dato y se
        # declara el rango en la etiqueta, que es lo que transmite "casi cero".
        fig, ax = plt.subplots(1, 2, figsize=(10.5, 3.9))
        im0 = ax[0].pcolormesh(np.degrees(a3), np.degrees(a2), Z6, cmap="plasma",
                               vmin=0, vmax=Z6.max(), shading="auto")
        ax[0].set_title(r"(a) 6-D min-norm: worst-direction $\lambda_{\min}(P_{ff})$", fontsize=9)
        cb0 = fig.colorbar(im0, ax=ax[0])
        cb0.set_label(f"force fraction returned (max {Z6.max():.3f})", fontsize=8)
        im1 = ax[1].pcolormesh(np.degrees(a3), np.degrees(a2), ZS, cmap="viridis",
                               shading="auto")
        ax[1].set_title(r"(b) point-contact model: $\sigma_{\min}(J_v)$", fontsize=9)
        cb1 = fig.colorbar(im1, ax=ax[1])
        cb1.set_label("noise gain limit", fontsize=8)
        for k in (0, 1):
            ax[k].set_xlabel(r"$q_3$ [deg]"); ax[k].set_ylabel(r"$q_2$ [deg]")
            ax[k].plot(np.degrees(-0.9), np.degrees(1.2), "w*", ms=12, mec="k")
        fig.suptitle("Force estimation over the workspace: the 6-D inverse loses force "
                     "where the point-contact model does not", fontsize=10)
        print(f"  panel (a): lam_min en [{Z6.min():.4f}, {Z6.max():.4f}]"
              f"   panel (b): sig_min en [{ZS.min():.4f}, {ZS.max():.4f}]")
        fig.tight_layout()
        out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "figures")
        os.makedirs(out, exist_ok=True)
        fp = os.path.join(out, "observability_map.png")
        fig.savefig(fp, dpi=160)
        print(f"\n  figura -> {os.path.normpath(fp)}")
    except Exception as ex:
        print(f"\n  [figura omitida: {ex}]")
