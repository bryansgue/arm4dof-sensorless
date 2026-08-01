"""
Figuras del paper. Genera ../figures/*.png a partir de los experimentos vivos
(nada hardcodeado salvo la tabla de E4-bis, que viene de test_e4bis.py).

Run: python3 make_figures.py
"""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import casadi as ca
from casadi import SX, vertcat

from dq_math import ln_dual, se3_error_decoupled, dq_error, dq_from_pose

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "figures")
os.makedirs(OUT, exist_ok=True)
plt.rcParams.update({"font.size": 9, "axes.grid": True, "grid.alpha": 0.3})


# ── Fig. divergencia de metricas + ablacion E4-bis ────────────────────────────
def fig_metric():
    dqe = SX.sym("dqe", 8)
    f_ln = ca.Function("f_ln", [dqe], [ln_dual(dqe)])
    f_dc = ca.Function("f_dc", [dqe], [se3_error_decoupled(dqe)])
    quat = SX.sym("q", 4); tr = SX.sym("t", 3)
    f_dq = ca.Function("f_dq", [quat, tr], [dq_from_pose(quat, tr)])
    a_, b_ = SX.sym("a", 8), SX.sym("b", 8)
    f_er = ca.Function("f_er", [a_, b_], [dq_error(a_, b_)])

    def dq(axis, ang, t):
        ax = np.array(axis, float); ax /= np.linalg.norm(ax)
        q = np.concatenate([[np.cos(ang/2)], np.sin(ang/2)*ax])
        return np.array(f_dq(q, np.array(t, float))).flatten()

    I = dq([0, 0, 1], 0.0, [0, 0, 0])
    ang = np.linspace(0.5, 150, 200)
    rel = []
    for a in ang:
        A = dq([0.3, 0.5, 0.81], np.radians(a), [0.10, -0.05, 0.08])
        e = np.array(f_er(I, A)).flatten()
        v1 = np.array(f_ln(e)).flatten(); v2 = np.array(f_dc(e)).flatten()
        rel.append(100*np.linalg.norm(v1[3:]-v2[3:])/np.linalg.norm(v2[3:]))
    rel = np.array(rel)

    # de test_e4bis.py
    alpha = np.array([0, 15, 30, 45, 60, 75, 90])
    dpos  = np.array([1.1, 0.9, 0.8, 1.2, 1.6, 2.1, 2.8])

    fig, ax = plt.subplots(1, 2, figsize=(9, 3.3))
    ax[0].plot(ang, rel, "k-", lw=1.6)
    ax[0].axvline(6.68, color="tab:red", ls="--", lw=1.2)
    ax[0].annotate("regimen del\nbanco original\n(6.7 deg, 2.8 %)", xy=(6.68, 2.8),
                   xytext=(30, 12), fontsize=7.5, color="tab:red",
                   arrowprops=dict(arrowstyle="->", color="tab:red", lw=0.8))
    ax[0].set_xlabel(r"error de orientacion $\|\varphi\|$ [deg]")
    ax[0].set_ylabel(r"$\|\rho_{\log}-t_{err}\|/\|t_{err}\|$ [%]")
    ax[0].set_title("(a) divergencia entre las dos metricas", fontsize=9)

    ax[1].plot(alpha, dpos, "o-", color="tab:blue", lw=1.6, ms=5)
    ax[1].axhline(0, color="k", lw=0.8)
    ax[1].set_xlabel(r"desalineacion de la referencia $\alpha$ [deg]")
    ax[1].set_ylabel("ventaja del log dual en posicion [%]")
    ax[1].set_title("(b) ablacion en lazo cerrado", fontsize=9)
    ax[1].set_ylim(0, 3.2)
    fig.tight_layout()
    fp = os.path.join(OUT, "metric_ablation.png"); fig.savefig(fp, dpi=160)
    print("  ->", os.path.normpath(fp))


# ── Fig. trazas del lazo de interaccion ───────────────────────────────────────
def fig_interaction():
    """Barrido DIRECCIONAL, no una traza suelta. El eje ya no es el tiempo sino la
    direccion del empuje, asi se ven los dos lados y todas las direcciones a la vez.

      (a),(b) polares: fraccion de fuerza recuperada segun la direccion, en dos
              planos ortogonales. En el plano que contiene la direccion ciega el
              6-D se estrangula; en el de las dos observables da un circulo.
      (c)     el momento FANTASMA que el 6-D inventa, tambien contra la direccion
      (d)     respuesta compliant en lazo cerrado, para CUATRO direcciones
    """
    import numpy as np
    import test_interaction_mil as T
    from arm_kinematics import build_jac_fn, build_fk_fn
    fJ = build_jac_fn(); fk = build_fk_fn()

    q = np.array([0., 1.2, -0.9, 0.4])
    J = np.array(fJ(q)); Jv = J[:3, :]
    P = J @ np.linalg.lstsq(J.T @ J, J.T, rcond=None)[0]
    Pff, Pmf = P[:3, :3], P[3:, :3]
    pee = np.array(fk(q)[1]).flatten()

    # El plano del brazo contiene el eje de la base (z) y la direccion azimutal
    # del efector. La normal es horizontal y perpendicular a esa azimutal.
    az = np.array([pee[0], pee[1], 0.0]); az /= np.linalg.norm(az)
    nrm = np.cross(np.array([0., 0., 1.]), az)      # normal al plano del brazo
    zax = np.array([0., 0., 1.])

    # angulo FUERA del plano: 0 deg = empuje contenido en el plano del brazo
    ang = np.linspace(0, 90, 181)
    rec6 = np.zeros_like(ang); rec3 = np.zeros_like(ang); mom6 = np.zeros_like(ang)
    for i, a in enumerate(ang):
        r = np.radians(a)
        u = np.cos(r)*az + np.sin(r)*nrm          # gira desde el plano hacia la normal
        u /= np.linalg.norm(u)
        rec6[i] = np.linalg.norm(Pff @ u)
        mom6[i] = np.linalg.norm(Pmf @ u)
        rec3[i] = np.linalg.norm(np.linalg.lstsq(Jv.T, Jv.T @ u, rcond=None)[0])

    # Solo el lazo cerrado: es lo unico que esta figura aporta y no esta en las
    # tablas. El barrido direccional vive en la Tabla de tres direcciones y en la
    # figura del barrido; repetirlo aca era redundante, y ademas mezclaba unidades
    # (fuerza en % contra momento en N.m/N sobre el mismo eje).
    fig, ax = plt.subplots(1, 2, figsize=(8.4, 3.0), sharex=True)
    COL = ("tab:blue", "tab:green", "tab:orange", "tab:red")
    for a_deg, c in zip([0, 30, 60, 90], COL):
        r = np.radians(a_deg)
        d = 6.0*(np.cos(r)*az + np.sin(r)*nrm)
        L = T.run(list(d), estimator="force")
        t = np.arange(T.T)*T.DT
        lab = f"${a_deg}^\\circ$"
        ax[0].plot(t, 1000*np.linalg.norm(L["f3"]-L["ftrue"], axis=1),
                   lw=1.4, color=c, label=lab)
        ax[1].plot(t, 1000*L["err"], lw=1.6, color=c, label=lab)
    for a in ax:
        a.axvspan(T.T0*T.DT, T.T1*T.DT, color="0.88", zorder=0)
        a.set_xlabel("t [s]", fontsize=8.5); a.tick_params(labelsize=7.5)
        a.grid(alpha=.3)
    ax[0].set_ylabel(r"estimation error $\|\hat f-f\|$ [mN]", fontsize=8.5)
    ax[0].set_title("(a) sensorless estimate, applied force $6$ N", fontsize=8.5)
    ax[0].axhline(0, color="k", lw=.8)
    ax[0].legend(fontsize=7, ncol=2, title="push angle out of\nthe arm plane",
                 title_fontsize=6.3, loc="upper left")
    ax[1].set_ylabel("displacement from task [mm]", fontsize=8.5)
    ax[1].set_title("(b) compliant response", fontsize=8.5)
    ax[1].text(.5*(T.T0+T.T1)*T.DT, 3, "push", ha="center", fontsize=7.5,
               color="0.35")

    fig.tight_layout()
    fp = os.path.join(OUT, "interaction_traces.png")
    fig.savefig(fp, dpi=165, bbox_inches="tight")
    print("  ->", os.path.normpath(fp))


if __name__ == "__main__":
    print("figuras:")
    fig_metric()
    fig_interaction()
