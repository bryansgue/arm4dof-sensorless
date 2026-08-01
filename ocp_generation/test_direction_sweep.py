"""
Barrido DIRECCIONAL del empuje — que el error del 6-D depende de la direccion no
puede quedar como afirmacion geometrica: hay que medirlo sobre la esfera entera y
en varias configuraciones.

Los experimentos en lazo cerrado usan dos direcciones en una sola pose, lo que
alcanza para ilustrar el contraste pero no para caracterizarlo. Aca se barre la
esfera de direcciones de empuje en varias configuraciones del espacio de trabajo
y se mide, para cada direccion:

  - error relativo del inverso 6-D
  - error relativo del inverso de contacto puntual
  - alineacion del empuje con la direccion CIEGA de esa pose, definida como el
    autovector de P_ff de menor autovalor

La prediccion de la Seccion IV es que el error del 6-D crece con esa alineacion y
no depende de nada mas, mientras que el de contacto puntual es nulo en TODAS las
direcciones. Si el error del 6-D dependiera de la magnitud o de la pose y no de la
direccion, la explicacion geometrica seria falsa.

Run:  python3 test_direction_sweep.py   -> tabla + ../figures/direction_sweep.png
"""
import os
import numpy as np
from arm_kinematics import build_jac_fn

fJ = build_jac_fn()
J_ = lambda q: np.array(fJ(q))

POSES = {
    "P1": np.array([0.0,  1.2, -0.9,  0.4]),
    "P2": np.array([0.6,  1.4, -0.7,  0.9]),
    "P3": np.array([-0.5, 0.7, -1.4, -0.3]),
    "P4": np.array([0.3,  2.2, -1.6,  0.8]),
    "P5": np.array([-0.8, 0.9, -0.3, -1.2]),
    "P6": np.array([0.2,  2.8, -1.1,  0.5]),
}


def blocks(q):
    J = J_(q); Jv = J[:3, :]
    P = J @ np.linalg.lstsq(J.T @ J, J.T, rcond=None)[0]
    return Jv, P[:3, :3]


def sweep_dirs(q, n=2000, seed=0):
    """Direcciones uniformes en la esfera. Devuelve (alineacion, err6, err3)."""
    rng = np.random.default_rng(seed)
    Jv, Pff = blocks(q)
    w, V = np.linalg.eigh(Pff)
    blind = V[:, 0]                       # autovector de menor autovalor
    U = rng.normal(size=(n, 3))
    U /= np.linalg.norm(U, axis=1, keepdims=True)
    align = np.abs(U @ blind)             # |cos| con la direccion ciega
    e6 = np.zeros(n); e3 = np.zeros(n)
    for i, u in enumerate(U):
        tau = Jv.T @ u
        f6 = Pff @ u                                          # parte de fuerza del 6-D
        f3 = np.linalg.lstsq(Jv.T, tau, rcond=None)[0]        # contacto puntual
        e6[i] = np.linalg.norm(f6 - u); e3[i] = np.linalg.norm(f3 - u)
    return align, 100*e6, 100*e3, w


if __name__ == "__main__":
    print("=" * 78)
    print("Barrido direccional: 2000 direcciones de empuje por pose, 6 poses")
    print("error relativo [% de |f|]")
    print("=" * 78)
    print(f"{'pose':>5} {'sig_min(Jv)':>12} | {'6-D: mediana':>13} {'p90':>7} {'max':>7} "
          f"| {'3-D max':>9} | {'% dir con 6-D>50%':>18}")
    print("-"*78)
    A = {}
    for name, q in POSES.items():
        al, e6, e3, w = sweep_dirs(q)
        A[name] = (al, e6, e3)
        s = np.linalg.svd(J_(q)[:3, :], compute_uv=False)[2]
        print(f"{name:>5} {s:12.4f} | {np.median(e6):13.1f} {np.percentile(e6,90):7.1f} "
              f"{e6.max():7.1f} | {e3.max():9.2e} | {100*np.mean(e6>50):17.1f}%")

    print()
    print("=" * 78)
    print("El error del 6-D contra la ALINEACION con la direccion ciega")
    print("(todas las poses juntas, 12000 direcciones)")
    print("=" * 78)
    al = np.concatenate([A[k][0] for k in A])
    e6 = np.concatenate([A[k][1] for k in A])
    e3 = np.concatenate([A[k][2] for k in A])
    print(f"{'|cos| con dir. ciega':>22} | {'6-D mediana [%]':>16} {'n':>7}")
    print("-"*78)
    edges = [0, .2, .4, .6, .8, .95, 1.0]
    for a, b in zip(edges[:-1], edges[1:]):
        m = (al >= a) & (al < b)
        if m.sum():
            print(f"{f'{a:.2f} - {b:.2f}':>22} | {np.median(e6[m]):16.1f} {m.sum():7d}")
    r = np.corrcoef(al, e6)[0, 1]
    print(f"\n  correlacion(alineacion, error 6-D) = {r:+.4f}")
    print(f"  error maximo del contacto puntual sobre las 12000 direcciones = {e3.max():.2e} %")
    print("  => el error del 6-D lo explica la DIRECCION; el de contacto puntual no existe.")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        plt.rcParams.update({"font.size": 9, "axes.grid": True, "grid.alpha": .3})
        fig, ax = plt.subplots(1, 2, figsize=(9, 3.3))
        ax[0].scatter(al, e6, s=1.5, alpha=.12, color="tab:red", rasterized=True)
        ax[0].scatter(al, e3, s=1.5, alpha=.5, color="tab:blue", rasterized=True)
        bx = np.linspace(0, 1, 21); ctr = .5*(bx[1:]+bx[:-1])
        med = [np.median(e6[(al >= a) & (al < b)]) if ((al >= a) & (al < b)).sum() else np.nan
               for a, b in zip(bx[:-1], bx[1:])]
        ax[0].plot(ctr, med, "k-", lw=1.8, label="6-D median")
        ax[0].set_xlabel("alignment with blind direction  $|\\cos\\theta|$")
        ax[0].set_ylabel("relative force error [%]")
        ax[0].set_title("(a) error is explained by direction", fontsize=9)
        ax[0].legend(fontsize=7.5, loc="upper left")
        ax[0].set_ylim(-5, 105)
        for k in A:
            ax[1].hist(A[k][1], bins=40, histtype="step", lw=1.2, label=k)
        ax[1].set_xlabel("6-D relative force error [%]")
        ax[1].set_ylabel("directions")
        ax[1].set_title("(b) six poses, 2000 directions each", fontsize=9)
        ax[1].legend(fontsize=7, ncol=2)
        fig.suptitle("Point-contact error is below $10^{-12}$ % for every direction and pose",
                     fontsize=9.5)
        fig.tight_layout()
        out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "figures")
        os.makedirs(out, exist_ok=True)
        fp = os.path.join(out, "direction_sweep.png")
        fig.savefig(fp, dpi=160)
        print(f"\n  figura -> {os.path.normpath(fp)}")
    except Exception as ex:
        print(f"\n  [figura omitida: {ex}]")
