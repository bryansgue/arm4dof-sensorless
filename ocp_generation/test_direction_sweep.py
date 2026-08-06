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

    # ── figura ────────────────────────────────────────────────────────────────
    # ⚠️ La version anterior de esta figura no comunicaba nada. El panel (a) era un
    # scatter de 12000 puntos que se veia como UNA banda roja —seis rectas casi
    # identicas superpuestas— y desde que la identidad esta DEMOSTRADA, confirmar
    # una recta con puntos no agrega. El panel (b) eran seis histogramas escalonados
    # con 40-70 cuentas por bin: ruido visual. Ahora:
    #   (a) la relacion por pose, una recta por pose, con su pendiente a la vista
    #   (b) como esa pendiente 1-lam depende de l_c, que es el resultado nuevo
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        plt.rcParams.update({"font.size": 9, "axes.grid": True, "grid.alpha": .3})

        def Mw(q, lc):
            """Bloque de fuerza del inverso 6-D ponderado. lc=1 es el sin ponderar."""
            J = J_(q)
            W2i = np.diag([1., 1., 1., lc*lc, lc*lc, lc*lc])
            return (W2i @ J @ np.linalg.pinv(J.T @ W2i @ J) @ J[:3, :].T)[:3, :3]

        # ⚠️ El panel de "error contra alineacion" se ELIMINO. Eran seis rectas de
        # la misma pendiente superpuestas, o sea la ec. del error dibujada: no hay
        # nada que leer ahi que la ecuacion no diga mejor. Queda UN panel, con lo
        # unico que la ecuacion NO dice: como su pendiente depende de la metrica.
        cmap = plt.get_cmap("tab10")
        fig, ax = plt.subplots(figsize=(4.6, 3.4))

        LCS = np.logspace(np.log10(0.03), np.log10(3.0), 60)
        for k, (name, q) in enumerate(POSES.items()):
            slope = [100*(1 - np.linalg.eigvalsh(Mw(q, lc))[0]) for lc in LCS]
            ax.plot(LCS, slope, lw=1.7, color=cmap(k), label=name)
        for x, lab in [(0.233, "arm scale"), (1.0, "unweighted SI")]:
            ax.axvline(x, color="0.4", ls="--", lw=.9)
            ax.annotate(lab, xy=(x, 4), rotation=90, fontsize=7.5,
                        color="0.3", ha="right", va="bottom")
        ax.set_xscale("log")
        ax.set_xlabel(r"characteristic length  $\ell_c$  [m]")
        ax.set_ylabel(r"error per unit alignment,  $100\,(1-\lambda_{\min})$  [%]")
        ax.legend(fontsize=7.5, ncol=2, loc="upper left")
        ax.set_ylim(0, 104); ax.set_xlim(LCS[0], LCS[-1])
        ax.set_title("The six poses become indistinguishable\n"
                     "only under the unweighted-SI metric", fontsize=9)

        fig.tight_layout()
        out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "figures")
        os.makedirs(out, exist_ok=True)
        fp = os.path.join(out, "direction_sweep.png")
        fig.savefig(fp, dpi=160)
        print(f"\n  figura -> {os.path.normpath(fp)}")
    except Exception as ex:
        print(f"\n  [figura omitida: {ex}]")
