"""
Figura de arquitectura: que se mide, que se estima, que se comanda.

El punto que la figura tiene que dejar claro de un vistazo: el servo devuelve TRES
señales, y la tercera (esfuerzo/corriente) es la que hace las veces de sensor de
fuerza. No hay sensor F/T en ningun lado del lazo.

Run: python3 make_arch_figure.py  -> ../figures/architecture.png
"""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "figures")
os.makedirs(OUT, exist_ok=True)

BLUE = "#2a5d8f"; ORANGE = "#c8641e"; GRAY = "#555555"; GREEN = "#2e7d4f"


def box(ax, x, y, w, h, text, fc="white", ec=GRAY, fs=8.5, lw=1.3, tc="black"):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.012",
                                fc=fc, ec=ec, lw=lw, zorder=2))
    ax.text(x+w/2, y+h/2, text, ha="center", va="center", fontsize=fs,
            zorder=3, color=tc, linespacing=1.35)


def arrow(ax, p0, p1, text=None, color=GRAY, fs=7.5, off=(0, 0.028), style="-|>",
          rad=0.0, lw=1.3):
    ax.add_patch(FancyArrowPatch(p0, p1, arrowstyle=style, mutation_scale=11,
                                 lw=lw, color=color, zorder=1,
                                 connectionstyle=f"arc3,rad={rad}"))
    if text:
        ax.text((p0[0]+p1[0])/2+off[0], (p0[1]+p1[1])/2+off[1], text,
                ha="center", va="bottom", fontsize=fs, color=color, zorder=3)


fig, ax = plt.subplots(figsize=(9.0, 3.5))
ax.set_xlim(0, 1); ax.set_ylim(0, 1); ax.axis("off")

# ── planta ───────────────────────────────────────────────────────────────────
box(ax, 0.40, 0.06, 0.20, 0.17,
    "MX-28R servos\n(velocity mode)", fc="#f2f2f2", fs=8.5)
box(ax, 0.70, 0.06, 0.20, 0.17, "4-DoF arm\n" + r"$M\ddot q+k_v\dot q+h=k_vu+J^{\!\top}F$",
    fc="#f2f2f2", fs=7.6)
arrow(ax, (0.60, 0.145), (0.70, 0.145), r"$\tau$", off=(0, 0.012))

# empuje humano
ax.annotate("", xy=(0.80, 0.23), xytext=(0.80, 0.36),
            arrowprops=dict(arrowstyle="-|>", lw=1.6, color=ORANGE))
ax.text(0.815, 0.30, r"human push $f$", fontsize=8.5, color=ORANGE, va="center")

# ── las tres señales ─────────────────────────────────────────────────────────
box(ax, 0.055, 0.06, 0.28, 0.17,
    "servo feedback (no F/T sensor)\n"
    r"$q$    $\dot q$    $\tau_{\rm act}$ (current)",
    fc="#eef4fa", ec=BLUE, fs=8.2)
arrow(ax, (0.70, 0.115), (0.335, 0.115), color=BLUE, rad=-0.18, lw=1.5)

# ── estimador ────────────────────────────────────────────────────────────────
box(ax, 0.055, 0.44, 0.28, 0.20,
    "residual + point contact\n"
    r"$\tau_{\rm ext}=M\ddot q+h-\tau_{\rm act}$" "\n"
    r"$\hat f=(J_vJ_v^{\!\top})^{-1}J_v\tau_{\rm ext}$",
    fc="#eef4fa", ec=BLUE, fs=7.8, lw=1.8)
arrow(ax, (0.195, 0.23), (0.195, 0.44), color=BLUE, lw=1.5)

# ── admitancia ───────────────────────────────────────────────────────────────
box(ax, 0.395, 0.44, 0.20, 0.20,
    "admittance\n" r"$D\nu=\hat f-k\,e_{\rm task}$" "\n"
    r"$\Rightarrow\ q_{\rm ref}$", fc="#f4fbf6", ec=GREEN, fs=8.0)
arrow(ax, (0.335, 0.54), (0.395, 0.54), r"$\hat f$", color=BLUE, off=(0, 0.012))

# ── NMPC ─────────────────────────────────────────────────────────────────────
box(ax, 0.655, 0.44, 0.245, 0.20,
    "DQ-NMPC\n" r"$\min\sum\|[\,e;\dot q;\tau]\|^2_W$" "\n"
    r"$e=\log(q_d^*\otimes q_{ee})$",
    fc="#fdf3ec", ec=ORANGE, fs=7.8)
arrow(ax, (0.595, 0.54), (0.655, 0.54), r"$q_{\rm ref}$", color=GREEN,
      off=(0, 0.012))

# comando de velocidad al servo
arrow(ax, (0.777, 0.44), (0.50, 0.23), r"$u=\dot q^{\star}_1$", color=ORANGE,
      rad=0.16, off=(0.055, -0.005), lw=1.5)

# etiquetas de regimen
ax.text(0.50, 0.90,
        "the third servo signal replaces the force/torque sensor",
        ha="center", fontsize=9.5, color=BLUE, weight="bold")
ax.text(0.50, 0.80,
        r"below $\approx\!4$–$6$ N the servo resists and compliance is synthetic;"
        "\n"
        r"above it the actuator saturates and compliance is physical",
        ha="center", fontsize=8.2, color=GRAY, linespacing=1.4)

fig.tight_layout()
fp = os.path.join(OUT, "architecture.png")
fig.savefig(fp, dpi=170, bbox_inches="tight")
print("->", os.path.normpath(fp))
