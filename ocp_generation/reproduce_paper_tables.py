"""
Regenera las tablas del paper que salian de scripts INLINE nunca guardados.

⚠️ Por que existe. La seccion "Reproducibility" del paper afirma que cada figura y
tabla la produce un script con nombre. Era FALSO para cinco tablas: se habian
calculado con codigo ad-hoc en la terminal que no quedo en el repositorio. Esto lo
arregla; el resto de las tablas ya tenian su script y estan mapeadas en
paper/SUBMISSION.md.

Run:  python3 reproduce_paper_tables.py
"""
import numpy as np
import casadi as ca
from casadi import SX, vertcat

import arm_params as P
from arm_dynamics import build_dynamics
from arm_kinematics import build_fk_fn, build_jac_fn
from dq_math import ln_dual, se3_error_decoupled, dq_error, dq_from_pose

fM, fh = build_dynamics(); fk = build_fk_fn(); fJ = build_jac_fn()
M_ = lambda q: np.array(fM(q))
h_ = lambda q, qd: np.array(fh(q, qd)).flatten()
J_ = lambda q: np.array(fJ(q))
Jv_ = lambda q: J_(q)[:3, :]

Q_LO = np.array([-3.1416, -0.436332, -2.00713, -2.00713])
Q_HI = np.array([3.1416, 3.57792, 2.00713, 2.00713])
Q_TASK = np.array([0.0, 1.2, -0.9, 0.4])
DT, TAU_MAX = 0.01, 1.4
G = 9.81


def rule(t):
    print("\n" + "=" * 74); print(t); print("=" * 74)


# ── Tabla: donde vive la señal segun la rigidez del lazo ──────────────────────
def tab_velocity_split():
    rule("TABLA: reparto del residuo segun kv (empuje de 6 N)")
    u = np.array([0., 1., 0.])
    F6 = np.concatenate([6.0*u, np.zeros(3)])
    print(f"{'kv':>6} {'|M qdd|':>10} {'|h-tau|':>10} {'% en tau':>10} {'despl [mm]':>12}")
    for kv in [1., 4., 20., 100.]:
        x = np.concatenate([Q_TASK, np.zeros(4)]); qd_cmd = np.zeros(4)
        nsub = 200
        for _ in range(120):
            qb = x[4:].copy(); acc = np.zeros(4); ds = DT/nsub
            for _ in range(nsub):
                q, qd = x[:4], x[4:]
                tau = kv*(qd_cmd - qd); acc += tau
                a = np.linalg.solve(M_(q), tau + J_(q).T @ F6 - h_(q, qd))
                x = np.concatenate([q + ds*qd, qd + ds*a])
            qdd = (x[4:] - qb)/DT; ta = acc/nsub
        q, qd = x[:4], x[4:]
        A = np.linalg.norm(M_(q) @ qdd); B = np.linalg.norm(h_(q, qd) - ta)
        d = 1000*np.linalg.norm(np.array(fk(q)[1]).flatten()
                                - np.array(fk(Q_TASK)[1]).flatten())
        print(f"{kv:6.0f} {A:10.5f} {B:10.4f} {100*B/(A+B):9.1f}% {d:12.2f}")


# ── Tabla: capacidad de fuerza del actuador ──────────────────────────────────
def tab_force_capability(n=9, ndir=60):
    rule("TABLA: fuerza maxima ejercible (tau_max = 1.4 N.m)")

    def fmax(q, u):
        t = Jv_(q).T @ u; g = h_(q, np.zeros(4)); s = np.inf
        for j in range(4):
            if abs(t[j]) < 1e-12:
                continue
            for lim in (TAU_MAX, -TAU_MAX):
                v = (lim - g[j])/t[j]
                if v > 0:
                    s = min(s, v)
        return s

    rng = np.random.default_rng(0); W = []
    g = [np.linspace(Q_LO[i], Q_HI[i], n) for i in (1, 2, 3)]
    for a in g[0]:
        for b in g[1]:
            for c in g[2]:
                q = np.array([0., a, b, c])
                W.append(min(fmax(q, u/np.linalg.norm(u))
                             for u in rng.normal(size=(ndir, 3))))
    W = np.array(W); W = W[np.isfinite(W)]
    print(f"  {len(W)} configuraciones, PEOR direccion en cada una:")
    print(f"  mediana {np.median(W):.2f} N   p5 {np.percentile(W,5):.2f} N   "
          f"min {W.min():.2f} N")


# ── Tabla: tres direcciones con nombre fisico ────────────────────────────────
def tab_named_directions():
    rule("TABLA: fuerza recuperada por el 6-D en tres direcciones con nombre")
    POSES = {"P1": Q_TASK, "P2": np.array([0.6, 1.4, -0.7, 0.9]),
             "P3": np.array([-0.5, 0.7, -1.4, -0.3]),
             "P4": np.array([0.3, 2.2, -1.6, 0.8])}
    print(f"{'pose':>5} | {'along arm':>10} {'vertical':>9} {'out of plane':>13} "
          f"| {'momento':>8}")
    for nm, q in POSES.items():
        J = J_(q); Pm = J @ np.linalg.lstsq(J.T @ J, J.T, rcond=None)[0]
        Pff, Pmf = Pm[:3, :3], Pm[3:, :3]
        pee = np.array(fk(q)[1]).flatten()
        az = np.array([pee[0], pee[1], 0.]); az /= np.linalg.norm(az)
        nr = np.cross(np.array([0., 0., 1.]), az); zx = np.array([0., 0., 1.])
        r = [100*np.linalg.norm(Pff @ v) for v in (az, zx, nr)]
        print(f"{nm:>5} | {r[0]:9.1f}% {r[1]:8.1f}% {r[2]:12.1f}% "
              f"| {np.linalg.norm(Pmf@nr):8.2f}")
    print("  (el inverso de contacto puntual devuelve 100.0 % en las tres)")


# ── Tabla: inyeccion de fallas ───────────────────────────────────────────────
def tab_fault_injection():
    rule("TABLA: fuerza espuria bajo perturbacion de un parametro")
    import sys, os
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    "..", "hw"))
    from s1_gravity_check import POSES as PS
    def peak(pert):
        return max(np.linalg.norm(np.linalg.lstsq(Jv_(q).T, pert(q), rcond=None)[0])
                   for q in PS)
    hq = lambda q: h_(q, np.zeros(4))
    tests = [("signo de par invertido", lambda q: -2.00*hq(q)),
             ("constante de par x2", lambda q: -1.00*hq(q)),
             ("offset de par 0.05 N.m", lambda q: np.array([0, .05, 0, 0.])),
             ("stiction 0.05 N.m", lambda q: 0.05*np.ones(4)),
             ("constante de par +20 %", lambda q: -0.20*hq(q)),
             ("offset de par 0.02 N.m", lambda q: np.array([0, .02, 0, 0.])),
             ("constante de par +5 %", lambda q: -0.05*hq(q))]
    for nm, p in tests:
        print(f"  {nm:<32} {peak(p):8.2f} N")
    for li in range(4):
        m0 = P.LINKS[li]['mass']; P.LINKS[li]['mass'] = 1.20*m0
        _, fh2 = build_dynamics(); h2 = lambda q: np.array(fh2(q, np.zeros(4))).flatten()
        print(f"  {'masa eslabon '+str(li+1)+' +20 %':<32} "
              f"{peak(lambda q: h2(q)-hq(q)):8.2f} N")
        P.LINKS[li]['mass'] = m0
    for off in (0.05, 0.10):
        print(f"  {'cero de junta corrido '+f'{off:.2f}'+' rad':<32} "
              f"{peak(lambda q, o=off: hq(q+np.array([0,o,0,0.]))-hq(q)):8.2f} N")


# ── Divergencia entre las dos metricas de pose ───────────────────────────────
def tab_metric_divergence():
    rule("TABLA: divergencia entre el error acoplado y el desacoplado")
    d = SX.sym("d", 8)
    f_ln = ca.Function("a", [d], [ln_dual(d)])
    f_dc = ca.Function("b", [d], [se3_error_decoupled(d)])
    qs = SX.sym("q", 4); ts = SX.sym("t", 3)
    f_dq = ca.Function("c", [qs, ts], [dq_from_pose(qs, ts)])
    x, y = SX.sym("x", 8), SX.sym("y", 8)
    f_er = ca.Function("e", [x, y], [dq_error(x, y)])

    def dq(axis, ang, t):
        ax = np.array(axis, float); ax /= np.linalg.norm(ax)
        return np.array(f_dq(np.concatenate([[np.cos(ang/2)], np.sin(ang/2)*ax]),
                             np.array(t, float))).flatten()

    I = dq([0, 0, 1], 0.0, [0, 0, 0])
    print(f"  {'|phi| [deg]':>12} {'divergencia':>13}")
    for a in (1, 5, 10, 45, 90):
        A = dq([0.3, 0.5, 0.81], np.radians(a), [0.10, -0.05, 0.08])
        e = np.array(f_er(I, A)).flatten()
        v1 = np.array(f_ln(e)).flatten(); v2 = np.array(f_dc(e)).flatten()
        rel = 100*np.linalg.norm(v1[3:]-v2[3:])/np.linalg.norm(v2[3:])
        print(f"  {a:12d} {rel:12.2f} %")
    print("  (crece ~0.38 % por grado)")


if __name__ == "__main__":
    tab_velocity_split()
    tab_force_capability()
    tab_named_directions()
    tab_metric_divergence()
    tab_fault_injection()
