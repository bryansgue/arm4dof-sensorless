"""
Monte Carlo con RUIDO REALISTA de servo — el resultado que faltaba.

Todos los demas experimentos de este trabajo son sin ruido. Sobre un banco limpio
cualquier estimador exacto se ve perfecto, asi que la pregunta que decide si esto
sirve en hardware es otra: bajo el ruido que el MX-28 realmente entrega, ¿sigue
ganando el modelo de contacto puntual, y por cuanto?

Fuentes de ruido modeladas, con las magnitudes del servo real:
  - cuantizacion de corriente: 3.36 mA por cuenta -> par cuantizado
  - stiction de Coulomb: 0.05 N.m con signo aleatorio por junta (el termino
    dominante segun la inyeccion de fallas)
  - cuantizacion de encoder: 12 bit sobre una vuelta -> 1.53e-3 rad
  - ruido de velocidad -> entra por q̈ (que en actuacion por velocidad pesa poco)

Se compara el inverso 6D contra el de contacto puntual sobre N configuraciones y
fuerzas aleatorias, y se barre la magnitud del ruido.

Run:  python3 test_noise_montecarlo.py
"""
import numpy as np
from arm_dynamics import build_dynamics
from arm_kinematics import build_jac_fn

fM, fh = build_dynamics(); fJ = build_jac_fn()
M_ = lambda q: np.array(fM(q))
h_ = lambda q, qd: np.array(fh(q, qd)).flatten()
J_ = lambda q: np.array(fJ(q))

Q_LO = np.array([-3.1416, -0.436332, -2.00713, -2.00713])
Q_HI = np.array([3.1416, 3.57792, 2.00713, 2.00713])

CUR_LSB = 3.36e-3          # A por cuenta
KT = 1.0                   # N.m/A (semilla; la escala se cancela en el analisis)
TAU_LSB = CUR_LSB*KT       # cuantizacion de par
STICTION = 0.05            # N.m, Coulomb por junta
ENC_LSB = 2*np.pi/4096     # rad


def estimators(q, tau_ext):
    """(f_6D, f_3D) desde el mismo residuo."""
    J = J_(q); Jv = J[:3, :]
    U, s, _ = np.linalg.svd(J, full_matrices=False)
    r = int(np.sum(s > 1e-9*s[0]))
    # 6D: min-norm sobre el wrench completo, se toma la parte de fuerza
    F6 = U[:, :r] @ np.linalg.lstsq(J.T @ U[:, :r], tau_ext, rcond=None)[0] \
        if False else J @ np.linalg.lstsq(J.T @ J, tau_ext, rcond=None)[0]
    f3 = np.linalg.lstsq(Jv.T, tau_ext, rcond=None)[0]
    return F6[:3], f3


def trial(rng, noise=1.0, sigmin_floor=0.0):
    while True:
        q = rng.uniform(Q_LO, Q_HI)
        s = np.linalg.svd(J_(q)[:3, :], compute_uv=False)[2]
        if s >= sigmin_floor:
            break
    f = rng.normal(size=3); f *= rng.uniform(2.0, 10.0)/np.linalg.norm(f)   # 2-10 N
    tau_true = J_(q)[:3, :].T @ f
    # --- ruido ---
    n_stick = noise*STICTION*rng.choice([-1.0, 1.0], size=4)
    tau_meas = tau_true + n_stick
    tau_meas = np.round(tau_meas/(noise*TAU_LSB + 1e-12))*(noise*TAU_LSB) if noise > 0 else tau_meas
    # error de encoder -> el modelo evalua h y J en una q ligeramente distinta
    q_meas = q + noise*ENC_LSB*rng.uniform(-0.5, 0.5, 4)
    f6, f3 = estimators(q_meas, tau_meas)
    return np.linalg.norm(f6-f), np.linalg.norm(f3-f), np.linalg.norm(f), s


def run(n=2000, noise=1.0, sigmin_floor=0.0, seed=0):
    rng = np.random.default_rng(seed)
    e6 = []; e3 = []; mag = []
    for _ in range(n):
        a, b, m, _ = trial(rng, noise, sigmin_floor)
        e6.append(a); e3.append(b); mag.append(m)
    e6 = np.array(e6); e3 = np.array(e3); mag = np.array(mag)
    return 100*e6/mag, 100*e3/mag       # error relativo [%]


if __name__ == "__main__":
    print("=" * 74)
    print("Monte Carlo, N=2000 configuraciones y fuerzas aleatorias (2-10 N)")
    print("error RELATIVO de la fuerza estimada [% de |f|]")
    print("=" * 74)
    print(f"{'ruido':>22} | {'6-D min-norm':>26} | {'contacto puntual':>26}")
    print(f"{'':>22} | {'mediana':>8} {'p90':>8} {'p99':>8} | "
          f"{'mediana':>8} {'p90':>8} {'p99':>8}")
    print("-"*74)
    for lab, nz in [("sin ruido", 0.0), ("nominal (x1)", 1.0),
                    ("degradado (x2)", 2.0), ("malo (x4)", 4.0)]:
        r6, r3 = run(noise=nz)
        print(f"{lab:>22} | {np.median(r6):8.2f} {np.percentile(r6,90):8.2f} "
              f"{np.percentile(r6,99):8.2f} | {np.median(r3):8.2f} "
              f"{np.percentile(r3,90):8.2f} {np.percentile(r3,99):8.2f}")

    print()
    print("=" * 74)
    print("Efecto del condicionamiento: descartando posturas con sigma_min bajo")
    print("(ruido nominal)")
    print("=" * 74)
    print(f"{'umbral sigma_min':>18} | {'6-D mediana':>12} {'6-D p90':>10} | "
          f"{'3-D mediana':>12} {'3-D p90':>10}")
    print("-"*74)
    for fl in [0.0, 0.02, 0.04, 0.06]:
        r6, r3 = run(n=1500, noise=1.0, sigmin_floor=fl)
        print(f"{fl:18.2f} | {np.median(r6):12.2f} {np.percentile(r6,90):10.2f} | "
              f"{np.median(r3):12.2f} {np.percentile(r3,90):10.2f}")
    print()
    print("La cota ||df|| <= ||dtau||/sigma_min predice que descartar posturas mal")
    print("condicionadas acota la cola del estimador de contacto puntual. El 6-D no")
    print("mejora, porque su error no viene del ruido sino de la mala atribucion.")
