"""
P1 — Estimador de WRENCH externo SIN SENSOR (residuo de momento).

Modelo:  M(q) q̈ + h(q,q̇) = τ_act + Jᵀ F_ext
  => τ_ext = M q̈ + h − τ_act = Jᵀ F_ext        (parte del torque por el wrench externo)
  => F_est = (Jᵀ)⁺ τ_ext = J (Jᵀ J)⁻¹ τ_ext     (wrench 6D, min-norm)

HONESTO: J es 6×4 → solo el subespacio 4D OBSERVABLE del wrench se recupera (dual del
rank(J)=4<6). Las 2 direcciones no-observables (null de Jᵀ) no se sienten. Físicamente
correcto: el brazo solo percibe el wrench que puede resistir.

Inputs reales: q,q̇ encoders; q̈ por diff-finita de q̇; τ_act del servo (MuJoCo actuator_force
/ MX-28 Present_Load calibrado). MiL: verificamos la matemática con el modelo validado.
"""
import numpy as np
from arm_dynamics import build_dynamics
from arm_kinematics import build_jac_fn

_fM, _fh = build_dynamics()
_fJ = build_jac_fn()

def M_of(q):  return np.array(_fM(q))
def h_of(q,qd): return np.array(_fh(q,qd)).flatten()
def J_of(q):  return np.array(_fJ(q))           # 6x4

def estimate_wrench(q, qd, qdd, tau_act, damp=1e-6):
    """F_est (6) = wrench externo observable en el efector (world)."""
    M = M_of(q); h = h_of(q,qd); J = J_of(q)
    tau_ext = M @ qdd + h - tau_act              # = Jᵀ F_ext  (4,)
    Jt = J.T                                      # 4x6
    # F_est = J (JᵀJ + λI)⁻¹ τ_ext  (min-norm, observable)
    JtJ = Jt @ J + damp*np.eye(4)                 # 4x4  (= Jᵀ_row J_col? cuidado)
    # NOTA: tau_ext = Jᵀ F  (Jᵀ es 4x6). Min-norm F = J (Jᵀ J... ) -> usar (JJᵀ)? No:
    # A=Jᵀ (4x6). A F = tau_ext. min-norm F = Aᵀ(AAᵀ)⁻¹ tau_ext = J (Jᵀ J)⁻¹? AAᵀ=Jᵀ J (4x4).
    AAt = Jt @ J + damp*np.eye(4)                 # 4x4
    F_est = J @ np.linalg.solve(AAt, tau_ext)     # 6
    return F_est, tau_ext

def project_observable(q, F, damp=1e-6):
    """Proyeccion de un wrench F(6) al subespacio observable col(J) (lo recuperable
    SI no se asume nada sobre F). Ver estimate_force() para el caso de fuerza pura."""
    J = J_of(q); Jt = J.T
    tau = Jt @ F
    AAt = Jt @ J + damp*np.eye(4)
    return J @ np.linalg.solve(AAt, tau)


def estimate_force(q, qd, qdd, tau_act):
    """F_ext (3) suponiendo CONTACTO DE FUERZA PURA en el efector.

    Este es el estimador correcto para un empuje humano, y NO es lo mismo que tomar
    las 3 primeras componentes de estimate_wrench(). Razon:

      estimate_wrench resuelve  Jᵀ F = tau  con F ∈ R⁶ y J ∈ R⁶ˣ⁴. Son 6 incognitas
      desde 4 medidas: el sistema esta INDETERMINADO y la solucion de norma minima
      reparte el torque entre fuerza y momento. Una fuerza pura se mal-atribuye
      parcialmente a un momento. Medido sobre 2197 configuraciones, ese reparto
      pierde el 50 % de la fuerza en promedio (hasta 99.6 %).

      Si se sabe que el contacto es una fuerza pura en un punto CONOCIDO (el efector),
      el modelo correcto es  Jvᵀ f = tau  con f ∈ R³ y Jvᵀ ∈ R⁴ˣ³: SOBRE-determinado.
      rank(Jv)=3 en el 100 % del espacio de trabajo de este brazo, asi que f se
      recupera EXACTA (3.6e-15 sobre el mismo barrido).

    ⚠️ Requiere conocer el punto de contacto. Vale para hand-guiding en el efector;
    no vale para un contacto en cualquier lugar del eslabon. Y si el contacto aplica
    ademas un momento (objeto agarrado), vuelve la ambiguedad: ahi hay que usar
    estimate_wrench y aceptar la mezcla.

    El limite practico NO es observabilidad sino CONDICIONAMIENTO: cerca de una
    singularidad de Jv, sigma_min cae (mediana 0.0496, minimo 7.6e-5 en el barrido)
    y el ruido de tau se amplifica por 1/sigma_min.
    """
    M = M_of(q); h = h_of(q, qd); Jv = J_of(q)[:3, :]     # 3x4
    tau_ext = M @ qdd + h - tau_act                        # (4,)
    f, *_ = np.linalg.lstsq(Jv.T, tau_ext, rcond=None)     # min-cuadrados: 4 ec., 3 incog.
    return f


def force_conditioning(q):
    """(sigma_min, cond) de Jv: cuanto amplifica el ruido el estimador de fuerza pura."""
    s = np.linalg.svd(J_of(q)[:3, :], compute_uv=False)
    return float(s[2]), float(s[0]/max(s[2], 1e-16))

if __name__ == "__main__":
    # ── MiL: aplicar wrench conocido, recuperarlo ──
    rng = np.random.default_rng(0)
    print("== MiL estimador de wrench (modelo validado) ==")
    maxe_obs=0; maxe_tau=0
    for k in range(6):
        q  = rng.uniform(-1,1,4); qd = rng.uniform(-1,1,4); qdd = rng.uniform(-2,2,4)
        F_true = rng.uniform(-5,5,6)                       # wrench "de la mano" (6D)
        # plant: τ_act que produce ese (q̈, con ese F_ext)
        tau_act = M_of(q)@qdd + h_of(q,qd) - J_of(q).T@F_true
        F_est, tau_ext = estimate_wrench(q, qd, qdd, tau_act)
        F_obs = project_observable(q, F_true)             # lo recuperable de F_true
        e_obs = np.max(np.abs(F_est - F_obs))             # F_est vs proyeccion observable
        e_tau = np.max(np.abs(J_of(q).T@F_est - J_of(q).T@F_true))  # torque reproducido
        maxe_obs=max(maxe_obs,e_obs); maxe_tau=max(maxe_tau,e_tau)
        unobs = np.linalg.norm(F_true - F_obs)            # parte NO observable (no se siente)
        print(f" cfg{k}: F_est==F_obs err={e_obs:.2e}  Jᵀ-reproduce err={e_tau:.2e}  |F_noobs|={unobs:.2f}N")
    print("-"*50)
    print(f"MAX |F_est−F_obs|={maxe_obs:.2e} (validacion)  MAX Jᵀ-repro={maxe_tau:.2e} (~λ damping)")
    # validacion = F_est recupera la proyeccion observable; Jᵀ-repro es el piso de la regularizacion λ
    print("ESTIMADOR WRENCH OK (recupera el subespacio observable)" if (maxe_obs<1e-8 and maxe_tau<1e-2) else "REVISAR")
