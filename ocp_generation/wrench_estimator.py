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
    """Proyeccion de un wrench F(6) al subespacio observable col(J) (lo recuperable)."""
    J = J_of(q); Jt = J.T
    tau = Jt @ F
    AAt = Jt @ J + damp*np.eye(4)
    return J @ np.linalg.solve(AAt, tau)

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
