"""
P2 — Ley de ADMITANCIA en se(3) (dual quaternion).

Wrench externo F_est = [fuerza(3); torque(3)] (world)  ->  twist compliant:
    Λ ξ̇ + D ξ = F_ext        (admitancia masa-amortiguamiento en se(3))
    version velocidad (Λ=0):  ξ = D⁻¹ F_ext  = [v(3); ω(3)]
La referencia de pose CEDE integrando ese twist -> hand-guiding:
    p_ref += v·dt ;  q_ref = exp(ω·dt) ⊗ q_ref
D puede acoplar (6×6): fuerza fuera de eje -> movimiento de TORNILLO (rot+trans
acopladas) = donde el se(3)/DQ tiene proposito. Diagonal = primer corte.

numpy (corre fuera del OCP, 100 Hz). Convencion: quat [w,x,y,z], twist [v;ω], wrench [f;τ].
"""
import numpy as np

def qmul(a, b):
    w1,x1,y1,z1=a; w2,x2,y2,z2=b
    return np.array([w1*w2-x1*x2-y1*y2-z1*z2,
                     w1*x2+x1*w2+y1*z2-z1*y2,
                     w1*y2-x1*z2+y1*w2+z1*x2,
                     w1*z2+x1*y2-y1*x2+z1*w2])

def axisangle_quat(v):
    """vector de rotacion v (=ω·dt) -> quaternion."""
    th = np.linalg.norm(v)
    if th < 1e-12: return np.array([1.0,0,0,0])
    n = v/th
    return np.array([np.cos(th/2), *(np.sin(th/2)*n)])

def dq_from_pose_np(quat, t):
    """DQ unitario = [q_r; 0.5 t⊗q_r]."""
    tq = np.array([0.0,*t])
    qd = 0.5*qmul(tq, quat)
    return np.concatenate([quat, qd])

class DQAdmittance:
    """Genera la referencia compliant que cede al wrench externo."""
    def __init__(self, D_trans=40.0, D_rot=4.0, p0=None, q0=None,
                 k_return=0.0, p_task=None, q_task=None):
        # D: amortiguamiento virtual (alto = rigido, bajo = blando). 6x6 (aqui diag).
        self.Dinv = np.diag([1/D_trans]*3 + [1/D_rot]*3)   # 6x6
        self.p = np.array(p0 if p0 is not None else [0,0,0], float)
        self.q = np.array(q0 if q0 is not None else [1,0,0,0], float)
        # opcional: resorte que retorna a la pose de tarea al soltar (k_return>0)
        self.k_return = k_return
        self.p_task = np.array(p_task if p_task is not None else self.p, float)
        self.q_task = np.array(q_task if q_task is not None else self.q, float)

    def update(self, F_ext, dt):
        """F_ext=[f(3);τ(3)] (world) -> avanza la referencia compliant, devuelve DQ_ref."""
        xi = self.Dinv @ np.asarray(F_ext, float)     # [v; ω]
        v, w = xi[:3], xi[3:]
        # resorte de retorno (opcional): tira de la ref hacia la tarea
        if self.k_return > 0:
            v = v + self.k_return*(self.p_task - self.p)
        self.p = self.p + v*dt
        self.q = qmul(axisangle_quat(w*dt), self.q)
        self.q = self.q/np.linalg.norm(self.q)
        return dq_from_pose_np(self.q, self.p), self.p.copy(), self.q.copy()


if __name__ == "__main__":
    # ── MiL: empujar -> la referencia cede en la direccion del empuje ──
    print("== MiL admitancia DQ (hand-guiding a nivel referencia) ==")
    adm = DQAdmittance(D_trans=40.0, D_rot=4.0, p0=[0.2,0,0.2], q0=[1,0,0,0])
    dt=0.01
    # fase 1: empujo en +y 5 N por 1 s ; fase 2: suelto 0.5 s
    log=[]
    for t in range(150):
        F = np.array([0,5.0,0, 0,0,0.0]) if t<100 else np.zeros(6)   # fuerza +y, luego suelto
        dq,p,q = adm.update(F, dt)
        log.append(p.copy())
    log=np.array(log)
    print(f"p inicial = {np.round(log[0],4)}")
    print(f"p a t=1s (empujando +y) = {np.round(log[99],4)}   (debe moverse +y)")
    print(f"p final (soltado) = {np.round(log[-1],4)}   (sin resorte: se queda)")
    moved_y = log[99,1]-log[0,1]
    print(f"desplazamiento +y bajo 5N/1s = {moved_y:.4f} m  (= 5/40·1 = 0.125 esperado)")
    print("ADMITANCIA DQ OK" if abs(moved_y-0.125)<2e-3 else "REVISAR")
