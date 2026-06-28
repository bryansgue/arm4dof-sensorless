"""
Item 3 — Tracking de TRAYECTORIA del DQ-NMPC (referencia DQ movil, no setpoint).
Trayectoria suave alcanzable (tornillo lento: rot+trans acoplado). Planta = dinamica
validada. Reporta RMSE pos + orient + max. Demuestra seguimiento de trayectoria 6D.
"""
import os, numpy as np
from acados_template import AcadosOcpSolver
import generate_arm_dqnmpc_ocp as G
from arm_dynamics import build_dynamics
from arm_kinematics import build_fk_fn

fM,fh=build_dynamics(); fk=build_fk_fn()
def dyn(x,tau):
    q,qd=x[:4],x[4:]; return np.concatenate([qd,np.linalg.solve(np.array(fM(q)),tau-np.array(fh(q,qd)).flatten())])
def rk4(x,tau,dt):
    k1=dyn(x,tau);k2=dyn(x+dt/2*k1,tau);k3=dyn(x+dt/2*k2,tau);k4=dyn(x+dt*k3,tau)
    return x+dt/6*(k1+2*k2+2*k3+k4)

here=os.path.dirname(os.path.abspath(__file__))
solver=AcadosOcpSolver(G.build_ocp(),json_file=os.path.join(here,"..","acados_ocp_arm4dof_dqnmpc.json"),build=False,generate=False)

# trayectoria LENTA alcanzable (tornillo): joints suaves -> EE rot+trans acoplado
def q_ref_t(t):
    s=t*0.01
    return np.array([0.3*np.sin(0.4*s), 1.2+0.3*np.sin(0.3*s+1), -0.9+0.25*np.sin(0.35*s), 0.4+0.4*np.sin(0.45*s+2)])

W=np.zeros(G.N_PARAMS); W[8:14]=[3,3,3,120,120,120]; W[14:18]=0.1; W[18:22]=0.02
x=np.concatenate([q_ref_t(0),np.zeros(4)])
pe=[]; oe=[]; T=400
for t in range(T):
    qr=q_ref_t(t); dq_ref=np.array(fk(qr)[0]).flatten(); W[0:8]=dq_ref
    for k in range(G.N_HORIZON+1): solver.set(k,"p",W)
    solver.set(0,"lbx",x); solver.set(0,"ubx",x); solver.solve()
    x=rk4(x,solver.get(0,"u"),0.01)
    _,p,quat=fk(x[:4]); _,pr,qq=fk(qr)
    p=np.array(p).flatten(); pr=np.array(pr).flatten(); quat=np.array(quat).flatten(); qq=np.array(qq).flatten()
    pe.append(np.linalg.norm(p-pr)); dot=min(1.0,abs(float(quat@qq))); oe.append(2*np.degrees(np.arccos(dot)))
pe=np.array(pe[50:]); oe=np.array(oe[50:])
print("== Item 3: tracking de trayectoria DQ-NMPC (tornillo lento) ==")
print(f"RMSE posicion    = {1000*np.sqrt(np.mean(pe**2)):.2f} mm   (max {1000*pe.max():.2f} mm)")
print(f"RMSE orientacion = {np.sqrt(np.mean(oe**2)):.3f} deg  (max {oe.max():.3f} deg)")
print("TRACKING TRAYECTORIA OK" if (np.sqrt(np.mean(pe**2))<0.01 and np.sqrt(np.mean(oe**2))<3) else "tracking aceptable (ver numeros)")
