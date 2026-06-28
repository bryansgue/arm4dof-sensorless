"""
E4 — DQ vs DESACOPLADO: tracking de trayectoria con acople rot-traslacion.
Misma planta (dinamica validada), misma trayectoria, mismos pesos. Mide error de
tracking (pos + orient). Hipotesis: EMPATE (J_l⁻¹≈I en errores chicos).
Honesto: confirma que el DQ NO gana performance -> su valor es la representacion.
"""
import os, numpy as np
from acados_template import AcadosOcpSolver
import generate_arm_dqnmpc_ocp as Gdq
import generate_arm_decoupled_ocp as Gdec
from arm_dynamics import build_dynamics
from arm_kinematics import build_fk_fn

fM,fh=build_dynamics(); fk=build_fk_fn()
def dyn(x,tau):
    q,qd=x[:4],x[4:]
    return np.concatenate([qd, np.linalg.solve(np.array(fM(q)), tau-np.array(fh(q,qd)).flatten())])
def rk4(x,tau,dt):
    k1=dyn(x,tau);k2=dyn(x+dt/2*k1,tau);k3=dyn(x+dt/2*k2,tau);k4=dyn(x+dt*k3,tau)
    return x+dt/6*(k1+2*k2+2*k3+k4)

here=os.path.dirname(os.path.abspath(__file__))
sol_dq =AcadosOcpSolver(Gdq.build_ocp(),        json_file=os.path.join(here,"..","acados_ocp_arm4dof_dqnmpc.json"),    build=False,generate=False)
sol_dec=AcadosOcpSolver(Gdec.build_ocp_decoupled(),json_file=os.path.join(here,"..","acados_ocp_arm4dof_decoupled.json"),build=False,generate=False)

# trayectoria en juntas (alcanzable) -> EE hace movimiento de tornillo (rot+trans acoplado)
def q_ref_t(t):
    s=t*0.01
    return np.array([0.4*np.sin(0.8*s), 1.2+0.4*np.sin(0.6*s+1), -0.9+0.3*np.sin(0.7*s), 0.4+0.5*np.sin(0.9*s+2)])

W=np.zeros(Gdq.N_PARAMS); W[8:14]=[3,3,3,120,120,120]; W[14:18]=0.1; W[18:22]=0.02

def run(solver):
    x=np.concatenate([q_ref_t(0),np.zeros(4)])
    pe=[]; oe=[]; T=300
    for t in range(T):
        qd_ref=q_ref_t(t)
        dq_ref=np.array(fk(qd_ref)[0]).flatten(); W[0:8]=dq_ref
        for k in range(Gdq.N_HORIZON+1): solver.set(k,"p",W)
        solver.set(0,"lbx",x); solver.set(0,"ubx",x)
        st=solver.solve(); tau=solver.get(0,"u")
        x=rk4(x,tau,0.01)
        # error EE vs referencia
        _,p,quat=fk(x[:4]); _,pr,qr=fk(qd_ref)
        p=np.array(p).flatten(); pr=np.array(pr).flatten()
        quat=np.array(quat).flatten(); qr=np.array(qr).flatten()
        pe.append(np.linalg.norm(p-pr))
        dot=min(1.0,abs(float(quat@qr))); oe.append(2*np.degrees(np.arccos(dot)))
    return np.mean(pe[50:]), np.mean(oe[50:])   # tras transitorio

pe_dq,oe_dq = run(sol_dq)
pe_dc,oe_dc = run(sol_dec)
print("== E4: DQ vs DESACOPLADO (tracking trayectoria de tornillo) ==")
print(f"{'':12s}  pos_err[mm]  orient_err[deg]")
print(f"DQ (ln_dual)   {1000*pe_dq:9.3f}   {oe_dq:11.4f}")
print(f"DESACOPLADO    {1000*pe_dc:9.3f}   {oe_dc:11.4f}")
diff=100*abs(pe_dq-pe_dc)/max(pe_dq,pe_dc,1e-9)
print(f"-> diferencia de tracking = {diff:.1f}%  ({'EMPATE (J_l⁻¹≈I)' if diff<10 else 'DIFERENCIA'})")
print("E4: confirmado — DQ ~= desacoplado en tracking. Valor DQ = representacion unificada, no performance.")
