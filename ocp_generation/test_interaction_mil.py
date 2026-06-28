"""
P3 — LAZO DE INTERACCION completo, en MiL (modelo validado como planta, sin MuJoCo binario).

Arquitectura (100 Hz):
  planta: M q̈ + h = τ_act + Jᵀ F_mano      (dinamica validada + fuerza humana)
  servo:  τ_act = kv (q̇_cmd − q̇)            (modelo del lazo velocidad MX-28)
  P1 estima: F_est = (Jᵀ)⁺ (M q̈ + h − τ_act)   (sin sensor)
  P2 admitancia: F_est -> cede la referencia DQ (hand-guiding) + resorte de retorno
  NMPC: trackea DQ_ref compliant -> q̇_cmd = q̇*[1]

Escenario: el brazo SOSTIENE una tarea; un humano lo EMPUJA (ventana de fuerza); el efector
CEDE (sigue la mano), el NMPC se mantiene factible, y al SOLTAR retorna a la tarea.
Demuestra: estimacion sin sensor + compliance + factibilidad + retorno. El keystone del paper.
"""
import os, numpy as np
from acados_template import AcadosOcpSolver
import generate_arm_dqnmpc_ocp as G
from arm_dynamics import build_dynamics
from arm_kinematics import build_fk_fn, build_jac_fn
from wrench_estimator import estimate_wrench
from dq_admittance import DQAdmittance

fM,fh = build_dynamics(); fk=build_fk_fn(); fJ=build_jac_fn()
def M_(q): return np.array(fM(q))
def h_(q,qd): return np.array(fh(q,qd)).flatten()
def J_(q): return np.array(fJ(q))

# solver
json = os.path.join(os.path.dirname(os.path.abspath(__file__)),"..","acados_ocp_arm4dof_dqnmpc.json")
solver = AcadosOcpSolver(G.build_ocp(), json_file=json, build=False, generate=False)

# tarea: sostener pose en q_task
q_task = np.array([0.0, 1.2, -0.9, 0.4])
dq_task, p_task, quat_task = fk(q_task)
dq_task=np.array(dq_task).flatten(); p_task=np.array(p_task).flatten(); quat_task=np.array(quat_task).flatten()

# admitancia (con resorte de retorno suave -> vuelve a la tarea al soltar)
adm = DQAdmittance(D_trans=60.0, D_rot=8.0, p0=p_task, q0=quat_task,
                   k_return=2.0, p_task=p_task, q_task=quat_task)

W = np.zeros(G.N_PARAMS)
W[8:14]=np.array([2.,2.,2., 80.,80.,80.]); W[14:18]=0.3; W[18:22]=0.02

# planta inicial = en la tarea
x = np.concatenate([q_task, np.zeros(4)])
dt=0.01; kv=4.0
qd_cmd=np.zeros(4); qd_prev=np.zeros(4)

# fuerza humana: empuje en +y (y un poco -z) entre t=0.5s y 2.0s
def F_hand(t):
    return np.array([0,6.0,-2.0, 0,0,0]) if (50<=t<200) else np.zeros(6)

logs={'t':[],'err_task':[],'Fhat':[],'Ftrue':[],'status':[],'p':[]}
F_est=np.zeros(6)
for t in range(350):
    q,qd = x[:4], x[4:]
    # P2: admitancia con el wrench estimado del paso ANTERIOR -> referencia compliant
    dq_ref,p_ref,q_ref = adm.update(F_est, dt)
    W[0:8]=dq_ref
    # NMPC
    for k in range(G.N_HORIZON+1): solver.set(k,"p",W)
    solver.set(0,"lbx",x); solver.set(0,"ubx",x)
    st=solver.solve()
    qd_cmd = solver.get(1,"x")[4:8]
    # planta: integra servo + fuerza humana real; mide q̈ y τ_act aplicados (momentum observer)
    Ftrue=F_hand(t); qd_before=qd.copy(); tau_accum=np.zeros(4)
    for _ in range(5):
        q,qd=x[:4],x[4:]
        tau=kv*(qd_cmd-qd); tau_accum+=tau
        qdd_p=np.linalg.solve(M_(q), tau + J_(q).T@Ftrue - h_(q,qd))
        x=np.concatenate([q+ (dt/5)*qd, qd+(dt/5)*qdd_p])
    qdd_meas=(x[4:]-qd_before)/dt; tau_act_meas=tau_accum/5
    # P1: estimar wrench (sin sensor) con cantidades MEDIDAS consistentes
    F_est,_ = estimate_wrench(x[:4], x[4:], qdd_meas, tau_act_meas)
    _,pee,_=fk(x[:4]); pee=np.array(pee).flatten()
    logs['t'].append(t*dt); logs['err_task'].append(np.linalg.norm(pee-p_task))
    logs['Fhat'].append(F_est[:3].copy()); logs['Ftrue'].append(Ftrue[:3].copy())
    logs['status'].append(st); logs['p'].append(pee.copy())

import numpy as np
err=np.array(logs['err_task']); Fhat=np.array(logs['Fhat']); Ftrue=np.array(logs['Ftrue']); P=np.array(logs['p'])
push=slice(60,190); rel=slice(260,350)
print("== MiL lazo de interaccion (hand-guiding compliant) ==")
print(f"status NMPC: {sum(s==0 for s in logs['status'])}/{len(logs['status'])} OK")
print(f"error a tarea: reposo={err[40]:.4f}  empujado(t~1.5s)={err[150]:.4f}  tras soltar(final)={err[-1]:.4f} m")
print(f"  -> cede al empujar: {'SI' if err[150]>3*err[40] else 'NO'}   retorna al soltar: {'SI' if err[-1]<2*err[40]+0.005 else 'NO'}")
# estimacion vs real (componente y, la dominante) durante el empuje
fy_corr = np.corrcoef(Fhat[push,1], Ftrue[push,1])[0,1]
print(f"estimacion F_y sin sensor vs real (empuje): corr={fy_corr:.3f}, |F_y| real~6N est~{np.mean(Fhat[push,1]):.1f}N")
desp = np.max(np.abs(P[push,1]-p_task[1]))
print(f"desplazamiento compliant max en +y = {desp:.4f} m")
ok = (sum(s==0 for s in logs['status'])>0.95*len(logs['status'])
      and err[150]>3*err[40] and err[-1]<2*err[40]+0.005 and fy_corr>0.8)
print("LAZO DE INTERACCION MiL OK (estima + cede + factible + retorna)" if ok else "REVISAR")
