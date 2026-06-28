"""
Test lazo cerrado del DQ-NMPC del brazo:
  NMPC -> τ* (y q̇*[1] = comando de velocidad al servo MX-28)
  integro con la dinamica VALIDADA (q̈=M⁻¹(τ−h), RK4)
  -> el efector debe converger a la pose target.
Reporta: status del solver, error de pose vs tiempo, q̇* (salida velocidad).
"""
import os, numpy as np, casadi as ca
from acados_template import AcadosOcpSolver
import arm_params as P
from arm_dynamics import build_dynamics
from arm_kinematics import build_fk_fn
import generate_arm_dqnmpc_ocp as G

f_M, f_h = build_dynamics()
fk = build_fk_fn()

def dyn(x, tau):
    q, qd = x[:4], x[4:]
    qdd = np.linalg.solve(np.array(f_M(q)), tau - np.array(f_h(q, qd)).flatten())
    return np.concatenate([qd, qdd])

def rk4(x, tau, dt):
    k1=dyn(x,tau); k2=dyn(x+dt/2*k1,tau); k3=dyn(x+dt/2*k2,tau); k4=dyn(x+dt*k3,tau)
    return x + dt/6*(k1+2*k2+2*k3+k4)

def quat_pos(dq):
    dq=np.array(dq).flatten(); return dq

# target alcanzable
q_tgt = np.array([0.6, 1.4, -0.9, 0.3])
dq_ref, p_tgt, quat_tgt = fk(q_tgt)
dq_ref = np.array(dq_ref).flatten(); p_tgt=np.array(p_tgt).flatten()

# solver
json = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                    "acados_ocp_arm4dof_dqnmpc.json")
solver = AcadosOcpSolver(G.build_ocp(), json_file=json, build=False, generate=False)

# params: dq_ref + pesos default
p = np.zeros(G.N_PARAMS)
p[0:8]  = dq_ref
p[8:14] = np.array([2.0,2.0,2.0, 80.0,80.0,80.0])   # W_se3 [rot,trans]
p[14:18]= 0.3       # W_qd
p[18:22]= 0.02      # W_u
for k in range(G.N_HORIZON+1):
    solver.set(k, "p", p)

# lazo cerrado
x = np.array([0.0, 1.0, -1.0, 0.5, 0,0,0,0])
dt = 0.01   # 100 Hz
T = 250
errs=[]; statuses=[]; qd_cmds=[]
for t in range(T):
    solver.set(0, "lbx", x); solver.set(0, "ubx", x)
    st = solver.solve()
    statuses.append(st)
    tau0 = solver.get(0, "u")
    qd_cmd = solver.get(1, "x")[4:8]    # q̇*[1] = comando de velocidad al servo MX-28
    qd_cmds.append(qd_cmd.copy())
    x = rk4(x, tau0, dt)
    _,p_ee,_ = fk(x[:4]); p_ee=np.array(p_ee).flatten()
    errs.append(np.linalg.norm(p_ee - p_tgt))

errs=np.array(errs)
print(f"target q = {q_tgt}")
print(f"target pos efector = {np.round(p_tgt,4)}")
print(f"status solver: {sum(s==0 for s in statuses)}/{T} OK (0=success)")
print(f"error pos efector:  inicial={errs[0]:.4f} m  ->  final={errs[-1]:.5f} m")
print(f"  min={errs.min():.5f}  @paso {errs.argmin()}")
print(f"q final = {np.round(x[:4],3)}  (target {np.round(q_tgt,3)})")
print(f"|q̇*| final (cmd velocidad servo) = {np.round(qd_cmds[-1],4)} rad/s")
ok = errs[-1] < 0.01 and all(s==0 for s in statuses)
print("DQ-NMPC LAZO CERRADO OK" if ok else ("REVISAR  (status fail)" if any(s!=0 for s in statuses) else "REVISAR (no converge)"))
