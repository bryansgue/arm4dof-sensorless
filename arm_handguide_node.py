#!/usr/bin/env python3
"""
Nodo ROS2 — HAND-GUIDING en vivo (compliant, sin sensor).
Arrastrá el efector en el visor MuJoCo (ctrl + arrastrar) -> el brazo CEDE y lo seguís;
soltá -> vuelve a la tarea. Observer de wrench (sin sensor) + admitancia DQ + NMPC.

  sub  /quadrotor/base/joint_states  (q, q̇)
  obs  F_est = (Jᵀ)⁺ (M q̈ + h − τ_act)     [q̈ = EMA(dif-finita q̇), τ_act = kv(qd_cmd−q̇)]
  adm  F_est -> cede la referencia DQ + resorte de retorno
  pub  /quadrotor/ctrl_cmd = [q̇*1..4, 0]

OJO: este nodo es pal DEMO visual. La validación rigurosa está en C (mj_interaction).
El q̈ por dif-finita es ruidoso -> se filtra (EMA). En HW real: τ_act de Present_Load.

Correr:  T1: ros2 launch drone_teleop mujoco_only.launch.py scene:=arm4dof_vel
         T2: python3 arm_handguide_node.py
         (en el visor: ctrl+arrastrar el efector)
"""
import os, sys, numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "ocp_generation"))
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

import generate_arm_dqnmpc_ocp as G
from arm_dynamics import build_dynamics
from arm_kinematics import build_fk_fn, build_jac_fn
from dq_admittance import DQAdmittance
from acados_template import AcadosOcpSolver

JOINTS=["m1","m2","m3","m4"]
KV=3.0          # ganancia del actuador velocity (de la escena)
LAM=0.01        # regularizacion del wrench (4DOF mal-condicionado)
ALPHA_QDD=0.2   # EMA de q̈ (dif-finita ruidosa)
ALPHA_F=0.3     # EMA del wrench estimado

class HandGuide(Node):
    def __init__(self):
        super().__init__("arm_handguide")
        self.fM,self.fh=build_dynamics(); self.fk=build_fk_fn(); self.fJ=build_jac_fn()
        json=os.path.join(os.path.dirname(os.path.abspath(__file__)),"ocp_generation","..","acados_ocp_arm4dof_dqnmpc.json")
        self.solver=AcadosOcpSolver(G.build_ocp(),json_file=json,build=False,generate=False)
        self.W=np.zeros(G.N_PARAMS); self.W[8:14]=[2,2,2,80,80,80]; self.W[14:18]=0.3; self.W[18:22]=0.02
        # tarea
        q_task=np.array(self.declare_parameter("task_q",[0.0,1.2,-0.9,0.4]).value,float)
        _,p0,q0=self.fk(q_task); self.p_task=np.array(p0).flatten(); self.q_task=np.array(q0).flatten()
        self.adm=DQAdmittance(D_trans=8.0,D_rot=2.0,p0=self.p_task,q0=self.q_task,
                              k_return=1.5,p_task=self.p_task,q_task=self.q_task)
        self.q=q_task.copy(); self.qd=np.zeros(4); self.qd_prev=np.zeros(4)
        self.qdd=np.zeros(4); self.qd_cmd=np.zeros(4); self.F=np.zeros(6); self.have=False
        self.create_subscription(JointState,"/quadrotor/base/joint_states",self.on_js,10)
        self.pub=self.create_publisher(Float64MultiArray,"/quadrotor/ctrl_cmd",10)
        self.create_timer(0.01,self.step)
        self.get_logger().info("Hand-guiding listo. En el visor: ctrl+arrastrar el efector.")

    def on_js(self,msg):
        idx={n:i for i,n in enumerate(msg.name)}
        try:
            self.q=np.array([msg.position[idx[j]] for j in JOINTS])
            self.qd=np.array([msg.velocity[idx[j]] for j in JOINTS]) if msg.velocity else np.zeros(4)
            self.have=True
        except (KeyError,IndexError): pass

    def step(self):
        if not self.have: return
        q,qd=self.q,self.qd
        # observer (sin sensor): q̈ dif-finita filtrada, τ_act del propio comando
        qdd_raw=(qd-self.qd_prev)/0.01; self.qd_prev=qd.copy()
        self.qdd=(1-ALPHA_QDD)*self.qdd+ALPHA_QDD*qdd_raw
        tau_act=KV*(self.qd_cmd-qd)
        M=np.array(self.fM(q)); h=np.array(self.fh(q,qd)).flatten(); J=np.array(self.fJ(q))
        tau_ext=M@self.qdd+h-tau_act
        AAt=J.T@J+LAM*np.eye(4)
        F_raw=J@np.linalg.solve(AAt,tau_ext)
        self.F=(1-ALPHA_F)*self.F+ALPHA_F*F_raw       # EMA del wrench
        # admitancia -> referencia compliant
        dq_ref,_,_=self.adm.update(self.F,0.01); self.W[0:8]=dq_ref
        # NMPC
        x=np.concatenate([q,qd])
        for k in range(G.N_HORIZON+1): self.solver.set(k,"p",self.W)
        self.solver.set(0,"lbx",x); self.solver.set(0,"ubx",x)
        st=self.solver.solve()
        self.qd_cmd=self.solver.get(1,"x")[4:8] if st==0 else np.zeros(4)
        m=Float64MultiArray(); m.data=[float(v) for v in self.qd_cmd]+[0.0]; self.pub.publish(m)

def main():
    rclpy.init(); n=HandGuide()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally: n.destroy_node(); rclpy.shutdown()

if __name__=="__main__": main()
