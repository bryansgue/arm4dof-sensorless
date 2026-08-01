#!/usr/bin/env python3
"""
Nodo ROS2 del DQ-NMPC del brazo 4DOF (control por VELOCIDAD).

  sub  /quadrotor/base/joint_states  (q, q̇ de m1..m4)
  ->   resuelve el OCP acados (DQ-NMPC)
  pub  /quadrotor/ctrl_cmd  Float64MultiArray = [q̇*1..4, 0]  -> actuadores velocity

Target: pose del efector = FK(target_q).  target_q por parametro o topic /arm/target_q.

Correr:
  ros2 launch drone_teleop mujoco_only.launch.py scene:=arm4dof_vel   # T1 (sim)
  python3 arm_dqnmpc_node.py                                          # T2 (este nodo)
"""
import os, sys, numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "ocp_generation"))

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

import generate_arm_dqnmpc_ocp as G
from arm_kinematics import build_fk_fn
from acados_template import AcadosOcpSolver

JOINTS = ["m1","m2","m3","m4"]

class ArmDQNMPC(Node):
    def __init__(self):
        super().__init__("arm_dqnmpc")
        self.fk = build_fk_fn()
        json = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "ocp_generation","..","acados_ocp_arm4dof_dqnmpc.json")
        self.solver = AcadosOcpSolver(G.build_ocp(), json_file=json, build=False, generate=False)

        # pesos. OJO: el OCP es NONLINEAR_LS, asi que el peso NO viaja en p[8:22]
        # (esos slots quedaron muertos al migrar desde EXTERNAL). Se setean con
        # cost_set sobre la W del solver; p solo lleva dq_ref en p[0:8].
        W_se3 = [2., 2., 2., 80., 80., 80.]; W_qd = [0.3]*4; W_u = [0.02]*4
        for k in range(G.N_HORIZON):
            self.solver.cost_set(k, "W", np.diag(W_se3 + W_qd + W_u))
        self.solver.cost_set(G.N_HORIZON, "W", np.diag(W_se3 + W_qd))
        self.W = np.zeros(G.N_PARAMS)

        # target inicial (config de juntas -> pose efector)
        tq = self.declare_parameter("target_q", [0.6,1.4,-0.9,0.3]).value
        self.set_target_q(np.array(tq, float))

        self.q = np.zeros(4); self.qd = np.zeros(4); self.have_state = False
        self.create_subscription(JointState, "/quadrotor/base/joint_states", self.on_js, 10)
        self.create_subscription(Float64MultiArray, "/arm/target_q", self.on_tgt, 10)
        self.pub = self.create_publisher(Float64MultiArray, "/quadrotor/ctrl_cmd", 10)
        self.create_timer(0.01, self.step)   # 100 Hz
        self.get_logger().info("DQ-NMPC brazo listo (100 Hz, control velocidad).")

    def set_target_q(self, tq):
        dq_ref = np.array(self.fk(tq)[0]).flatten()
        self.W[0:8] = dq_ref
        self.get_logger().info(f"target_q = {np.round(tq,3)}")

    def on_tgt(self, msg):
        if len(msg.data) >= 4: self.set_target_q(np.array(msg.data[:4], float))

    def on_js(self, msg):
        idx = {n:i for i,n in enumerate(msg.name)}
        try:
            self.q  = np.array([msg.position[idx[j]] for j in JOINTS])
            self.qd = np.array([msg.velocity[idx[j]] for j in JOINTS]) if msg.velocity else np.zeros(4)
            self.have_state = True
        except (KeyError, IndexError):
            pass

    def step(self):
        if not self.have_state: return
        x = np.concatenate([self.q, self.qd])
        for k in range(G.N_HORIZON+1):
            self.solver.set(k, "p", self.W)
        self.solver.set(0, "lbx", x); self.solver.set(0, "ubx", x)
        st = self.solver.solve()
        qd_cmd = self.solver.get(1, "x")[4:8]
        if st != 0:
            qd_cmd = np.zeros(4)   # solver fail -> frenar
        msg = Float64MultiArray()
        msg.data = [float(qd_cmd[0]),float(qd_cmd[1]),float(qd_cmd[2]),float(qd_cmd[3]), 0.0]
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = ArmDQNMPC()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()
