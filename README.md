# arm4dof_dqnmpc — Compliant DQ-NMPC sin sensor para brazo 4DOF de bajo costo

Control de interacción física **sin sensor de fuerza** para un manipulador 4DOF de servos
comerciales (Dynamixel MX-28R). Estimación de wrench externo desde la dinámica + encoders
(observability-aware), control compliant DQ-NMPC. Target: **Mechatronics** (ver `PLAN_MECHATRONICS.md`).

Proyecto SEPARADO del dron (`nmpc_controller`). Reusa álgebra DQ + numérica del DQ-NMPC del dron.

## Estructura

```
ocp_generation/
  arm_params.py           params del brazo extraídos de MuJoCo (validado)
  arm_dynamics.py         M(q), h(q,q̇) simbólicos CasADi → validado vs MuJoCo 1e-10
  arm_kinematics.py       FK DQ + Jacobiano del efector → validado 1e-13 / 4e-17
  dq_math.py              álgebra dual quaternion + log se(3) (de mpcc_controller del dron)
  dq_admittance.py        ley de admitancia se(3) (P2)
  wrench_estimator.py     estimador de wrench sin sensor (P1)
  generate_arm_dqnmpc_ocp.py    OCP acados DQ-NMPC
  generate_arm_decoupled_ocp.py OCP baseline desacoplado (E4)
  test_*.py               tests MiL / comparación (E4, trayectoria, interacción)
  mj_*.c                  validación rigurosa en MuJoCo REAL (libmujoco + libacados)
arm_dqnmpc_node.py        nodo ROS: NMPC tracking (vivo)
arm_handguide_node.py     nodo ROS: hand-guiding compliant (vivo, demo)
```

## Sim (escena MuJoCo)

En `~/mujoco_ws/src/acp_mujoco_simulator/model/arm4dof/` (infra compartida):
`scene_arm4dof_vel.xml(.xacro)` — actuadores velocity + plugins ROS.

```bash
ros2 launch drone_teleop mujoco_only.launch.py scene:=arm4dof_vel   # sim
python3 arm_handguide_node.py                                       # hand-guiding vivo
```

## Build de los tests C (libmujoco + libacados)

```bash
# regenerar escena temp (se borra de /tmp):
#   python3 -c "import re;s=open('scene_arm4dof_vel.xml').read();...; open('/tmp/vel.xml','w').write(s)"
# .so del solver: cd c_generated_code_arm_dqnmpc && make shared_lib
gcc mj_interaction.c -I$MUJOCO/include -I$ACADOS/include[...] \
  -lmujoco -lacados_ocp_solver_arm4dof_dqnmpc -lacados -lhpipm -lblasfeo -lm
```

## Estado

Núcleo experimental SIM completo y validado. Falta: hardware (MX-28 real) + escribir.
Ver `PLAN_MECHATRONICS.md`.
