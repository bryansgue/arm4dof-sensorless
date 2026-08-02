# arm4dof_dqnmpc — estimación de fuerza sin sensor para un brazo 4DOF de bajo costo

Control de interacción física **sin sensor de fuerza** para un manipulador 4DOF de
servos comerciales (Dynamixel MX-28R), actuado en **velocidad**. Estimación de la
fuerza de contacto desde la dinámica y los encoders, y control compliant
predictivo.

> **Empezar por [`CLAUDE.md`](CLAUDE.md)**: estado, correcciones que no hay que
> deshacer, y cómo reproducir cada número.

Paper listo para **IEEE Access** en [`paper/`](paper/) (14 páginas, compila
limpio). Todo lo reportado es **simulación**; el hardware está pendiente.

## Resultado principal

Inverter el residuo de momento para un wrench completo está **mal especificado**
cuando el brazo tiene menos de seis juntas: el sistema queda indeterminado y la
solución de norma mínima reparte la fuerza de contacto como un momento espurio.
Sobre 2197 configuraciones eso descarta el **50.5%** de la fuerza aplicada en
promedio, y falla en silencio. Con un modelo de contacto puntual el problema es
sobre-determinado y la fuerza se recupera exacta.

Contra MuJoCo como planta independiente: RMSE **2.227 → 0.157 N**.

## Estructura

```
paper/              manuscrito IEEE Access + checklist de envío
hw/                 bring-up del brazo real: driver, S0-S3, banco MuJoCo
ocp_generation/     modelo, estimador, OCPs y todos los experimentos
figures/            figuras del paper (regenerables)
```

## Simulador

Escena en `~/mujoco_ws/src/acp_mujoco_simulator/model/arm4dof/scene_arm4dof_vel.xml`
(actuadores de velocidad con los límites reales del MX-28). La prepara sola
`hw/mj_arm.ensure_scene()`.

```bash
ros2 launch drone_teleop mujoco_only.launch.py scene:=arm4dof_vel
python3 arm_handguide_node.py        # hand-guiding compliant
```

## Reproducir

Ver la sección correspondiente de [`CLAUDE.md`](CLAUDE.md). Todos los números del
paper salen de un script con nombre; ninguno está escrito a mano.
