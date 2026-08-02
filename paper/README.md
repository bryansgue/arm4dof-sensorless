# Manuscrito — IEEE Access

**Contact-Model-Aware Sensorless Force Estimation and Compliant Dual Quaternion
Predictive Control for Low-Cost Under-Actuated Manipulators**

9 páginas. Formato `ieeeaccess.cls` tomado de
`~/python/Time_optimal_planing-NMPC/ACCESS_latex` (solo el formato; nada de
contenido de ahí).

## Compilar

```bash
pdflatex main && bibtex main && pdflatex main && pdflatex main
```

Compila con 0 errores y 0 referencias sin resolver.

⚠️ Los ~21 avisos `Overfull \hbox (505.12pt)` son del **`.cls`**, no del texto: el
template original los produce igual. No tocarlos.

## La tesis, en una frase

En brazos con menos juntas que dimensiones de tarea, lo que limita la estimación
sin sensor **no es el rango del Jacobiano sino el modelo de contacto que asume el
estimador**. El pseudo-inverso 6D estándar está indeterminado y reparte la fuerza
de contacto como momento espurio; con contacto puntual en punto conocido el
problema es sobre-determinado y la fuerza se recupera exacta si `rank(Jv)=3`, que
se cumple en el 100% del espacio de trabajo.

Números duros: el 6D pierde 50.5% de la fuerza en promedio (peor caso 99.6%);
contra MuJoCo el RMSE baja de 2.227 N a 0.157 N, factor 14.

⚠️ Esto **corrige** los dos borradores anteriores (Mechatronics y L-CSS), que
afirmaban que el brazo tenía una dirección de fuerza ciega. Ver
`../PLAN_MECHATRONICS.md`, sección "CORRECCIÓN IMPORTANTE".

## Antes de enviar

1. **Autores, afiliaciones, correo de contacto, financiación, agradecimiento** —
   todo son marcadores `[...]`.
2. **Biografías con foto**: IEEE Access las pide. El template las trae como
   `\begin{IEEEbiography}[{\includegraphics{...}}]{Nombre}`. Hoy no están.
3. **Cabecera**: el desborde de 9.3 pt en `\markboth` desaparece al poner nombres
   reales (hoy dice `[Author]`).
4. **Figuras** se regeneran:
   ```bash
   cd ../ocp_generation
   python3 observability_map.py    # figures/observability_map.png
   python3 make_figures.py         # figures/metric_ablation.png, interaction_traces.png
   ```
   ⚠️ `metric_ablation.png` ya no se cita en el texto: la ablación quedó como
   Tabla V. Si se quiere como figura, agregarla en Sec. VI-F.

## De dónde sale cada número

Nada escrito a mano.

| sección | qué reporta | script |
|---|---|---|
| III-D, Tab. 1 | validación del modelo vs MuJoCo | `arm_dynamics.py`, `arm_kinematics.py` |
| IV, Tab. 2 | los dos estimadores, barrido 2197 configs, condicionamiento | `observability_map.py` |
| IV-C | exactitud del estimador de contacto puntual | `wrench_estimator.py` |
| VI-A, Tab. 3 | comparación en lazo cerrado | `test_interaction_mil.py` |
| VI-B | validación contra MuJoCo (2.227 vs 0.157 N) | `mj_wrench_test.c` |
| VI-C, Tab. 4 | compliant vs rígido, pareado N=6 | `mj_stats.c` |
| VI-D | regulación y trayectoria | `test_arm_dqnmpc.py`, `test_trajectory.py` |
| VI-D, Tab. 5 | timing, grafo vs escalar | `timing_test.c` |
| VI-F, Tab. 6 | ablación de la métrica se(3) | `test_e4bis.py` |

Tests en C: preparar la escena primero.

```bash
S=~/mujoco_ws/src/acp_mujoco_simulator/model/arm4dof
python3 -c "s=open('$S/scene_arm4dof_vel.xml').read(); \
  open('/tmp/vel.xml','w').write(s.replace('meshdir=\"assets/\"','meshdir=\"$S/assets/\"'))"
```

## Qué falta

Todo es simulación, y el manuscrito lo declara en el abstract, en la introducción
y en Limitations.

- **HW-1** estimación contra peso colgado conocido, en configuración **bien y mal
  condicionada** (`σ_min(Jv)` alto y bajo). Es la predicción falsable del paper.
- **HW-2/3/4** hand-guiding, compliant vs rígido (**pareado**), trayectoria.
- **Código**: extender el estimador a punto de contacto desconocido a lo largo del
  eslabón (Remark 3), en la línea de la referencia [5].
