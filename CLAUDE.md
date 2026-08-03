# CLAUDE.md — arm4dof_dqnmpc

Estimación de fuerza de contacto **sin sensor** y control compliant para un brazo
4DOF de servos comerciales (Dynamixel MX-28R). Paper listo para IEEE Access.

> **Entrada rápida:** el paper está en `paper/` (14 pág, compila limpio). Lo único
> que bloquea el envío es confirmar la **autoría**. Ver `paper/SUBMISSION.md`.

---

## Estado (01/08/2026)

| | |
|---|---|
| paper | `paper/main.tex`, 14 pág, IEEE Access, 0 errores, lint clean |
| bloqueo | **autoría** (decisión del autor). ORCID en el portal, fotos en cámara lista |
| hardware | **nada probado en el brazo real.** `hw/` listo, procedimientos validados |
| rama | `master` (mergeado desde `sensorless-contact-model`), sin pushear |

⚠️ **El paper declara que TODO es simulación**, en abstract, introducción, alcance
y limitaciones. No suavizar eso: es el punto que un revisor va a atacar y la
defensa es que está declarado.

---

## ⛔ REVISIÓN DEL ASESOR (02/08/2026) — leer antes de tocar el paper

El asesor revisó y **tiene razón en todo lo verificable**. No enviar hasta
resolverlo. Detalle y citas en `.claude/paper-domain.md`.

| hallazgo | verificado |
|---|---|
| **La diferenciación con Magrini es indefendible.** Ya establece el hecho dimensional (`rank J_c = 6` exige `n ≥ 6`), reduce a `m = 3`, dice que `N(J_c^T)` nunca se recupera, y su Fig. 7 muestra `4 < 6` con fuerza mal y momento espurio en hardware | ✅ contra la fuente |
| El abstract dice *"under the quantization and stiction of the real servo"* y esos efectos están **simulados**. Se lee como datos reales | ✅ |
| **"under-actuated" es incorrecto**: 4 juntas actuadas = totalmente actuado. Usar *low-DoF* o *task-dimension-deficient* | ✅ |
| Inconsistencia 0.99 N (intro) vs 0.264 N (teoría): son experimentos distintos y el lector no puede saberlo | ✅ |
| Autoría: debe salir con **un solo autor**. Quitar afiliaciones 2 y 3, corresponding, financiamiento, agradecimiento y biografías heredadas | criterio IEEE |
| Esperar el hardware: las afirmaciones aplicadas dependen justo de lo que MuJoCo no valida (corriente como proxy de par, stiction, holgura, calibración de kt) | razonable |
| 14 páginas con tres historias diluyen el aporte principal | razonable |

⚠️ **Causa raíz del error grande: se citó a Magrini SIN LEERLO.** El extracto está
en `.claude/magrini2014_extract.txt`. No volver a afirmar qué hace un paper sin
abrirlo.

**Lo que sigue en pie como aporte** está listado en `.claude/paper-domain.md`:
cuantificación global, estructura direccional, fallo silencioso de la correlación,
gating por condicionamiento, y todo el régimen de actuación en velocidad.

## Las tres contribuciones, y su experimento

1. **Para `n<6` decide el modelo de contacto, no el rango del Jacobiano.**
   El inverso 6-D min-norm está indeterminado y reparte la fuerza como momento
   espurio: pierde **50.5%** en promedio sobre 2197 configuraciones (peor caso
   99.6%) y **falla en silencio**, con correlación 0.997 contra verdad. El inverso
   de contacto puntual es exacto si `rank(Jv)=3`, que se cumple en el 100%.
   Contra MuJoCo: RMSE **2.227 → 0.157 N**, factor 14.
   → `observability_map.py`, `test_direction_sweep.py`, `mj_wrench_test.c`

2. **El modelo dinámico lo necesita el ESTIMADOR, no el controlador.**
   De las tres formulaciones posibles bajo actuación en velocidad, la dinámica
   (V1) es mal planteada sin identificar el lazo del servo y falla en todos los
   solves; la cinemática (V0) no usa `M`, `h` ni `kv` y **iguala a la de par a
   1/25 del costo**.
   → `generate_arm_vel_ocp.py`, `test_vel_vs_torque.py`

3. **Compliance en dos regímenes, separados por la capacidad del actuador.**
   El brazo ejerce 3.9-5.9 N en su peor dirección; el guiado manual pide 5-15 N.
   Por debajo la compliance es sintética, por encima el actuador satura y es
   física. Transición medida: `ee/ref` pasa de ~1 a ~1.9 y la saturación de 0% a
   100% entre 5 y 6 N.
   → `reproduce_paper_tables.py`

Más: la cota `‖δf‖ ≤ ‖δτ‖/σ_min(Jv)` **verificada en la planta MuJoCo** con ruido
inyectado — pendiente −0.872, R² 0.980. → `hw/test_conditioning_law.py`

---

## ⛔ Correcciones que NO hay que deshacer

Cada una nació de descubrir que algo no reproducía. Si algo parece raro, leer esto
antes de "arreglarlo".

**1. El modelo va en SX, no en MX.** `ca.solve` sobre MX emite un solver QR
NUMÉRICO por evaluación, con derivadas por diferenciación implícita. Costaba
**101.6 ms/solve, de los cuales 101.5 era linealización**; el QP nunca fue el
cuello. En SX: 2.5 ms. Equivalencia verificada a `2.2e-15` N·m sobre 250 pasos.

**2. `p[8:22]` está MUERTO en el OCP.** Al migrar de `EXTERNAL` a `NONLINEAR_LS`
los pesos dejaron de viajar en los parámetros: van en `ocp.cost.W` y se cambian
con `cost_set`. Escribirlos en `p` no hace nada y no avisa. Los nodos ROS ya usan
`cost_set`.

**3. El estimador correcto es `estimate_force()`, no `estimate_wrench()`.**
El segundo resuelve 6 incógnitas desde 4 medidas. Ver contribución 1.

**4. Los dual quaternions se SACARON del paper.** Dos motivos: no sostenían
ninguna contribución (la ablación daba 0.8-2.8%), y el paper llamaba a la ec. (6)
*"the se(3) logarithm"* y **no lo es** — verificado contra el log matricial:
`φ` difiere por factor 2 y `ρ` por factores no constantes. `dq_math.ln_dual`
**sigue en el repo y sigue mal nombrado**; lo usa el código pero ya no lo describe
el paper.

**5. Las poses de calibración salen de `hw/select_poses.py`, con MuJoCo de filtro.**
Las elegidas analíticamente tenían **tres de seis dentro del piso** (`ncon` 10, 10,
54). El banco analítico no tiene geometría y no puede verlo. También hay 0.15 rad
de margen contra topes: sin él, la junta se apoya y el actuador no ve la carga
(`kt` daba 6275% de error).

---

## 🪤 Trampas que picaron dos veces

**La línea que imprime el veredicto debe depender del dato.** Pasó dos veces:
`test_e4_compare.py` imprimía *"E4: confirmado — empate"* hardcodeado, y un test
mío imprimió *"son el mismo mapa"* cuando la diferencia era 1.31. Se detecta
leyendo el número, no la conclusión.

**Una métrica agregada puede estar sana con el resultado mal.** Correlación 0.997
convivía con 2.227 N de error: mide forma, no ganancia.

**Un test que no puede fallar tampoco puede pasar.** El MiL correlacionaba contra
una constante: `nan > 0.8` es False siempre.

**Cambiar una constante obliga a recalcular lo que dependía de ella.** La tabla de
inyección de fallas quedó con las poses viejas (las del piso) hasta la auditoría.
Por eso existe `reproduce_paper_tables.py`.

**Cada número es UNA corrida hasta que se repite.** El timing varía 17% entre
corridas del mismo binario.

---

## Reproducir los números del paper

```bash
cd ocp_generation
python3 arm_dynamics.py            # modelo vs MuJoCo: 8.78e-11 / 2.73e-09
python3 arm_kinematics.py          # FK: 6.93e-13 m
python3 observability_map.py       # barrido 2197 configs
python3 test_direction_sweep.py    # 12000 direcciones, corr 0.9993
python3 test_interaction_mil.py    # lazo compliant, dos estimadores
python3 test_noise_montecarlo.py   # ruido de servo + gating
python3 test_vel_vs_torque.py      # T vs V1 vs V0
python3 test_e4bis.py              # ablación de la métrica
python3 reproduce_paper_tables.py  # las 5 tablas que antes eran inline
python3 make_figures.py            # figuras 3 y 4
python3 make_arch_figure.py        # figura 1

cd ../hw
python3 test_conditioning_law.py   # ley 1/sigma_min en planta MuJoCo
python3 validate_procedures.py     # S1/S2 contra MuJoCo
python3 select_poses.py            # poses con filtro de colisión
```

Tests en C (MuJoCo). La escena la prepara `hw/mj_arm.ensure_scene()`:

```bash
MJ=~/mujoco_ws/mujoco-3.4.0
gcc mj_wrench_test.c -I$MJ/include -I$ACADOS_SOURCE_DIR/include \
  -I$ACADOS_SOURCE_DIR/include/{acados,blasfeo/include,hpipm/include} \
  -I../c_generated_code_arm_dqnmpc -L$MJ/lib -L../c_generated_code_arm_dqnmpc \
  -L$ACADOS_SOURCE_DIR/lib -lmujoco -lacados_ocp_solver_arm4dof_dqnmpc \
  -lacados -lhpipm -lblasfeo -lm -o /tmp/mj_wrench_test
```

Regenerar solvers: `python3 generate_arm_dqnmpc_ocp.py`, `generate_arm_vel_ocp.py`,
`generate_arm_decoupled_nls_ocp.py`.

---

## Hardware — nada probado todavía

`hw/README.md` tiene el plan completo. Lo esencial:

**Lo único que hace falta correr para el paper: S3 parte A.**
`hw/s3_conditioning.py --port /dev/ttyUSB0`. Par deshabilitado, sin pesa, sin lazo
de control, bus a 10 Hz alcanza. Cuatro posturas a mano. Una tarde.

Convierte *"todo es simulación"* en *"la predicción central se verificó en
hardware"*.

⚠️ **Puede FALLAR.** Pendiente ~0 significa que la Sec. IV del paper está mal.
Está diseñado para decirlo fuerte, no para disimularlo.

⚠️ **La tabla de control de `hw/dxl_io.py` NO está verificada** contra el servo
real. S0 la sondea sin habilitar par. Correr S0 antes de mover un motor.

⚠️ La junta de la base **no se puede calibrar colgando pesas**: fuerza vertical no
hace momento sobre eje vertical. Necesita tiro horizontal.

---

## Mapa de documentación

| archivo | qué tiene |
|---|---|
| **`CLAUDE.md`** | este archivo: estado, correcciones, trampas |
| `paper/SUBMISSION.md` | checklist de envío, auditoría de reproducibilidad, defensas |
| `paper/README.md` | cómo compilar, de dónde sale cada número |
| `hw/README.md` | plan de bring-up, auditoría del banco MuJoCo, poder de detección |
| `PLAN_MECHATRONICS.md` | **histórico.** El venue cambió a Access; conserva las correcciones de E1/E4 con su contexto |
| `.claude/paper-domain.md` | competidores, deltas, riesgos de revisor |
| `.claude/prose-ledger.txt` | términos vedados; lo consume el linter |

---

## Pendientes, en orden

1. **Autoría** — decisión del autor. Los coautores actuales se replicaron del otro
   paper de Access y **eso no se hereda**.
2. **S3 parte A** en hardware.
3. Corpus de referencia en `paper_refs/src_corpus/` para el barrido léxico del
   revisor (`lint_prose.py --sweep`). Hoy solo cubre ortografía y AI-tells.
4. Repetir la Sec. VII con V0 en vez de T. Declarado equivalente y demostrado en
   el lazo compliant, pero un revisor puede pedirlo.

**Lo que sacaría si hay que acortar:** Sec. VII-F (compliant vs rígido, N=6) y la
ablación de la métrica. No son contribuciones, respaldan. Ahí hay página y media.
