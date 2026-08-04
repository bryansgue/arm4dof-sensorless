# CLAUDE.md — arm4dof_dqnmpc

Estimación de fuerza de contacto **sin sensor** y control compliant para un brazo
4DOF de servos comerciales (Dynamixel MX-28R). Paper listo para IEEE Access.

> **Entrada rápida:** el paper está en `paper/` (18 pág, compila limpio). Lo único
> que bloquea el envío es confirmar la **afiliación**. Ver `paper/SUBMISSION.md`,
> y `paper/RESPONSE.md` para la respuesta a la segunda revisión del asesor.

---

## Estado (04/08/2026)

| | |
|---|---|
| paper | `paper/main.tex`, **18 pág**, 12 tablas, IEEE Access, 0 errores, lint clean, **33 refs** |
| bloqueo | **la afiliación** (decisión del autor). Autoría única ya aplicada. ORCID en el portal, fotos en cámara lista |
| hardware | **nada probado en el brazo real.** `hw/` listo, procedimientos validados |
| rama | `master` (mergeado desde `sensorless-contact-model`), sin pushear |

⚠️ **El paper declara que TODO es simulación**, en abstract, introducción, alcance
y limitaciones. No suavizar eso: es el punto que un revisor va a atacar y la
defensa es que está declarado.

---

## ⛔ SEGUNDA REVISIÓN DEL ASESOR (04/08/2026) — Major Revision

**Respuesta punto por punto: `paper/RESPONSE.md`.** Todo lo verificable de esa
revisión se comprobó contra el código y **era correcto**. Resumen de lo que cambió:

| hallazgo | estado |
|---|---|
| **La norma del wrench no es dimensionalmente homogénea.** `‖f‖²+‖m‖²` suma N con N·m: presupone una longitud, y la pseudoinversa sin ponderar la fija en **1 m sin declararlo**, con el brazo midiendo 0.233 m de alcance medio | ✅ Sec. IV-C nueva: formulación ponderada, punto de referencia declarado, tabla de sensibilidad, Proposición 2 |
| validación solo simulada | ⛔ abierto, requiere el brazo. Lenguaje calibrado |
| el gating no informaba qué fracción del espacio conserva | ✅ columna nueva: el umbral 0.06 retiene **31.4 %** |
| Tablas 12 y 13 discrepaban sin gating | ✅ era N=2000 vs N=1500; igualadas |
| faltaba variabilidad entre semillas | ✅ 5 semillas, intervalos reportados |
| *"independent physics engine"* y el 1e-10 se leían como validación física | ✅ reescrito en las 4 apariciones |
| *"the noise the MX-28R actually delivers"* con efectos simulados | ✅ ahora dice que es un modelo desde hoja de datos |
| 15 tablas diluyen el aporte | ✅ **16 → 12** |
| README y checklist decían 9 y 14 páginas, y tres autores | ✅ sincronizados |

### Lo que SOBREVIVE a la métrica, y es el resultado nuevo

⚠️ **No es todo o nada.** Verificado numéricamente sobre 2197 configuraciones y 9
valores de ℓ_c (`ocp_generation/metric_sensitivity.py`):

| | |
|---|---|
| **invariante** | el conjunto de fuerzas recuperadas exacto (probado analíticamente) |
| **invariante** | los autovectores de `M(ℓ_c)`: QUÉ direcciones se pierden. Ángulo principal máximo **2.6e-6 grados** |
| **invariante** | el peor caso, 99.6 % para todo ℓ_c ≥ 0.15 m |
| **invariante** | `‖M f − f‖ = (1−λ_min)·|cos θ|`, exacta POR POSE |
| **NO invariante** | los autovalores, el 50.5 % (→ 38.0 % a la escala del brazo), el "100 % de configuraciones con λ_min<0.10" (→ 6 % a ℓ_c=0.05) |

✅ **Y quedó DEMOSTRADA, no observada (ronda 3).** Con la junta 1 sobre `e_z` y
las juntas 2-4 sobre un eje horizontal común `a`:

    ker Jw = {x_1 = 0, x_2+x_3+x_4 = 0}   ->   P = Jv(ker Jw) = a^⊥   si dim P = 2
    toda f en P se recupera exacta para todo l_c   ->   M(l_c) = I − (1−λ) a aᵀ

Los autovectores no dependen de `ℓ_c` **porque los tres ejes de pitch son
paralelos**. Para un Jacobiano genérico es FALSO, y el paper lo dice.

⚠️ La hipótesis `dim Jv(ker Jw)=2` **es** el espectro `{λ,1,1}`: coinciden en el
100 % de las 2197 configuraciones, y ambas valen en el 92.3 %. El resto son las
singulares del borde. El 92 % dejó de ser un número sin explicación.

⚠️ **El rango del estimador ponderado es `col(W⁻²J)`, NO `W⁻¹col(J)`.** El error
estaba en la primera versión de la demostración. Corregido, la parte (i) sale en
una línea: `[f;0] ∈ col(W⁻²J) ⟺ Jw x = 0 y Jv x = f`, donde `ℓ_c` no puede
aparecer.

⚠️ Si se verifica numéricamente, hacerlo sobre **autoespacios**: la base del
degenerado es arbitraria y comparar autovectores uno a uno da 90°, artefacto.

⚠️ **La correlación 0.9993 está favorecida por la métrica**, y el paper ahora lo
dice. Por pose es exactamente 1.000000 a cualquier ℓ_c (es una identidad
algebraica); agrupada mide cuánto se parecen las pendientes `1−λ_min` de las seis
poses. A ℓ_c=0.05 m las mismas 12000 direcciones dan **0.4475**.

⚠️ El nominal quedó en **ℓ_c = 1 m**, llamado *unweighted-SI baseline* y no
"métrica natural". El asesor lo respaldó: cambiarlo a 0.233 m haría que el paper
dejara de cuantificar la práctica que critica.

⚠️ **Los residuos numéricos del manuscrito tienen que ser los de la sección G de
`metric_sensitivity.py`, y de ninguna otra prueba.** Hubo una versión con `9e-8` y
`9e-12`, que salían de 400 configuraciones al azar SIN filtrar por
`dim Jv(ker Jw)=2`: con las casi singulares adentro el residuo se degrada seis
órdenes. Los correctos son 6e-14 sobre el plano y 8e-15 sobre el eje de pitch.

⚠️ El aviso `A NumPy version >=1.17.3 and <1.25.0 is required for this version of
SciPy` es de **scipy 1.8.0**, y es cosmético: **ningún script del repo importa
scipy**, entra por `acados_template`. Se cierra con `scipy>=1.11`, que es
preferible a bajar numpy porque no toca la versión con la que se generó todo.

⚠️ `requirements.txt` lista **dos entornos y no son el mismo**: el ORIGINAL, donde
se produjeron los números (scipy 1.8.0, con aviso), y el RECOMENDADO para
reproducir limpio (scipy>=1.11). No colapsarlos: el primero es un registro, el
segundo una instrucción. Y ahí está fijado el commit de acados
(`e0759960cae76e4f2e8177cf2866a6ab088f4e86`, `v0.5.4-6`), que es la dependencia más
sensible del repo porque el código C generado cambia entre versiones.

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

## Reencuadre acordado con el asesor (02/08/2026)

El paquete alcanza para Access **como contribución integrada, no como cinco
novedades independientes**. El marco:

> *A quantitative design framework for point-contact force estimation on low-DoF,
> velocity-actuated manipulators.*

Y una sola pregunta aplicada:

> *Given a low-DoF arm with velocity-controlled smart servos, what contact force
> is identifiable, with what accuracy, and how can it be used for compliant
> interaction?*

**Jerarquía. No son tres contribuciones al mismo nivel:**

| | qué es | rol |
|---|---|---|
| **1** | caracterización cuantitativa y geométrica de la inversión bajo modelo de contacto conocido | **principal** |
| **2** | arquitectura de estimación y compliance para brazos low-DoF actuados en velocidad | **co-principal** |
| **3** | el NMPC | **demostrador de integración**, no contribución matemática |

Dentro del marco 1: `P_ff` explica y **predice** la estructura direccional; el
barrido cuantifica magnitud y prevalencia; `σ_min(Jv)` lo vuelve criterio
operativo; el estudio de correlación muestra por qué una validación aparentemente
buena esconde error grande.

⚠️ **El 50.5% NO es contribución por sí solo.** Pertenece a este brazo, este
espacio de trabajo y esta distribución de fuerzas. Vale cuando `P_ff` lo
**explica y predice**, no cuando solo se mide. Escribirlo así.

⚠️ **La contribución 2 no puede sostenerse en *"un controlador cinemático no
necesita `M` ni `h`"***, que aislado es conocido. Lo defendible es la **cadena
completa**:

```
corriente del servo → τ_act → τ_ext → f̂ → q̇_cmd
```

y demostrar que: el lazo interno no aparece en la reconstrucción si el par
aplicado se mide; su ganancia no hace falta identificarla; el modelo dinámico
queda **confinado al observador**; la aceleración aporta poco en este régimen; la
implementación cinemática conserva el comportamiento compliant; y todo eso
sobrevive a corriente real, fricción, stiction y cuantización.

**Tres condiciones para que alcance**, y ninguna está cumplida hoy:

1. la revisión bibliográfica confirma que nadie reunió antes esa caracterización;
2. el hardware valida **la predicción direccional, el efecto del condicionamiento
   y la viabilidad del par inferido desde corriente**;
3. las afirmaciones se calibran: el gating y el fallo de correlación son
   *consecuencias útiles*, no "métodos nuevos".

**Compresión acordada:** conservar `P_ff`, barrido direccional, workspace,
hardware, arquitectura de velocidad y la comparación cinemático vs dinámico.
Reducir dual quaternions, ablaciones de error de pose y comparaciones de control
que no sostengan las dos contribuciones. **Eliminar el lenguaje de *"exact
recovery"* fuera del modelo ideal**: en hardware se habla de consistencia, error y
sensibilidad.

## Evidencia por contribución

**C1** — el inverso 6-D min-norm está indeterminado y reparte la fuerza como
momento espurio: pierde **50.5%** en promedio sobre 2197 configuraciones bajo la
métrica sin ponderar de la práctica estándar (**38.0%** a la escala del brazo,
peor caso 99.6% con las dos) y **falla en silencio**, con correlación 0.997 contra
verdad. El de contacto puntual es exacto si `rank(Jv)=3`, que se cumple en el 100%,
y **no admite elección de métrica**, que es un argumento a su favor independiente
de la exactitud. Contra MuJoCo: RMSE **2.227 → 0.157 N**.
→ `observability_map.py`, `test_direction_sweep.py`, `metric_sensitivity.py`,
`mj_wrench_test.c`

Más: la cota `‖δf‖ ≤ ‖δτ‖/σ_min(Jv)` **verificada en la planta MuJoCo** con ruido
inyectado — pendiente −0.872, R² 0.980. → `hw/test_conditioning_law.py`

**C2** — de las tres formulaciones bajo actuación en velocidad, la dinámica (V1)
es mal planteada sin identificar el lazo del servo y falla en todos los solves; la
cinemática (V0) no usa `M`, `h` ni `kv` y **iguala a la de par a 1/25 del costo**.
Compliance en dos regímenes separados por la capacidad del actuador: el brazo
ejerce 3.9-5.9 N en su peor dirección, el guiado manual pide 5-15 N; `ee/ref` pasa
de ~1.34 a ~2.65 y la saturación de 0% a >97% **entre 6 y 8 N**, que es donde cae
la capacidad estática de esa pose y esa dirección (6.53 N).
→ `generate_arm_vel_ocp.py`, `test_vel_vs_torque.py`, `test_compliance_regimes.py`

⚠️ Esos números de régimen **cambiaron el 04/08/2026**: la tabla publicada no la
reproducía ningún script. Y en la ronda 3 volvieron a cambiar, porque la meseta de
0.9 s no era estado estacionario: ahora dura 3.0 s = seis constantes de tiempo y
el comandado coincide con `‖f‖/(D·k)` a 0.3 %. La fila de 15 N se eliminó y el
script exige `status == 0` por fila. Ver `paper/RESPONSE.md`.

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

**Una tabla sin script guardado es una tabla sin verificar, y el `reproduce_` no
garantiza nada por existir.** La auditoría del 01/08/2026 creó
`reproduce_paper_tables.py` para cerrar exactamente este defecto, y `tab:regimes`
se le escapó: al escribirle el script (04/08/2026) **los números no reprodujeron**
y la transición entre regímenes se movió de 5-6 N a 6-8 N. La forma de detectarlo
no fue leer el código sino **mapear cada tabla del PDF a un script y no encontrar
uno**.

**Comparar contra el número que corresponde, no contra el que está a mano.** El
texto validaba la transición de esa tabla contra la mediana del espacio de trabajo
en la PEOR dirección (5.88 N), cuando el experimento es en UNA pose y UNA
dirección, cuya capacidad estática es 6.53 N. La mediana acota por abajo, no
predice.

**Una ventana de medición tiene que ser más larga que la constante de tiempo del
sistema.** Promediar el tramo plano completo subestimaba el desplazamiento un 35 %
porque la admitancia tiene τ = 1/k_return = 0.5 s y el tramo dura 0.9 s. Lo delató
comparar contra el valor analítico `‖f‖/(D_trans·k_return)`.

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
python3 metric_sensitivity.py      # sensibilidad a l_c + invariancia (NUEVO)
python3 test_compliance_regimes.py # los dos regímenes, con saturación (NUEVO)
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

**Lo que hace falta correr para el paper: S3, LAS DOS PARTES.**

```bash
python3 hw/s3_conditioning.py --port /dev/ttyUSB0 --mass 0.2 --pull 2.0 \
        --pull-tol 0.1 --repeats 3
```

Par deshabilitado, sin lazo de control, bus a 10 Hz alcanza, cuatro posturas a
mano. No hace falta sensor F/T: una masa (~0.2 kg) y una balanza de equipaje.

**Lo que el script registra**, para que el ensayo sea auditable y no una anécdota:
dirección y punto de aplicación (posición del efector por FK, que es el punto de
contacto que el estimador supone conocido), incertidumbre del instrumento,
**corriente en reposo** y su ruido por postura, y **repetibilidad** entre ensayos
repetidos. Con eso calcula un **piso de resolución** = incertidumbre +
repetibilidad, y se niega a declarar una separación entre estimadores menor que
ese piso.

| parte | qué valida | necesita |
|---|---|---|
| **A** | la ley `‖δf‖ ≤ ‖δτ‖/σ_min` — amplificación de ruido | nada colgado |
| **B** | exactitud **y mal-atribución fuerza–momento** | carga conocida, **dos direcciones** |

⚠️ **Correr solo la parte A NO ALCANZA** — corregido tras la revisión del asesor.
Un ensayo sin carga conocida valida ruido, deriva y dependencia postural, pero no
puede validar ni la exactitud de la fuerza ni la mal-atribución, que es la tesis
central. La parte B corre **los dos estimadores sobre el mismo residuo** y reporta
el momento espurio del 6-D, que debería ser cero.

⚠️ **Dos direcciones, no una.** La pesa colgada deja la junta de la base sin
excitar (fuerza vertical no hace momento sobre eje vertical) y una sola dirección
no distingue un estimador sesgado de uno que solo acierta en vertical. La segunda
es tiro horizontal con balanza de equipaje: `--pull 2.0 --pull-axis x`.

⚠️ **Puede FALLAR, y las dos partes están diseñadas para decirlo fuerte.**
Pendiente ~0 en A significa que la Sec. IV está mal. Empate entre estimadores en B
significa que la contribución principal no se sostiene en hardware.

⚠️ **El veredicto de B está gateado por tres cosas, y calla si alguna falla:**

| reja | por qué | umbral |
|---|---|---|
| `dJ/J` | el brazo se hunde ⇒ el Jacobiano cambió entre las dos medidas y el número mezcla el efecto con el hundimiento | 3% |
| `qd_max` | si el brazo se mueve, `τ` lleva inercia y no compara contra la lectura del instrumento | 0.02 rad/s |
| deriva | la carga no se sostuvo constante, o la stiction se reacomodó | 10% de `‖δτ‖` |

Por eso la carga se aplica **cuasi-estáticamente**. En el ensayo en seco contra
MuJoCo da 8/8 contaminadas por `dJ/J` —el banco tiene lazo de velocidad blando—;
el modo posición del MX-28 es mucho más rígido. Sostener rígido o bajar la carga.

⚠️ **En simulación la repetibilidad y el ruido de reposo salen 0.000 por
construcción** (MuJoCo es determinista). Esos dos números solo significan algo en
hardware; el script lo avisa al arrancar en modo `--sim`.

⚠️ **La tabla de control de `hw/dxl_io.py` NO está verificada** contra el servo
real. S0 la sondea sin habilitar par. Correr S0 antes de mover un motor.

⚠️ La junta de la base **no se puede calibrar colgando pesas**: fuerza vertical no
hace momento sobre eje vertical. Necesita tiro horizontal.

---

## Mapa de documentación

| archivo | qué tiene |
|---|---|
| **`CLAUDE.md`** | este archivo: estado, correcciones, trampas |
| `paper/RESPONSE.md` | **respuesta a la revisión del asesor del 04/08/2026** |
| `paper/SUBMISSION.md` | checklist de envío, auditoría de reproducibilidad, defensas |
| `paper/README.md` | cómo compilar, de dónde sale cada número |
| `hw/README.md` | plan de bring-up, auditoría del banco MuJoCo, poder de detección |
| `PLAN_MECHATRONICS.md` | **histórico.** El venue cambió a Access; conserva las correcciones de E1/E4 con su contexto |
| `.claude/paper-domain.md` | competidores, deltas, riesgos de revisor |
| `.claude/prose-ledger.txt` | términos vedados; lo consume el linter |

---

## Pendientes, en orden

⛔ **LO ÚNICO QUE BLOQUEA EL ENVÍO HOY: la afiliación.** Quedó LASER/UFPB, marcada
con comentario en `main.tex:57`. Es decisión del autor.

1. ✅ **Posicionamiento contra Magrini** reescrito (commit `68b104c`).
2. ✅ **Jerarquía acordada aplicada** (C1 principal, C2 co-principal, NMPC
   demostrador) y comprimido.
3. **S3, las dos partes**, en hardware. Requiere el brazo montado. Es lo que el
   asesor estima que mueve la aceptación de 40-50 % a 65-75 %.
4. ✅ **Búsqueda bibliográfica — HECHA (03/08/2026).** `refs.bib` 14 → **33**,
   metadatos verificados contra Crossref/Semantic Scholar, cero referencias sin
   citar, más el barrido de citas hacia adelante (130 revisadas). Aparecieron
   **cinco trabajos que faltaban**, los cinco ya citados con su delta escrito:
   `wong2024sensorobs` (T-RO 2024, direcciones ciegas por configuración),
   `lu2023configopt` (condicionamiento como criterio de postura),
   `dewolde2024current` (compliance sin F/T comandando corriente),
   `wahrburg2018motorcurrent` (corriente → wrench cartesiano) y `yen2019virtual`
   (contacto puntual como default en hardware barato). Los tres primeros **leídos
   enteros**. ⚠️ De Wahrburg solo se leyó el título — se cita a ese nivel y nada
   más. Registro completo en `.claude/lit-review-2026-08.md`.
5. ✅ **Autoría única** aplicada, y correcciones puntuales del asesor (abstract,
   *under-actuated* → *low-DoF*, 0.99 N vs 0.264 N desambiguado). ⛔ Falta
   **confirmar la afiliación**.
6. ✅ **Sec. VII repetida con V0 en vez de T** — es `sec:res-model`, Tabla IX: el
   lazo compliant completo con controlador cinemático, estimador idéntico.
7. Corpus de referencia en `paper_refs/src_corpus/` para el barrido léxico
   (`lint_prose.py --sweep`). Hoy solo cubre ortografía y AI-tells.
8. Conseguir el texto de `wahrburg2018motorcurrent` si alguna vez hace falta
   afirmar algo de su método. Hoy se cita al nivel del título.

9. ✅ **Métrica del wrench** (04/08/2026): Sec. IV-C, Proposición 2, tabla de
   sensibilidad, `metric_sensitivity.py`. ⚠️ El nominal ℓ_c = 1 m es decisión
   abierta.
10. ✅ **`tab:regimes` ahora se reproduce** (`test_compliance_regimes.py`), y sus
   números cambiaron. ⚠️ Confirmar con el asesor: se cambió una tabla de
   resultados contra una reimplementación, porque el código original no existía.

⚠️ **Ya no queda material fácil de cortar.** Se sacaron la ablación de la métrica,
"compliant vs rígido" (N=6), la nota de implementación en tiempo real, y en la
ronda del 04/08/2026 cuatro tablas más (`timing`, `validation`, `threedirs`,
`velocity`, sus números pasaron a prosa) y dos subsecciones del bloque del
controlador. **16 → 12 tablas.** Aun así el material de la métrica cuesta ~2
páginas netas y el PDF quedó en 17. Lo que queda sostiene C1 o C2: recortar más
cuesta evidencia, y los candidatos están listados en `paper/RESPONSE.md`.

⚠️ **La razón de costo V0 vs T tiene DOS valores y los dos son correctos:** ~27×
en el test de regulación (Tabla IV) y **25×** en el lazo compliant completo
(Tabla IX). El del resumen y las conclusiones es el segundo. Son experimentos
distintos; no "unificarlos".
