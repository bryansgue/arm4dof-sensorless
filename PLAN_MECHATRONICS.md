> ⚠️ **DOCUMENTO HISTÓRICO.** El venue cambió a **IEEE Access** y el paper ya está
> escrito (`paper/`). Este archivo se conserva porque contiene las correcciones de
> E1 y E4 con su contexto, que explican por qué varios resultados previos no
> reproducían. **Para el estado actual, leer [`CLAUDE.md`](CLAUDE.md).**

# PLAN — Paper

> ⚠️ **VENUE CAMBIADO el 01/08/2026: va a IEEE ACCESS.** Decisión del usuario,
> explícita. Esto **revierte** la línea de abajo, que decía lo contrario y quedaba
> de una decisión previa. Manuscrito en `paper/main.tex`, formato `ieeeaccess.cls`
> tomado de `~/python/Time_optimal_planing-NMPC/ACCESS_latex`.
>
> [histórico] Target: **Mechatronics** (Elsevier, IF ~3.3). Red de respaldo:
> IJCAS / Robotica. NO MDPI, NO IEEE Access (estigma).

## Framing (cómo se vende)

Sistema mecatrónico integrado de **control de interacción física SIN sensor de fuerza**
para un manipulador de **bajo costo** (servos comerciales Dynamixel MX-28R). Estima el
wrench externo desde la dinámica + encoders (**observability-aware**, brazo sub-actuado
rank 4<6) y lo usa para control compliant DQ-NMPC, validado en **hardware real**.

- **Estrella:** estimación de wrench sin sensor + análisis de observabilidad (sub-actuado).
- **DQ:** formulación unificada (herramienta, no el claim de novedad — E4 dio empate, honesto).
- **Admitancia:** la aplicación (no la contribución — es madura).
- **Valor mechatronics:** sin F/T sensor → ahorro; servos baratos → democratización; HW real.

## Por qué Mechatronics y no RA-L/ISA

- RA-L/ISA exigen novedad fuerte → el DQ es flaco (E4 empate) → riesgo de "incremental".
- Mechatronics premia **sistema sólido + sin sensor + barato + HW + validación** = lo que SÍ tenemos.
- El E4 empate y el límite 4DOF NO lastiman ahí; la honestidad suma.

## ⛔ CORRECCIÓN IMPORTANTE (01/08/2026) — la sección de abajo estaba MAL

Lo que sigue afirmaba que el brazo tiene una **dirección de fuerza ciega** en el
100% del espacio de trabajo, y que eso era un límite estructural del 4DOF. **Es
falso.** Verificado:

```
empuje f=[0,6,0] en q_task, la supuesta direccion ciega:
  tau = Jvᵀ f = [1.2867, 0, 0]   <- NO es cero. Produce 1.29 N.m en la base.
  rank(Jv) = 3   en el 100% de 2197 configuraciones
  estimador de FUERZA PURA:  f = [0, 6, 0]   EXACTO
```

El empuje **no es invisible**: el torque de junta está. Lo que fallaba era el
**estimador**, que resuelve `Jᵀ F = tau` con F ∈ R⁶ desde 4 medidas. Está
indeterminado, y la solución de norma mínima **reparte la fuerza como momento**
(0.264 N de fuerza + 1.230 N·m de momento en ese ejemplo).

Con el modelo de contacto correcto (fuerza pura en punto conocido) el sistema es
`Jvᵀ f = tau`, 4 ecuaciones y 3 incógnitas: **sobre-determinado**, y la fuerza
sale exacta.

| error estimando fuerza unitaria, 2197 configs | media | max |
|---|---|---|
| WRENCH-6D min-norm (lo que usaba el código) | **0.5046** | 0.9961 |
| FUERZA-3D contacto puntual | **3.6e-15** | 1.4e-12 |

**El 6D pierde la mitad de la fuerza en promedio.** En MuJoCo: RMSE 2.227 N contra
0.157 N, factor 14.

**El límite real es CONDICIONAMIENTO, no observabilidad:** `σ_min(Jv)` mediana
0.0496, mínimo 7.6e-5; `cond(Jv)` mediana 6.7, p95 29.4.

⇒ Es mejor resultado que el equivocado: no es una disculpa sobre un brazo barato,
es un error de método en el observador estándar, y es accionable. Es la tesis del
paper (`paper/main.tex`). El código corregido está en `wrench_estimator.py`
(`estimate_force`) y `observability_map.py`.

⚠️ Lo que SÍ sigue en pie de la sección de abajo: `trace(P)=4`, el reparto de
`P_ff`, y que el proyector hay que calcularlo por SVD. Pero describen el
comportamiento del **estimador malo**, no del mecanismo.

⚠️ Y el costo del modelo correcto, que hay que declarar: exige **conocer el punto
de contacto**. Vale para hand-guiding en el efector; no vale para contacto en
cualquier punto del eslabón. Si el contacto además aplica momento (objeto
agarrado), vuelve la ambigüedad y hay que usar el 6D.

## [SUPERADO — leer la corrección de arriba] Observabilidad del wrench 6D

`observability_map.py`. Es lo que convierte el límite 4DOF de disculpa en análisis.

**Invariante estructural.** El estimador recupera `F_est = P F` con
`P = J(JᵀJ)⁻¹Jᵀ` el proyector ORTOGONAL sobre `col(J)`. Como `P` es proyector
ortogonal de rango n, **`trace(P) = 4` SIEMPRE** (verificado a 4.000000). Corolario
incómodo: promediando sobre direcciones uniformes de R⁶ la fracción observable es
`4/6` CONSTANTE, independiente de q. **Cualquier "observabilidad promedio" no
informa nada** — la estructura está en CÓMO se reparte la pérdida.

**Fuerzas puras** (un empuje humano): la parte recuperada es `P_ff f`, con `P_ff`
el bloque 3×3 superior-izquierdo. Autovalores en [0,1]; `rho(u) = uᵀ P_ff u` es la
fracción del empuje en dirección `u` que el brazo SIENTE.

| pose | lam_min | lam_med | lam_max |
|---|---|---|---|
| q_task | 0.0440 | 1.0000 | 1.0000 |
| Q_TGT | 0.0039 | 1.0000 | 1.0000 |
| X0 | 0.0346 | 1.0000 | 1.0000 |

**Barrido 13³ = 2197 configuraciones:** `lam_min` media 0.0232, mediana 0.0172,
p5 0.0002, min 0.0. `lam_max = 1.0000` en todas. **El 100% del espacio de trabajo
tiene una dirección de empuje casi ciega.** rank(J)=4 en 92.3%; el 7.7% restante es
SINGULAR, y ahí el subespacio observable se achica todavía más.

⚠️ En singularidad `J(JᵀJ)⁻¹Jᵀ` explota. El proyector se calcula por **SVD**
(`P = U_r U_rᵀ`), que es correcto ahí también. No es detalle: ese caso es el 7.7%.

**Explicación mecánica (y es lo que lo hace paper, no tabla).** Las juntas 2-4 son
de cabeceo COPLANARES: solo resisten carga en el plano del brazo. La componente
normal a ese plano llega a los actuadores por UN solo camino, el momento sobre la
junta de base, `|p_ee × f|`. Ese brazo de palanca es el offset horizontal del
efector respecto del eje de la base, y **se anula cuando el brazo se pliega sobre su
propia base**. Predicción confirmada: el menor offset (0.064 m) tiene el menor
`lam_min` (0.0039); el mayor offset (0.215 m) el mayor (0.0440).

⇒ No se arregla con mejor observador. Solo con una junta cuyo eje salga del plano,
o restringiendo la tarea al plano observable.

**Consecuencia de diseño, todavía NO implementada:** la admitancia debe restringirse
al subespacio observable. Ceder en una dirección ciega es compliance en lazo abierto
guiada por ruido del estimador.

## ⚠️ E1 / MiL de interacción (01/08/2026) — el estimador está BIEN, el test mide mal

`test_interaction_mil.py` imprime **REVISAR** (`corr=nan`, `|F_y|` real ~6N est ~0.4N).
Verificado que **no** es regresión: el código MX previo da exactamente lo mismo.

**El estimador no tiene nada malo.** Con cantidades instantáneas exactas recupera la
proyección observable a **7.11e-15**, y la discretización del banco (5 subpasos, q̈ por
diferencia finita, τ promediado) **no aporta error**: da el mismo número.

**Lo que pasa es física, y es el claim del paper.** El empuje elegido —`[0, 6, −2]` N en
`q_task = [0, 1.2, −0.9, 0.4]`— cae casi entero en el núcleo de `Jᵀ`:

```
|F_real| = 6.325 N    |F_obs| = 2.361 N    |NO observable| = 5.867 N   (93%)
componente y:  real 6.000  ->  observable 0.264
```

El test compara `F_est` contra los **6 N aplicados** en vez de contra los **0.264 N
observables**, o sea contra una cantidad que el brazo no puede medir por construcción
(`rank(J)=4<6`, lo que `wrench_estimator.py:8-10` ya documenta). Los "0.4 N" son el
resultado CORRECTO.

⚠️ Y hay un segundo defecto, independiente: `test_interaction_mil.py:88` correlaciona
contra `Ftrue[push,1]`, que es **constante 6.0** en toda la ventana (`F_hand`, línea 51).
Correlación contra una constante es `nan`, y el criterio es `fy_corr > 0.8` ⇒ `nan > 0.8`
es False. **Este test no puede pasar nunca, con ningún estimador.**

**Qué hacer (y es mejor paper que lo que había):**
1. Comparar `F_est` contra `project_observable(q, F_true)`, no contra `F_true`.
2. Perfil de fuerza NO constante en la ventana, para que la correlación exista.
3. Elegir el empuje en una dirección OBSERVABLE — o, mejor, **reportar las dos**.
4. ⭐ **Mapa de observabilidad sobre el espacio de trabajo**: qué fracción de un empuje
   típico es recuperable según configuración y dirección. Que el 93% de este empuje sea
   invisible NO es un detalle a esconder: es el resultado que convierte la limitación
   4DOF de disculpa en ANÁLISIS, y es exactamente el "observability-aware" que el paper
   vende como estrella. Sin ese mapa, un revisor pregunta "¿y cuándo NO funciona?" y no
   hay respuesta.

⚠️ Pendiente de verificar: E1 ("corr 0.96–0.998") sale de `mj_wrench_test.c`, no de este
test. Hay que confirmar bajo qué direcciones se midió — si fueron observables, el número
vale pero está incompleto sin el punto 4.

⚠️ `test_trajectory.py` da hoy **6.82 mm / 4.785°**, no los 3.82 mm / 2.72° de la tabla
(también verificado idéntico en MX). Número viejo: usar el que produce el código.

## Los 4 experimentos de HARDWARE que cierran el paper (lo único que falta)

```
HW-1  Estimación sin sensor vs fuerza CONOCIDA (peso colgado por hilo/polea → F_est vs mg).
      EL experimento clave. Valida E1 en real.
HW-2  Hand-guiding cualitativo + métrica de fuerza/esfuerzo (E3 en real).
HW-3  Compliant vs rígido en real, mismo empuje → confirma los ~30%.
HW-4  Tracking de trayectoria en real → confirma los ~3.8mm.
```
4-5 repeticiones c/u → stats. El sim ya dio la validación rigurosa; HW la confirma en real.

## Outline

```
I.   Intro — interacción física sin F/T sensor en manipuladores de bajo costo (gap + costo)
II.  Sistema — brazo 4DOF MX-28, modelo dinámico (validado a 1e-10), arquitectura
III. Estimación de wrench sin sensor + ANÁLISIS DE OBSERVABILIDAD (rank 4<6)  ← corazón
IV.  Control compliant DQ-NMPC (admitancia se(3), salida velocidad → servo)
V.   Implementación tiempo real — 100 Hz, timing del solver, [CPU/embebido]
VI.  Experimentos: SIM (E1,E3,E4,trayectoria,stats N=6) + HW (HW-1..4)
VII. Conclusión + límites (4DOF/observabilidad, declarado)
```

## Estado

```
[✓] II, III, IV, VI-sim   HECHO y validado (la mayoría del paper)
[✓] V  implementación      MEDIDO: 2.5-3.0 ms/solve (era 12 Hz; MX->SX)
[✓] III observabilidad     NUEVO y es la contribución: observability_map.py
[✓] escribir               paper/main.tex — 12 pág, compila. Falta autores + HW
[ ] VI-HW  HW-1..4         los 4 experimentos en el MX-28 real
[ ] admitancia restringida al subespacio observable (código, no hardware)
```

⚠️ **El framing cambió** (01/08/2026). La estrella ya no es "estimamos wrench sin
sensor" sino **qué puede y qué no puede sentir el brazo**. Motivo: el estimador es
exacto (7.1e-15) y por lo tanto no es contribución; lo que nadie había cuantificado
es el subespacio observable. Ver `paper/main.tex` sec. 3 y la sección de
observabilidad más abajo.

~70% del paper ya está en código validado. Cuello de botella = HARDWARE.

## Resultados sim (resumen, para el paper)

| Experimento | Resultado |
|---|---|
| Modelo (FK/Jac/EOM/dinámica) | validado vs MuJoCo a 1e-10..1e-16 |
| E1 observer sin sensor | corr 0.961±0.021 — ⚠️ pero corr mide FORMA: en MuJoCo hay 1.08 N de error de amplitud al 41% de observabilidad |
| E3 compliant vs rígido (N=6) | **PAREADO**: fuerza +1.15±0.64 N (t=4.42), seguimiento +14.4±8.0 mm (t=4.43), favorable 6/6. Sin parear las dispersiones se solapan y no concluye |
| E4 DQ vs desacoplado | **ver abajo — el E4 original NO era válido** |
| Trayectoria | RMSE 3.82 mm / 2.72° |
| Observabilidad 4DOF | rank(J)=4<6, declarado |
| Tiempo real | **2.53 ms/solve = 395 Hz media, 306 Hz worst-case** |

---

## ⚠️ E4 revisado (31/07/2026) — el "empate al 1%" no era medición

`test_e4_compare.py` **no podía detectar una diferencia**, por cuatro defectos
independientes. Cualquiera de ellos solo ya garantiza el empate:

1. **Pesos distintos entre los dos controladores.** Al migrar el DQ de `EXTERNAL` a
   `NONLINEAR_LS`, los parámetros `p[8:22]` (`W_se3`,`W_qd`,`W_u`) quedaron **muertos**:
   se declaran en `generate_arm_dqnmpc_ocp.py:85-87` y no se usan — el peso sale de
   `ocp.cost.W`, fijado en generación. El baseline es `EXTERNAL` y sí los usa. El test
   seteaba `[3,3,3,120,120,120]`/`qd 0.1`, así que corría **DQ rot2/trans80/qd0.3 contra
   DEC rot3/trans120/qd0.1**. Reejecutado hoy da **40.8% con el DQ PERDIENDO** — no 1%.
2. **Referencia alcanzable.** `q_ref_t` es trayectoria de JUNTAS ⇒ error → 0 ⇒
   `J_l⁻¹(φ) → I` ⇒ las dos funciones de costo son **idénticas en el límite medido**.
3. **Error inicial cero** (`x0 = q_ref(0)`): no hay régimen de error grande en ningún
   momento de la corrida.
4. **Transitorio descartado** (`pe[50:]`): se tira justo donde las métricas se separan.

⚠️ Y `test_e4_compare.py:59` imprime `"E4: confirmado — DQ ~= desacoplado"` **hardcodeado**,
sin depender del resultado. Reejecutarlo mostraba la conclusión vieja con números nuevos.

**Cota que hace falsable el claim.** Las dos métricas difieren solo en `ρ = J_l⁻¹ t_err`
vs `t_err`, y esa diferencia es **0.38% por grado** de error de orientación (lineal, como
manda `J_l⁻¹ ≈ I − φ×/2`): 1.9% a 5°, 3.8% a 10°, 16.9% a 45°, 34.1% a 90°.
E4 corrió a **6.68°** ⇒ los dos costos eran **97.2% idénticos por construcción**.

**E4-FAIR** (mismos pesos, todo lo demás igual): 9.035 mm vs 8.710 mm — empate real
(DQ −3.7% pos, +1.5% orient). Los 40.8% eran el bug de pesos.

**E4-BIS** (baseline también `NONLINEAR_LS` ⇒ sin confound de Hessiano; referencia con
orientación perturbada α ⇒ INALCANZABLE, que es la condición real del 4DOF; arranque lejos;
transitorio incluido; 250 pasos, 0 fallos de solver):

| α | DQ pos | DEC pos | Δpos | DQ ori | DEC ori |
|---|---|---|---|---|---|
| 0° (control positivo) | 1.038 mm | 1.050 mm | +1.1% | 0.236° | 0.263° |
| 30° | 2.472 | 2.492 | +0.8% | 29.44 | 29.44 |
| 60° | 6.120 | 6.220 | +1.6% | 58.47 | 58.46 |
| 90° | 17.729 | 18.246 | **+2.8%** | 85.38 | 85.32 |

**Lectura honesta:** el DQ gana, y la ventaja **crece monótona con α** (0.8→1.2→1.6→2.1→2.8%)
en una sim determinista, así que es señal y no dispersión. Pero es **chica**: donde las
funciones de costo difieren 34%, el lazo cerrado difiere 2.8%. La saturación de par y la
variedad alcanzable del 4DOF dominan el compromiso.

⇒ **El DQ no se vende como ganancia de desempeño (2.8% no sostiene un paper).** Se vende
como el compromiso rotación/traslación correcto por construcción, a costo de cómputo CERO
(2.46 ms en los dos), y queda ATADO al claim estrella: la métrica se(3) importa justamente
cuando la tarea es inalcanzable, que es la condición que define a un manipulador sub-actuado.

## ✅ Tiempo real (item V) — cerrado el 31/07/2026: era 12 Hz, ahora 395 Hz

`timing_test.c` daba **86.1 ms = 12 Hz** e imprimía `AJUSTAR`. El docstring de
`generate_arm_dqnmpc_ocp.py:95` afirmaba que pasar a `NONLINEAR_LS` había llevado el
solver a 100 Hz: **falso** — daba exactamente los 86 ms del caso que decía haber
arreglado. Desglose: `time_lin` 101.46 ms de 101.63 total; `time_qp` **0.13 ms**. El QP
nunca fue el problema.

**Causa:** todo el modelo se construía en **MX**, y `ca.solve(M, τ−h)`
(`generate_arm_dqnmpc_ocp.py:62`) sobre MX genera un nodo `LinsolQr` — un solver QR
NUMÉRICO por evaluación, con derivadas por diferenciación implícita, que ni siquiera se
puede expandir (`eval_sx not defined for LinsolQr`). Con 20 nodos × 4 etapas ERK con
sensibilidades, eso se evalúa ~80 veces por solve.

**Arreglo:** el modelo entero en **SX** (`arm_dynamics`, `arm_kinematics`, `dq_math` y los
dos generadores). En SX el `solve` es eliminación gaussiana simbólica y el C generado queda
plano.

| | media | worst-case |
|---|---|---|
| MX (antes) | 95.4 ms → **10 Hz** | 104.9 ms |
| SX (ahora) | **2.53 ms → 395 Hz** | 3.27 ms → **306 Hz** |

**Equivalencia verificada, no asumida:** mismo x0, misma referencia, 250 pasos de lazo
cerrado ⇒ `max|Δu| = 2.2e-15 Nm`, `max|Δx| = 3.1e-15`. Es el mismo controlador. Y la
regresión del modelo sigue en pie: `max|dM|=8.78e-11`, `max|dh|=2.73e-09` vs MuJoCo,
FK `6.93e-13 m / 0.00°`.

⚠️ Defecto que queda en `timing_test.c:19-24`: **x0 fijo** en las 2000 corridas, así que
con RTI + warm start el QP arranca en el óptimo. Sobreestimaba ~10% (86 vs 95 ms reales en
lazo cerrado). Ya no bloquea — hay 3x de margen — pero el número que va al paper es el de
lazo cerrado, no el de x0 fijo.
