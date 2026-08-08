# Domain profile — Contact-Model-Aware Sensorless Force Estimation (IEEE Access)

Draft: `paper/main.tex`
Venue: **IEEE Access** (no es RA-L: sin límite de 6-8 páginas, sin doble anonimato
obligatorio). Las reglas duras de RA-L de la skill NO aplican; sí aplican el rubro
de revisor (C), la higiene de claims (F) y el pase de vocabulario (H).

## SOTA (must-cite + delta)

- **De Luca & Mattone, ICRA 2005** — residuo sin sensor para control híbrido
  fuerza/movimiento — brazos con par y n≥6; no trata la inversión bajo n<6 —
  [must-cite]
- **De Luca et al., IROS 2006** — detección de colisión y reacción segura, DLR-III
  (7 DoF) — mismo régimen: par, n≥6 — [must-cite]
- **Magrini et al., IROS 2014** — ⛔ **LEER EL PAPER ANTES DE ESCRIBIR NADA SOBRE
  ÉL.** Texto extraído en `.claude/magrini2014_extract.txt`.

  ⚠️ **02/08/2026: se verificó contra la fuente y la diferenciación que tenía el
  paper era INDEFENDIBLE.** Magrini ya establece, en 2014, casi toda la
  observación dimensional que yo reclamaba como nueva. Citas literales:

  > *"in order to estimate properly Γc, i.e., both a contact force F_c ... and a
  > contact torque M_c ..., we should have rank J_c = 6, which is true only if the
  > robot has n ≥ 6 joints"*

  > *"The dimension of the task related to the contact force is thus m = 3"*

  > *"all forces F_c ∈ N(J_c^T) will never be recovered"*

  > *"(J_c^T)# J_c^T ≠ I, so that part of the contact forces may not be identified"*

  Y el experimento de la Fig. 7, con masa colgada en un KUKA LWR real:

  > *"When attempting to estimate Γc ∈ R6 with the complete Jacobian, the number of
  > informative residual components is too low (4 < 6), and this results in a wrong
  > estimation of both F_c and M_c (the latter should be zero while it is not)"*

  O sea: `4 < 6`, fuerza mal, momento espurio. Exactamente el "hallazgo" que el
  borrador reclamaba. El caso `4 < 6` les aparece por contacto en un eslabón
  intermedio, no por tener pocas juntas, pero **la matemática y la consecuencia son
  las mismas**.

  ⛔ **Frases del borrador que hay que borrar:** *"the consequences for the
  inversion step are, to our knowledge, not characterized"* y *"there the
  six-dimensional inversion is already well posed"*. Las dos son falsas.

  **CAUSA RAÍZ: se citó el paper SIN LEERLO**, afirmando qué hacía desde memoria.
  No repetir. — [must-cite, reposicionar como extensión, no como descubrimiento]
- **Haddadin et al., T-RO 2017** — survey de detección/aislamiento/identificación —
  contexto, delimita el estado del arte — [must-cite]
- **Yoshikawa, IJRR 1985** — elipsoides de manipulabilidad — problema DUAL
  (producir vs percibir) — [context]

⚠️ **Los dual quaternions se SACARON del paper (01/08/2026).** Motivo doble:
(1) no sostenían ninguna de las tres contribuciones, y la ablación daba 0.8-2.8%;
(2) el paper llamaba a la ec. (6) "the se(3) logarithm" y **NO lo es** — verificado
contra el logaritmo matricial de SE(3) sobre 200 poses: φ difiere por factor 2 y ρ
por factores no constantes (0.63, 1.14, 1.17). `dq_math.ln_dual` calcula
`φ = (ángulo/2)·eje` pero aplica `J_l⁻¹` evaluado en ese MEDIO ángulo y a `t_err`
sin dividir: no coincide ni con el log DQ estándar ni con el de SE(3). Sigue siendo
una métrica de pose válida, pero el nombre era falso. La ablación se conserva,
reencuadrada como error acoplado vs desacoplado. Se eliminaron figueredo2013dq y
adorno2021dqrobotics.
- **Chiacchio et al., 1997** — politopos de fuerza — se usa en su rol clásico para
  el umbral de saturación (Sec. V-C) — [context]
- **Wong & Suleiman, IEEE T-RO 2024** — ⛔ **vecino directo de C1, encontrado el
  03/08/2026.** *Sensor observability analysis*: índice y elipsoide por
  configuración de qué ejes de tarea observan los sensores de par; demuestra en un
  Baxter una postura donde la fuerza en x es invisible con verdad de terreno; ya
  conecta con `N(Jᵀ)`. **Leído entero.** No cuantifica el error en newtons, no
  trata la mal-atribución fuerza→momento, no reduce a contacto puntual, y su
  deficiencia viene del montaje de sensores, no de `n<6`. Ver
  `.claude/lit-review-2026-08.md` — [must-cite, delta ya escrito en Sec. II-B]
- **Lu, Shen & Zhuang, Automatika 2023** — ⛔ vecino del gating: optimiza
  configuraciones de un robot de 6 juntas por **número de condición del
  Jacobiano** para mejorar la estimación de fuerza. ⚠️ **Solo resumen leído**
  (T&F 403). — [must-cite]
- **de Wolde et al., IROS 2024** — ⛔ vecino directo de C2: control de impedancia
  en brazo comandado por CORRIENTE, sin sensor F/T, calibrando la relación
  corriente/par y la fricción. ⚠️ **Solo resumen leído.** La diferencia que
  sostiene C2 (ellos comandan par, acá se comanda `q̇`) es real pero más fina de
  lo que decía el borrador. — [must-cite]
- **Scherzinger et al., 2022 (FDCC)** — la interfaz de VELOCIDAD da mejor
  compliance que la de posición en robots comerciales. ⇒ *"velocidad para
  compliance"* no se puede presentar como hallazgo — [context]

## Positioning

- **Novedad de titular:** el régimen `n<6` + actuación en VELOCIDAD, que es lo que
  son los brazos de servos comerciales. No hay trabajo previo en ese cruce.
- **Diferenciar sobre todo de Magrini 2014:** ellos ya restringen a contacto
  puntual, pero para estimar el PUNTO en brazos redundantes. Acá el punto es
  conocido y el problema es la DIMENSIÓN del desconocido bajo n<6. Hay que decirlo
  explícito o un revisor lo lee como incremental.
- **Evidencia más fuerte:** (1) barrido direccional, 12000 direcciones, correlación
  0.9993 entre alineación y error; (2) Monte Carlo con ruido de servo, 18.0% contra
  69.6%; (3) el gating por σ_min separa los dos mecanismos de error.
- **Lo que NO se reclama, y está escrito en el paper:** que `JᵀF=τ` esté
  indeterminado con n<6 es álgebra elemental; el modelo de contacto puntual es
  práctica estándar. Mantener ese párrafo — es lo que evita que se lea inflado.

## Lo que SÍ queda como aporte propio (tras verificar Magrini)

Magrini establece el hecho dimensional y muestra UN caso. No hace:

- cuantificación global sobre el espacio de trabajo (2197 configs, 50.5% medio)
- descomposición espectral de `P_ff` y estructura direccional (12000 direcciones,
  correlación 0.9993 con la alineación)
- demostración del **fallo silencioso**: correlación 0.997 conviviendo con 2.227 N
- análisis de condicionamiento y criterio de gating por `σ_min(Jv)`
- el régimen de **actuación en velocidad**: asignación de modelo (estimador sí,
  controlador no) y los dos regímenes de compliance

⇒ Reformular como **extensión cuantitativa y orientada a diseño**, no como
descubrimiento del problema dimensional.

## El marco acordado (02/08/2026)

> *A quantitative design framework for point-contact force estimation on low-DoF,
> velocity-actuated manipulators.*
>
> *Given a low-DoF arm with velocity-controlled smart servos, what contact force
> is identifiable, with what accuracy, and how can it be used for compliant
> interaction?*

Jerarquía: **C1** caracterización cuantitativa y geométrica de la inversión
(principal) · **C2** arquitectura de estimación y compliance bajo actuación en
velocidad (co-principal) · **NMPC** demostrador de integración, no contribución
matemática.

⚠️ El **50.5% no es contribución por sí solo**: es de este brazo, este workspace y
esta distribución de fuerzas. Su valor aparece cuando `P_ff` lo **explica y
predice**.

⚠️ **C2 no se sostiene en *"el controlador cinemático no necesita M ni h"***, que
aislado es conocido. Lo defendible es la cadena `corriente → τ_act → τ_ext → f̂ →
q̇_cmd` con las seis demostraciones listadas en `CLAUDE.md`.

⚠️ **Fuera del modelo ideal no se dice *"exact recovery"***. En hardware:
consistencia, error y sensibilidad.

Condiciones para que alcance: (1) la búsqueda bibliográfica confirma que nadie
reunió antes la caracterización; (2) el hardware valida predicción direccional,
efecto del condicionamiento y viabilidad del par desde corriente; (3) gating y
fallo de correlación se presentan como consecuencias, no como métodos nuevos.

**Estado de la condición 1 (03/08/2026): PARCIAL.** Se corrió la búsqueda,
`refs.bib` pasó de 14 a 31 entradas, y los tres vecinos nuevos ya están citados con
su delta en el manuscrito. Falta: texto completo de Lu 2023 y de Wolde 2024, y el
barrido de citas HACIA ADELANTE de Magrini 2014 y Wong 2024. Registro completo en
`.claude/lit-review-2026-08.md`.

⚠️ **Lo que la búsqueda quitó de la mesa:** que existan direcciones ciegas
dependientes de la configuración, medibles con un elipsoide (Wong 2024); que el
condicionamiento del Jacobiano gobierne la exactitud y sirva para elegir posturas
(Lu 2023); que la cadena corriente→par→compliance sin sensor F/T sea nueva
(de Wolde 2024); y que la velocidad sea mejor interfaz que la posición para
compliance (Scherzinger 2022). Lo que **queda**: la magnitud y prevalencia de la
pérdida sobre el espacio de trabajo, la **mal-atribución** fuerza→momento espurio
que `P_ff` predice, el fallo silencioso de la correlación, y el cruce `n<6` +
actuación en velocidad.

⚠️ **Un ensayo sin carga conocida no valida la tesis.** Valida ruido, deriva y
dependencia postural. Para exactitud y mal-atribución hace falta carga conocida en
**dos direcciones** y posturas bien/mal condicionadas — sin sensor F/T. Ya está en
`hw/s3_conditioning.py` (`--mass`, `--pull`).

## Riesgos de revisor identificados

1. **"Esto es elemental"** — mitigado por el párrafo de "no reclamado como nuevo" y
   porque la contribución declarada es el régimen + la cuantificación.
2. **Todo es simulación.** Declarado en abstract, intro y limitaciones. Es el
   riesgo mayor y no tiene mitigación salvo hardware.
3. **Circularidad en el barrido direccional**: la "dirección ciega" sale del mismo
   modelo que produce el error. Mide que la dirección predice el error, no que el
   error se prediga a sí mismo, pero conviene declararlo.
4. **La Sec. VII-C (compliant vs rígido) y la ablación DQ no son contribuciones.**
   Están como validación. Si un revisor pregunta qué aportan, la respuesta honesta
   es que respaldan, no aportan.

## Pendiente

- **No hay corpus de referencia local.** Para el pase H.1 (barrido fuera-de-corpus)
  hace falta bajar los fuentes LaTeX de los trabajos de arriba que estén en arXiv
  (`curl -L arxiv.org/e-print/<id>`) a `paper_refs/src_corpus/`. Sin eso, el
  linter solo cubre ortografía, gramática y AI-tells, no el registro del campo.
