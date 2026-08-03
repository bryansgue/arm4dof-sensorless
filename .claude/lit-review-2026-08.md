# Búsqueda bibliográfica — 03/08/2026

Condición 1 del asesor: *"la revisión bibliográfica confirma que nadie reunió antes
esa caracterización"*. Este archivo registra qué se buscó, qué se encontró, qué se
leyó de verdad, y qué afirmaciones del borrador quedan **tocadas**.

⚠️ **Veredicto corto: la condición 1 NO está cumplida por el borrador actual.**
Aparecieron **dos vecinos que hoy no se citan** y que un revisor del área encuentra
en una búsqueda de diez minutos. Con ellos citados y las afirmaciones recalibradas,
el encuadre acordado sobrevive — pero más angosto.

`refs.bib` pasó de 14 a **31** entradas. Todos los metadatos nuevos salen de
Crossref y Semantic Scholar por DOI, no de memoria.

---

## Estado de lectura — qué se puede afirmar de cada uno

Regla de la casa: no afirmar qué hace un paper sin abrirlo. Esta tabla dice hasta
dónde llega el derecho a afirmar.

| ref | leído | se puede afirmar |
|---|---|---|
| `wong2024sensorobs` | **texto completo, pág. 1-12** | todo lo de abajo |
| `magrini2014virtual` | extracto verificado (sesión anterior) | todo |
| `lu2023configopt` | **texto completo, 32 pág.** (vía figshare, ver abajo) | todo lo de abajo |
| `dewolde2024current` | **texto completo** (arXiv:2403.13079) | todo lo de abajo |
| resto | resumen | contexto, no diferenciación |

✅ **Los tres vecinos que pueden mover una contribución de sitio están leídos
enteros.** Los dos que faltaban se consiguieron el mismo día: Lu por figshare, de
Wolde por arXiv. El resto se cita solo como contexto, y para eso el resumen alcanza.

⚠️ **Y en los dos casos el resumen daba una impresión equivocada** — en Lu
subestimaba el solape, en de Wolde lo exageraba. Es la misma regla de la casa: no
afirmar qué hace un paper sin abrirlo. Un resumen no es abrirlo.

---

## 🔴 EL VECINO QUE FALTABA — Wong & Suleiman, IEEE T-RO 2024

`wong2024sensorobs`, *Sensor Observability Analysis for Maximizing Task-Space
Observability of Articulated Robots*, T-RO 40:4102-4116. Leído entero.

**Qué hace.** Define un índice y un **elipsoide de "observabilidad sensorial"**:
transforma los ejes de sensores distribuidos (típicamente sensores de par en las
juntas) al marco de tarea y mide, **por configuración**, qué tan bien se observa
cada eje del espacio de tarea. Cita textual del resumen:

> *"certain joint configurations may align joint torque sensors in such a way that
> they are unable to observe interaction forces in one or more task-space (e.g.,
> Cartesian) axes"*

Y el experimento en hardware (Fig. 4b, Baxter de 7 juntas, con sensor de fuerza
externo como verdad):

> *"the robot is unable to observe interaction forces in the x-axis at t ≈ 3 s and
> t ≈ 17 s, despite the ground truth force sensor showing interaction forces"*

O sea: **una dirección ciega, demostrada en hardware, con verdad de terreno.**

**Además tienen un umbral que se parece a mi gating.** Sec. III-C define un umbral
por ruido de sensor, `s*ᵢⱼ = σ_ε / Φᵢⱼ`, y descarta ejes por debajo:

> *"Sensor observability values below the threshold are set to 0 ... either sⱼ = 0
> or o = 0 indicates that at least one task space axis is no longer observable"*

**Y saben del vínculo con el espacio nulo de `Jᵀ`:**

> *"it is sometimes possible to extract similar information using the end effector
> force and joint torque relationship τ = Jᵀf and examining the null space of Jᵀ"*

### Qué NO hacen — y ahí vive lo que queda de C1

Verificado leyendo, no suponiendo:

1. **No cuantifican el error de fuerza.** Su índice es una **alineación
   normalizada** en [0,1] (o una suma sin cota), no Newtons. Nunca aparece "cuánta
   fuerza se pierde".
2. **No tratan la mal-atribución fuerza↔momento.** El objeto que estudian es qué
   eje se observa, no cómo la pseudoinversa de mínima norma **reparte** la fuerza
   no observada como momento espurio. Ese es exactamente el mecanismo de
   `magrini2014virtual` Fig. 7 y el mío.
3. **No reducen a contacto puntual con punto conocido.** No aparece.
4. **El régimen `n<6` no es su tema.** Sus robots son Baxter (7 juntas) y un RRR
   plano de 3 con celdas de carga; la deficiencia les entra por *dónde están
   montados los sensores*, no por tener pocas juntas. Su propio texto separa las
   dos cosas: una configuración singular de observabilidad **no** implica
   singularidad cinemática, y viceversa (Fig. 2c-e, Fig. 5).
5. **El uso es prescriptivo, no descriptivo**: maximizar el índice en el espacio
   nulo o por QP para *evitar* configuraciones ciegas. No hay estadística sobre el
   espacio de trabajo ni predicción del error.

### Consecuencia para el borrador

⚠️ **"La estructura direccional del error es nueva" ya no se puede escribir así.**
Que existan direcciones ciegas, que dependan de la configuración, y que eso se
pueda medir con un elipsoide, está publicado en T-RO en 2024.

Lo que sí queda, y hay que decirlo con esas palabras:

- `P_ff` es un **proyector en el espacio de fuerza** derivado de `J_v`, no un índice
  de alineación de ejes de sensor. Predice **cuánta fuerza** (en N) se pierde y
  **hacia dónde va**, no solo si un eje se observa.
- La **mal-atribución** fuerza→momento espurio, que es la tesis central, no está en
  Wong.
- La **cuantificación global** (2197 configuraciones, 50.5% medio, peor caso 99.6%)
  es de otra naturaleza que un índice local por configuración.
- El **fallo silencioso** (correlación 0.997 conviviendo con 2.227 N) no aparece en
  ningún trabajo encontrado.

⛔ Y hay que **citar el vínculo que ellos ya hicieron** con el espacio nulo de `Jᵀ`,
porque si el borrador lo presenta como observación propia, se lee mal.

---

## 🟠 EL SEGUNDO VECINO — Lu, Shen & Zhuang, Automatika 2023

`lu2023configopt`. **Texto completo leído** (32 pág.). ⚠️ T&F da 403 y hrcak pide
captcha; el manuscrito aceptado sale por **figshare**, que no bloquea:

```bash
curl -s https://api.figshare.com/v2/articles/21878553/files      # lista archivos
curl -sL https://ndownloader.figshare.com/files/38814204 -o lu2023.pdf
```

(La ruta general: Unpaywall `api.unpaywall.org/v2/<DOI>?email=...` lista todos los
espejos abiertos. Sirvió acá y sirve para el resto.)

**Lo que hace, verificado en el texto — es MÁS solapado de lo que decía el
resumen:**

- su cota (Ec. 21) es la **relativa clásica** del número de condición:
  `‖δF‖/‖F‖ ≤ cond(Jᵀ)·‖δτ‖/‖τ‖`, y la reconstrucción (Ec. 22) es
  `F̂ = [J(q)ᵀ]⁻¹ τ̂` — o sea **`Jᵀ` cuadrado e invertible**, robot de 6 juntas;
- **sí mapean el espacio de trabajo**: 500 000 configuraciones por Monte Carlo,
  distribución del JCN y su función de densidad (Fig. 7). JCN medio 1052.9,
  mínimo 9.17, el 96.55% por encima de 11;
- lo usan **prescriptivamente**: optimizan dirección de la trayectoria y pose del
  efector para bajar el JCN medio sobre el camino;
- y ellos mismos declaran el hueco que llenan: *"the relationship between the JCN
  and the force estimation error has not yet been investigated"* (2023).

**El delta, ahora preciso y escrito en la Sec. IV del manuscrito:**

1. su inverso **no existe** con `n<6` — `Jᵀ` es 6×4. La cota de acá es **absoluta**
   y sobre el inverso **sobredeterminado** de contacto puntual, con `σ_min(Jv)`, no
   un número de condición de `J` completo;
2. ellos mapean el **índice**, acá se mapea el **error** — y el componente dominante
   del error es la mal-atribución estructural, que **sobrevive con buen
   condicionamiento** y por lo tanto ninguna elección de postura la quita. Su marco
   entero supone error ∝ condicionamiento;
3. un brazo de 6 juntas siguiendo un camino 1-D tiene ángulo redundante para gastar
   en condicionamiento; uno de 4 siguiendo una pose no tiene ninguno ⇒ acá el índice
   es **reja**, no objetivo.

⚠️ **No repetir la afirmación negativa de ellos.** Con este paper citado, la ruta
del condicionamiento está establecida; lo de acá es el régimen deficiente.

---

## 🟠 EL VECINO DE C2 — de Wolde et al., IROS 2024

`dewolde2024current`. **Texto completo leído** (arXiv:2403.13079, libre). Brazo
Kinova GEN3 Lite sobre base móvil.

⚠️ **El resumen daba una impresión equivocada, y a favor nuestro esta vez.** Leído
el método, la diferencia es mucho más grande de lo que parecía:

1. **Su calibración va en el sentido CONTRARIO.** Estiman la razón `r` entre
   corriente y par **del modelo** para convertir un par COMANDADO por el controlador
   de impedancia en una corriente (su Ec. 6, `c = h(τ)`). La corriente es
   **salida**, no medición de entrada.
2. **Nunca estiman la fuerza externa.** Y lo dicen: eligen impedancia justamente
   porque *"Admittance control faces the challenge of needing sensors to detect
   external forces"*. No hay `f̂`.
3. **Su ley SÍ lleva el modelo dinámico**: Ec. (3) es
   `f = Λ(q)ẍ_d + µ(q,q̇)ẋ_d + f_g − K_d e − D_d ė`, o sea inercia cartesiana y
   Coriolis dentro del controlador.
4. El brazo es **comandado por corriente** (interfaz de par). Lo velocity-controlled
   ahí es la **base móvil**, no el brazo.

**Delta, ya escrito en Related Work:** acá la corriente es una **medición** de la
que se infiere `τ_ext` y `f̂`, lo comandado es `q̇`, el actuador cierra su lazo
abajo, y el modelo dinámico queda **confinado al observador**. Son cadenas
opuestas, no variantes.

⚠️ Y `scherzinger2022fdcc` ya midió que la **interfaz de velocidad es mejor que la
de posición** para compliance en robots comerciales. O sea: *"velocidad para
compliance"* tampoco se puede presentar como hallazgo. Lo que se presenta es la
**asignación de modelo** (el modelo dinámico confinado al observador; el controlador
cinemático iguala al de par a 1/25 del costo) y los **dos regímenes** acotados por
la capacidad del actuador.

---

## Lo que la búsqueda NO encontró — y sostiene lo que queda

Buscado por diez ángulos distintos (ver "Consultas" abajo), sin resultado:

- ningún trabajo que **cuantifique sobre el espacio de trabajo** la fracción de
  fuerza que pierde la inversión 6-D de mínima norma en un brazo de pocas juntas;
- ningún trabajo que documente el **fallo silencioso**: una métrica de validación
  agregada sana (correlación ≈1) conviviendo con error de ganancia grande, en
  estimación de fuerza sin sensor. La crítica general "correlación mide asociación,
  no acuerdo" es clásica fuera de robótica (`bland1986agreement`) — **citarla, y no
  reclamar el mecanismo como propio**, solo su instancia en este problema;
- ningún trabajo en el cruce **`n<6` + actuación en velocidad + par inferido de
  corriente de servo comercial**.

⚠️ Una afirmación negativa nunca se prueba buscando. Lo que sí se puede escribir es
*"we are not aware of"*, con los vecinos citados al lado.

---

## Referencias nuevas agregadas (17)

| clave | rol |
|---|---|
| `wong2024sensorobs` | ⛔ vecino directo de C1 — direcciones ciegas, T-RO 2024 |
| `lu2023configopt` | ⛔ vecino directo del gating — JCN como criterio de configuración |
| `dewolde2024current` | ⛔ vecino directo de C2 — corriente→par→compliance sin F/T |
| `scherzinger2022fdcc` | velocidad vs posición para compliance |
| `likar2014contact` | fuerza + punto de contacto desde par de juntas |
| `manuelli2016cpf` | localización de contacto (lo que acá se supone conocido) |
| `magrini2016hybrid`, `gaz2018polishing` | el residuo aplicado, línea De Luca |
| `shan2024fine`, `shan2024payload` | wrench desde señales internas, RA-L 2024 |
| `liu2021sensorless`, `zhang2021forcefree` | observador + fricción; corriente en enseñanza directa |
| `osburg2022dnnwrench` | wrench estimation y el problema en singularidades |
| `keemink2018admittance`, `zhu2025compliantsurvey` | contexto de compliance |
| `khatib1987operational` | clásico de espacio operacional |
| `bland1986agreement` | acuerdo vs correlación — el fallo silencioso |

## Consultas corridas

sensorless force estimation + momentum observer + low-DoF · Dynamixel corriente
como par · inversión subdeterminada Jᵀ y espacio nulo · condicionamiento /
número de condición / valores singulares y exactitud de fuerza · admitancia con
interfaz de velocidad sin dinámica inversa · fricción y stiction en servos ·
enseñanza kinestésica sin sensor F/T · punto de contacto conocido y reducción a
3-D · evaluación estadística del error sobre el espacio de trabajo · NMPC con
estimación de fuerza de contacto · exoesqueletos de pocos grados de libertad.

⚠️ El endpoint de búsqueda de Semantic Scholar devuelve 429; el de resolución por
DOI/arXiv funciona. Descubrimiento por WebSearch, metadatos por DOI.

## Lo que falta para cerrar la condición 1

1. ✅ **Texto completo de los tres vecinos** — hecho el 03/08/2026.
2. ✅ **Posicionamiento reescrito** contra los tres, en Related Work y en Sec. IV.
3. ⛔ Barrer las citas **hacia adelante** de `magrini2014virtual` y
   `wong2024sensorobs` (quién los citó): es donde vive el trabajo que esta búsqueda
   pudo no ver. **Es lo único que queda de la condición 1.**
4. Bajar los fuentes LaTeX de los que estén en arXiv a `paper_refs/src_corpus/`
   para el pase léxico de `lint_prose.py --sweep` (pendiente ya registrado).
