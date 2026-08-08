# Respuesta a las revisiones del asesor — 04/08/2026

> ⚠️ **Este documento es un REGISTRO HISTÓRICO POR RONDA, no una descripción del
> manuscrito actual.** Cada sección conserva el estado en que se escribió, incluidos
> números y decisiones que rondas posteriores cambiaron. Está a propósito: lo que
> documenta es cómo se resolvió cada objeción, y borrar los estados intermedios
> haría ilegible el razonamiento. Cuando una sección quedó superada, hay un aviso
> apuntando a la ronda que la corrige.

## Estado ACTUAL del manuscrito, que es el que vale

| | |
|---|---|
| PDF | **18 páginas, 11 tablas**, 4 figuras, 33 referencias |
| compilación | 0 errores, 0 referencias sin resolver, `lint_prose.py` clean |
| autoría | ⛔ **propuesta**: dos autores, **pendiente de confirmación escrita**. Ver `SUBMISSION.md` |
| bloqueos | autoría por escrito · biografía de Angélica Quito · autor de correspondencia en la carta · ORCID |
| carta de presentación | borrador en `COVER_LETTER.md` |

⚠️ Cualquier número de páginas, tablas o autoría que aparezca MÁS ABAJO pertenece a
la ronda en que se escribió y **no describe el estado de hoy**.

## Índice de rondas

- **Ronda 2** (primera Major Revision): secciones 1 a 5.
- **Ronda 3** (con el código ejecutado): "Ronda 3". Los tres puntos pedidos quedaron
  cerrados, y el primero terminó **mejor** de lo recomendado: hay demostración
  analítica, así que no hizo falta degradar la invariancia a observación empírica.
- **Ronda 4** (ajustes documentales): "Ronda 4".

---

Todo lo verificable de la revisión se comprobó contra el código antes de tocar el
texto. **Los dos comentarios mayores y los siete menores eran correctos.** Abajo,
qué se hizo con cada uno y qué queda pendiente de decisión.

---

## Comentario mayor 1 — la métrica del wrench

**Confirmado, y era peor de lo señalado.** `P` sale de la SVD de `J ∈ R⁶ˣ⁴`, cuyas
tres primeras filas (`Jv`) van en metros y las tres últimas (`Jw`) son
adimensionales. Cambiar la unidad de longitud cambia `col(J)`, y con ella `P_ff`.
La pseudoinversa sin ponderar en unidades SI fija esa longitud en **1 m sin
declararlo**, mientras que el alcance medio de este brazo es **0.233 m**.

Medido sobre las mismas 2197 configuraciones, con norma `‖f‖² + ‖m‖²/ℓ_c²`:

| ℓ_c [m] | media perdida | mediana | p90 | max | configs con λ_min<0.10 |
|---|---|---|---|---|---|
| 0.05 | 12.8 % | 5.4 | 37.1 | 99.1 | 6 % |
| 0.15 | 30.2 % | 24.9 | 63.3 | 99.6 | 17 % |
| 0.233 (alcance) | **38.0 %** | 35.9 | 73.0 | 99.6 | 29 % |
| 0.50 | 47.3 % | 47.9 | 84.0 | 99.6 | 63 % |
| 1.00 (práctica estándar) | **50.5 %** | 51.1 | 89.2 | 99.6 | 100 % |
| 2.00 | 51.4 % | 52.2 | 91.1 | 99.6 | 100 % |

**Lo que se agregó al paper** (Sec. IV-C nueva, `\label{sec:metric}`):

1. Discusión explícita de la incompatibilidad dimensional.
2. Formulación ponderada `W_ℓc = diag(I, I/ℓ_c)`, ec. (10)-(11). El inverso sin
   ponderar es el caso ℓ_c = 1 m.
3. Tabla de sensibilidad completa (Tabla II del PDF).
4. **Punto de referencia declarado**: el momento se toma en el origen del efector,
   el mismo punto donde se evalúa `J` y donde se supone el contacto.
5. **Proposición 2, separando invariante de dependiente de la métrica.**

### Lo que se rescata, y es el resultado nuevo de esta ronda

> ⚠️ **Superado en la ronda 3.** Lo de abajo se escribió como verificación
> numérica; en la ronda 3 pasó a ser **demostración analítica** (Proposiciones 2
> y 3), y el argumento de rango se corrigió a `col(W⁻²J)`. Ver la sección
> "Ronda 3".

| | |
|---|---|
| **invariante** | el conjunto de fuerzas recuperadas exacto (probado analíticamente) |
| **invariante** | los autovectores de `M(ℓ_c)`: **qué** direcciones se pierden. Ángulo principal máximo **2.6e-6 grados** sobre 2197 configuraciones y 9 valores de ℓ_c |
| **invariante** | el peor caso, 99.6 % para todo ℓ_c ≥ 0.15 m |
| **invariante** | la ley `‖M f − f‖ = (1−λ_min)·|cos θ|`, exacta por pose |
| **depende de la métrica** | los autovalores, el 50.5 %, la fracción convertida en momento, el "100 % de configuraciones" |

⚠️ La verificación de invariancia hay que hacerla sobre **autoespacios**, no sobre
autovectores sueltos: el espectro es `{λ_min, 1, 1}` en el 92 % de las
configuraciones y la base del subespacio degenerado es arbitraria. Comparar
autovectores uno a uno da hasta 90° y es un artefacto.

### Y una autocrítica que salió de aquí, que el asesor no había pedido

**La correlación 0.9993 está favorecida por la métrica.** Por pose la relación es
la identidad algebraica de arriba, o sea correlación **exactamente 1.000000 a
cualquier ℓ_c**. Al agrupar seis poses, cada una aporta una recta de pendiente
`1−λ_min`, y la correlación agrupada mide cuánto se parecen esas pendientes. Con
ℓ_c = 1 m todos los `λ_min` caen bajo 0.09, las pendientes casi coinciden, y por
eso sale 0.9993. **Con ℓ_c = 0.05 m las mismas 12000 direcciones dan 0.4475**, con
todas las correlaciones por pose todavía en 1.000000.

Esto está escrito en el paper como remark propio, no escondido. La afirmación
predictiva pasa a ser la identidad por pose; el número agrupado se presenta como
lo que es.

### Decisión que queda abierta

El paper usa **ℓ_c = 1 m como nominal**, y lo justifica así: no es la escala
natural del brazo, pero **es la métrica que el estimador bajo examen realmente
implementa**. El objeto de estudio es la práctica estándar. La fila de 0.233 m se
reporta en paralelo en abstract, contribuciones, resultados y conclusiones.

⚠️ **Si preferís que el nominal sea 0.233 m**, el cambio es mecánico pero toca ~15
números en el texto. Decilo y lo hago.

**Script:** `ocp_generation/metric_sensitivity.py` reproduce las seis secciones
(escala del brazo, tabla de sensibilidad, invariancia, el ejemplo de 6 N, el
barrido direccional contra ℓ_c, y el control de que el inverso de contacto puntual
no admite este problema).

---

## Comentario mayor 2 — validación solo simulada

**No se puede cerrar sin el brazo.** Lo que sí se hizo es calibrar el lenguaje
para que nada se lea como dato real:

| antes | ahora |
|---|---|
| *"under the noise the MX-28R actually delivers"* | *"under a noise model built from the published specifications of the MX-28R, not from measurements of one"*, y se dice que la magnitud de stiction es una suposición, no un dato de hoja |
| *"independent physics engine"* (4 apariciones) | *"a second and independently written dynamic implementation"*, con la aclaración de que coincidir a 1e-10 prueba que dos implementaciones de la misma mecánica coinciden, **no realismo físico** |
| límite *"Simulation only"* | ampliado: ningún simulador puede dictaminar si la corriente sirve de proxy de par en un servo concreto |
| abstract | ahora nombra explícitamente los tres puntos que el hardware tiene que probar, incluido el par inferido desde corriente |

El experimento estático mínimo que el asesor describe **ya está especificado y
scripteado**: `hw/s3_conditioning.py`, las dos partes (dispersión sin carga, masa
conocida vertical + tiro horizontal con balanza), los dos estimadores sobre el
mismo residuo, con piso de resolución y tres rejas que hacen callar el veredicto
si el ensayo se contamina. Sigue sin correrse: **requiere el brazo montado.**

---

## Comentarios menores

| # | comentario | qué se hizo |
|---|---|---|
| 1 | el gating debe informar qué fracción del espacio conserva cada umbral | Tabla de gating con columna **workspace kept**: 100 / 88.6 / 68.7 / **31.4 %**. Párrafo nuevo diciendo que el umbral que parte la cola al medio rechaza dos tercios del espacio, que 0.04 conserva 68.7 % y ya da un tercio de la mejora, y que el gate se usa mejor como bandera de confianza que como restricción de postura. El costo también entra en abstract y conclusiones |
| 2 | Tablas 12 y 13 discrepan en el caso sin gating | **Causa encontrada**: la de ruido corría N=2000 y la de gating N=1500, misma semilla. Igualadas a N=2000; ahora la fila `0.00` reproduce **exacto** la fila `nominal` (69.6 / 149.1 / 18.0 / 58.9), y el paper lo dice |
| 3 | reportar variabilidad entre semillas del Monte Carlo | Agregado: 5 semillas. p90 de contacto puntual en `[58.9, 62.0] %` sin gate y `[28.4, 29.8] %` con gate 0.06, intervalos disjuntos. Mediana 6-D `[68.5, 72.8]` → `[73.2, 77.6]` bajo el mismo gate. Se declara que diferencias de pocos puntos porcentuales no deben interpretarse |
| 4 | *"independent physics engine"* | reescrito en las 4 apariciones, ver arriba |
| 5 | el 1e-10 da impresión contraproducente | ahora siempre acompañado de qué prueba y qué no |
| 6 | demasiadas tablas | **16 → 12**. Eliminadas: `timing`, `validation`, `threedirs`, `velocity` (sus números pasaron a prosa, sin perder evidencia). Eliminadas también las subsecciones *Real-time implementation* (queda como remark) y *Tracking, regulation and computation* (queda en 4 líneas). El bloque del controlador es ahora lo mínimo que sostiene la contribución 2 |
| 7 | README y checklist desactualizados (9 y 14 páginas) | sincronizados |
| 8 | la doc de envío habla de tres autores | corregido en esa ronda: el manuscrito tenía autoría única y `SUBMISSION.md` se sincronizó. ⚠️ **Superado**: la autoría cambió después, ver el encabezado |

---

## ⚠️ Hallazgo NO pedido por la revisión: la tabla de regímenes no se reproducía

Al mapear cada tabla a su script para actualizar el README apareció que
**`tab:regimes` (los dos regímenes de compliance) no la producía ningún script
guardado**. Es el mismo defecto que la auditoría del 01/08/2026 decía haber
cerrado con `reproduce_paper_tables.py`, que tiene cinco funciones de tabla y
ninguna es esta.

Se escribió el script (`ocp_generation/test_compliance_regimes.py`: planta con
saturación, admitancia y NMPC reales, barrido de magnitud) y **no reproduce los
números publicados**:

| ‖f‖ [N] | publicado (comandado / real / razón / saturación) | reproducido |
|---|---|---|
| 1 | 8.3 / 8.6 / 1.03 / 0 % | 7.2 / 9.3 / 1.29 / 0 % |
| 5 | 41.4 / 37.8 / 0.91 / 0 % | 35.9 / 43.4 / 1.21 / 0 % |
| 6 | 49.7 / 86.0 / **1.73 / 100 %** | 43.1 / 52.2 / **1.21 / 0 %** |
| 8 | 66.2 / 124.5 / 1.88 / 98 % | 57.5 / 138.5 / 2.41 / 91 % |
| 10 | 82.8 / 162.3 / 1.96 / 98 % | 71.8 / 264.2 / 3.68 / 100 % |

**La conclusión cualitativa se sostiene** (dos regímenes, la razón salta justo
donde aparece la saturación), pero **la transición cae entre 6 y 8 N, no entre 5 y
6**. Y ahí estaba además un error de argumento en el texto: decía que la
transición coincide *"con la capacidad del actuador calculada independientemente"*,
citando la **mediana del espacio de trabajo en la PEOR dirección (5.88 N)**. Eso
no es lo que corresponde comparar: el experimento es en UNA pose y UNA dirección,
cuya capacidad estática es **6.53 N**, y la transición reproducida cae justo
encima. La mediana del espacio de trabajo acota por abajo, no predice.

**Qué se hizo:** se reemplazó la tabla por la reproducible y se reescribió el
párrafo para comparar contra la capacidad de esa pose y esa dirección, dejando la
mediana del espacio de trabajo como contexto y diciendo explícitamente qué papel
cumple.

⚠️ **Esto es una decisión tuya para confirmar.** Cambié una tabla de resultados
apoyándome en una reimplementación mía, no en el código original, que no existe.
Las dos lecturas posibles son que mi reconstrucción difiera en algún ajuste del
inline perdido, o que la tabla publicada estuviera mal. No puedo distinguirlas.
Lo que sí es seguro es que la versión que queda **se reproduce corriendo un
script**, y la anterior no.

⚠️ Un detalle de método que salió de esto y vale para cualquier medición futura:
la primera versión de mi script promediaba el tramo plano completo y subestimaba
el desplazamiento un 35 %, porque la admitancia tiene constante de tiempo
`1/k_return = 0.5 s` y el tramo dura 0.9 s. El valor analítico de referencia es
`‖f‖/(D_trans·k_return)`, y compararlo contra eso fue lo que delató el sesgo.

---

## Lo que sigue bloqueando, en orden

1. **S3 en hardware, las dos partes.** Es lo que el asesor estima que mueve la
   aceptación de 40-50 % a 65-75 %. Requiere el brazo montado.
2. **La afiliación.** Sigue en LASER/UFPB, marcada con comentario en
   `main.tex:57`. Decisión del autor.
3. **17 páginas.** Se cortaron cuatro tablas y dos subsecciones y aun así el
   material de la métrica cuesta ~2 páginas netas. Lo que queda sostiene C1 o C2;
   recortar más cuesta evidencia. Si hay que llegar a 15, decime **qué bloque**
   sacrificar: los candidatos son la sección de sensibilidad a fallas (Tabla de
   perturbaciones), el barrido de seis poses (`tab:dirsweep`, redundante en parte
   con la figura), o el demostrador NMPC entero.

## Lo que NO se cambió, y por qué

- **El nominal ℓ_c = 1 m.** Ver arriba: es la métrica de la práctica que se está
  auditando. Cambiarlo es una decisión, no una corrección.
- **La estructura de tres contribuciones con jerarquía** (C1 principal, C2
  co-principal, NMPC demostrador). Venía acordada de la ronda anterior y esta
  revisión no la objetó.


---
---

# Ronda 3 — los tres puntos que pediste cerrar

## 1. La invariancia direccional: no la degradé, la demostré

Tenías razón en que **no se puede presentar como proposición universal**: para un
Jacobiano arbitrario los autovectores sí se mueven con ℓ_c. Recomendabas
degradarla a resultado empírico salvo que hubiera demostración algebraica limpia.

**La hay, y sale de la arquitectura del brazo.** Con la junta 1 girando sobre la
vertical `e_z` y las juntas 2-4 sobre un eje horizontal común `a`:

```
ker Jw = {x : x_1 = 0,  x_2+x_3+x_4 = 0}            (e_z y a independientes)
P  = Jv(ker Jw) = span{v_2-v_3, v_3-v_4} ⊆ a^⊥      (v_i = a × r_i ⊥ a)
dim P = 2  ⟹  P = a^⊥
```

Y como toda `f ∈ P` se recupera exacta para todo ℓ_c (Prop. 2), `M(ℓ_c)` es la
identidad sobre `a^⊥`. Con `M` simétrica, deja `span{a}` invariante:

```
M(ℓ_c) = I − (1 − λ(ℓ_c)) · a aᵀ
```

Los autovectores no dependen de ℓ_c **porque los tres ejes de pitch son
paralelos**, no por casualidad numérica. La dirección menos observable *es* el eje
de pitch, y el plano del brazo pasa intacto.

La simetría de `M` también quedó demostrada en vez de observada. Con
`S = W⁻¹` y usando que el bloque de fuerza de `S` es la identidad:

```
M(ℓ_c) = [Π_ℓc]_{1:3,1:3},   Π_ℓc = proyector ORTOGONAL sobre col(S J)
```

o sea que `M` es el bloque de fuerza de un proyector ortogonal para cualquier ℓ_c,
igual que `P_ff` lo era para ℓ_c = 1 m. De ahí salen simetría y `0 ⪯ M ⪯ I` de una.

**La hipótesis `dim P = 2` es exactamente lo que los números reportaban como
espectro `{λ,1,1}`:** las dos condiciones coinciden en el **100 %** de las 2197
configuraciones, y ambas se cumplen en el **92.3 %**; el resto son las
configuraciones singulares del borde de la grilla, donde los tres eslabones se
alinean. O sea que el 92 % dejó de ser un número sin explicación.

La identidad del error ya no está condicionada a un espectro observado, sale de la
estructura demostrada:

```
‖M f − f‖ = (1 − λ(ℓ_c)) · |aᵀ f| = (1 − λ(ℓ_c)) · |cos θ|
```

Y agregué explícitamente que **para un Jacobiano genérico esto es falso**: la
invariancia es una propiedad de esta arquitectura, demostrada, no una regla
general.

**En el paper:** Proposición 2 (conjunto recuperado, general), Proposición 3
(estructura direccional, arquitectura yaw + ejes de pitch PARALELOS), corolario del error,
y el párrafo que dice qué NO se generaliza. Verificación numérica en la sección G
de `metric_sensitivity.py`.

## 2. El argumento de rango: corregido, y quedó más fuerte

Tenías razón: el rango es `col(W⁻²J)`, no `W⁻¹ col(J)`. Corregido, y al hacerlo la
demostración de la parte (i) se volvió de una línea y **el resultado más nítido**:

```
[f;0] ∈ col(W⁻²J)  con  W⁻² = diag(I, ℓ_c² I)
    ⟺  ∃x : Jw x = 0  y  Jv x = f
    ⟺  f ∈ Jv(ker Jw)
```

ℓ_c no puede aparecer en esa condición porque solo multiplica un bloque que se
anula. Y el conjunto recuperado tiene ahora interpretación física: **las fuerzas
que produce un movimiento articular que no genera rotación.**

`metric_sensitivity.py` calcula `M` por las dos vías (`W⁻¹` y `W⁻²`) como control
cruzado: coinciden a 1.6e-11.

## 3. La tabla de regímenes: estado estacionario de verdad, y solver controlado

Apliqué las dos correcciones, y elegí extender la meseta como preferías.

**Meseta de 3.0 s = seis constantes de tiempo** (antes 0.9 s). Ahora el valor
comandado coincide con el equilibrio analítico `‖f‖/(D·k) = 8.33 mm/N` **a 0.3 %**,
y el título "steady-state" es correcto:

| ‖f‖ [N] | comandado [mm] | analítico [mm] | real [mm] | razón | saturado | solver |
|---|---|---|---|---|---|---|
| 1 | 8.3 | 8.3 | 11.4 | 1.38 | 0 % | OK |
| 3 | 24.9 | 25.0 | 32.9 | 1.32 | 0 % | OK |
| 5 | 41.5 | 41.6 | 54.9 | 1.32 | 0 % | OK |
| 6 | 49.8 | 50.0 | 66.1 | 1.33 | 0 % | OK |
| 8 | 66.5 | 66.6 | 160.5 | 2.42 | 97 % | OK |
| 10 | 83.1 | 83.3 | 239.2 | 2.88 | 100 % | OK |

**Fila de 15 N eliminada**, como recomendabas: no agregaba evidencia y era donde
estaba la anomalía.

**El script ahora registra `solver.solve()`**, cuenta fallos por fila, **invalida
la fila** si hay alguno, y **no imprime "confirmed"** si quedó alguna invalidada.
Con la meseta larga y sin la fila de 15 N: **cero fallos en las seis filas.**

La conclusión no cambió: transición entre 6 y 8 N, sobre la capacidad estática de
esa pose y dirección (6.53 N).

## 4. ℓ_c: adopté tu terminología

El baseline sigue en 1 m y ahora se llama sistemáticamente **"unweighted-SI
baseline"**, no métrica natural. La escala propia del brazo sigue en paralelo con
la misma visibilidad (50.5 % / 38.0 %).

## 5. Páginas: 18, y no recorté

Seguí tu criterio de no recortar solo para volver a 15. El +1 respecto de la ronda
anterior es la demostración de las Proposiciones 2 y 3, que es justamente lo que
pediste. Si después hace falta reducir, respeto tu orden: primero el demostrador
NMPC, después la sensibilidad a perturbaciones a material suplementario, y nunca la
tabla de métricas ni gating/semillas.

## Reproducir

```bash
cd ocp_generation
python3 metric_sensitivity.py       # secciones A-G; la G verifica las Props. 2 y 3
python3 test_compliance_regimes.py  # tabla nueva, con estado del solver por fila
```


---

# Ronda 4 — los dos ajustes documentales

**Veredicto de la ronda 3: Minor Revision, científicamente cerrado dentro del
alcance simulado.** Los dos detalles que quedaban:

## 1. Los residuos numéricos: eran pruebas distintas, y ahora es una sola

Tenías razón en la discrepancia. Los `9e-8` y `9e-12` del manuscrito venían de una
verificación anterior sobre **400 configuraciones al azar SIN filtrar por la
hipótesis** `dim Jv(ker Jw) = 2`: al incluir configuraciones casi singulares el
residuo se degrada, y esos números eran los peores casos de esa mezcla. La sección
G del script barre las **2197 configuraciones de la grilla** y evalúa la estructura
solo donde la hipótesis se cumple, que es lo correcto.

No tenía sentido reportar dos pruebas, así que **eliminé la vieja y el manuscrito
cita ahora exactamente la salida de la sección G**:

| | manuscrito ahora | sección G |
|---|---|---|
| `‖M f − f‖` sobre `P` | 6e-14 | 5.96e-14 |
| componente de `M a` fuera de `span{a}` | 8e-15 | 8.09e-15 |
| `‖M − Mᵀ‖` | 6e-14 | 5.40e-14 |
| autoespacios contra ℓ_c = 1 m | 2.6e-6 grados | 2.6e-6 grados |

El texto declara explícitamente el alcance: la grilla completa, los nueve valores
de ℓ_c de la Tabla II, restringido a las configuraciones que satisfacen la
hipótesis.

## 2. NumPy / SciPy, y el commit de acados

Diagnosticado y documentado en `requirements.txt` nuevo, que lista **dos entornos
separados** para no confundir dónde se produjeron los números con qué conviene
instalar:

| | |
|---|---|
| **entorno original** | Python 3.10.12 · numpy 1.26.4 · **scipy 1.8.0** (el del aviso) · casadi 3.7.2 · acados `v0.5.4-6-ge0759960c`, commit `e0759960cae76e4f2e8177cf2866a6ab088f4e86` · acados_template 0.5.1 desde ese checkout |
| **entorno recomendado** | igual, con `scipy>=1.11` |

El commit de acados quedó registrado con las instrucciones de checkout: es la
dependencia más sensible, porque el código C generado y la API de
`AcadosOcpSolver` cambian entre versiones, así que forma parte del resultado.

Sobre el aviso en sí:

- El aviso lo emite **`scipy 1.8.0`**, que declara tope `numpy < 1.25`, contra el
  `numpy 1.26.4` instalado.
- **Ningún script de este repositorio importa scipy** (verificado con `grep` sobre
  `ocp_generation/` y `hw/`): entra como dependencia transitiva de
  `acados_template`. Por eso el aviso es cosmético y ninguna corrida se ve
  afectada.
- Se cierra por cualquiera de los dos lados: `numpy<1.25`, o `scipy>=1.11`
  conservando numpy. **Preferible el segundo**, porque no toca la versión de numpy
  con la que se produjeron todos los números.

## Estado al cerrar esa ronda

18 páginas, 12 tablas, 0 errores, lint clean. No recorté, siguiendo tu criterio.

⚠️ Superado: rondas posteriores eliminaron `tab:sweep`, rehicieron las dos figuras
y cambiaron la autoría. El estado vigente está en el encabezado de este documento.
