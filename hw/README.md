# Bring-up a hardware — MX-28R

Plan por etapas para pasar el control compliant al brazo real. **Cada etapa es
falsable, barata, y no depende de la siguiente.** Si una falla, no se sigue.

Esa regla no es burocracia: en el proyecto del dron, mover cinco parámetros a la
vez rompió una corrida entera sin poder atribuir el daño a ninguno.

---

## La duda de fondo: el empuje es par, la acción es velocidad

No hay contradicción, y conviene tenerlo claro antes de tocar nada.

El servo tiene un lazo interno que convierte la velocidad comandada en el par que
haga falta. Ese par **lo reporta** (`Present_Current`), y es exactamente el
`τ_act` que el estimador resta. O sea: **el lazo de velocidad puede quedar como
caja negra**. No hace falta conocerlo ni identificarlo.

Verificado con rampa de fuerza de 1 a 14 N, servo limitado a 1.4 N·m por junta:

```
|f| [N]   |tau_act|  satura?   |f_est|    error
    1.0       0.407     no       1.000     0.0%
    6.0       1.310     no       5.999     0.0%
   14.0       1.197     no      13.998     0.0%
```

**El estimador no satura en ningún punto del rango de guiado manual.**

### Pero la compliance tiene DOS regímenes, y solo uno es sintético

| fuerza | qué pasa | de dónde sale la compliance |
|---|---|---|
| **< 4-6 N** | el servo resiste, el brazo casi no se mueve | **sintética**: sentir → mover referencia → seguir. Manda la latencia del lazo |
| **> 4-6 N** | el servo **satura de par** y el brazo se reacomoda a una postura donde puede sostener | **física, y gratis** |

El umbral sale de la capacidad real del brazo con `τ_max = 1.4 N·m`:

```
fuerza maxima ejercible, PEOR direccion, barrido 729 configs:
  mediana 5.88 N     p5 4.29 N     min 3.93 N
```

Un guiado manual normal son 5-15 N ⇒ **en el régimen que importa el brazo es
naturalmente backdriveable**. Eso no es un defecto: es una especificación, y hace
que el brazo sea intrínsecamente seguro.

### Qué implica para el plan

1. **No cambiar de modo.** Nada de PWM ni de inventar control de par. El modo
   velocidad más el reporte de corriente alcanza.
2. **Ablandar el lazo interno del servo** (ganancias P/I de velocidad, registros
   de la tabla de control) para que el régimen suave también ceda algo
   físicamente. Es una escritura de registro, cuesta cero, y mejora el tacto.
   Costo: peor seguimiento de trayectoria en S4. Hay que elegir el compromiso.
3. **La exigencia de latencia se relaja.** La latencia solo manda por debajo de
   ~5 N; arriba la física ya cede sola. Si el bus da 50-100 Hz alcanza, y eso
   ablanda el criterio de S0.

## La decisión de arquitectura, y por qué

El servo MX-28 se comanda en **velocidad**, no en par. Eso NO obliga a cambiar el
OCP, pero sí cambia de qué depende el estimador. Medido en simulación:

| kv del lazo de velocidad | señal de fuerza en `M q̈` | en `τ_act` | desplazamiento |
|---|---|---|---|
| 1 | 1.3% | 98.7% | 200 mm |
| 4 | 0.1% | 99.9% | 43 mm |
| 20 | 0.006% | **100.0%** | 8 mm |
| 100 | ~0 | **100.0%** | 1.6 mm |

**Con lazo de velocidad, la fuerza aparece ENTERA como par del servo, no como
aceleración.** El brazo casi no se mueve: el servo la absorbe.

Dos consecuencias, y las dos mandan sobre el plan:

1. **`q̈` deja de importar.** Aporta 0.006% de la señal. No hace falta filtrar
   encoders ni pelear con la diferencia finita. Era la preocupación principal y
   resultó irrelevante.
2. **Todo pasa por `τ_act`.** Y la sensibilidad es **1:1**: 5% de error en la
   constante de par ⇒ 5% de error en fuerza. Sin atenuación.

### Qué se mantiene y qué no

| | decisión |
|---|---|
| OCP con **par** como entrada | **SE MANTIENE.** Los límites de par (1.4 N·m) son físicos y hacen trabajo real: sin ellos el NMPC pide aceleraciones que el servo no da. Se sigue enviando `q̇*[1]` al servo, que es el patrón estándar de NMPC sobre lazo interno rápido |
| modelo dinámico `M`, `h` | **OBLIGATORIO.** No es solo del controlador: el estimador lo necesita |
| `q̈` por diferencia finita | se calcula igual, pero **no es crítico**. No invertir esfuerzo ahí |
| `τ_act` | **el punto crítico.** Ver etapas 1 y 2 |

### Piso de detección

`‖δf‖ ≤ ‖δτ‖ / σ_min(Jv)`. Con 0.01 N·m de error de par:

| postura | `σ_min(Jv)` | fuerza equivalente |
|---|---|---|
| mediana del workspace | 0.0496 | **0.20 N** |
| p5 (mal condicionada) | 0.0127 | **0.79 N** |
| peor caso | 0.00008 | 131.71 N (inservible) |

La stiction de un servo chico con reductora anda por 0.05-0.12 N·m ⇒ **1-2 N de
piso** en postura mediana. Para hand-guiding con empujes de 5-10 N alcanza, pero
ese es el número que manda, no el ruido del encoder.

⇒ **Hay que monitorear `σ_min(Jv)` en línea** (cuesta microsegundos) y ensanchar la
banda muerta donde cae.

---

## S3 — lo unico que hace falta correr para el paper

`s3_conditioning.py`. **Verificado de punta a punta** con `--sim` (ensayo en seco
contra MuJoCo, sin prompts): el flujo corre, las cuatro posturas se alcanzan y los
`sigma_min` medidos (0.0108, 0.0204, 0.0435, 0.0918) coinciden con los de
seleccion.

```bash
python3 s3_conditioning.py --sim              # ensayo en seco, sin hardware
python3 s3_conditioning.py --port /dev/ttyUSB0             # parte A, sin pesa
python3 s3_conditioning.py --port /dev/ttyUSB0 --mass 0.1  # A y B
```

**Parte A** es la que importa: par deshabilitado, sin pesa, sin lazo de control,
bus a 10 Hz alcanza. Una tarde.

⚠️ La parte A **no se puede ensayar en simulacion**: MuJoCo es determinista y
`std(tau)` da exactamente 0. El script lo avisa. Esta diseñada para medir algo que
el simulador no tiene.

⚠️ **Parte B: sostener en modo POSICION.** Con lazo de velocidad blando el brazo se
hunde y ninguna masa util baja del 3 % de contaminacion del Jacobiano. El script
reporta `dq` y `dJ/J` y avisa.

⚠️ **S3 puede FALLAR**, y esta diseñado para decirlo fuerte. Pendiente ~0 significa
que la Seccion IV del paper esta mal. Es el desenlace mas informativo posible.

## Etapas

### S0 — Sondeo del bus y tasa alcanzable ⛔ BLOQUEA TODO

`s0_probe_and_rate.py`. No mueve nada.

Verifica: modelo y firmware de cada servo, tabla de control, y sobre todo **a qué
frecuencia se puede cerrar el lazo** con lectura sincronizada de 4 servos más
escritura.

⚠️ **Esto puede ser el bloqueo real y nadie lo verificó.** El NMPC resuelve en
2.5 ms, así que el cómputo no es el límite: el bus sí. Y hay una trampa clásica de
FTDI: el *latency timer* viene en **16 ms** por defecto, lo que topea el lazo a
62 Hz sin importar el baudrate. Se baja a 1 ms:

```bash
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

**Criterio:** ≥100 Hz sostenido con jitter < 20% del período. Si no se llega, hay
que decidir entre bajar la tasa del NMPC o cambiar de interfaz, y eso reescribe el
resto del plan.

### S1 — Control negativo: sin contacto, `f_est ≈ 0`

`s1_gravity_check.py`. El brazo se mueve solo entre poses, **nadie lo toca**.

Con el brazo quieto y sin contacto, `τ_ext = h − τ_act`. Si `f_est` no da ≈0, hay
algo mal antes de empezar, y **con una pesa colgada ya no se podrá separar la
causa**.

Sale de acá: el sesgo por postura, y la fricción de Coulomb + viscosa por junta.

**Poder de detección, medido** inyectando cada falla sobre el modelo validado:

| falla | `f_est` falso |
|---|---|
| signo de par invertido | 7.38 N |
| constante de par ×2 | 3.69 N |
| offset de par 0.05 N·m | 1.66 N |
| **stiction 0.05 N·m** | **0.90 N** |
| constante de par +20% | 0.74 N |
| offset de par 0.02 N·m | 0.67 N |
| masa eslabón 4 +20% | 0.42 N |
| cero de junta corrido 0.10 rad | 0.33 N |
| *— criterio 0.3 N —* | |
| constante de par +5% | 0.19 N |
| masa eslabón 1 +20% | 0.00 N |

⚠️ **S1 NO valida la constante de par**: un 5% de error da 0.19 N, bajo el
criterio. Los pares de gravedad de este brazo son chicos (|h| ~0.3 N·m). La
constante sale de S2, con carga.

⚠️ Los **offsets** son lo más detectable y los errores de **gravedad** se rechazan
casi enteros: el error de gravedad cae en las direcciones bien condicionadas de
`Jv` y los mínimos cuadrados lo absorben; un offset no. Por eso **la stiction es
el término dominante**, no el ruido.

⚠️ Ciego al eslabón 1, y está bien: gira sobre eje vertical y la gravedad no le
hace par.

Las poses se eligieron **por barrido** maximizando sensibilidad. Las elegidas a
ojo dieron 3-7× menos: un control negativo poco sensible pasa por el motivo
equivocado.

**Criterio:** `|f_est| < 0.3 N` en todo el barrido, tras compensar fricción.

### S2 — Constante de par contra pesas conocidas

`s2_torque_constant.py`. Cuelga masas conocidas del efector en postura **bien
condicionada** y ajusta por regresión la constante corriente→par.

⚠️ **No usar el valor de datasheet.** Es la misma lección que el `hover_throttle`
del dron: figuraba "validado" y estaba 9.7% bajo, y el lazo cerrado lo tapaba.
Medirlo cuesta minutos.

⚠️ Medir en **orden alternado** de masas, no creciente: cualquier deriva térmica
del motor queda correlacionada con la carga y entra entera en la pendiente.

**Criterio:** R² > 0.98 y dispersión entre corridas < 5%.

### S3 — Validación del estimador, bien y mal condicionado

`s3_estimator_check.py`. Repite S2 en dos posturas: `σ_min(Jv)` alto y bajo.

**Es la predicción falsable del paper.** La teoría dice que el error de fuerza
escala con `1/σ_min`. Si no escala, el análisis de condicionamiento no vale.

**Criterio:** el cociente de errores entre las dos posturas debe seguir al cociente
de `1/σ_min` dentro de un factor 2.

### S4 — NMPC siguiendo trayectoria, SIN compliance

`s4_track.py`. Valida el controlador solo, con el estimador apagado.

**Criterio:** sigue una trayectoria lenta sin fallos de solver, error comparable al
de simulación (6.8 mm) degradado a lo sumo 3x.

### S5 — Lazo compliant completo

`s5_handguide.py`. Recién acá se cierra todo.

**Criterio:** cede al empujar, retorna al soltar, y el esfuerzo de guiado baja
respecto de rígido en un **diseño pareado** (mismas condiciones en las dos ramas).
No repetir la presentación no-pareada: con N=6 oculta el efecto.

---

## Seguridad, antes de energizar

1. **Límite de corriente por servo** en la tabla de control, no en software.
2. **Watchdog de bus**: si se pierde una lectura, velocidad cero inmediata.
3. **Caja de espacio de trabajo** y límites de junta verificados ANTES de habilitar par.
4. **Parada física al alcance** durante S4 y S5.
5. Arrancar con `kv` **bajo** (servo blando): falla suave, y además da más
   desplazamiento por empuje, que es lo que se quiere sentir en hand-guiding.

---

## ¿Hace falta arreglar el brazo de MuJoCo? — auditoría (01/08/2026)

**No para lo que se usa acá**, y está medido, no opinado.

**Lo que coincide con el modelo del estimador:** masas de los 4 eslabones + fingers
y `ARMATURE = 0.012`. Por eso `M` y `h` validan a 1e-10.

**Lo que MuJoCo tiene y el estimador NO** — y es deseable, es la discrepancia
planta-modelo que hace honesta la validación:

| | XML | ¿en `arm_params.py`? |
|---|---|---|
| `armature` | 0.012 | sí |
| `damping` | 0.12 | **no** |
| `frictionloss` | 0.02 | **no** |

⚠️ Me preocupaba que `frictionloss=0.02` fuera **optimista**: un MX-28 con reductora
193:1 anda más por 0.05-0.15 N·m, y la stiction es el término dominante del error.
Barrido:

| frictionloss | damping | error kt j2 | j3 | j4 | R² min |
|---|---|---|---|---|---|
| 0.02 | 0.12 | 0.07% | 0.01% | 0.20% | 1.0000 |
| 0.05 | 0.12 | 0.05% | 0.08% | 0.21% | 1.0000 |
| 0.10 | 0.12 | 0.07% | 0.07% | 0.21% | 1.0000 |
| 0.15 | 0.12 | 0.06% | 0.08% | 0.22% | 1.0000 |
| 0.10 | **0.30** | 0.07% | 0.07% | 0.21% | 1.0000 |

**Insensible a la fricción en un rango de 7.5×.** La razón: `kt` es la **pendiente**
de la regresión, y la fricción es ~constante entre masas (misma pose, misma
velocidad), así que no la toca. Por eso no hace falta subir `frictionloss` para que
el banco sea creíble, y por eso en hardware la stiction degrada el ESTIMADOR pero
no la CALIBRACIÓN.

### Lo que sí falta en el banco, y hay que saberlo

1. **El actuador `<velocity>` de MuJoCo es proporcional puro, sin integrador.** Con
   comando cero da par cero y el brazo cae; el MX-28 real tiene lazo PI y sostiene.
   Obliga a mantener el lazo de posición cerrado mientras se mide. No invalida la
   identificación (el estimador usa el par MEDIDO), pero cambia cómo se escriben
   los scripts.
2. **Zona muerta de corriente** cerca de cero: el servo real no reporta corriente
   linealmente ahí. MuJoCo da `actuator_force` exacto; `mj_arm.py` solo agrega
   cuantización.
3. **Backlash** de la reductora: no está.
4. **Deriva térmica**: no está. Es lo que justifica el orden alternado de masas, y
   por lo tanto ese control **no se puede validar en este banco**.

Ninguna de las cuatro invalida lo medido; las cuatro son razones para no leer el
0.22% como predicción de hardware.

## Validación de los procedimientos, sin hardware (01/08/2026)

`dxl_sim.py` emula el bus sobre la dinámica validada, con constante de par,
stiction y offsets de cero **inyectados y desconocidos para los scripts**.
`validate_procedures.py` corre S1 y S2 contra él y compara contra la verdad.

**Resultado: S2 recupera la constante inyectada con ≤2.24% de error, R²=0.9999.**
Y S1 **falla correctamente** (1.089 N contra criterio 0.3 N) con la constante
semilla puesta: es justo lo que tiene que detectar antes de colgar pesas.

⚠️ **Esto NO valida el hardware.** Valida que los scripts y las regresiones hacen
lo que dicen. El banco no modela reductora, backlash, deriva térmica ni retardo
del bus.

### Tres defectos de diseño que encontró, y que se habrían descubierto con el brazo desarmado

**1. La junta de la BASE es inidentificable colgando pesas.** Dio `kt = 0.0000`,
error 100%. Es estructural: una fuerza vertical **no produce momento sobre un eje
vertical**, así que la regresión no tiene señal. Su constante exige otra
excitación — tiro horizontal por polea. Queda excluida y declarado.

**2. La dependencia circular S1↔S2 que creí ver NO existe, pero el término sobraba.**
S2 restaba un modelo de fricción que viene de S1, y S1 necesita `kt`, que viene de
S2. Resulta que en reposo `q̇≈0` hace que ese término valga ~0 cualquiera sea su
valor: por eso S2 daba bien **con la fricción mal identificada**. Se quitó el
término. No hay circularidad.

**3. El "arreglo" de promediar atravesando la pose en los dos sentidos NO funciona.**
Lo intenté para cancelar la fricción seca sin conocerla. Medido: **76-81% de error,
R² 0.10-0.35**. El par reportado en movimiento arrastra el estado del integrador
del lazo de velocidad del servo y términos dinámicos que el miembro derecho no
modela, y eso no se cancela al invertir el sentido. Reposo es mejor.

⚠️ Antes probé una variante peor todavía: mover **una sola junta**. La fricción
seca solo se invierte en la que se mueve; las quietas conservan la suya. 37-83%.

### Lo que el banco NO puede probar

En reposo la fricción **estática** real es indeterminada dentro de la banda de
despegue, y el banco la modela como `tanh(q̇/1e-3)`, o sea ~0 en reposo. **El banco
no puede probar ese caso.** En hardware hay que esperar degradación acotada por
`τ_coulomb/σ_min(Jv)`: con 0.05 N·m y `σ_min` mediano 0.0496, **~1 N de
incertidumbre**. Ese es el número a vigilar con el brazo real.

## Estado

⚠️ **Nada de esto está probado contra hardware.** No hay SDK instalado, no hay
driver previo en el workspace y no hay servo conectado. Los scripts están escritos
contra la documentación del protocolo y **la tabla de control debe confirmarse con
S0 antes de mover un motor**.

```bash
pip install dynamixel-sdk
python3 s0_probe_and_rate.py --port /dev/ttyUSB0 --baud 1000000
```
