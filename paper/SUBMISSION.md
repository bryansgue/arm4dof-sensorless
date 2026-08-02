# Checklist de envío — IEEE Access

Estado del PDF: **14 páginas, 0 errores, 0 citas o referencias sin resolver.**

```bash
pdflatex main && bibtex main && pdflatex main && pdflatex main
```

---

## ⚠️ Lo que TENÉS que decidir vos antes de subir

### 1. Autoría — la única cosa que no puedo decidir
El bloque de autores replica el de tu otro paper de Access
(`~/python/Time_optimal_planing-NMPC/ACCESS_latex`): Guevara, Brandão,
Varela-Aldás, con esas tres afiliaciones y Varela-Aldás como autor de
correspondencia.

**Eso lo puse para que compile, no porque sepa quién firma este trabajo.** La
autoría no se hereda de un paper anterior. Hay un comentario `⚠️ CONFIRMAR
AUTORIA` en `main.tex` justo arriba del bloque. Revisalo.

Lo mismo vale para el financiamiento (`\tfootnote`) y el agradecimiento: copiados
del otro paper.

### 2. ORCID
IEEE Access lo pide para **todos** los autores. Se carga en el portal, no en el
`.tex`.

### 3. Fotos de biografía
Las tres biografías están escritas pero usan `IEEEbiographynophoto`, porque **no
hay fotos en ningún repositorio**. Para cámara lista:

```latex
\begin{IEEEbiography}[{\includegraphics[width=1in,height=1.25in,clip,
  keepaspectratio]{Fig/<nombre>.png}}]{Nombre}
```

Hay un comentario con la sintaxis exacta antes de las biografías.

---

## Auditoría de reproducibilidad (01/08/2026)

Se re-corrió **todo** script que produce un número del paper. Resultado:

**Reproducen exacto:** modelo (8.78e-11 / 2.73e-09 / 6.93e-13), barrido de 2197
configs, barrido direccional (correlación 0.9993), MiL, Monte Carlo de ruido,
gating, MuJoCo wrench (2.227 → 0.157 N), pareado N=6 (t=4.42), trayectoria
(6.82 mm), regulación (1.65 mm), ley de condicionamiento (−0.872 / R² 0.980),
ablación, capacidad de fuerza, direcciones con nombre, divergencia de métricas.

**Tres defectos encontrados y corregidos:**

1. **Cinco tablas salían de scripts inline nunca guardados.** La sección
   "Reproducibility" del paper afirmaba que cada tabla la produce un script con
   nombre, y era **falso**. Creado `ocp_generation/reproduce_paper_tables.py`.
2. **La tabla de inyección de fallas se había calculado con las poses VIEJAS**,
   tres de las cuales estaban dentro del piso (descubierto después, con MuJoCo).
   Recalculada con las poses válidas: 8.75 / 4.38 / 1.31 / 1.09 en vez de
   7.38 / 3.69 / 1.66 / 0.90. **El orden y la conclusión no cambian**, y el umbral
   de 0.3 N sigue separando lo mismo.
3. **La Tabla IV no la reproducía su script.** Los números (T 0.180, V0 0.122)
   venían de 1000 pasos con `W_u` ajustado; el script corría 250 pasos con el peso
   por defecto y daba 0.754 y 5.637. Alineado el script; V0 pasa a 0.120.

⚠️ La columna de desplazamiento de la tabla de reparto por `kv` también se
recalculó (350 / 106 / 18 / 3.5 mm en vez de 200 / 43 / 8 / 1.6): dependía del
número de pasos de asentamiento, que no estaba fijado.

## Lo que ya está resuelto

| | |
|---|---|
| formato | `ieeeaccess.cls` + `IEEEtran.cls` + `IEEEtran.bst` + fuentes, todo versionado |
| compilación | limpia, sin errores |
| referencias | 16, todas citadas y resueltas |
| figuras | 4, todas citadas |
| tablas | 12, todas citadas |
| ortografía / gramática | `lint_prose.py` da `clean` |

⚠️ Los 2 avisos `Overfull \hbox (9.2679pt)` en `\maketitle` y los avisos de fuente
`T1/formata/m/sl` **son del template**: el paper de referencia produce exactamente
los mismos. No tocarlos.

⚠️ Los ~21 avisos `Overfull \hbox (505.12pt)` también son del `.cls`.

---

## Lo que el paper declara, y hay que sostener en la carta de presentación

**Todo es simulación.** Está dicho en el abstract, en la introducción, en la
sección de alcance y en limitaciones. No lo suavices: es el punto que un revisor
va a atacar, y la defensa es que está declarado, no escondido.

Las defensas concretas, por si el revisor insiste:

1. **MuJoCo es implementación independiente**, con damping, armature y
   frictionloss que el modelo del estimador no tiene. Ahí la corrección mejora
   14×.
2. **El Monte Carlo con ruido de servo** responde cuantitativamente al "pero los
   sensores reales": 18.0% contra 69.6% de error mediano.
3. **La contribución central es estructural**, no empírica: que el min-norm 6-D
   reparta fuerza como momento con `n<6` es cierto para cualquier brazo.

**Dos de los ingredientes son elementales y el paper lo dice** (Sec. I). Ese
párrafo es lo que evita que se lea inflado — no lo saques.

---

## Lo que lo mejoraría, en orden

1. **S3 en hardware** (`../hw/s3_conditioning.py`). Es lo más barato que convierte
   "todo es simulación" en "la predicción central se verificó en hardware": sin
   lazo de control, sin NMPC, par deshabilitado, y la parte A ni siquiera necesita
   la pesa. Una tarde.
2. **Corpus de referencia** para el barrido H.1 del revisor (registro léxico del
   campo). Hoy el linter solo cubre ortografía y AI-tells. Bajar los fuentes de
   arXiv de las referencias a `paper_refs/src_corpus/`.
3. **Repetir la Sec. VII con la formulación V0** en vez de T. Está declarado que
   son equivalentes y demostrado en el lazo compliant, pero un revisor puede
   pedirlo.

---

## Lo que sacaría si hay que acortar

La Sec. VII-D (compliant vs rígido, N=6) y la ablación de la métrica DQ **no son
contribuciones** — respaldan que el sistema funciona. Si un revisor pregunta qué
aportan, la respuesta honesta es que nada, y ahí hay página y media.
