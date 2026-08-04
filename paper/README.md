# Manuscrito — IEEE Access

**Sensorless Contact-Force Estimation on Low-DoF, Velocity-Actuated Manipulators:
A Quantitative Design Framework**

18 páginas, 12 tablas, 4 figuras, 33 referencias. Autoría única. Formato
`ieeeaccess.cls` tomado de `~/python/Time_optimal_planing-NMPC/ACCESS_latex`
(solo el formato; nada de contenido de ahí).

> **Respuesta punto por punto a la revisión del asesor (04/08/2026): `RESPONSE.md`.**
> **Checklist de envío: `SUBMISSION.md`.**

## Compilar

```bash
pdflatex main && bibtex main && pdflatex main && pdflatex main
```

Compila con 0 errores y 0 referencias sin resolver.

⚠️ Los ~21 avisos `Overfull \hbox (505.12pt)` son del **`.cls`**, no del texto: el
template original los produce igual. No tocarlos.

Lint de prosa (términos vedados, ortografía americana, AI-tells):

```bash
python3 ~/.claude/plugins/cache/huao/ral-reviewer/*/skills/ral-reviewer/lint_prose.py main.tex
```

## La tesis, en una frase

En brazos con menos juntas que dimensiones de tarea, lo que limita la estimación
sin sensor **no es el rango del Jacobiano sino el modelo de contacto que asume el
estimador**. El pseudo-inverso 6D estándar está indeterminado y reparte la fuerza
de contacto como momento espurio; con contacto puntual en punto conocido el
problema es sobre-determinado y la fuerza se recupera exacta si `rank(Jv)=3`, que
se cumple en el 100 % del espacio de trabajo.

Números duros: el 6D pierde **50.5 %** de la fuerza en promedio bajo la métrica sin
ponderar de la práctica estándar, **38.0 %** bajo la escala propia del brazo, y
99.6 % en el peor caso bajo cualquiera de las dos; contra MuJoCo el RMSE baja de
2.227 N a 0.157 N, factor 14.

⚠️ **El escalar depende de la métrica del wrench y el paper lo declara.** La norma
`‖f‖² + ‖m‖²` suma N con N·m, así que presupone una longitud; la pseudoinversa sin
ponderar la fija en 1 m sin decirlo. Lo que NO depende de esa elección: qué
direcciones se pierden, el conjunto de fuerzas recuperadas exacto, y el peor caso.
Ver Sec. IV-C y `RESPONSE.md`.

⚠️ Esto **corrige** los dos borradores anteriores (Mechatronics y L-CSS), que
afirmaban que el brazo tenía una dirección de fuerza ciega. Ver
`../PLAN_MECHATRONICS.md`, sección "CORRECCIÓN IMPORTANTE".

## Antes de enviar

1. ⛔ **La afiliación** — único bloqueo. LASER/UFPB, comentario en `main.tex:57`.
2. **ORCID**: se carga en el portal, no en el `.tex`.
3. **Biografía con foto**: hoy `IEEEbiographynophoto`, que compila. La foto va en
   cámara lista.
4. **Figuras** se regeneran:
   ```bash
   cd ../ocp_generation
   python3 observability_map.py     # figures/observability_map.png
   python3 test_direction_sweep.py  # figures/direction_sweep.png
   python3 make_figures.py          # figures/interaction_traces.png
   python3 make_arch_figure.py      # figures/architecture.png
   ```

## De dónde sale cada número

Nada escrito a mano.

| tabla | qué reporta | script |
|---|---|---|
| `tab:gap` | comparación con el trabajo previo | literatura, sin script |
| `tab:metric` | **sensibilidad a ℓ_c**, la métrica del wrench | `metric_sensitivity.py` |
| `tab:sweep` | barrido de 2197 configs, rank y condicionamiento | `observability_map.py` |
| `tab:dirsweep` | 12000 direcciones de empuje, seis poses | `test_direction_sweep.py` |
| `tab:regimes` | los dos regímenes de compliance | `test_compliance_regimes.py` ⚠️ |
| `tab:formulations` | T vs V1 vs V0 | `test_vel_vs_torque.py` |
| `tab:mil` | lazo cerrado, dos modelos de contacto sobre el MISMO residuo | `test_interaction_mil.py` |
| `tab:v0loop` | lazo compliant completo con V0 en vez de T | `test_vel_vs_torque.py` |
| `tab:noise` | Monte Carlo con ruido de servo, N=2000 | `test_noise_montecarlo.py` |
| `tab:gating` | gating por `σ_min(Jv)` **+ workspace retenido** | `test_noise_montecarlo.py` |
| `tab:condlaw` | ley `1/σ_min` en la planta MuJoCo | `../hw/test_conditioning_law.py` |
| `tab:faults` | inyección de fallas de modelo y calibración | `reproduce_paper_tables.py` |

⚠️ `tab:regimes` NO se reproducía con ningún script guardado hasta el
04/08/2026, y al escribirlo los números cambiaron. Ver `RESPONSE.md`.

Números en prosa que antes eran tabla (se comprimió el bloque del controlador):
validación del modelo vs MuJoCo (`arm_dynamics.py`, `arm_kinematics.py`), reparto
del residuo por `kv` y timing grafo-vs-escalar (`timing_test.c`), y las tres
direcciones con nombre (`reproduce_paper_tables.py`).

Validación contra MuJoCo (2.227 → 0.157 N): `mj_wrench_test.c`. Tests en C:
preparar la escena primero con `hw/mj_arm.ensure_scene()`.

## Qué falta

Todo es simulación, y el manuscrito lo declara en el abstract, en la introducción,
en el alcance y en Limitations.

- **S3, LAS DOS PARTES** (`../hw/s3_conditioning.py`): dispersión sin carga (valida
  la ley de condicionamiento) **y** carga conocida en dos direcciones (valida
  exactitud y mal-atribución). ⚠️ La parte A sola no alcanza.
- **Extender el estimador** a punto de contacto desconocido a lo largo del eslabón
  (Remark 3).
