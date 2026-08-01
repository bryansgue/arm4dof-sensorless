# Domain profile — Contact-Model-Aware Sensorless Force Estimation (IEEE Access)

Draft: `paper/main.tex`
Venue: **IEEE Access** (no es RA-L: sin límite de 6-8 páginas, sin doble anonimato
obligatorio). Las reglas duras de RA-L de la skill NO aplican; sí aplican el rubro
de revisor (C), la higiene de claims (F) y el pase de vocabulario (H).
Corpus: **no hay corpus local todavía.** Ver "Pendiente" abajo.

## SOTA (must-cite + delta)

- **De Luca & Mattone, ICRA 2005** — residuo sin sensor para control híbrido
  fuerza/movimiento — brazos con par y n≥6; no trata la inversión bajo n<6 —
  [must-cite]
- **De Luca et al., IROS 2006** — detección de colisión y reacción segura, DLR-III
  (7 DoF) — mismo régimen: par, n≥6 — [must-cite]
- **Magrini et al., IROS 2014** — sensor de fuerza virtual; estima fuerza Y punto
  de contacto en brazos redundantes — **el competidor más cercano**: ya usa modelo
  de contacto puntual, pero con n≥6 donde la inversión 6-D no es singular, y con
  actuación en par — [must-cite, diferenciar explícitamente]
- **Haddadin et al., T-RO 2017** — survey de detección/aislamiento/identificación —
  contexto, delimita el estado del arte — [must-cite]
- **Yoshikawa, IJRR 1985** — elipsoides de manipulabilidad — problema DUAL
  (producir vs percibir) — [context]
- **Chiacchio et al., 1997** — politopos de fuerza — se usa en su rol clásico para
  el umbral de saturación (Sec. V-C) — [context]

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
