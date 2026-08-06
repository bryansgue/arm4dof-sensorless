# Carta de presentación — IEEE Access

⚠️ **Para qué existe este archivo.** La mayor amenaza de rechazo de este manuscrito
**no es que sea solo simulación** — eso produce Major Revision, no rechazo. Es que
un editor o revisor lo lea como *"una cuantificación rigurosa de un hecho
conocido"* y lo rechace temprano. Ese riesgo **no lo resuelve el hardware**: se
gana o se pierde en el primer párrafo que lee el editor.

El manuscrito ya distingue lo conocido de lo nuevo en la Sección I, y esa
honestidad es parte de su fortaleza. **No inflar las afirmaciones aquí.** Lo que la
carta hace es que el editor no tenga que deducirlo.

---

## Borrador

Dear Editor,

We submit for consideration in IEEE Access the manuscript *"Sensorless
Contact-Force Estimation on Low-DoF, Velocity-Actuated Manipulators: A
Quantitative Design Framework."*

That a residual observer cannot resolve a full six-dimensional wrench from fewer
than six joints is established, and we say so explicitly in the paper. What was
not available to a designer, and what this manuscript supplies, is everything that
follows from that fact in practice:

- **how much** of an applied force the conventional inversion discards, measured
  over a sampled workspace of $2\,197$ configurations rather than asserted;
- **in which directions**, through a law that predicts the error from the push
  direction alone rather than describing it after the fact;
- **that the quantification is not scale-free.** The minimum-norm criterion adds
  newtons to newton-metres, so it presupposes a characteristic length that the
  unweighted pseudoinverse fixes silently. We parameterize that choice, report the
  sensitivity, and separate the conclusions that survive it from those that do not;
- **which part is structural.** For an arm whose first joint rotates about the
  vertical and whose remaining joint axes are parallel to a common horizontal
  direction, we prove that the directions lost, and the set of forces recovered
  exactly, are
  independent of that length, while the magnitude of the loss is not. We state
  plainly that this invariance does not hold for a general Jacobian;
- **that the failure is silent.** The conventional inversion correlates with ground
  truth at $0.997$ while discarding half of the applied force under the
  unweighted-SI baseline, so a validation of the usual kind does not detect it;
- **that structural error and noise amplification are operationally separable.**
  Gating on the smallest singular value of the translational Jacobian halves the
  tail of one and leaves the other untouched, and we report the cost of that gate
  in retained workspace rather than only its benefit;
- **the velocity-actuated regime**, in which the dynamic model turns out to be
  required by the estimator and by nothing else.

The paper also states what it does not establish. All results are obtained in
simulation. The experiment that would test the directional prediction, the
conditioning law, and the viability of torque inferred from motor current is
specified in full but has not been run, and we identify it as the principal
limitation rather than leaving it to the reader to notice.

Every table and figure is produced by a named script maintained in the
accompanying reproducibility package, with the environment pinned, including the
exact commit of the solver framework.

The work suits IEEE Access because its contribution is a design framework of
immediate use to anyone building compliant interaction on low-cost hardware, and
because its central result is corrective: it identifies an error in current
practice, quantifies it, and explains it.

Sincerely,
[autor de correspondencia]

---

## Notas para armarla

- ⚠️ **El orden importa.** El primer párrafo concede lo conocido antes de que el
  editor lo piense. Conceder primero es lo que hace creíble el resto.
- ⚠️ **No sacar el párrafo de limitaciones.** Un editor que descubre solo la
  simulación por su cuenta la lee como omisión; declarada, la lee como alcance.
- ⚠️ **"reproducibility package" obliga a entregarlo.** La frase solo vale si el
  código se adjunta o queda en un repositorio accesible. Si no va a haber ninguno
  de los dos, hay que sacarla, no suavizarla.
- ⚠️ **Ni "coplanar" ni "comparten un eje".** La primera es más débil que lo que se
  demuestra; la segunda se lee como **coaxial**, y las rectas articulares son
  distintas. La formulación exacta es: *ejes de pitch **paralelos a una dirección
  horizontal común** `a`*. Lo que la demostración usa es que las columnas
  angulares comparten la dirección, no que las rectas coincidan.
- ⚠️ **El 50.5 % nunca va suelto**, ni aquí ni en el manuscrito: siempre con el
  baseline sin ponderar declarado al lado.
- ⛔ Falta reemplazar `[autor de correspondencia]` por el nombre real. Ver
  `SUBMISSION.md`.
- Si el portal permite carta libre, recortar notas redundantes hasta ~1 página.
  **Recortar notas, no contribuciones.**
