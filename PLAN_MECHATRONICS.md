# PLAN — Paper a Mechatronics (Elsevier)

> Target: **Mechatronics** (Elsevier, IF ~3.3). Red de respaldo: IJCAS / Robotica.
> NO MDPI, NO IEEE Access (estigma). Decisión tomada con el usuario.

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
[~] V  implementación      medir timing del solver (item en curso)
[ ] VI-HW  HW-1..4         los 4 experimentos en el MX-28 real
[ ] escribir
```

~70% del paper ya está en código validado. Cuello de botella = HARDWARE.

## Resultados sim (resumen, para el paper)

| Experimento | Resultado |
|---|---|
| Modelo (FK/Jac/EOM/dinámica) | validado vs MuJoCo a 1e-10..1e-16 |
| E1 observer sin sensor | corr 0.96–0.998 vs ground truth |
| E3 compliant vs rígido (N=6) | 30%±  menos fuerza + mejor seguimiento |
| E4 DQ vs desacoplado | 1% (empate — DQ = representación, honesto) |
| Trayectoria | RMSE 3.82 mm / 2.72° |
| Observabilidad 4DOF | rank(J)=4<6, declarado |
