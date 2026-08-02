"""
S0 — Sondeo del bus y tasa alcanzable.  ⛔ BLOQUEA TODAS LAS DEMAS ETAPAS.

NO MUEVE NADA. El par queda deshabilitado de punta a punta.

Responde tres preguntas, en orden:
  1. ¿Qué servos hay, con qué modelo y firmware? (define la tabla de control)
  2. ¿Las lecturas son plausibles? (posición dentro de rango, velocidad ~0 en reposo,
     corriente ~0 sin par)
  3. ¿A qué frecuencia se puede cerrar el lazo con lectura sincronizada + escritura?

⚠️ El punto 3 es el que puede bloquear el proyecto y nadie lo verificó. El NMPC
resuelve en 2.5 ms, asi que el computo NO es el limite: el bus si. Y el latency
timer de FTDI viene en 16 ms, lo que topea el lazo en ~62 Hz sin importar el
baudrate.

Criterio de aprobacion: >=100 Hz sostenido con jitter < 20 % del periodo.

Run:  python3 s0_probe_and_rate.py --port /dev/ttyUSB0 --baud 1000000
"""
import argparse
import time
import numpy as np

import dxl_io


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--ids", type=int, nargs="+", default=[1, 2, 3, 4])
    ap.add_argument("--n", type=int, default=2000, help="iteraciones del test de tasa")
    args = ap.parse_args()

    print("=" * 68)
    print("S0 — sondeo del bus.  NO se habilita par: nada se mueve.")
    print("=" * 68)

    dev = args.port.split("/")[-1]
    dxl_io.set_ftdi_latency(dev, 1)

    arm = dxl_io.DxlArm(port=args.port, baud=args.baud, ids=args.ids)
    try:
        arm.enable(False)          # explicito: par apagado

        # ── 1. identidad ─────────────────────────────────────────────────────
        print("\n-- 1. servos en el bus --")
        info = arm.ping()
        for i, (model, fw) in info.items():
            print(f"   id {i}:  model={model}  firmware={fw}")
        print("\n   ⚠️ CONFIRMAR la tabla de control de dxl_io.py contra el e-Manual")
        print("      de ESTE modelo y firmware ANTES de habilitar par.")

        # ── 2. plausibilidad ─────────────────────────────────────────────────
        print("\n-- 2. lecturas en reposo, sin par --")
        Q = []; QD = []; T = []
        for _ in range(200):
            q, qd, tau = arm.read()
            Q.append(q); QD.append(qd); T.append(tau)
            time.sleep(0.002)
        Q = np.array(Q); QD = np.array(QD); T = np.array(T)
        print(f"   {'junta':>6} {'q [rad]':>12} {'ruido q':>10} "
              f"{'|qd| max':>10} {'tau medio':>11} {'ruido tau':>10}")
        for k in range(len(args.ids)):
            print(f"   {k+1:6d} {Q[:,k].mean():12.4f} {Q[:,k].std():10.2e} "
                  f"{np.abs(QD[:,k]).max():10.4f} {T[:,k].mean():11.4f} {T[:,k].std():10.2e}")
        print("\n   esperado: |qd| ~ 0 (brazo quieto), tau ~ 0 (par deshabilitado).")
        print("   ⚠️ tau NO nulo con par apagado = escala o direccion mal en la tabla.")
        print(f"   resolucion de posicion: {dxl_io.POS2RAD:.2e} rad/cuenta")

        # ── 3. tasa de lazo ──────────────────────────────────────────────────
        print(f"\n-- 3. tasa de lazo alcanzable ({args.n} iteraciones) --")
        print("   lectura sincronizada + escritura de velocidad CERO (no se mueve)")
        zero = np.zeros(len(args.ids))
        dt = np.empty(args.n)
        fails = 0
        t_prev = time.perf_counter()
        for k in range(args.n):
            try:
                arm.read()
                arm.write_velocity(zero)
            except IOError:
                fails += 1
            t = time.perf_counter()
            dt[k] = t - t_prev
            t_prev = t
        dt = dt[50:] * 1e3       # ms, descartando el arranque

        hz_mean = 1000.0/dt.mean()
        hz_worst = 1000.0/dt.max()
        jitter = 100.0*dt.std()/dt.mean()
        print(f"   periodo: media {dt.mean():.3f} ms   p95 {np.percentile(dt,95):.3f}   "
              f"max {dt.max():.3f}")
        print(f"   tasa   : media {hz_mean:.0f} Hz   peor caso {hz_worst:.0f} Hz")
        print(f"   jitter : {jitter:.1f} % del periodo")
        print(f"   fallos de comunicacion: {fails}/{args.n}")

        print("\n" + "=" * 68)
        ok = (hz_mean >= 100 and jitter < 20 and fails == 0)
        if ok:
            print("S0 APROBADO — el bus sostiene el lazo. Seguir con S1.")
        else:
            print("S0 RECHAZADO. No seguir a S1 hasta resolver:")
            if hz_mean < 100:
                print(f"   - tasa {hz_mean:.0f} Hz < 100 Hz.")
                print("     revisar: latency_timer de FTDI (16 ms por defecto),")
                print("     baudrate, y cuantos registros se leen por ciclo.")
            if jitter >= 20:
                print(f"   - jitter {jitter:.1f} % alto. Revisar carga del sistema y USB.")
            if fails:
                print(f"   - {fails} fallos de comunicacion. Cableado o alimentacion.")
        print("=" * 68)
    finally:
        arm.close()


if __name__ == "__main__":
    main()
