"""
Banco de servos EMULADO — para correr S0..S5 sin hardware.

Por que existe. Los scripts de bring-up nunca se ejecutaron contra nada, y el
PROCEDIMIENTO DE IDENTIFICACION en si mismo esta sin validar. Si la regresion de
S2 no recupera una constante de par CONOCIDA en un banco donde la respuesta se
sabe, tampoco la va a recuperar en el brazo real, y ahi el error se descubre con
las pesas colgadas y sin forma de separar la causa.

Expone la MISMA interfaz que dxl_io.DxlArm, asi que los scripts corren sin
cambios: se les pasa --sim y ni se enteran.

Lo que el banco inyecta, y los scripts NO conocen:
    KT_TRUE      constante corriente->par real, distinta de la semilla
    TAU_COULOMB  stiction por junta
    TAU_VISCOUS  friccion viscosa por junta
    cuantizacion de corriente y de encoder, con las magnitudes del MX-28
    ZERO_OFFSET  desalineacion de cero de montaje

⚠️ Esto NO sustituye al hardware. No modela: dinamica de la reductora, backlash,
deriva termica, retardo del bus, ni la estructura real del lazo de velocidad
(que es PI, no P). Sirve para validar la LOGICA de los scripts y el PODER de los
procedimientos de identificacion, no para producir resultados publicables.
"""
from __future__ import annotations
import os
import sys
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "ocp_generation"))
from arm_dynamics import build_dynamics
from arm_kinematics import build_jac_fn

_fM, _fh = build_dynamics(); _fJ = build_jac_fn()

# ── verdad de terreno del banco. Los scripts no la leen. ──────────────────────
KT_TRUE     = np.array([1.37, 1.41, 1.35, 1.39])     # N.m/A  (semilla del driver: 1.0)
TAU_COULOMB = np.array([0.042, 0.055, 0.038, 0.031])  # N.m
TAU_VISCOUS = np.array([0.021, 0.018, 0.024, 0.016])  # N.m.s
ZERO_OFFSET = np.array([0.004, -0.006, 0.003, 0.005]) # rad, desalineacion de montaje

CUR_LSB = 3.36e-3        # A por cuenta
ENC_LSB = 2*np.pi/4096   # rad por cuenta
TAU_MAX = 1.4
KV_P, KV_I = 18.0, 60.0  # lazo de velocidad PI del servo (los scripts no lo saben)
G = 9.81


class SimArm:
    """Emula DxlArm. Misma interfaz publica."""

    def __init__(self, port=None, baud=None, ids=(1, 2, 3, 4),
                 zeros_rad=None, signs=None, torque_const=None,
                 q0=None, seed=0):
        self.ids = list(ids); self.n = len(self.ids)
        self.signs = np.ones(self.n)
        self.zeros = np.zeros(self.n)
        # el driver arranca con la semilla, NO con la verdad
        self.kt = np.ones(self.n) if torque_const is None else np.asarray(torque_const, float)
        self.rng = np.random.default_rng(seed)

        self.q = np.array([0.0, 1.2, -0.9, 0.4]) if q0 is None else np.asarray(q0, float)
        self.qd = np.zeros(self.n)
        self.u = np.zeros(self.n)          # comando de velocidad vigente
        self._ei = np.zeros(self.n)        # integrador del lazo del servo
        self.tau_applied = np.zeros(self.n)
        self.payload = 0.0                 # kg colgados del efector
        self.enabled = False
        self._dt = 0.002                   # paso interno del banco

    # ── interfaz ─────────────────────────────────────────────────────────────
    def ping(self):
        return {i: (30, 45) for i in self.ids}     # modelo/firmware ficticios

    def set_velocity_mode(self):
        pass

    def enable(self, on=True):
        self.enabled = bool(on)
        if not on:
            self._ei[:] = 0.0

    def write_velocity(self, qd_cmd):
        self.u = np.clip(np.asarray(qd_cmd, float), -5.969, 5.969)
        self._step()

    def stop(self):
        self.write_velocity(np.zeros(self.n))

    def close(self):
        self.enable(False)

    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.close()

    def read(self):
        """(q, qd, tau) como los devolveria el driver: con cuantizacion, offsets
        de cero, y el par derivado de la corriente con la constante SEMILLA."""
        self._step()
        q_raw = self.q + ZERO_OFFSET
        q_meas = np.round(q_raw/ENC_LSB)*ENC_LSB
        qd_meas = np.round(self.qd/(ENC_LSB/self._dt))*(ENC_LSB/self._dt)
        # corriente medida = par aplicado / constante VERDADERA, cuantizada
        i_true = self.tau_applied/KT_TRUE
        i_meas = np.round(i_true/CUR_LSB)*CUR_LSB
        # el driver la convierte con SU constante (la semilla, o la calibrada)
        tau_meas = i_meas*self.kt
        return q_meas, qd_meas, tau_meas

    # ── dinamica del banco ───────────────────────────────────────────────────
    def _step(self, n=1):
        for _ in range(n):
            if not self.enabled:
                self.tau_applied[:] = 0.0
                return
            e = self.u - self.qd
            self._ei += e*self._dt
            self._ei = np.clip(self._ei, -2.0, 2.0)            # anti-windup
            tau_cmd = KV_P*e + KV_I*self._ei
            # friccion: se opone al movimiento y la produce el motor
            f_fric = TAU_COULOMB*np.tanh(self.qd/1e-3) + TAU_VISCOUS*self.qd
            tau = np.clip(tau_cmd, -TAU_MAX, TAU_MAX)
            self.tau_applied = tau
            M = np.array(_fM(self.q))
            h = np.array(_fh(self.q, self.qd)).flatten()
            w_ext = self._payload_torque()
            qdd = np.linalg.solve(M, tau - f_fric + w_ext - h)
            self.qd = self.qd + self._dt*qdd
            self.q = self.q + self._dt*self.qd

    def _payload_torque(self):
        if self.payload <= 0.0:
            return np.zeros(self.n)
        Jv = np.array(_fJ(self.q))[:3, :]
        return Jv.T @ np.array([0.0, 0.0, -self.payload*G])

    # ── utilidades del banco, no del driver ──────────────────────────────────
    def hang(self, mass_kg):
        self.payload = float(mass_kg)

    def settle(self, seconds=2.0):
        self._step(int(seconds/self._dt))


def truth():
    return dict(kt=KT_TRUE, coulomb=TAU_COULOMB, viscous=TAU_VISCOUS,
                zero_offset=ZERO_OFFSET, kv_p=KV_P, kv_i=KV_I)
