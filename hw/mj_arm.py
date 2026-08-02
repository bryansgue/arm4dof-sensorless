"""
Banco de servos sobre MuJoCo — la planta INDEPENDIENTE.

Por que reemplaza a dxl_sim.py como banco de referencia. Aquel usaba la MISMA
dinamica que el estimador, asi que planta y modelo coincidian por construccion y
la validacion era parcialmente circular. MuJoCo es otro integrador, con su propia
resolucion de restricciones, y la escena trae:

    <velocity kv=3 ctrlrange=+-5.969 forcerange=+-1.4>   comando de VELOCIDAD,
                                                        limites reales del MX-28
    <joint damping=0.12 armature=0.012 frictionloss=0.02>
    <actuatorfrc>                                       = Present_Load

Tres de esos terminos —damping, armature y sobre todo frictionloss— NO estan en
arm_dynamics.py, que es el modelo que usa el estimador. O sea que hay discrepancia
planta-modelo REAL, que es lo que se quiere probar.

⚠️ frictionloss de MuJoCo es una restriccion de friccion seca: en reposo toma el
valor que haga falta hasta el limite. Eso es exactamente la indeterminacion de
friccion estatica que dxl_sim NO podia modelar.

Expone la misma interfaz que dxl_io.DxlArm.
"""
from __future__ import annotations
import os
import numpy as np

SCENE = "/tmp/vel.xml"
JOINTS = ("m1", "m2", "m3", "m4")
ACTS = ("a_m1", "a_m2", "a_m3", "a_m4")
TIP_BODY = "tip"

# verdad de terreno del banco; los scripts no la leen
KT_TRUE = np.array([1.37, 1.41, 1.35, 1.39])     # N.m/A
CUR_LSB = 3.36e-3                                 # A por cuenta
ENC_LSB = 2*np.pi/4096                            # rad por cuenta
G = 9.81


def ensure_scene():
    """Prepara /tmp/vel.xml con meshdir absoluto, si no existe."""
    if os.path.exists(SCENE):
        return SCENE
    src = os.path.expanduser(
        "~/mujoco_ws/src/acp_mujoco_simulator/model/arm4dof/scene_arm4dof_vel.xml")
    d = os.path.dirname(src)
    s = open(src).read().replace('meshdir="assets/"', f'meshdir="{d}/assets/"')
    open(SCENE, "w").write(s)
    return SCENE


class MjArm:
    """Interfaz DxlArm sobre MuJoCo. Comando = velocidad de junta."""

    def __init__(self, port=None, baud=None, ids=(1, 2, 3, 4),
                 torque_const=None, q0=None, quantize=True, seed=0,
                 cur_noise_A=0.0):
        """cur_noise_A: sigma del ruido ADITIVO de corriente, en amperios.

        ⚠️ MuJoCo es determinista: en equilibrio el par es exactamente constante
        y la cuantizacion sola da varianza CERO. Un servo real tiene ruido
        electrico aditivo ademas de la cuantizacion. Sin este termino no se puede
        estudiar la amplificacion de ruido en este banco.
        Referencia: el LSB de corriente del MX-28 es 3.36 mA, asi que un sigma
        de 1-3 LSB (3-10 mA) es el orden razonable."""
        import mujoco
        self.mj = mujoco
        self.m = mujoco.MjModel.from_xml_path(ensure_scene())
        self.d = mujoco.MjData(self.m)
        self.n = 4
        self.ids = list(ids)
        self.quantize = quantize
        self.cur_noise_A = float(cur_noise_A)
        self.kt = np.ones(self.n) if torque_const is None \
            else np.asarray(torque_const, float)
        self.qadr = [self.m.jnt_qposadr[mujoco.mj_name2id(
            self.m, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in JOINTS]
        self.dadr = [self.m.jnt_dofadr[mujoco.mj_name2id(
            self.m, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in JOINTS]
        self.aid = [mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_ACTUATOR, a)
                    for a in ACTS]
        self.tip = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, TIP_BODY)
        self.payload = 0.0
        self.enabled = False
        self.rng = np.random.default_rng(seed)
        mujoco.mj_forward(self.m, self.d)
        if q0 is not None:
            self.set_state(q0)

    # ── interfaz ─────────────────────────────────────────────────────────────
    def ping(self):
        return {i: (30, 45) for i in self.ids}

    def set_velocity_mode(self):
        pass

    def enable(self, on=True):
        self.enabled = bool(on)
        if not on:
            self.d.ctrl[:] = 0.0

    def write_velocity(self, qd_cmd):
        if not self.enabled:
            return
        for k, a in enumerate(self.aid):
            self.d.ctrl[a] = float(np.clip(qd_cmd[k], -5.969, 5.969))
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
        """(q, qd, tau) con la cuantizacion del servo real.
        tau sale de la CORRIENTE: actuator_force / KT_TRUE, cuantizada, y el
        driver la reconvierte con SU constante."""
        q = np.array([self.d.qpos[i] for i in self.qadr])
        qd = np.array([self.d.qvel[i] for i in self.dadr])
        tau_true = np.array([self.d.actuator_force[a] for a in self.aid])
        i_meas = tau_true/KT_TRUE
        if self.cur_noise_A > 0.0:
            i_meas = i_meas + self.rng.normal(0.0, self.cur_noise_A, self.n)
        if self.quantize:
            q = np.round(q/ENC_LSB)*ENC_LSB
            i_meas = np.round(i_meas/CUR_LSB)*CUR_LSB
        return q, qd, i_meas*self.kt

    # ── utilidades del banco ─────────────────────────────────────────────────
    def _step(self, n=1):
        if self.payload > 0.0:
            self.d.xfrc_applied[self.tip, :3] = [0.0, 0.0, -self.payload*G]
        else:
            self.d.xfrc_applied[self.tip, :] = 0.0
        for _ in range(n):
            self.mj.mj_step(self.m, self.d)

    def hang(self, mass_kg):
        self.payload = float(mass_kg)

    def set_state(self, q):
        for k, i in enumerate(self.qadr):
            self.d.qpos[i] = q[k]
        for i in self.dadr:
            self.d.qvel[i] = 0.0
        self.mj.mj_forward(self.m, self.d)

    def settle(self, n=2000):
        for _ in range(n):
            self._step()


def truth():
    return dict(kt=KT_TRUE, note="damping/armature/frictionloss vienen del XML")
