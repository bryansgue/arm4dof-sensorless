"""
Capa de I/O para los MX-28R. Lectura sincronizada de q, q̇ y corriente, escritura
sincronizada de velocidad objetivo.

⚠️ LEER ANTES DE ENERGIZAR ⚠️
La TABLA DE CONTROL de abajo está escrita contra la documentación del Protocolo
2.0 y **NO está verificada contra el servo real**. Las direcciones difieren entre
MX-28 con firmware 1.0 y con firmware 2.0, y una dirección equivocada puede
escribir en el registro incorrecto. Correr `s0_probe_and_rate.py` PRIMERO: sondea
modelo y firmware, y vuelca los campos para confirmarlos contra el e-Manual antes
de que nada se mueva.

Las unidades también hay que confirmarlas. Un factor de escala mal puesto en la
corriente entra 1:1 en la fuerza estimada (ver hw/README.md).
"""
from __future__ import annotations
import time
import numpy as np

# ── TABLA DE CONTROL — CONFIRMAR CON S0 ──────────────────────────────────────
# Protocolo 2.0. Verificar contra el e-Manual del firmware instalado.
ADDR = {
    "operating_mode":   (11,  1),   # 1 = control de velocidad
    "torque_enable":    (64,  1),
    "goal_velocity":    (104, 4),
    "present_current":  (126, 2),   # con signo
    "present_velocity": (128, 4),   # con signo
    "present_position": (132, 4),   # con signo
}
# bloque contiguo para lectura sincronizada: current+velocity+position = 10 bytes
READ_BLOCK = (126, 10)

MODE_VELOCITY = 1

# ── UNIDADES — CONFIRMAR CON S0 ──────────────────────────────────────────────
POS_PER_REV      = 4096.0          # cuentas por vuelta (MX-28: 12 bit)
VEL_UNIT_RPM     = 0.229           # por cuenta de velocidad
CUR_UNIT_A       = 3.36e-3         # A por cuenta de corriente
# ⚠️ NO es el valor de datasheet: se MIDE en S2. Este es solo semilla.
TORQUE_CONST_SEED = 1.0            # N.m por amperio

POS2RAD = 2.0*np.pi/POS_PER_REV
VEL2RPS = VEL_UNIT_RPM*2.0*np.pi/60.0


class DxlArm:
    """4 servos en un bus. Todas las lecturas/escrituras son sincronizadas."""

    def __init__(self, port="/dev/ttyUSB0", baud=1_000_000, ids=(1, 2, 3, 4),
                 zeros_rad=None, signs=None, torque_const=None):
        try:
            import dynamixel_sdk as dxl
        except ImportError as e:
            raise ImportError(
                "falta dynamixel_sdk.  pip install dynamixel-sdk") from e
        self._dxl = dxl
        self.ids = list(ids)
        self.n = len(self.ids)
        # offset de cero y sentido: la convención del modelo NO es la del servo
        self.zeros = np.zeros(self.n) if zeros_rad is None else np.asarray(zeros_rad, float)
        self.signs = np.ones(self.n) if signs is None else np.asarray(signs, float)
        self.kt = np.full(self.n, TORQUE_CONST_SEED) if torque_const is None \
            else np.asarray(torque_const, float)

        self.port = dxl.PortHandler(port)
        self.pk = dxl.PacketHandler(2.0)
        if not self.port.openPort():
            raise IOError(f"no se pudo abrir {port}")
        if not self.port.setBaudRate(baud):
            raise IOError(f"no se pudo fijar {baud} baudios")

        addr, ln = READ_BLOCK
        self._sync_read = dxl.GroupSyncRead(self.port, self.pk, addr, ln)
        for i in self.ids:
            if not self._sync_read.addParam(i):
                raise IOError(f"addParam falló para el id {i}")
        a, l = ADDR["goal_velocity"]
        self._sync_write = dxl.GroupSyncWrite(self.port, self.pk, a, l)

        self._last_ok = time.perf_counter()

    # ── utilidades de bajo nivel ─────────────────────────────────────────────
    def _w1(self, i, key, val):
        a, _ = ADDR[key]
        r, e = self.pk.write1ByteTxRx(self.port, i, a, val)
        if r != self._dxl.COMM_SUCCESS or e != 0:
            raise IOError(f"write {key} id {i}: comm={r} err={e}")

    def ping(self):
        """(id -> (model, firmware)). No mueve nada."""
        out = {}
        for i in self.ids:
            model, r, e = self.pk.ping(self.port, i)
            if r != self._dxl.COMM_SUCCESS:
                raise IOError(f"ping id {i} sin respuesta (comm={r})")
            fw, _, _ = self.pk.read1ByteTxRx(self.port, i, 6)
            out[i] = (model, fw)
        return out

    # ── configuración ────────────────────────────────────────────────────────
    def set_velocity_mode(self):
        """El modo solo se puede cambiar con el par DESHABILITADO."""
        for i in self.ids:
            self._w1(i, "torque_enable", 0)
            self._w1(i, "operating_mode", MODE_VELOCITY)

    def enable(self, on=True):
        for i in self.ids:
            self._w1(i, "torque_enable", 1 if on else 0)

    # ── lazo ─────────────────────────────────────────────────────────────────
    def read(self):
        """(q[rad], qd[rad/s], tau[N.m]) en la convención del MODELO.

        tau es el par estimado desde la corriente. NO es el par externo: es lo que
        el servo está aplicando, o sea el tau_act que el estimador de wrench resta.
        """
        r = self._sync_read.txRxPacket()
        if r != self._dxl.COMM_SUCCESS:
            raise IOError(f"sync read falló (comm={r})")
        addr, _ = READ_BLOCK
        q = np.zeros(self.n); qd = np.zeros(self.n); tau = np.zeros(self.n)
        for k, i in enumerate(self.ids):
            ac, lc = ADDR["present_current"]
            av, lv = ADDR["present_velocity"]
            ap, lp = ADDR["present_position"]
            if not self._sync_read.isAvailable(i, addr, READ_BLOCK[1]):
                raise IOError(f"datos no disponibles para id {i}")
            cur = _signed(self._sync_read.getData(i, ac, lc), lc)
            vel = _signed(self._sync_read.getData(i, av, lv), lv)
            pos = _signed(self._sync_read.getData(i, ap, lp), lp)
            q[k]   = self.signs[k]*(pos*POS2RAD) - self.zeros[k]
            qd[k]  = self.signs[k]*(vel*VEL2RPS)
            tau[k] = self.signs[k]*(cur*CUR_UNIT_A*self.kt[k])
        self._last_ok = time.perf_counter()
        return q, qd, tau

    def write_velocity(self, qd_cmd):
        """qd_cmd en rad/s, convención del modelo."""
        self._sync_write.clearParam()
        a, l = ADDR["goal_velocity"]
        for k, i in enumerate(self.ids):
            raw = int(round(self.signs[k]*qd_cmd[k]/VEL2RPS))
            self._sync_write.addParam(i, _to_bytes(raw, l))
        r = self._sync_write.txPacket()
        if r != self._dxl.COMM_SUCCESS:
            raise IOError(f"sync write falló (comm={r})")

    def stop(self):
        self.write_velocity(np.zeros(self.n))

    def close(self):
        try:
            self.stop(); self.enable(False)
        finally:
            self.port.closePort()

    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.close()


def _signed(v, nbytes):
    bits = 8*nbytes
    return v - (1 << bits) if v >= (1 << (bits-1)) else v


def _to_bytes(v, nbytes):
    if v < 0:
        v += (1 << (8*nbytes))
    return [(v >> (8*k)) & 0xFF for k in range(nbytes)]


def set_ftdi_latency(port="ttyUSB0", ms=1):
    """El latency timer de FTDI viene en 16 ms y topea el lazo en ~62 Hz sin
    importar el baudrate. Es la trampa clásica del bring-up de Dynamixel."""
    p = f"/sys/bus/usb-serial/devices/{port}/latency_timer"
    try:
        cur = int(open(p).read().strip())
        if cur != ms:
            print(f"⚠️ latency_timer = {cur} ms.  Bajar a {ms}:")
            print(f"   echo {ms} | sudo tee {p}")
        return cur
    except Exception as e:
        print(f"[no se pudo leer {p}: {e}]")
        return None
