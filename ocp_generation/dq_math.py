"""
Algebra de dual quaternions (CasADi) — COPIADO verbatim de
mpcc_controller/ocp_generation/generate_dq_mpcc_ocp.py (DQ-NMPC del dron, ACP lab).
Reusable, sin dependencia del repo original. Convencion: q=[w,x,y,z].
DQ = q_r + eps q_d (8-vector). log-map se(3) con norma suavizada ARM-safe.
"""
import numpy as np
import casadi as ca
from casadi import MX, DM, vertcat, horzcat, if_else, atan2


def H_plus_matrix(q):
    """Hamilton izq: H+(q)·p = q ⊗ p."""
    return vertcat(
        horzcat(q[0], -q[1], -q[2], -q[3]),
        horzcat(q[1],  q[0], -q[3],  q[2]),
        horzcat(q[2],  q[3],  q[0], -q[1]),
        horzcat(q[3], -q[2],  q[1],  q[0]),
    )

def quat_product(p, q):
    return H_plus_matrix(p) @ q

def dq_from_pose(quat, trans):
    """DQ unitario desde rotacion (quat) + traslacion. Q = q_r + eps ½ t⊗q_r."""
    t_pure = vertcat(MX(0), trans)
    q_d = 0.5 * quat_product(t_pure, quat)
    return vertcat(quat, q_d)

def dq_conjugate(dq):
    return vertcat(dq[0], -dq[1], -dq[2], -dq[3], dq[4], -dq[5], -dq[6], -dq[7])

def dq_product(dq1, dq2):
    q1r, q1d = dq1[0:4], dq1[4:8]
    q2r, q2d = dq2[0:4], dq2[4:8]
    real = quat_product(q1r, q2r)
    dual = quat_product(q1r, q2d) + quat_product(q1d, q2r)
    return vertcat(real, dual)

def dq_error(dq_desired, dq_actual):
    """Q_err = Q_d* ⊗ Q."""
    dq_d_conj = dq_conjugate(dq_desired)
    qr_c, qd_c = dq_d_conj[0:4], dq_d_conj[4:8]
    H_r = H_plus_matrix(qr_c); H_d = H_plus_matrix(qd_c)
    zeros = DM.zeros(4, 4)
    H_dual = vertcat(horzcat(H_r, zeros), horzcat(H_d, H_r))
    return H_dual @ dq_actual

def dq_get_position(dq):
    qr, qd = dq[0:4], dq[4:8]
    qr_c = vertcat(qr[0], -qr[1], -qr[2], -qr[3])
    t = 2.0 * quat_product(qd, qr_c)
    return t[1:4]

def left_jacobian_SO3_inv(phi):
    """J_l(φ)⁻¹ (Solà 2018). Norma suavizada -> AD finito en φ=0 (ARM-safe)."""
    EPS_M = np.finfo(np.float64).eps
    I3 = DM.eye(3)
    theta = ca.sqrt(phi[0]**2 + phi[1]**2 + phi[2]**2 + EPS_M)
    phi_x = vertcat(
        horzcat(MX(0),  -phi[2],  phi[1]),
        horzcat(phi[2],  MX(0),  -phi[0]),
        horzcat(-phi[1], phi[0],  MX(0)),
    )
    n_hat = phi / theta
    n_hat_col = ca.reshape(n_hat, 3, 1)
    nnT = n_hat_col @ n_hat_col.T
    n_hat_x = phi_x / theta
    half = 0.5 * theta
    alpha = half * ca.cos(half) / (ca.sin(half) + EPS_M)
    return alpha * I3 - half * n_hat_x + (1 - alpha) * nnT

def ln_dual(dq_err):
    """log(Q_err)=[φ;ρ]∈se(3). φ=eje-angulo, ρ=J_l⁻¹·t_err (acople rot-trans)."""
    sign = if_else(dq_err[0] < 0, MX(-1.0), MX(1.0))
    q_real = sign * dq_err[0:4]
    q_dual = sign * dq_err[4:8]
    q_real_c = vertcat(q_real[0], -q_real[1], -q_real[2], -q_real[3])
    EPS_M = np.finfo(np.float64).eps
    norm_v = ca.sqrt(q_real[1]**2 + q_real[2]**2 + q_real[3]**2 + EPS_M)
    angle = 2.0 * atan2(norm_v, q_real[0])
    phi = vertcat(0.5*angle*q_real[1]/norm_v, 0.5*angle*q_real[2]/norm_v, 0.5*angle*q_real[3]/norm_v)
    t_err = 2.0 * quat_product(q_dual, q_real_c)
    rho = left_jacobian_SO3_inv(phi) @ t_err[1:4]
    return vertcat(phi, rho)

def se3_error_decoupled(dq_err):
    """Baseline DESACOPLADO (E4): [φ; t_err] — rotacion log + traslacion CRUDA,
    SIN el acople J_l⁻¹. Para errores chicos J_l⁻¹≈I => casi == ln_dual (empate esperado)."""
    sign = if_else(dq_err[0] < 0, MX(-1.0), MX(1.0))
    q_real = sign * dq_err[0:4]; q_dual = sign * dq_err[4:8]
    q_real_c = vertcat(q_real[0], -q_real[1], -q_real[2], -q_real[3])
    EPS_M = np.finfo(np.float64).eps
    norm_v = ca.sqrt(q_real[1]**2 + q_real[2]**2 + q_real[3]**2 + EPS_M)
    angle = 2.0 * atan2(norm_v, q_real[0])
    phi = vertcat(0.5*angle*q_real[1]/norm_v, 0.5*angle*q_real[2]/norm_v, 0.5*angle*q_real[3]/norm_v)
    t_err = 2.0 * quat_product(q_dual, q_real_c)
    return vertcat(phi, t_err[1:4])   # SIN J_l⁻¹
