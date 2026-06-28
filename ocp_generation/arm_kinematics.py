"""
FK del efector en DUAL QUATERNION (CasADi) — q(4) -> DQ_ee (8).
Reusa la cadena de arm_dynamics. Valida vs site_xpos/xquat de MuJoCo (en arm_params.TESTS).
"""
import numpy as np
import casadi as ca
from casadi import MX, vertcat, horzcat, mtimes
import arm_params as P
from arm_dynamics import quat2R, axisangle2R
from dq_math import dq_from_pose, dq_get_position

NQ = 4

def R2quat(R):
    """Matriz rotacion 3x3 -> quaternion [w,x,y,z] (CasADi, rama estable w>0)."""
    w = 0.5*ca.sqrt(ca.fmax(1e-12, 1 + R[0,0] + R[1,1] + R[2,2]))
    x = (R[2,1]-R[1,2])/(4*w); y = (R[0,2]-R[2,0])/(4*w); z = (R[1,0]-R[0,1])/(4*w)
    return vertcat(w,x,y,z)

def fk_ee_dq(q):
    """q(4) -> (DQ_ee(8), pos(3), quat(4)) del efector (site 'ee' = tip)."""
    R_prev = quat2R(MX(P.BASE_QUAT)); p_prev = MX(P.BASE_POS)
    for i,L in enumerate(P.LINKS):
        Rf = quat2R(MX(L['body_quat'])); pf = MX(L['body_pos'])
        R_fix = mtimes(R_prev, Rf); p_fix = p_prev + mtimes(R_prev, pf)
        Rj = axisangle2R(MX(L['jnt_axis']), q[i])
        R_prev = mtimes(R_fix, Rj); p_prev = p_fix
    # tip (efector) fijo sobre hand
    Rt = quat2R(MX(P.TIP['body_quat'])); pt = MX(P.TIP['body_pos'])
    R_ee = mtimes(R_prev, Rt); p_ee = p_prev + mtimes(R_prev, pt)
    quat = R2quat(R_ee)
    dq = dq_from_pose(quat, p_ee)
    return dq, p_ee, quat

def jacobian_ee(q):
    """Jacobiano geometrico 6x4 del efector: [Jv; Jw], twist world = J q̇.
    Construccion screw (validada a 4e-17 vs mj_jacSite)."""
    R_prev = quat2R(MX(P.BASE_QUAT)); p_prev = MX(P.BASE_POS)
    axis_w = []; anch_w = []
    for i,L in enumerate(P.LINKS):
        Rf = quat2R(MX(L['body_quat'])); pf = MX(L['body_pos'])
        R_fix = mtimes(R_prev, Rf); p_fix = p_prev + mtimes(R_prev, pf)
        axis_w.append(mtimes(R_fix, MX(L['jnt_axis'])))
        anch_w.append(p_fix + mtimes(R_fix, MX(L['jnt_pos'])))
        R_prev = mtimes(R_fix, axisangle2R(MX(L['jnt_axis']), q[i])); p_prev = p_fix
    Rt = quat2R(MX(P.TIP['body_quat'])); p_ee = p_prev + mtimes(R_prev, MX(P.TIP['body_pos']))
    Jv = MX.zeros(3, NQ); Jw = MX.zeros(3, NQ)
    for j in range(NQ):
        Jw[:, j] = axis_w[j]
        Jv[:, j] = ca.cross(axis_w[j], p_ee - anch_w[j])
    return ca.vertcat(Jv, Jw)   # 6x4

def build_fk_fn():
    q = MX.sym("q", NQ)
    dq, p, quat = fk_ee_dq(q)
    return ca.Function("fk_ee", [q], [dq, p, quat])

def build_jac_fn():
    q = MX.sym("q", NQ)
    return ca.Function("jac_ee", [q], [jacobian_ee(q)])

if __name__ == "__main__":
    fk = build_fk_fn()
    maxp=0; maxo=0
    print("== Validacion FK DQ (CasADi) vs MuJoCo site ==")
    for k,(q,qd,M,h,ep,eq) in enumerate(P.TESTS):
        dq,p,quat = fk(q)
        p=np.array(p).flatten(); quat=np.array(quat).flatten()
        ep=np.array(ep); eq=np.array(eq)
        pe=np.linalg.norm(p-ep)
        dot=abs(float(quat@eq)); dot=min(1.0,dot); oe=2*np.degrees(np.arccos(dot))
        maxp=max(maxp,pe); maxo=max(maxo,oe)
        print(f" cfg{k}: pos_err={pe:.2e} m  orient_err={oe:.2e} deg")
    print("-"*44); print(f"MAX pos={maxp:.2e}  orient={maxo:.2e}")
    print("FK DQ CasADi VALIDADA" if (maxp<1e-6 and maxo<1e-3) else "REVISAR")
