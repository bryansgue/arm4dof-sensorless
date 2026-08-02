"""
Dinamica simbolica (CasADi) del brazo 4DOF — M(q), h(q,qd) = Coriolis+gravedad.

Enfoque: Jacobianos geometricos por link (screw/Plücker, ya validado a 4e-17 vs
mj_jacSite) -> M(q) = Σ m_i Jv_i^T Jv_i + Jw_i^T (R_i I_i R_i^T) Jw_i.
Bias via Lagrangiano:  h = Mdot·qd - ½ ∂(qd^T M qd)/∂q + ∂PE/∂q.

Valida M,h contra los valores MuJoCo dumpeados en arm_params.TESTS (objetivo ~1e-10).

Reusa convenciones DQ del DQ-MPCC del dron (mpcc_controller) — copiado, no importado.
"""
import numpy as np
import casadi as ca
from casadi import SX, vertcat, horzcat, mtimes, cross, jacobian, gradient
import arm_params as P

NQ = 4

def quat2R(q):
    """Quaternion [w,x,y,z] -> matriz de rotacion 3x3 (CasADi)."""
    w,x,y,z = q[0],q[1],q[2],q[3]
    return vertcat(
        horzcat(1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)),
        horzcat(2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)),
        horzcat(2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)),
    )

def axisangle2R(axis, ang):
    """Rotacion alrededor de eje unitario por angulo (Rodrigues)."""
    a = axis
    K = vertcat(horzcat(SX(0),-a[2],a[1]), horzcat(a[2],SX(0),-a[0]), horzcat(-a[1],a[0],SX(0)))
    return SX.eye(3) + ca.sin(ang)*K + (1-ca.cos(ang))*mtimes(K,K)

def build_dynamics():
    q  = SX.sym("q", NQ)
    qd = SX.sym("qd", NQ)
    g_vec = SX(P.GRAVITY)            # [0,0,-9.81]

    # --- FK de la cadena: frame world de cada link + eje/ancla de cada junta ---
    # T_0 = base fija
    Rb = quat2R(SX(P.BASE_QUAT)); pb = SX(P.BASE_POS)
    R_prev, p_prev = Rb, pb
    axis_w = []   # eje world de junta j (en config q)
    anch_w = []   # ancla world de junta j
    Rlink = []    # rotacion world del frame de cada link (tras la junta)
    plink = []    # origen world del frame de cada link
    pcom_w = []   # COM world de cada link
    Rcom_w = []   # rotacion world del frame de inercia de cada link

    for i,L in enumerate(P.LINKS):
        Rf = quat2R(SX(L['body_quat'])); pf = SX(L['body_pos'])
        # frame fijo padre->body
        R_fix = mtimes(R_prev, Rf)
        p_fix = p_prev + mtimes(R_prev, pf)
        # junta: eje local a través de jnt_pos, angulo q[i]
        ax_l = SX(L['jnt_axis'])
        ax_world = mtimes(R_fix, ax_l)
        anchor = p_fix + mtimes(R_fix, SX(L['jnt_pos']))
        Rj = axisangle2R(ax_l, q[i])
        # frame del link tras la junta
        R_i = mtimes(R_fix, Rj)
        p_i = p_fix                      # origen del frame del link = p_fix (junta en jnt_pos)
        axis_w.append(ax_world); anch_w.append(anchor)
        # COM world + frame inercia
        Rcom = mtimes(R_i, quat2R(SX(L['iquat'])))
        pcom = p_i + mtimes(R_i, SX(L['ipos']))
        Rlink.append(R_i); plink.append(p_i); pcom_w.append(pcom); Rcom_w.append(Rcom)
        R_prev, p_prev = R_i, p_i

    # --- fingers (gripper) rigido sobre hand, m5=0: payload fijo movido por m1-m4 ---
    Fg = P.FINGERS
    R_hand, p_hand = Rlink[3], plink[3]
    R_fg = mtimes(R_hand, quat2R(SX(Fg['body_quat'])))          # m5=0 -> sin rot extra
    p_fg = p_hand + mtimes(R_hand, SX(Fg['body_pos']))
    pcom_fg = p_fg + mtimes(R_fg, SX(Fg['ipos']))
    Rcom_fg = mtimes(R_fg, quat2R(SX(Fg['iquat'])))
    # añadir como link extra movido por joints 0..3
    EXTRA = [dict(mass=Fg['mass'], idiag=Fg['idiag'], pcom=pcom_fg, Rcom=Rcom_fg, upto=3)]

    # --- lista unificada de cuerpos: 4 links jointed + fingers (payload) ---
    BODIES = [dict(mass=L['mass'], idiag=L['idiag'], pcom=pcom_w[i], Rcom=Rcom_w[i], upto=i)
              for i,L in enumerate(P.LINKS)] + EXTRA

    # --- Jacobianos geometricos por cuerpo COM ---
    M = SX.zeros(NQ, NQ)
    PE = SX(0)
    for B in BODIES:
        mi = B['mass']
        # inercia about COM en world: R_com diag(idiag) R_com^T
        Ib = ca.diag(SX(B['idiag']))
        Iw = mtimes(mtimes(B['Rcom'], Ib), B['Rcom'].T)
        # columnas j<=upto: revoluta -> Jw col = axis_j ; Jv col = axis_j x (pcom - anchor_j)
        Jv = SX.zeros(3, NQ); Jw = SX.zeros(3, NQ)
        for j in range(B['upto']+1):
            Jw[:,j] = axis_w[j]
            Jv[:,j] = cross(axis_w[j], B['pcom']-anch_w[j])
        M += mi*mtimes(Jv.T,Jv) + mtimes(mtimes(Jw.T,Iw),Jw)
        # energia potencial: PE = -m g·pcom  (g_vec ya negativo en z)
        PE += -mi * ca.dot(g_vec, B['pcom'])

    # armature (inercia del rotor del servo) -> diagonal de M (igual que mj_fullM)
    M += ca.diag(SX(P.ARMATURE))

    # --- bias h(q,qd) = Mdot·qd - ½ d(qd^T M qd)/dq + dPE/dq ---
    Mqd = mtimes(M, qd)
    Mdot_qd = mtimes(jacobian(Mqd, q), qd)          # (dM/dq · qd) qd = Ṁ qd
    KE = 0.5*mtimes(qd.T, mtimes(M, qd))
    dKE_dq = gradient(KE, q)                          # ½ d(qd^T M qd)/dq
    g_term = gradient(PE, q)
    h = Mdot_qd - dKE_dq + g_term

    f_M = ca.Function("M", [q], [M])
    f_h = ca.Function("h", [q,qd], [h])
    return f_M, f_h

if __name__ == "__main__":
    f_M, f_h = build_dynamics()
    maxM=0; maxh=0
    print("== Validacion dinamica CasADi vs MuJoCo (%d configs) ==" % len(P.TESTS))
    for k,(q,qd,Mref,href,_ep,_eq) in enumerate(P.TESTS):
        Mc = np.array(f_M(q)); hc = np.array(f_h(q,qd)).flatten()
        eM = np.max(np.abs(Mc - np.array(Mref)))
        eh = np.max(np.abs(hc - np.array(href)))
        maxM=max(maxM,eM); maxh=max(maxh,eh)
        print(f" cfg{k}: max|M-Mmj|={eM:.2e}  max|h-hmj|={eh:.2e}")
    print("-"*48)
    print(f"MAX M_err={maxM:.2e}   MAX h_err={maxh:.2e}")
    # umbral 1e-7: el dump MuJoCo es %.12g (~1e-10 redondeo) -> ese es el piso
    print("DINAMICA SIMBOLICA VALIDADA (= precision del dump)" if (maxM<1e-7 and maxh<1e-7) else "REVISAR")
