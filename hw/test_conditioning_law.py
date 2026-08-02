"""
Verificacion DIRECTA de la cota de condicionamiento, a traves de la planta MuJoCo.

    ||delta_f||  <=  ||delta_tau|| / sigma_min(Jv)

El Monte Carlo del paper (test_noise_montecarlo.py) es ANALITICO: aplica ruido
sobre el Jacobiano sin pasar por la dinamica. Aca el estimador se maneja a traves
del motor de fisica completo, con damping, armature y friccion seca que el modelo
del estimador NO tiene, y el ruido se inyecta en la LECTURA DE CORRIENTE, que es
donde lo lleva el servo real.

⚠️ MuJoCo es determinista: en equilibrio el par es exactamente constante y la
cuantizacion sola da varianza CERO. Por eso el ruido se MODELA (mj_arm.MjArm
acepta cur_noise_A). Lo que esto establece es que la cota gobierna al estimador
cuando se lo maneja a traves de la planta no lineal completa, no solo en el
planteo algebraico. Reemplazar el ruido modelado por el de un servo real es el
paso que falta, y es una medicion estatica con los actuadores deshabilitados
(ver s3_conditioning.py, parte A).

Resultado medido: pendiente -0.872 (R2 0.980) con sigma de 1 LSB, y -0.790
(R2 0.987) con 3 LSB, contra la prediccion de -1. La dispersion queda por DEBAJO
de la cota (razon 0.48-0.77), que es lo correcto: la cota se alcanza solo cuando
el error se alinea con la direccion menos observable.

Run:  python3 test_conditioning_law.py
"""
import numpy as np, sys
sys.path.insert(0,'.'); sys.path.insert(0,'../ocp_generation')
import mj_arm
from s3_conditioning import POSES, Jv_
from validate_procedures import hold
print("="*76)
print("LEY DE ESCALA a traves de la planta MuJoCo, con ruido de corriente inyectado")
print("="*76)
print("MuJoCo aporta la dinamica (damping, armature, frictionloss) y el ruido se")
print("inyecta en la lectura de corriente, que es donde lo tiene el servo real.")
print()
for noise, lab in [(0.003,"1 LSB (3.4 mA)"),(0.010,"3 LSB (10 mA)")]:
    print(f"--- sigma de corriente = {lab} ---")
    print(f"  {'sigma_min':>10} {'std(tau)':>10} {'std(f_est)':>11} {'predicho':>10} {'razon':>7}")
    S=[];D=[]
    for p,(qr,_) in enumerate(POSES):
        a=mj_arm.MjArm(cur_noise_A=noise, seed=p)
        a.set_velocity_mode(); a.enable(True); a.set_state(qr); hold(a,qr)
        T=[];Q=[]
        for _ in range(600):
            q,qd,t=a.read(); a.write_velocity(np.clip(3.0*(qr-q),-0.4,0.4))
            T.append(t); Q.append(q)
        T=np.array(T); qm=np.mean(Q,0); Jv=Jv_(qm)
        s=np.linalg.svd(Jv,compute_uv=False)[2]
        F=np.array([np.linalg.lstsq(Jv.T,t,rcond=None)[0] for t in T])
        st=float(np.mean(np.std(T,0))); sf=float(np.mean(np.std(F,0)))
        S.append(s); D.append(sf)
        print(f"  {s:10.4f} {st:10.5f} {sf:11.4f} {st/s:10.4f} {sf/(st/s):7.2f}")
        a.close()
    S=np.array(S); D=np.array(D)
    A=np.column_stack([np.log(S),np.ones(len(S))])
    c,*_=np.linalg.lstsq(A,np.log(D),rcond=None)
    r2=1-np.sum((np.log(D)-A@c)**2)/np.sum((np.log(D)-np.log(D).mean())**2)
    ok = -1.4<=c[0]<=-0.6 and r2>0.8
    print(f"  pendiente = {c[0]:+.3f}  (prediccion -1)   R2 = {r2:.3f}   "
          f"{'LEY VERIFICADA' if ok else 'NO VERIFICA'}")
    print()
