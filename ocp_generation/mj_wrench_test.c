/* ================================================================
   P1 en MuJoCo REAL — estimacion de wrench SIN SENSOR vs ground truth.
   El NMPC sostiene una pose; YO aplico una fuerza al efector (xfrc_applied
   = "la mano"); el momentum observer la estima con cantidades EXACTAS de
   MuJoCo (qacc, qfrc_actuator, qfrc_passive, qfrc_bias). Comparo F_est vs F real.
   τ_ext = M qacc + qfrc_bias − qfrc_actuator − qfrc_passive = Jᵀ F_ext
   F_est = (Jᵀ)⁺ τ_ext   (subespacio observable 4D)
   ================================================================ */
#include <mujoco/mujoco.h>
#include <stdio.h>
#include <math.h>
#include "acados_solver_arm4dof_dqnmpc.h"
#include "acados_c/ocp_nlp_interface.h"

static void qmul(const double*a,const double*b,double*o){
  o[0]=a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3];
  o[1]=a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2];
  o[2]=a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1];
  o[3]=a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]; }

int main(){
  char e[1000]={0};
  mjModel* m=mj_loadXML("/tmp/vel.xml",NULL,e,1000);
  if(!m){printf("MJ ERR %s\n",e);return 1;}
  mjData* d=mj_makeData(m); int nv=m->nv;
  const char* jn[]={"m1","m2","m3","m4"}; int qadr[4],dadr[4],aid[4];
  for(int i=0;i<4;i++){int j=mj_name2id(m,mjOBJ_JOINT,jn[i]); qadr[i]=m->jnt_qposadr[j]; dadr[i]=m->jnt_dofadr[j];
    char an[5]={'a','_','m','1'+i,0}; aid[i]=mj_name2id(m,mjOBJ_ACTUATOR,an);}
  int site=mj_name2id(m,mjOBJ_SITE,"ee"); int tip=mj_name2id(m,mjOBJ_BODY,"tip");

  /* dq_ref de la tarea (sostener pose q_task) */
  double q_task[4]={0.0,1.2,-0.9,0.4};
  mj_resetData(m,d); for(int i=0;i<4;i++) d->qpos[qadr[i]]=q_task[i]; mj_forward(m,d);
  double qr[4]; mju_mat2Quat(qr,d->site_xmat+9*site);
  double pt[3]={d->site_xpos[3*site],d->site_xpos[3*site+1],d->site_xpos[3*site+2]};
  double tq[4]={0,pt[0],pt[1],pt[2]},qd_[4]; qmul(tq,qr,qd_);
  double dq_ref[8]={qr[0],qr[1],qr[2],qr[3],0.5*qd_[0],0.5*qd_[1],0.5*qd_[2],0.5*qd_[3]};

  arm4dof_dqnmpc_solver_capsule* cap=arm4dof_dqnmpc_acados_create_capsule();
  arm4dof_dqnmpc_acados_create(cap);
  ocp_nlp_config* cfg=arm4dof_dqnmpc_acados_get_nlp_config(cap);
  ocp_nlp_dims* dim=arm4dof_dqnmpc_acados_get_nlp_dims(cap);
  ocp_nlp_in* in=arm4dof_dqnmpc_acados_get_nlp_in(cap);
  ocp_nlp_out* out=arm4dof_dqnmpc_acados_get_nlp_out(cap);
  int N=ARM4DOF_DQNMPC_N;
  double prm[22]={0}; for(int i=0;i<8;i++) prm[i]=dq_ref[i];
  double Wse3[6]={2,2,2,80,80,80}; for(int i=0;i<6;i++) prm[8+i]=Wse3[i];
  for(int i=0;i<4;i++){prm[14+i]=0.3; prm[18+i]=0.02;}
  for(int s=0;s<=N;s++) arm4dof_dqnmpc_acados_update_params(cap,s,prm,22);

  /* arranque en la tarea */
  mj_resetData(m,d); for(int i=0;i<4;i++) d->qpos[qadr[i]]=q_task[i]; mj_forward(m,d);

  double* jp=mju_malloc(3*nv*sizeof(double));
  double* jr=mju_malloc(3*nv*sizeof(double));
  double* M=mju_malloc(nv*nv*sizeof(double));
  int sub=(int)(0.01/m->opt.timestep+0.5);

  /* fuerza "mano" en el efector: +y, -z entre t=1.0s y 2.5s */
  double sum_dot=0,sum_tt=0,sum_ee=0; int ncnt=0;
  double f_at_peak[3]={0,0,0}, f3_at_peak[3]={0,0,0}, ftrue_peak[3]={0,0,0};
  double e6_acc=0, e3_acc=0; int nerr=0;
  for(int t=0;t<400;t++){
    double x[8]; for(int i=0;i<4;i++){x[i]=d->qpos[qadr[i]]; x[4+i]=d->qvel[dadr[i]];}
    ocp_nlp_constraints_model_set(cfg,dim,in,out,0,"lbx",x);
    ocp_nlp_constraints_model_set(cfg,dim,in,out,0,"ubx",x);
    arm4dof_dqnmpc_acados_solve(cap);
    double x1[8]; ocp_nlp_out_get(cfg,dim,out,1,"x",x1);
    for(int i=0;i<4;i++) d->ctrl[aid[i]]=x1[4+i];
    d->ctrl[mj_name2id(m,mjOBJ_ACTUATOR,"a_m5")]=0;
    /* fuerza de la mano (world) en el body tip */
    double Fh[6]={0,0,0,0,0,0};
    if(t>=100 && t<250){ Fh[1]=4.0; Fh[2]=-1.5; }
    for(int k=0;k<6;k++) d->xfrc_applied[6*tip+k]=Fh[k];
    for(int k=0;k<sub;k++) mj_step(m,d);

    /* momentum observer: recomputar qacc/qfrc consistentes con el estado actual + xfrc */
    mj_forward(m,d);
    mj_fullM(m,M,d->qM);
    double tau_ext[4];
    for(int i=0;i<4;i++){
      double Mqacc=0; for(int j2=0;j2<4;j2++) Mqacc+=M[dadr[i]*nv+dadr[j2]]*d->qacc[dadr[j2]];
      tau_ext[i]=Mqacc + d->qfrc_bias[dadr[i]] - d->qfrc_actuator[dadr[i]] - d->qfrc_passive[dadr[i]];
    }
    /* F_est = J (JᵀJ)⁻¹ tau_ext en el efector */
    mj_jacSite(m,d,jp,jr,site);
    double J[24]; /* 6x4: filas v(3)+w(3) */
    for(int c=0;c<4;c++){ for(int r=0;r<3;r++){J[r*4+c]=jp[r*nv+dadr[c]]; J[(r+3)*4+c]=jr[r*nv+dadr[c]];} }
    double JtJ[16]; for(int a=0;a<4;a++)for(int b=0;b<4;b++){double s=0;for(int r=0;r<6;r++)s+=J[r*4+a]*J[r*4+b]; JtJ[a*4+b]=s+(a==b?1e-6:0);}
    double L[16]; mju_copy(L,JtJ,16); mju_cholFactor(L,4,1e-12);
    double tmp[4]; mju_cholSolve(tmp,L,tau_ext,4);
    double Fest[6]; for(int r=0;r<6;r++){double s=0;for(int a=0;a<4;a++)s+=J[r*4+a]*tmp[a]; Fest[r]=s;}

    /* --- estimador de FUERZA PURA (3D) ---------------------------------------
       El de arriba resuelve JᵀF = tau con F∈R⁶ desde 4 medidas: indeterminado, y
       la solucion de norma minima reparte la fuerza como momento. Si el contacto
       es una fuerza pura en un punto conocido (el efector), el modelo correcto es
       Jvᵀ f = tau con f∈R³, que es SOBRE-determinado. Ecuaciones normales:
       (Jv Jvᵀ) f = Jv tau_ext,  Jv = 3x4. */
    double A[9]; for(int a=0;a<3;a++)for(int b=0;b<3;b++){double s=0;
      for(int c=0;c<4;c++) s+=J[a*4+c]*J[b*4+c]; A[a*3+b]=s+(a==b?1e-9:0);}
    double bb[3]; for(int a=0;a<3;a++){double s=0; for(int c=0;c<4;c++) s+=J[a*4+c]*tau_ext[c]; bb[a]=s;}
    double LA[9]; mju_copy(LA,A,9); mju_cholFactor(LA,3,1e-12);
    double Fest3[3]; mju_cholSolve(Fest3,LA,bb,3);

    if(t>=110 && t<240){ /* ventana estable de empuje: correlacion fuerza y */
      sum_dot+=Fest[1]*Fh[1]; sum_ee+=Fest[1]*Fest[1]; sum_tt+=Fh[1]*Fh[1]; ncnt++;
      for(int k=0;k<3;k++){ e6_acc+=(Fest[k]-Fh[k])*(Fest[k]-Fh[k]);
                            e3_acc+=(Fest3[k]-Fh[k])*(Fest3[k]-Fh[k]); }
      nerr++;
      if(t==180){ for(int k=0;k<3;k++){f_at_peak[k]=Fest[k]; f3_at_peak[k]=Fest3[k]; ftrue_peak[k]=Fh[k];}
        /* proyeccion OBSERVABLE de la fuerza real = lo recuperable = J(JtJ)^-1 Jt F */
        double Ft[6]={Fh[0],Fh[1],Fh[2],Fh[3],Fh[4],Fh[5]};
        double JtF[4]; for(int a=0;a<4;a++){double s=0;for(int r=0;r<6;r++)s+=J[r*4+a]*Ft[r]; JtF[a]=s;}
        double tmp2[4]; mju_cholSolve(tmp2,L,JtF,4);
        double Fobs[6]; for(int r=0;r<6;r++){double s=0;for(int a=0;a<4;a++)s+=J[r*4+a]*tmp2[a]; Fobs[r]=s;}
        double eo=0,nf=0,no=0; for(int k=0;k<6;k++){eo+=fabs(Fest[k]-Fobs[k]); nf+=Ft[k]*Ft[k]; no+=Fobs[k]*Fobs[k];}
        /* DEBUG: tau_ext vs Jt F_true vs constraint */
        double JtFtrue[4]; for(int a=0;a<4;a++){double s=0;for(int r=0;r<6;r++)s+=J[r*4+a]*Ft[r]; JtFtrue[a]=s;}
        double ncon_f=0; for(int i=0;i<4;i++) ncon_f+=fabs(d->qfrc_constraint[dadr[i]]);
        printf("DEBUG tau_ext   = [%.3f %.3f %.3f %.3f]\n",tau_ext[0],tau_ext[1],tau_ext[2],tau_ext[3]);
        printf("DEBUG Jt F_true = [%.3f %.3f %.3f %.3f]\n",JtFtrue[0],JtFtrue[1],JtFtrue[2],JtFtrue[3]);
        printf("DEBUG |qfrc_constraint(arm)|=%.3f  ncon=%d\n", ncon_f, d->ncon);
        printf("F_obs (proy. observable de la real) = [%.2f %.2f %.2f] N\n",Fobs[0],Fobs[1],Fobs[2]);
        printf("  |F_est - F_obs| = %.2e (estimador exacto)   observabilidad |F_obs|/|F_real| = %.0f%%\n",
               eo, 100*sqrt(no/nf));
      }
    }
  }
  printf("== P1 MuJoCo: estimacion sin sensor vs ground truth ==\n");
  printf("fuerza mano REAL (pico)      = [%6.2f %6.2f %6.2f] N\n", ftrue_peak[0],ftrue_peak[1],ftrue_peak[2]);
  printf("WRENCH-6D min-norm (pico)    = [%6.2f %6.2f %6.2f] N\n", f_at_peak[0],f_at_peak[1],f_at_peak[2]);
  printf("FUERZA-3D contacto puro(pico)= [%6.2f %6.2f %6.2f] N\n", f3_at_peak[0],f3_at_peak[1],f3_at_peak[2]);
  double corr=sum_dot/(sqrt(sum_ee*sum_tt)+1e-12);
  printf("corr(F_y est 6D, F_y real) en ventana = %.3f\n", corr);
  printf("RMSE contra la fuerza APLICADA (ventana):  6D %.3f N   3D %.3f N\n",
         sqrt(e6_acc/(3*nerr)), sqrt(e3_acc/(3*nerr)));
  printf("%s\n", (sqrt(e3_acc/(3*nerr)) < sqrt(e6_acc/(3*nerr)))
         ? "FUERZA-3D MEJOR QUE 6D EN MuJoCo" : "REVISAR");
  arm4dof_dqnmpc_acados_free(cap); arm4dof_dqnmpc_acados_free_capsule(cap);
  mju_free(jp);mju_free(jr);mju_free(M); mj_deleteData(d); mj_deleteModel(m); return 0;
}
