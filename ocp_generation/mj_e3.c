/* ================================================================
   E3 — Tracking CONSCIENTE DE INTERACCION vs baseline RIGIDO.
   Mismo empuje (xfrc) a dos controladores:
     RIGIDO   : NMPC trackea ref FIJA (sin admitancia) -> pelea la mano
     COMPLIANT: NMPC + admitancia DQ -> cede + retorna
   Metricas: esfuerzo de actuador resistiendo, desplazamiento (compliance),
   recuperacion post-soltar. Demuestra: compliant = seguro/blando, rigido = pelea.
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
static void axisangle_quat(const double*v,double*q){
  double th=sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
  if(th<1e-12){q[0]=1;q[1]=q[2]=q[3]=0;return;}
  double s=sin(th/2)/th; q[0]=cos(th/2); q[1]=s*v[0]; q[2]=s*v[1]; q[3]=s*v[2]; }
static void dq_from_pose(const double*quat,const double*t,double*dq){
  double tq[4]={0,t[0],t[1],t[2]},qd[4]; qmul(tq,quat,qd);
  for(int i=0;i<4;i++) dq[i]=quat[i]; for(int i=0;i<4;i++) dq[4+i]=0.5*qd[i]; }

typedef struct { double effort_push, disp_push, err_final; } Metrics;

static mjModel* m; static int qadr[4],dadr[4],aid[4],site,tip,am5;

static Metrics run(int stiff, arm4dof_dqnmpc_solver_capsule* cap){
  ocp_nlp_config* cfg=arm4dof_dqnmpc_acados_get_nlp_config(cap);
  ocp_nlp_dims* dim=arm4dof_dqnmpc_acados_get_nlp_dims(cap);
  ocp_nlp_in* in=arm4dof_dqnmpc_acados_get_nlp_in(cap);
  ocp_nlp_out* out=arm4dof_dqnmpc_acados_get_nlp_out(cap);
  int N=ARM4DOF_DQNMPC_N; int nv=m->nv;
  mjData* d=mj_makeData(m);
  double q_task[4]={0.0,1.2,-0.9,0.4};
  mj_resetData(m,d); for(int i=0;i<4;i++) d->qpos[qadr[i]]=q_task[i]; mj_forward(m,d);
  double qref0[4]; mju_mat2Quat(qref0,d->site_xmat+9*site);
  double p_task[3]={d->site_xpos[3*site],d->site_xpos[3*site+1],d->site_xpos[3*site+2]};
  double prm[22]={0}; double Wse3[6]={2,2,2,80,80,80}; for(int i=0;i<6;i++) prm[8+i]=Wse3[i];
  for(int i=0;i<4;i++){prm[14+i]=0.3; prm[18+i]=0.02;}
  double p_ref[3]={p_task[0],p_task[1],p_task[2]}, q_ref[4]={qref0[0],qref0[1],qref0[2],qref0[3]};
  double Dinv_t=1.0/6.0,Dinv_r=1.0/1.5,k_return=0.3,LAM=0.01;  /* guiar: D bajo, retorno casi nulo */
  double* jp=mju_malloc(3*nv*sizeof(double)); double* jr=mju_malloc(3*nv*sizeof(double)); double* M=mju_malloc(nv*nv*sizeof(double));
  int sub=(int)(0.01/m->opt.timestep+0.5); double dt=0.01;
  double eff=0,dispmax=0,errf=0; int npush=0;   /* eff=fuerza guia, dispmax=err seguir mano */
  for(int t=0;t<450;t++){
    /* la MANO = resorte a un objetivo que se mueve (intencion humana) */
    double p_hand[3]={p_task[0],p_task[1],p_task[2]};
    if(t>=100){ double a=(t<300)?(t-100)/200.0:1.0;     /* mueve la mano +z, -x, +y */
      p_hand[0]+=-0.03*a; p_hand[1]+=0.04*a; p_hand[2]+=0.05*a; }
    double dq_ref[8]; dq_from_pose(q_ref,p_ref,dq_ref);
    for(int i=0;i<8;i++) prm[i]=dq_ref[i];
    for(int s=0;s<=N;s++) arm4dof_dqnmpc_acados_update_params(cap,s,prm,22);
    double x[8]; for(int i=0;i<4;i++){x[i]=d->qpos[qadr[i]]; x[4+i]=d->qvel[dadr[i]];}
    ocp_nlp_constraints_model_set(cfg,dim,in,out,0,"lbx",x);
    ocp_nlp_constraints_model_set(cfg,dim,in,out,0,"ubx",x);
    arm4dof_dqnmpc_acados_solve(cap);
    double x1[8]; ocp_nlp_out_get(cfg,dim,out,1,"x",x1);
    for(int i=0;i<4;i++) d->ctrl[aid[i]]=x1[4+i]; d->ctrl[am5]=0;
    /* fuerza de la mano = k_hand (p_hand - p_ee) : depende de si el brazo sigue */
    double k_hand=80.0, Fh[6]={0,0,0,0,0,0};
    for(int k=0;k<3;k++) Fh[k]=k_hand*(p_hand[k]-d->site_xpos[3*site+k]);
    for(int k=0;k<6;k++) d->xfrc_applied[6*tip+k]=Fh[k];
    for(int k=0;k<sub;k++) mj_step(m,d);
    mj_forward(m,d); mj_fullM(m,M,d->qM);
    double tau_ext[4];
    for(int i=0;i<4;i++){ double Mq=0; for(int j2=0;j2<4;j2++) Mq+=M[dadr[i]*nv+dadr[j2]]*d->qacc[dadr[j2]];
      tau_ext[i]=Mq + d->qfrc_bias[dadr[i]] - d->qfrc_actuator[dadr[i]] - d->qfrc_passive[dadr[i]]; }
    if(!stiff){
      mj_jacSite(m,d,jp,jr,site);
      double J[24]; for(int c=0;c<4;c++)for(int r=0;r<3;r++){J[r*4+c]=jp[r*nv+dadr[c]]; J[(r+3)*4+c]=jr[r*nv+dadr[c]];}
      double JtJ[16]; for(int a=0;a<4;a++)for(int b=0;b<4;b++){double s=0;for(int r=0;r<6;r++)s+=J[r*4+a]*J[r*4+b]; JtJ[a*4+b]=s+(a==b?LAM:0);}
      double L[16]; mju_copy(L,JtJ,16); mju_cholFactor(L,4,1e-12);
      double tmp[4]; mju_cholSolve(tmp,L,tau_ext,4);
      double Fe[6]; for(int r=0;r<6;r++){double s=0;for(int a=0;a<4;a++)s+=J[r*4+a]*tmp[a]; Fe[r]=s;}
      double v[3]={Dinv_t*Fe[0],Dinv_t*Fe[1],Dinv_t*Fe[2]}, w[3]={Dinv_r*Fe[3],Dinv_r*Fe[4],Dinv_r*Fe[5]};
      for(int k=0;k<3;k++) v[k]+=k_return*(p_task[k]-p_ref[k]);
      double qrc[4]={q_ref[0],-q_ref[1],-q_ref[2],-q_ref[3]},qe[4]; qmul(qref0,qrc,qe);
      double sg=qe[0]<0?-1.0:1.0; for(int k=0;k<3;k++) w[k]+=k_return*2.0*sg*qe[1+k];
      for(int k=0;k<3;k++) p_ref[k]+=v[k]*dt;
      double wq[4],dvv[3]={w[0]*dt,w[1]*dt,w[2]*dt},nq[4]; axisangle_quat(dvv,wq); qmul(wq,q_ref,nq);
      double nn=sqrt(nq[0]*nq[0]+nq[1]*nq[1]+nq[2]*nq[2]+nq[3]*nq[3]); for(int k=0;k<4;k++) q_ref[k]=nq[k]/nn;
    }
    /* metricas: fuerza que la mano ejerce (esfuerzo guiar) + seguimiento de la mano */
    double Fmag=sqrt(Fh[0]*Fh[0]+Fh[1]*Fh[1]+Fh[2]*Fh[2]);
    double tx=d->site_xpos[3*site]-p_hand[0],ty=d->site_xpos[3*site+1]-p_hand[1],tz=d->site_xpos[3*site+2]-p_hand[2];
    double trk=sqrt(tx*tx+ty*ty+tz*tz);
    if(t>=200){ eff+=Fmag; dispmax+=trk; npush++; }   /* tras transitorio */
  }
  mju_free(jp);mju_free(jr);mju_free(M); mj_deleteData(d);
  Metrics mt={eff/npush, dispmax/npush, errf}; return mt;
}

int main(){
  char e[1000]={0};
  m=mj_loadXML("/tmp/vel.xml",NULL,e,1000); if(!m){printf("ERR %s\n",e);return 1;}
  const char* jn[]={"m1","m2","m3","m4"};
  for(int i=0;i<4;i++){int j=mj_name2id(m,mjOBJ_JOINT,jn[i]); qadr[i]=m->jnt_qposadr[j]; dadr[i]=m->jnt_dofadr[j];
    char an[5]={'a','_','m','1'+i,0}; aid[i]=mj_name2id(m,mjOBJ_ACTUATOR,an);}
  site=mj_name2id(m,mjOBJ_SITE,"ee"); tip=mj_name2id(m,mjOBJ_BODY,"tip"); am5=mj_name2id(m,mjOBJ_ACTUATOR,"a_m5");
  arm4dof_dqnmpc_solver_capsule* cap=arm4dof_dqnmpc_acados_create_capsule();
  arm4dof_dqnmpc_acados_create(cap);
  Metrics st=run(1,cap), co=run(0,cap);
  printf("== E3: guiar el brazo a una pose nueva (mano = resorte) ==\n");
  printf("%-12s  fuerza_guia[N]  error_seguir_mano[m]\n","");
  printf("RIGIDO        %12.2f   %15.4f\n", st.effort_push, st.disp_push);
  printf("COMPLIANT     %12.2f   %15.4f\n", co.effort_push, co.disp_push);
  printf("-> compliant: %.0f%% MENOS fuerza pra guiar,  %.0f%% mejor seguimiento de la mano\n",
         100*(1-co.effort_push/st.effort_push), 100*(1-co.disp_push/st.disp_push));
  int ok = (co.effort_push < 0.6*st.effort_push) && (co.disp_push < 0.6*st.disp_push);
  printf("%s\n", ok?"E3 OK: compliant se guia facil (poca fuerza) y sigue la mano; rigido pelea":"REVISAR");
  arm4dof_dqnmpc_acados_free(cap); arm4dof_dqnmpc_acados_free_capsule(cap); mj_deleteModel(m); return 0;
}
