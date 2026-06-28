/* ================================================================
   Lazo cerrado DQ-NMPC vs MuJoCo REAL (planta, no dinamica analitica).
   Control por VELOCIDAD: NMPC -> q̇*[1] -> ctrl (actuador velocity) -> mj_step.
   Valida el controlador contra la dinamica de verdad de MuJoCo.
   ================================================================ */
#include <mujoco/mujoco.h>
#include <stdio.h>
#include <math.h>
#include "acados_solver_arm4dof_dqnmpc.h"
#include "acados_c/ocp_nlp_interface.h"

/* quat helpers para construir dq_ref */
static void qmul(const double*a,const double*b,double*o){
  o[0]=a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3];
  o[1]=a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2];
  o[2]=a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1];
  o[3]=a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0];
}

int main(){
  char e[1000]={0};
  mjModel* m=mj_loadXML("/tmp/vel.xml",NULL,e,1000);
  if(!m){printf("MJ ERR %s\n",e);return 1;}
  mjData* d=mj_makeData(m);
  const char* jn[]={"m1","m2","m3","m4"};
  int qadr[4],dadr[4],aid[4];
  for(int i=0;i<4;i++){int j=mj_name2id(m,mjOBJ_JOINT,jn[i]); qadr[i]=m->jnt_qposadr[j]; dadr[i]=m->jnt_dofadr[j];
    char an[5]={'a','_','m','1'+i,0}; aid[i]=mj_name2id(m,mjOBJ_ACTUATOR,an);}
  int site=mj_name2id(m,mjOBJ_SITE,"ee");

  /* ---- target: dq_ref desde pose del site en q_tgt ---- */
  double q_tgt[4]={0.6,1.4,-0.9,0.3};
  mj_resetData(m,d);
  for(int i=0;i<4;i++) d->qpos[qadr[i]]=q_tgt[i];
  mj_forward(m,d);
  double p_tgt[3]={d->site_xpos[3*site],d->site_xpos[3*site+1],d->site_xpos[3*site+2]};
  double qr[4]; mju_mat2Quat(qr,d->site_xmat+9*site);
  double tq[4]={0,p_tgt[0],p_tgt[1],p_tgt[2]}, qd_[4]; qmul(tq,qr,qd_);
  double dq_ref[8]={qr[0],qr[1],qr[2],qr[3],0.5*qd_[0],0.5*qd_[1],0.5*qd_[2],0.5*qd_[3]};

  /* ---- acados solver ---- */
  arm4dof_dqnmpc_solver_capsule* cap=arm4dof_dqnmpc_acados_create_capsule();
  if(arm4dof_dqnmpc_acados_create(cap)){printf("acados create FAIL\n");return 1;}
  ocp_nlp_config* cfg=arm4dof_dqnmpc_acados_get_nlp_config(cap);
  ocp_nlp_dims* dim=arm4dof_dqnmpc_acados_get_nlp_dims(cap);
  ocp_nlp_in* in=arm4dof_dqnmpc_acados_get_nlp_in(cap);
  ocp_nlp_out* out=arm4dof_dqnmpc_acados_get_nlp_out(cap);
  int N=ARM4DOF_DQNMPC_N;

  double prm[22]={0};
  for(int i=0;i<8;i++) prm[i]=dq_ref[i];
  double Wse3[6]={2,2,2,80,80,80}; for(int i=0;i<6;i++) prm[8+i]=Wse3[i];
  for(int i=0;i<4;i++) prm[14+i]=0.3;   /* W_qd */
  for(int i=0;i<4;i++) prm[18+i]=0.02;  /* W_u  */
  for(int s=0;s<=N;s++) arm4dof_dqnmpc_acados_update_params(cap,s,prm,22);

  /* ---- estado inicial planta ---- */
  mj_resetData(m,d);
  double x0[8]={0,1.0,-1.0,0.5,0,0,0,0};
  for(int i=0;i<4;i++){d->qpos[qadr[i]]=x0[i]; d->qvel[dadr[i]]=0;}
  mj_forward(m,d);

  int T=300, okc=0; double e0=0,ef=0,emin=1e9;
  double dt_ctrl=0.01; int sub=(int)(dt_ctrl/m->opt.timestep+0.5);  /* substeps */
  for(int t=0;t<T;t++){
    double x[8];
    for(int i=0;i<4;i++){x[i]=d->qpos[qadr[i]]; x[4+i]=d->qvel[dadr[i]];}
    ocp_nlp_constraints_model_set(cfg,dim,in,out,0,"lbx",x);
    ocp_nlp_constraints_model_set(cfg,dim,in,out,0,"ubx",x);
    int st=arm4dof_dqnmpc_acados_solve(cap);
    if(st==0) okc++;
    double x1[8]; ocp_nlp_out_get(cfg,dim,out,1,"x",x1);
    double qd_cmd[4]={x1[4],x1[5],x1[6],x1[7]};   /* q̇*[1] = comando velocidad servo */
    for(int i=0;i<4;i++) d->ctrl[aid[i]]=qd_cmd[i];
    d->ctrl[mj_name2id(m,mjOBJ_ACTUATOR,"a_m5")]=0;
    for(int k=0;k<sub;k++) mj_step(m,d);
    /* error efector */
    double dx=d->site_xpos[3*site]-p_tgt[0], dy=d->site_xpos[3*site+1]-p_tgt[1], dz=d->site_xpos[3*site+2]-p_tgt[2];
    double err=sqrt(dx*dx+dy*dy+dz*dz);
    if(t==0)e0=err; ef=err; if(err<emin)emin=err;
  }
  printf("== Lazo cerrado DQ-NMPC vs MuJoCo REAL (control velocidad) ==\n");
  printf("target pos efector = [%.4f %.4f %.4f]\n",p_tgt[0],p_tgt[1],p_tgt[2]);
  printf("status solver: %d/%d OK\n",okc,T);
  printf("error efector: inicial=%.4f  ->  final=%.5f m  (min=%.5f)\n",e0,ef,emin);
  printf("q final = [%.3f %.3f %.3f %.3f]\n",d->qpos[qadr[0]],d->qpos[qadr[1]],d->qpos[qadr[2]],d->qpos[qadr[3]]);
  printf("%s\n",(ef<0.01 && okc==T)?"LAZO CERRADO MuJoCo OK":"REVISAR");
  arm4dof_dqnmpc_acados_free(cap); arm4dof_dqnmpc_acados_free_capsule(cap);
  mj_deleteData(d); mj_deleteModel(m); return 0;
}
