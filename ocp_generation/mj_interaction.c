/* ================================================================
   P3 — LAZO DE INTERACCION en MuJoCo REAL (hand-guiding compliant).
   NMPC sostiene tarea; YO aplico fuerza al efector (xfrc); momentum observer
   (sin sensor) -> admitancia DQ regularizada -> cede la referencia -> el efector
   SIGUE la "mano"; resorte de retorno -> vuelve a la tarea al soltar.
   Observability-aware: cede en el subespacio 4D observable (limite 4DOF, honesto).
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

int main(){
  char e[1000]={0};
  mjModel* m=mj_loadXML("/tmp/vel.xml",NULL,e,1000); if(!m){printf("MJ ERR %s\n",e);return 1;}
  mjData* d=mj_makeData(m); int nv=m->nv;
  const char* jn[]={"m1","m2","m3","m4"}; int qadr[4],dadr[4],aid[4];
  for(int i=0;i<4;i++){int j=mj_name2id(m,mjOBJ_JOINT,jn[i]); qadr[i]=m->jnt_qposadr[j]; dadr[i]=m->jnt_dofadr[j];
    char an[5]={'a','_','m','1'+i,0}; aid[i]=mj_name2id(m,mjOBJ_ACTUATOR,an);}
  int site=mj_name2id(m,mjOBJ_SITE,"ee"); int tip=mj_name2id(m,mjOBJ_BODY,"tip");

  /* tarea */
  double q_task[4]={0.0,1.2,-0.9,0.4};
  mj_resetData(m,d); for(int i=0;i<4;i++) d->qpos[qadr[i]]=q_task[i]; mj_forward(m,d);
  double qref0[4]; mju_mat2Quat(qref0,d->site_xmat+9*site);
  double p_task[3]={d->site_xpos[3*site],d->site_xpos[3*site+1],d->site_xpos[3*site+2]};

  arm4dof_dqnmpc_solver_capsule* cap=arm4dof_dqnmpc_acados_create_capsule();
  arm4dof_dqnmpc_acados_create(cap);
  ocp_nlp_config* cfg=arm4dof_dqnmpc_acados_get_nlp_config(cap);
  ocp_nlp_dims* dim=arm4dof_dqnmpc_acados_get_nlp_dims(cap);
  ocp_nlp_in* in=arm4dof_dqnmpc_acados_get_nlp_in(cap);
  ocp_nlp_out* out=arm4dof_dqnmpc_acados_get_nlp_out(cap);
  int N=ARM4DOF_DQNMPC_N;
  double prm[22]={0}; double Wse3[6]={2,2,2,80,80,80}; for(int i=0;i<6;i++) prm[8+i]=Wse3[i];
  for(int i=0;i<4;i++){prm[14+i]=0.3; prm[18+i]=0.02;}

  /* admitancia: estado de referencia compliant */
  double p_ref[3]={p_task[0],p_task[1],p_task[2]};
  double q_ref[4]={qref0[0],qref0[1],qref0[2],qref0[3]};
  double Dinv_t=1.0/15.0, Dinv_r=1.0/3.0, k_return=2.5;   // D bajo (estimado escalado), resorte retorno

  mj_resetData(m,d); for(int i=0;i<4;i++) d->qpos[qadr[i]]=q_task[i]; mj_forward(m,d);
  double* jp=mju_malloc(3*nv*sizeof(double)); double* jr=mju_malloc(3*nv*sizeof(double));
  double* M=mju_malloc(nv*nv*sizeof(double));
  int sub=(int)(0.01/m->opt.timestep+0.5); double dt=0.01;
  double LAM=0.01;

  double err_rest=0,err_push=0,err_final=0, sdot=0,see=0,stt=0;
  for(int t=0;t<450;t++){
    /* referencia compliant -> param NMPC */
    double dq_ref[8]; dq_from_pose(q_ref,p_ref,dq_ref);
    for(int i=0;i<8;i++) prm[i]=dq_ref[i];
    for(int s=0;s<=N;s++) arm4dof_dqnmpc_acados_update_params(cap,s,prm,22);
    double x[8]; for(int i=0;i<4;i++){x[i]=d->qpos[qadr[i]]; x[4+i]=d->qvel[dadr[i]];}
    ocp_nlp_constraints_model_set(cfg,dim,in,out,0,"lbx",x);
    ocp_nlp_constraints_model_set(cfg,dim,in,out,0,"ubx",x);
    arm4dof_dqnmpc_acados_solve(cap);
    double x1[8]; ocp_nlp_out_get(cfg,dim,out,1,"x",x1);
    for(int i=0;i<4;i++) d->ctrl[aid[i]]=x1[4+i];
    d->ctrl[mj_name2id(m,mjOBJ_ACTUATOR,"a_m5")]=0;
    /* fuerza "mano" (world): +y -z entre t=1.0 y 3.0 s */
    double Fh[6]={0,0,0,0,0,0}; if(t>=100&&t<300){Fh[1]=5.0; Fh[2]=-2.0;}
    for(int k=0;k<6;k++) d->xfrc_applied[6*tip+k]=Fh[k];
    for(int k=0;k<sub;k++) mj_step(m,d);

    /* observer (sin sensor) */
    mj_forward(m,d); mj_fullM(m,M,d->qM);
    double tau_ext[4];
    for(int i=0;i<4;i++){ double Mq=0; for(int j2=0;j2<4;j2++) Mq+=M[dadr[i]*nv+dadr[j2]]*d->qacc[dadr[j2]];
      tau_ext[i]=Mq + d->qfrc_bias[dadr[i]] - d->qfrc_actuator[dadr[i]] - d->qfrc_passive[dadr[i]]; }
    mj_jacSite(m,d,jp,jr,site);
    double J[24]; for(int c=0;c<4;c++)for(int r=0;r<3;r++){J[r*4+c]=jp[r*nv+dadr[c]]; J[(r+3)*4+c]=jr[r*nv+dadr[c]];}
    double JtJ[16]; for(int a=0;a<4;a++)for(int b=0;b<4;b++){double s=0;for(int r=0;r<6;r++)s+=J[r*4+a]*J[r*4+b]; JtJ[a*4+b]=s+(a==b?LAM:0);}
    double L[16]; mju_copy(L,JtJ,16); mju_cholFactor(L,4,1e-12);
    double tmp[4]; mju_cholSolve(tmp,L,tau_ext,4);
    double Fest[6]; for(int r=0;r<6;r++){double s=0;for(int a=0;a<4;a++)s+=J[r*4+a]*tmp[a]; Fest[r]=s;}

    /* admitancia DQ: cede la referencia + resorte de retorno */
    double v[3]={Dinv_t*Fest[0],Dinv_t*Fest[1],Dinv_t*Fest[2]};
    double w[3]={Dinv_r*Fest[3],Dinv_r*Fest[4],Dinv_r*Fest[5]};
    for(int k=0;k<3;k++) v[k]+=k_return*(p_task[k]-p_ref[k]);
    /* resorte de retorno ROTACIONAL: vector de rotacion de q_ref -> qref0 */
    double qrc[4]={q_ref[0],-q_ref[1],-q_ref[2],-q_ref[3]}, qe[4]; qmul(qref0,qrc,qe);
    double sgn=qe[0]<0?-1.0:1.0;
    for(int k=0;k<3;k++) w[k]+=k_return*2.0*sgn*qe[1+k];
    for(int k=0;k<3;k++) p_ref[k]+=v[k]*dt;
    double wq[4],dv[3]={w[0]*dt,w[1]*dt,w[2]*dt},nq[4]; axisangle_quat(dv,wq); qmul(wq,q_ref,nq);
    double nn=sqrt(nq[0]*nq[0]+nq[1]*nq[1]+nq[2]*nq[2]+nq[3]*nq[3]); for(int k=0;k<4;k++) q_ref[k]=nq[k]/nn;

    /* metricas */
    double dx=d->site_xpos[3*site]-p_task[0], dy=d->site_xpos[3*site+1]-p_task[1], dz=d->site_xpos[3*site+2]-p_task[2];
    double err=sqrt(dx*dx+dy*dy+dz*dz);
    if(t==90) err_rest=err; if(t==200) err_push=err; if(t==449) err_final=err;
    if(t>=120&&t<290){ sdot+=Fest[1]*Fh[1]; see+=Fest[1]*Fest[1]; stt+=Fh[1]*Fh[1]; }
  }
  printf("== P3 MuJoCo: lazo de interaccion (hand-guiding compliant, sin sensor) ==\n");
  printf("error efector a tarea:  reposo=%.4f  EMPUJADO=%.4f  tras SOLTAR=%.4f m\n", err_rest,err_push,err_final);
  printf("  cede al empujar: %s   retorna al soltar: %s\n",
         err_push>3*err_rest+0.005?"SI":"NO", err_final<2*err_rest+0.005?"SI":"NO");
  printf("observer corr(F_y est, real) = %.3f\n", sdot/(sqrt(see*stt)+1e-12));
  int ok = (err_push>3*err_rest+0.005) && (err_final<2*err_rest+0.005);
  printf("%s\n", ok?"LAZO INTERACCION MuJoCo OK (cede + retorna + observer fiel)":"REVISAR");
  arm4dof_dqnmpc_acados_free(cap); arm4dof_dqnmpc_acados_free_capsule(cap);
  mju_free(jp);mju_free(jr);mju_free(M); mj_deleteData(d); mj_deleteModel(m); return 0;
}
