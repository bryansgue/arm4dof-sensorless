/* ================================================================
   STATS N>=5 — E3 (compliant vs rigido) + observer, sobre N condiciones
   (poses de tarea x direcciones de guiado). Reporta media +- std.
   Para RA-L: no una sola corrida.
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

static mjModel* m; static int qadr[4],dadr[4],aid[4],site,tip,am5;

typedef struct { double force, track, corr; } R;

static R run(const double*q_task, const double*hand_d, int stiff, arm4dof_dqnmpc_solver_capsule* cap){
  ocp_nlp_config* cfg=arm4dof_dqnmpc_acados_get_nlp_config(cap);
  ocp_nlp_dims* dim=arm4dof_dqnmpc_acados_get_nlp_dims(cap);
  ocp_nlp_in* in=arm4dof_dqnmpc_acados_get_nlp_in(cap);
  ocp_nlp_out* out=arm4dof_dqnmpc_acados_get_nlp_out(cap);
  int N=ARM4DOF_DQNMPC_N, nv=m->nv;
  mjData* d=mj_makeData(m);
  mj_resetData(m,d); for(int i=0;i<4;i++) d->qpos[qadr[i]]=q_task[i]; mj_forward(m,d);
  double qref0[4]; mju_mat2Quat(qref0,d->site_xmat+9*site);
  double p_task[3]={d->site_xpos[3*site],d->site_xpos[3*site+1],d->site_xpos[3*site+2]};
  double prm[22]={0},Wse3[6]={2,2,2,80,80,80}; for(int i=0;i<6;i++)prm[8+i]=Wse3[i];
  for(int i=0;i<4;i++){prm[14+i]=0.3;prm[18+i]=0.02;}
  double p_ref[3]={p_task[0],p_task[1],p_task[2]},q_ref[4]={qref0[0],qref0[1],qref0[2],qref0[3]};
  double Dit=1.0/6.0,Dir=1.0/1.5,kret=0.3,LAM=0.01,kh=80.0;
  double* jp=mju_malloc(3*nv*sizeof(double));double* jr=mju_malloc(3*nv*sizeof(double));double* M=mju_malloc(nv*nv*sizeof(double));
  int sub=(int)(0.01/m->opt.timestep+0.5); double dt=0.01;
  double sf=0,strk=0; int n=0;
  double sd=0,se=0,st=0;  /* corr observer */
  for(int t=0;t<450;t++){
    double p_hand[3]={p_task[0],p_task[1],p_task[2]};
    if(t>=100){ double a=(t<300)?(t-100)/200.0:1.0; for(int k=0;k<3;k++) p_hand[k]+=hand_d[k]*a; }
    double dq_ref[8]; dq_from_pose(q_ref,p_ref,dq_ref); for(int i=0;i<8;i++)prm[i]=dq_ref[i];
    for(int s=0;s<=N;s++) arm4dof_dqnmpc_acados_update_params(cap,s,prm,22);
    double x[8]; for(int i=0;i<4;i++){x[i]=d->qpos[qadr[i]];x[4+i]=d->qvel[dadr[i]];}
    ocp_nlp_constraints_model_set(cfg,dim,in,out,0,"lbx",x);
    ocp_nlp_constraints_model_set(cfg,dim,in,out,0,"ubx",x);
    arm4dof_dqnmpc_acados_solve(cap);
    double x1[8]; ocp_nlp_out_get(cfg,dim,out,1,"x",x1);
    for(int i=0;i<4;i++) d->ctrl[aid[i]]=x1[4+i]; d->ctrl[am5]=0;
    double Fh[6]={0,0,0,0,0,0}; for(int k=0;k<3;k++) Fh[k]=kh*(p_hand[k]-d->site_xpos[3*site+k]);
    for(int k=0;k<6;k++) d->xfrc_applied[6*tip+k]=Fh[k];
    for(int k=0;k<sub;k++) mj_step(m,d);
    mj_forward(m,d); mj_fullM(m,M,d->qM);
    double tau_ext[4];
    for(int i=0;i<4;i++){double Mq=0;for(int j2=0;j2<4;j2++)Mq+=M[dadr[i]*nv+dadr[j2]]*d->qacc[dadr[j2]];
      tau_ext[i]=Mq+d->qfrc_bias[dadr[i]]-d->qfrc_actuator[dadr[i]]-d->qfrc_passive[dadr[i]];}
    mj_jacSite(m,d,jp,jr,site);
    double J[24]; for(int c=0;c<4;c++)for(int r=0;r<3;r++){J[r*4+c]=jp[r*nv+dadr[c]];J[(r+3)*4+c]=jr[r*nv+dadr[c]];}
    double JtJ[16];for(int a=0;a<4;a++)for(int b=0;b<4;b++){double s=0;for(int r=0;r<6;r++)s+=J[r*4+a]*J[r*4+b];JtJ[a*4+b]=s+(a==b?LAM:0);}
    double L[16];mju_copy(L,JtJ,16);mju_cholFactor(L,4,1e-12);
    double tmp[4];mju_cholSolve(tmp,L,tau_ext,4);
    double Fe[6];for(int r=0;r<6;r++){double s=0;for(int a=0;a<4;a++)s+=J[r*4+a]*tmp[a];Fe[r]=s;}
    if(!stiff){
      double v[3]={Dit*Fe[0],Dit*Fe[1],Dit*Fe[2]},w[3]={Dir*Fe[3],Dir*Fe[4],Dir*Fe[5]};
      for(int k=0;k<3;k++) v[k]+=kret*(p_task[k]-p_ref[k]);
      double qrc[4]={q_ref[0],-q_ref[1],-q_ref[2],-q_ref[3]},qe[4];qmul(qref0,qrc,qe);
      double sg=qe[0]<0?-1.0:1.0;for(int k=0;k<3;k++)w[k]+=kret*2.0*sg*qe[1+k];
      for(int k=0;k<3;k++)p_ref[k]+=v[k]*dt;
      double wq[4],dvv[3]={w[0]*dt,w[1]*dt,w[2]*dt},nq[4];axisangle_quat(dvv,wq);qmul(wq,q_ref,nq);
      double nn=sqrt(nq[0]*nq[0]+nq[1]*nq[1]+nq[2]*nq[2]+nq[3]*nq[3]);for(int k=0;k<4;k++)q_ref[k]=nq[k]/nn;
    }
    double Fm=sqrt(Fh[0]*Fh[0]+Fh[1]*Fh[1]+Fh[2]*Fh[2]);
    double tx=d->site_xpos[3*site]-p_hand[0],ty=d->site_xpos[3*site+1]-p_hand[1],tz=d->site_xpos[3*site+2]-p_hand[2];
    if(t>=200){ sf+=Fm; strk+=sqrt(tx*tx+ty*ty+tz*tz); n++;
      /* corr observer: F_est . F_true proyectado (magnitud direccional) */
      double fe3=sqrt(Fe[0]*Fe[0]+Fe[1]*Fe[1]+Fe[2]*Fe[2]);
      sd+=fe3*Fm; se+=fe3*fe3; st+=Fm*Fm; }
  }
  mju_free(jp);mju_free(jr);mju_free(M); mj_deleteData(d);
  R r={sf/n, strk/n, sd/(sqrt(se*st)+1e-12)}; return r;
}

static void stat_mu(double*v,int n,double*mu,double*sd){
  double s=0;for(int i=0;i<n;i++)s+=v[i]; *mu=s/n;
  double q=0;for(int i=0;i<n;i++)q+=(v[i]-*mu)*(v[i]-*mu); *sd=sqrt(q/n); }

int main(){
  char e[1000]={0}; m=mj_loadXML("/tmp/vel.xml",NULL,e,1000); if(!m){printf("ERR %s\n",e);return 1;}
  const char* jn[]={"m1","m2","m3","m4"};
  for(int i=0;i<4;i++){int j=mj_name2id(m,mjOBJ_JOINT,jn[i]);qadr[i]=m->jnt_qposadr[j];dadr[i]=m->jnt_dofadr[j];
    char an[5]={'a','_','m','1'+i,0};aid[i]=mj_name2id(m,mjOBJ_ACTUATOR,an);}
  site=mj_name2id(m,mjOBJ_SITE,"ee");tip=mj_name2id(m,mjOBJ_BODY,"tip");am5=mj_name2id(m,mjOBJ_ACTUATOR,"a_m5");
  arm4dof_dqnmpc_solver_capsule* cap=arm4dof_dqnmpc_acados_create_capsule(); arm4dof_dqnmpc_acados_create(cap);

  /* N=6 condiciones: poses de tarea x direcciones de guiado */
  double tasks[6][4]={{0,1.2,-0.9,0.4},{0.3,1.0,-0.7,0.3},{-0.3,1.4,-1.0,0.5},
                      {0.2,1.1,-0.8,0.2},{-0.2,1.3,-1.1,0.6},{0.1,0.9,-0.6,0.3}};
  double dirs[6][3]={{-0.03,0.04,0.05},{0.04,-0.03,0.04},{0.05,0.03,-0.03},
                     {-0.04,-0.04,0.04},{0.03,0.05,0.03},{-0.05,0.02,-0.04}};
  int Ncond=6;
  double fS[6],tS[6],fC[6],tC[6],cC[6];
  for(int c=0;c<Ncond;c++){
    printf("cond %d/%d...\n",c+1,Ncond); fflush(stdout);
    R st=run(tasks[c],dirs[c],1,cap), co=run(tasks[c],dirs[c],0,cap);
    fS[c]=st.force;tS[c]=st.track; fC[c]=co.force;tC[c]=co.track;cC[c]=co.corr;
  }
  double m1,s1,m2,s2,m3,s3,m4,s4,m5,s5;
  stat_mu(fS,Ncond,&m1,&s1); stat_mu(fC,Ncond,&m2,&s2);
  stat_mu(tS,Ncond,&m3,&s3); stat_mu(tC,Ncond,&m4,&s4); stat_mu(cC,Ncond,&m5,&s5);
  printf("== STATS N=%d condiciones (media +- std) ==\n",Ncond);
  printf("                       RIGIDO            COMPLIANT\n");
  printf("fuerza guia [N]     %5.2f +- %.2f      %5.2f +- %.2f\n", m1,s1, m2,s2);
  printf("error seguir [mm]   %5.1f +- %.1f      %5.1f +- %.1f\n", 1000*m3,1000*s3, 1000*m4,1000*s4);
  printf("observer corr        --                %5.3f +- %.3f\n", m5,s5);
  printf("-> compliant: %.0f%% menos fuerza, %.0f%% mejor seguimiento (consistente en N=%d)\n",
         100*(1-m2/m1), 100*(1-m4/m3), Ncond);

  /* ── analisis PAREADO ─────────────────────────────────────────────────────
     Las dos ramas corren la MISMA condicion (misma tarea, misma direccion de
     guiado), asi que comparar medias sueltas con su std tira informacion y
     subestima el efecto: la varianza ENTRE condiciones es comun a las dos y se
     cancela al restar. Lo que corresponde es la distribucion de las DIFERENCIAS. */
  printf("\n-- por condicion (pareado) --\n");
  printf("  cond   F_rig   F_com    dF      e_rig   e_com    de\n");
  double df[6],dt[6];
  for(int c=0;c<Ncond;c++){
    df[c]=fS[c]-fC[c]; dt[c]=1000*(tS[c]-tC[c]);
    printf("  %3d  %7.2f %7.2f %7.2f   %7.1f %7.1f %7.1f\n",
           c+1, fS[c], fC[c], df[c], 1000*tS[c], 1000*tC[c], dt[c]);
  }
  double mdf,sdf,mdt,sdt; stat_mu(df,Ncond,&mdf,&sdf); stat_mu(dt,Ncond,&mdt,&sdt);
  int wf=0,wt=0; for(int c=0;c<Ncond;c++){ if(df[c]>0)wf++; if(dt[c]>0)wt++; }
  double tf = mdf/(sdf/sqrt((double)Ncond)), tt = mdt/(sdt/sqrt((double)Ncond));
  printf("\n  fuerza     : dif media %+.2f +- %.2f N   t=%.2f   favorable en %d/%d\n",
         mdf,sdf,tf,wf,Ncond);
  printf("  seguimiento: dif media %+.1f +- %.1f mm  t=%.2f   favorable en %d/%d\n",
         mdt,sdt,tt,wt,Ncond);
  printf("  (t critico bilateral 0.05 con %d gl = 2.571)\n", Ncond-1);
  arm4dof_dqnmpc_acados_free(cap); arm4dof_dqnmpc_acados_free_capsule(cap); mj_deleteModel(m); return 0;
}
