/* Timing del DQ-NMPC: tiempo por solve (seccion implementacion tiempo real). */
#include <stdio.h>
#include <time.h>
#include <math.h>
#include "acados_solver_arm4dof_dqnmpc.h"
#include "acados_c/ocp_nlp_interface.h"

int main(){
  arm4dof_dqnmpc_solver_capsule* cap=arm4dof_dqnmpc_acados_create_capsule();
  arm4dof_dqnmpc_acados_create(cap);
  ocp_nlp_config* cfg=arm4dof_dqnmpc_acados_get_nlp_config(cap);
  ocp_nlp_dims* dim=arm4dof_dqnmpc_acados_get_nlp_dims(cap);
  ocp_nlp_in* in=arm4dof_dqnmpc_acados_get_nlp_in(cap);
  ocp_nlp_out* out=arm4dof_dqnmpc_acados_get_nlp_out(cap);
  int N=ARM4DOF_DQNMPC_N;
  double prm[22]={0}; prm[0]=1.0; double W[6]={2,2,2,80,80,80}; for(int i=0;i<6;i++)prm[8+i]=W[i];
  for(int i=0;i<4;i++){prm[14+i]=0.3;prm[18+i]=0.02;}
  for(int s=0;s<=N;s++) arm4dof_dqnmpc_acados_update_params(cap,s,prm,22);
  double x0[8]={0,1.2,-0.9,0.4,0,0,0,0};

  int M=2000; double tmin=1e9,tmax=0,tsum=0; struct timespec a,b;
  for(int k=0;k<M;k++){
    ocp_nlp_constraints_model_set(cfg,dim,in,out,0,"lbx",x0);
    ocp_nlp_constraints_model_set(cfg,dim,in,out,0,"ubx",x0);
    clock_gettime(CLOCK_MONOTONIC,&a);
    arm4dof_dqnmpc_acados_solve(cap);
    clock_gettime(CLOCK_MONOTONIC,&b);
    double ms=(b.tv_sec-a.tv_sec)*1e3+(b.tv_nsec-a.tv_nsec)*1e-6;
    if(k>50){ tsum+=ms; if(ms<tmin)tmin=ms; if(ms>tmax)tmax=ms; }
  }
  int n=M-51; double mean=tsum/n;
  printf("== Timing DQ-NMPC (N=%d, %d solves) ==\n",N,n);
  printf("solve: media=%.3f ms  min=%.3f  max=%.3f ms\n",mean,tmin,tmax);
  printf("frecuencia sostenible ~ %.0f Hz (media) / %.0f Hz (worst-case)\n",1000/mean,1000/tmax);
  printf("100 Hz (10 ms): %s\n", tmax<10?"OK con margen (worst-case < 10ms)":(mean<10?"OK en media":"AJUSTAR"));
  arm4dof_dqnmpc_acados_free(cap); arm4dof_dqnmpc_acados_free_capsule(cap); return 0;
}
