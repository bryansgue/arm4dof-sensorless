/* ================================================================
   Valida CONTACTO + estimacion de fuerza SIN SENSOR.
   Presiona el lapiz contra la pizarra (velocidad +x), mide:
     - fuerza de contacto REAL de MuJoCo (mj_contactForce) = ground truth
     - estimada por torque de juntas: F_est = (J^T)^+ (qfrc_actuator - qfrc_bias)
   Compara. Esta es la base del control de fuerza del paso 5.
   ================================================================ */
#include <mujoco/mujoco.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

int main(){
  char e[1000]={0};
  mjModel* m=mj_loadXML("/tmp/pen.xml",NULL,e,1000);
  if(!m){printf("ERR %s\n",e);return 1;}
  mjData* d=mj_makeData(m); int nv=m->nv;
  const char* jn[]={"m1","m2","m3","m4"}; int qadr[4],dadr[4],aid[4];
  for(int i=0;i<4;i++){int j=mj_name2id(m,mjOBJ_JOINT,jn[i]); qadr[i]=m->jnt_qposadr[j]; dadr[i]=m->jnt_dofadr[j];
    char an[5]={'a','_','m','1'+i,0}; aid[i]=mj_name2id(m,mjOBJ_ACTUATOR,an);}
  int pen=mj_name2id(m,mjOBJ_SITE,"pen_tip");
  int gpen=mj_name2id(m,mjOBJ_GEOM,"pen"), gboard=mj_name2id(m,mjOBJ_GEOM,"board");

  /* arranque: brazo extendido hacia la pizarra */
  mj_resetData(m,d);
  double q0[4]={0.0, 0.9, -0.7, 0.3};
  for(int i=0;i<4;i++) d->qpos[qadr[i]]=q0[i];
  mj_forward(m,d);

  double* jp=mju_malloc(3*nv*sizeof(double));
  double* jr=mju_malloc(3*nv*sizeof(double));
  double* M=mju_malloc(nv*nv*sizeof(double));

  /* lazo: comandar velocidad +x del pen_tip (presionar) via q̇=Jp^+ [v,0,0] */
  double vpress=0.03, dt=m->opt.timestep;
  for(int t=0; t<1500; t++){
    mj_jacSite(m,d,jp,jr,pen);
    /* Jp 3x4 (cols m1..m4) */
    double Jp[12]; for(int r=0;r<3;r++)for(int c=0;c<4;c++) Jp[r*4+c]=jp[r*nv+dadr[c]];
    /* q̇ = Jp^T (Jp Jp^T + λI)^-1 [v,0,0] */
    double JJt[9]; for(int a=0;a<3;a++)for(int b=0;b<3;b++){double s=0;for(int k=0;k<4;k++)s+=Jp[a*4+k]*Jp[b*4+k]; JJt[a*3+b]=s+(a==b?1e-4:0);}
    double Jinv[9]; mju_copy(Jinv,JJt,9); int r3=mju_cholFactor(Jinv,3,1e-12);(void)r3;
    double vvec[3]={vpress,0,0}, tmp[3]; mju_cholSolve(tmp,Jinv,vvec,3);
    double qd[4]; for(int c=0;c<4;c++){double s=0;for(int a=0;a<3;a++)s+=Jp[a*4+c]*tmp[a]; qd[c]=s;}
    for(int i=0;i<4;i++) d->ctrl[aid[i]]=qd[i];
    d->ctrl[mj_name2id(m,mjOBJ_ACTUATOR,"a_m5")]=0;
    mj_step(m,d);
  }

  /* ---- estado estacionario en contacto: medir ---- */
  mj_forward(m,d);
  printf("== Contacto lapiz-pizarra: fuerza real vs estimada sin sensor ==\n");
  printf("ncon=%d\n", d->ncon);

  /* fuerza de contacto REAL (suma sobre contactos pen-board), en world */
  double Fc_world[3]={0,0,0};
  for(int i=0;i<d->ncon;i++){
    int g1=d->contact[i].geom1,g2=d->contact[i].geom2;
    if((g1==gpen&&g2==gboard)||(g1==gboard&&g2==gpen)){
      double f6[6]; mj_contactForce(m,d,i,f6);
      /* f6 en frame de contacto: rotar a world con contact.frame (3x3 fila-major) */
      double* R=d->contact[i].frame; /* 9: ejes del contacto */
      /* fuerza = f6[0]*normal + f6[1]*t1 + f6[2]*t2 ; frame rows = [n; t1; t2] */
      for(int k=0;k<3;k++) Fc_world[k]+= f6[0]*R[0*3+k]+f6[1]*R[1*3+k]+f6[2]*R[2*3+k];
    }
  }
  double Fc_mag=sqrt(Fc_world[0]*Fc_world[0]+Fc_world[1]*Fc_world[1]+Fc_world[2]*Fc_world[2]);

  /* estimacion SIN SENSOR: tau_ext = qfrc_actuator - qfrc_bias = J^T F_ext */
  mj_fullM(m,M,d->qM);
  double tau_ext[4]; for(int i=0;i<4;i++) tau_ext[i]=d->qfrc_actuator[dadr[i]]-d->qfrc_bias[dadr[i]];
  /* F_est = (J^T)^+ tau_ext = (J J^T)^-1 J tau_ext  (J = Jp 3x4 en pen_tip) */
  mj_jacSite(m,d,jp,jr,pen);
  double Jp[12]; for(int r=0;r<3;r++)for(int c=0;c<4;c++) Jp[r*4+c]=jp[r*nv+dadr[c]];
  double Jt[3]; for(int a=0;a<3;a++){double s=0;for(int c=0;c<4;c++)s+=Jp[a*4+c]*tau_ext[c]; Jt[a]=s;}
  double JJt[9]; for(int a=0;a<3;a++)for(int b=0;b<3;b++){double s=0;for(int k=0;k<4;k++)s+=Jp[a*4+k]*Jp[b*4+k]; JJt[a*3+b]=s+(a==b?1e-6:0);}
  double L[9]; mju_copy(L,JJt,9); mju_cholFactor(L,3,1e-12);
  double Fest[3]; mju_cholSolve(Fest,L,Jt,3);
  double Fe_mag=sqrt(Fest[0]*Fest[0]+Fest[1]*Fest[1]+Fest[2]*Fest[2]);

  printf("F_contacto REAL (MuJoCo)  = [%.3f %.3f %.3f]  |F|=%.3f N\n",Fc_world[0],Fc_world[1],Fc_world[2],Fc_mag);
  printf("F_estimada (sin sensor)   = [%.3f %.3f %.3f]  |F|=%.3f N\n",Fest[0],Fest[1],Fest[2],Fe_mag);
  /* la fuerza que el brazo EJERCE sobre la pizarra = -reaccion; comparar magnitud + direccion x */
  double dot=(Fc_world[0]*Fest[0]+Fc_world[1]*Fest[1]+Fc_world[2]*Fest[2]);
  double cosang=dot/(Fc_mag*Fe_mag+1e-9);
  printf("normal-x: real=%.3f  est=%.3f N   | corr direccion (cos)=%.3f\n", Fc_world[0], Fest[0], cosang);
  printf("%s\n", (fabs(Fc_mag-Fe_mag)<0.15*Fc_mag+0.05)?"ESTIMACION FUERZA SIN SENSOR OK":"REVISAR");
  mju_free(jp);mju_free(jr);mju_free(M); mj_deleteData(d); mj_deleteModel(m); return 0;
}
