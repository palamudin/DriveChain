/* Cost benchmark for AERO_PLAN 12 (vehicle-vehicle interaction).
 * Per 240 Hz tick, for NP car panels and NV nearby vehicles:
 *  (A) grid map: transform panel to the other vehicle's frame, trilinear lookup of a
 *      precomputed (u,v,w,p) field (float4) on a 96x48x32 grid (~2.4 MB per vehicle type);
 *  (B) potential-flow line of NS point sources (ground-imaged by construction).
 * Synthetic data; timing only.  Build: cc -O2 -march=native bench_field_superpose.c -o /tmp/bfs -lm */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#define NX 96
#define NY 48
#define NZ 32
static double now(void){struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);return t.tv_sec+1e-9*t.tv_nsec;}
typedef struct{float x0,y0,z0,ih;float *g;} Grid;
static void grid_sample(const Grid*G,float x,float y,float z,float o[4]){
  float fx=(x-G->x0)*G->ih,fy=(y-G->y0)*G->ih,fz=(z-G->z0)*G->ih;
  if(fx<0||fy<0||fz<0||fx>=NX-1||fy>=NY-1||fz>=NZ-1){o[0]=o[1]=o[2]=o[3]=0;return;}
  int i=(int)fx,j=(int)fy,k=(int)fz;float a=fx-i,b=fy-j,c=fz-k;
  for(int q=0;q<4;q++)o[q]=0;
  for(int dk=0;dk<2;dk++)for(int dj=0;dj<2;dj++)for(int di=0;di<2;di++){
    float w=(di?a:1-a)*(dj?b:1-b)*(dk?c:1-c);
    const float*p=G->g+4*(((size_t)(k+dk)*NY+(j+dj))*NX+(i+di));
    for(int q=0;q<4;q++)o[q]+=w*p[q];}
}
int main(int argc,char**argv){
  int NP=argc>1?atoi(argv[1]):1024,NV=argc>2?atoi(argv[2]):4,NS=argc>3?atoi(argv[3]):32,R=2000;
  float*P=malloc(sizeof(float)*3*NP),*out=malloc(sizeof(float)*4*NP);
  srand(1);for(int i=0;i<3*NP;i++)P[i]=(float)rand()/RAND_MAX*4-2;
  Grid G={-30,-12,0,0.5f/1.0f,malloc(sizeof(float)*4*NX*NY*NZ)};G.ih=1/0.5f;
  for(size_t i=0;i<4ul*NX*NY*NZ;i++)G.g[i]=(float)rand()/RAND_MAX;
  float sx[256],sq[256];for(int s=0;s<NS;s++){sx[s]=8-0.9f*s;sq[s]=(s<4?10.f:-1.f);}
  double t=now(),sink=0;
  for(int r=0;r<R;r++)for(int v=0;v<NV;v++){float c=cosf(0.01f*v),s=sinf(0.01f*v),tx=5+3*v,ty=-3;
    for(int i=0;i<NP;i++){float x=c*P[3*i]-s*P[3*i+1]+tx,y=s*P[3*i]+c*P[3*i+1]+ty,o[4];
      grid_sample(&G,x,y,P[3*i+2]+1,o);out[4*i]+=o[0];out[4*i+1]+=o[1];out[4*i+2]+=o[2];out[4*i+3]+=o[3];}}
  double tA=(now()-t)/R;
  t=now();
  for(int r=0;r<R/4;r++)for(int v=0;v<NV;v++){float tx=5+3*v,ty=-3;
    for(int i=0;i<NP;i++){float x=P[3*i]+tx,y=P[3*i+1]+ty,z=P[3*i+2]+1,u=0,w=0,ww=0;
      for(int s=0;s<NS;s++){float dx=x-sx[s],r2=dx*dx+y*y+z*z+1e-6f,f=sq[s]/(r2*sqrtf(r2));u+=f*dx;w+=f*y;ww+=f*z;}
      out[4*i]+=u;out[4*i+1]+=w;out[4*i+2]+=ww;}}
  double tB=(now()-t)/(R/4);
  for(int i=0;i<4*NP;i++)sink+=out[i];
  printf("NP=%d NV=%d: grid map %.1f us/tick (%.2f us per sim-ms at 240 Hz); sources(NS=%d) %.1f us/tick (%.2f us/sim-ms)  [%g]\n",
    NP,NV,tA*1e6,tA*1e6*0.24,NS,tB*1e6,tB*1e6*0.24,sink*0);
  return 0;}
