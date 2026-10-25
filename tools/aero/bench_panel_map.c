/* bench_panel_map.c - cost of runtime per-panel aero-map evaluation (method (a)/(d) in docs/AERO_PLAN.md).
 * Synthetic data only (no car geometry).  Build: cc -O2 -march=native bench_panel_map.c -o /tmp/bpm -lm
 * Variant A: per-panel multilinear lookup in a 4-D map (heave, pitch, roll, yaw), 3 force comps per panel per node,
 *            then per-panel relative velocity v_rel = v_cg + w x r - wind(r) (wind sampled per panel), q per panel,
 *            and accumulation of the wrench (F, r x F) about the CG.
 * Variant B: reduced-order (POD) map: F_panel = sum_k a_k(state) * Phi_k[panel], a_k by multilinear lookup on a
 *            small table; same per-panel velocity/q and wrench accumulation.
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#define NH 7
#define NP 7
#define NR 5
#define NY 9
static double now(void){ struct timespec t; clock_gettime(CLOCK_MONOTONIC,&t); return t.tv_sec+1e-9*t.tv_nsec; }
static float frand(void){ return (float)rand()/RAND_MAX-0.5f; }
typedef struct { float r[3], n[3], A; } Panel;
static void wind_at(const float *r, double t, float *w){ /* smooth synthetic field: mean + 2 travelling modes */
  w[0]=3.0f+0.8f*sinf(0.7f*r[0]-1.3f*(float)t)+0.3f*sinf(2.1f*r[1]+0.4f*(float)t);
  w[1]=2.0f+0.5f*sinf(0.9f*r[0]+0.8f*(float)t); w[2]=0.1f*sinf(1.7f*r[0]-2.0f*(float)t);
}
int main(int argc,char**argv){
  int N = argc>1? atoi(argv[1]) : 1024, K = argc>2? atoi(argv[2]) : 16, iters=20000;
  Panel *P = malloc(sizeof(Panel)*N);
  for(int i=0;i<N;i++){ for(int j=0;j<3;j++){P[i].r[j]=2*frand(); P[i].n[j]=frand();} P[i].A=0.01f; }
  size_t nodes = (size_t)NH*NP*NR*NY;
  float *map = malloc(sizeof(float)*nodes*N*3);           /* [node][panel][3] */
  for(size_t i=0;i<nodes*N*3;i++) map[i]=frand();
  float *phi = malloc(sizeof(float)*K*N*3), *amap = malloc(sizeof(float)*nodes*K);
  for(long i=0;i<(long)K*N*3;i++) phi[i]=frand(); for(size_t i=0;i<nodes*K;i++) amap[i]=frand();
  double sink=0;
  for(int variant=0; variant<2; variant++){
    double t0=now();
    for(int it=0; it<iters; it++){
      double t=it/240.0; float st[4]={0.3f+0.1f*sinf(t),0.4f,0.5f,0.45f+0.2f*sinf(0.3f*t)}; /* normalised state */
      int idx[4]; float fr[4]; int dims[4]={NH,NP,NR,NY};
      for(int d=0;d<4;d++){ float x=st[d]*(dims[d]-1); int i0=(int)x; if(i0>dims[d]-2)i0=dims[d]-2; idx[d]=i0; fr[d]=x-i0; }
      size_t corner[16]; float wgt[16];
      for(int c=0;c<16;c++){ size_t o=0; float w=1; int s[4];
        for(int d=0;d<4;d++){ s[d]=(c>>d)&1; w*= s[d]? fr[d]:1-fr[d]; }
        o=(((size_t)(idx[0]+s[0])*NP+(idx[1]+s[1]))*NR+(idx[2]+s[2]))*NY+(idx[3]+s[3]); corner[c]=o; wgt[c]=w; }
      float a[64]; if(variant==1) for(int k=0;k<K;k++){ a[k]=0; for(int c=0;c<16;c++) a[k]+=wgt[c]*amap[corner[c]*K+k]; }
      float vcg[3]={30,0.5f,0}, om[3]={0.01f,0.02f,0.1f}; double F[3]={0,0,0}, M[3]={0,0,0};
      for(int i=0;i<N;i++){
        const float *r=P[i].r; float w[3]; wind_at(r,t,w);
        float vr[3]={vcg[0]+om[1]*r[2]-om[2]*r[1]-w[0], vcg[1]+om[2]*r[0]-om[0]*r[2]-w[1], vcg[2]+om[0]*r[1]-om[1]*r[0]-w[2]};
        float q=0.6f*(vr[0]*vr[0]+vr[1]*vr[1]+vr[2]*vr[2]);
        float c3[3]={0,0,0};
        if(variant==0){ for(int c=0;c<16;c++){ const float *m=map+(corner[c]*N+i)*3; c3[0]+=wgt[c]*m[0]; c3[1]+=wgt[c]*m[1]; c3[2]+=wgt[c]*m[2]; } }
        else { for(int k=0;k<K;k++){ const float *m=phi+((size_t)k*N+i)*3; c3[0]+=a[k]*m[0]; c3[1]+=a[k]*m[1]; c3[2]+=a[k]*m[2]; } }
        float f[3]={q*c3[0],q*c3[1],q*c3[2]};
        F[0]+=f[0];F[1]+=f[1];F[2]+=f[2]; M[0]+=r[1]*f[2]-r[2]*f[1]; M[1]+=r[2]*f[0]-r[0]*f[2]; M[2]+=r[0]*f[1]-r[1]*f[0];
      }
      sink+=F[0]+F[1]+F[2]+M[0]+M[1]+M[2];
    }
    double us=(now()-t0)/iters*1e6;
    printf("%s N=%d K=%d%s: %.2f us per aero evaluation  -> at 240 Hz: %.3f us per sim-ms (%.2f%% of a 100 us/sim-ms budget)\n",
      variant? "POD   ":"direct", N, K, variant? " POD-modes":" (4-D map 7x7x5x9)", us, us*0.240, us*0.240);
  }
  printf("map memory: direct %.1f MB, POD %.2f MB  (sink %g)\n", nodes*N*3*4/1e6, (K*N*3+nodes*K)*4/1e6, sink);
  return 0;
}
