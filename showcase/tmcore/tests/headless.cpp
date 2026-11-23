// headless.cpp - drive the stock 1990 Miata on the Ušće collider with the Tarmac core via tms.h (no renderer).
#include "tms.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <vector>
#include <chrono>
static bool readDc(const char* path,std::vector<float>& P,std::vector<uint32_t>& I,std::vector<uint8_t>& S){
  FILE* f=fopen(path,"rb"); if(!f) return false; char mg[8]; int h[3]; if(fread(mg,1,8,f)!=8||fread(h,4,3,f)!=3){fclose(f);return false;}
  std::vector<double> v(3*(size_t)h[0]); std::vector<int> t(3*(size_t)h[1]); S.resize(h[1]);
  fread(v.data(),8,v.size(),f); fread(t.data(),4,t.size(),f); fread(S.data(),1,S.size(),f); fclose(f);
  P.resize(v.size()); for(int i=0;i<h[0];i++){ P[3*i]=(float)v[3*i]; P[3*i+1]=(float)v[3*i+2]; P[3*i+2]=(float)-v[3*i+1]; } // DC(x E,y N,z up)->TM(X,Y up,Z)=(x,z,-y)
  I.assign(t.begin(),t.end()); return true;
}
int main(int argc,char**argv){
  const char* col=argc>1?argv[1]:"/workspace/drivechain/private/tracks/usce/collider.bin";
  int seconds=argc>2?atoi(argv[2]):30; int tune=argc>3?atoi(argv[3]):0;
  std::vector<float> P; std::vector<uint32_t> I; std::vector<uint8_t> S;
  if(!readDc(col,P,I,S)){ printf("read fail\n"); return 1; }
  auto t0=std::chrono::steady_clock::now();
  void* w=tms_world_create(P.data(),P.size()/3,I.data(),I.size()/3,S.data());
  printf("world %p (%zu tris) %.0f ms err=%s\n",w,I.size()/3,std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-t0).count(),tms_last_error());
  if(!w) return 1;
  // AI line for a simple pure-pursuit driver
  std::vector<double> lx,lz; { FILE* f=fopen("/workspace/drivechain/private/tracks/usce/line.csv","r"); char ln[256]; while(f&&fgets(ln,256,f)){ double a,b,c; if(sscanf(ln,"%lf,%lf,%lf",&a,&b,&c)==3){ lx.push_back(a); lz.push_back(-b);} } if(f)fclose(f); }
  double sx=lx[0],sz=lz[0],dx=lx[3]-lx[0],dz=lz[3]-lz[0]; double yaw=atan2(dx,dz);
  double y=tms_world_height(w,sx,200,sz); printf("spawn %.2f %.2f h=%.2f yaw=%.3f\n",sx,sz,y,yaw);
  void* c=tms_car_create(w,TMS_CAR_MIATA_NA,tune,sx,sz,yaw,y+1); printf("car %p err=%s\n",c,tms_last_error()); if(!c) return 1;
  float s[TMS_STATE_FLOATS]; size_t near=0; int gear=1; double tg=0;
  auto t1=std::chrono::steady_clock::now(); int ticks=0;
  for(int frame=0; frame<seconds*60; frame++){
    tms_car_state(c,s); double px=s[0],pz=s[2];
    // nearest line point then look ahead ~ 12 m + speed
    double best=1e18; for(size_t k=0;k<lx.size();k++){ size_t j=(near+k)%lx.size(); double d=(lx[j]-px)*(lx[j]-px)+(lz[j]-pz)*(lz[j]-pz); if(d<best){best=d; near=j;} if(k>60) break; }
    double look=8+s[10]*0.6; size_t j=near; double acc=0; while(acc<look){ size_t n=(j+1)%lx.size(); acc+=hypot(lx[n]-lx[j],lz[n]-lz[j]); j=n; }
    double qx=s[3],qy=s[4],qz=s[5],qw=s[6]; double fx=2*(qx*qz+qw*qy), fz=1-2*(qx*qx+qy*qy); double hx=1-2*(qy*qy+qz*qz), hz=2*(qx*qz-qw*qy); // body +Z, +X(left)
    double tx=lx[j]-px,tz=lz[j]-pz; double ahead=tx*fx+tz*fz, left=tx*hx+tz*hz; double ang=atan2(left,ahead);
    double steer=fmax(-1,fmin(1,ang*2.0)); // +left? sign checked below via log
    double target=argc>4?atof(argv[4]):22; double th=s[10]<target?0.7:0.0, br=s[10]>target+3?0.4:0.0;
    // gear: simple auto shift by rpm
    if(s[11]>5600&&gear<5&&frame/60.0-tg>1){gear++;tg=frame/60.0;} if(s[11]<2200&&gear>1&&frame/60.0-tg>1){gear--;tg=frame/60.0;}
    int r=tms_car_step6(c,th,br,-steer,0,0,gear,4); if(r<0){ printf("step err %d %s\n",r,tms_last_error()); break; } ticks+=r;
    if(frame%60==0) printf("t=%5.1f pos=(%7.1f %6.2f %7.1f) v=%5.1f rpm=%5.0f g=%d gear_cmd=%d near=%zu ang=%+.2f contact=%g%g%g%g load=%.0f\n",s[14],s[0],s[1],s[2],s[10],s[11],(int)s[12],gear,near,ang,s[16+14],s[32+14],s[48+14],s[64+14],s[16+10]);
  }
  double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-t1).count();
  printf("%d ticks in %.0f ms = %.1f us/tick (realtime needs < 4167)\n",ticks,ms,1000*ms/ticks);
  return 0;
}
