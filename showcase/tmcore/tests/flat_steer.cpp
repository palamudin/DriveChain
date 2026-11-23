#include "../api/tms.h"
#include <cstdio>
#include <cmath>
#include <vector>
int main(int argc,char**argv){
  int N=40; double S=25; std::vector<float> p; std::vector<uint32_t> ix; std::vector<uint8_t> sf;
  for(int j=0;j<=N;j++)for(int i=0;i<=N;i++){p.push_back((i-N/2)*S);p.push_back(i==0&&j==0?-1:0);p.push_back((j-N/2)*S);}
  for(int j=0;j<N;j++)for(int i=0;i<N;i++){uint32_t a=j*(N+1)+i,b=a+1,c=a+N+1,d=c+1;ix.insert(ix.end(),{a,c,b,b,c,d});sf.push_back(0);sf.push_back(0);}
  void* w=tms_world_create(p.data(),p.size()/3,ix.data(),ix.size()/3,sf.data()); if(!w){printf("%s\n",tms_last_error());return 1;}
  void* c=tms_car_create(w,1,0,0,0,0,1); if(!c){printf("%s\n",tms_last_error());return 1;}
  float s[80]; double SA=argc>1?atof(argv[1]):0.3; if(argc>2) tms_car_set_assists(c,atoi(argv[2])&1,atoi(argv[2])&2); int CAR=1;
  for(int t=0;t<240*12;t++){ double sec=t/240.0; double st=sec<6?0:SA; int r=tms_car_step7(c,sec<6?1:0.35,0,st,0,0,1,0,1); if(r<0){printf("err %s\n",tms_last_error());return 1;}
    if(t%240==0){tms_car_state(c,s); double h=atan2((s[18]+s[34])/2-(s[50]+s[66])/2,(s[16]+s[32])/2-(s[48]+s[64])/2); printf("%.1f st %.2f spd %.1f head %.3f rpm %.0f g %.0f\n",sec,st,s[10],h,s[11],s[12]);} }
}
