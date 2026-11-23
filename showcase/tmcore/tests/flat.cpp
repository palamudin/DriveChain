#include "tarmac_vehicle.h"
#include <cstdio>
#include <cstdlib>
extern "C" const char* tm_vehicle_get_research197_failure_v244(const void*);
extern "C" int32_t tm_vehicle_configure_car_powertrain_v104(void*,uint32_t,uint32_t,const TMCitySpawnV2*);
int main(int argc,char**argv){
  if(argc>1&&argv[1][0]=='1'){ setenv("TARMAC_PROVING","1",1);setenv("TARMAC_PROVING_SUBSTEPS","8",1);setenv("TARMAC_PROVING_NORMAL22","1",1);setenv("TARMAC_PROVING_RACK36","1",1);setenv("TARMAC_PROVING_TWIST27","1",1);setenv("TARMAC_PROVING_ROLLING49","1",1);setenv("TARMAC_PROVING_GEOMETRY56","1",1);}
  void* v=tm_vehicle_create(); printf("create %p %s\n",v,tm_vehicle_last_error());
  if(argc>2){int r=tm_vehicle_configure_car_powertrain_v104(v,atoi(argv[2]),1,nullptr); printf("cfg %d %s\n",r,tm_vehicle_last_error());}
  TMInput in{0,0,0,0,0,0,0}; TMState s{};
  for(int i=0;i<480;i++){ int r=tm_vehicle_step(v,&in); if(r<0){printf("tick %d err %s | %s\n",i,tm_vehicle_last_error(),tm_vehicle_get_research197_failure_v244(v));return 1;} }
  tm_vehicle_get_state(v,&s); printf("ok pos %.3f %.3f %.3f\n",s.position[0],s.position[1],s.position[2]);
}
