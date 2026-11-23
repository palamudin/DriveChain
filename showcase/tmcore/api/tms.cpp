// tms.cpp - thin C wrapper over the Tarmac native core (abi.cpp exports). No physics here:
// all dynamics, tyre contact, powertrain, assists run inside Jakov's NativeCurrent287 sources.
#include "tms.h"
#include "tarmac_vehicle.h"
#include "PedalActuation312/pedal_actuation_abi312.h"
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

extern "C" {
struct TMSAssistCfg { uint32_t size, abi, flags, reserved; };
int32_t tm_vehicle_set_assists_v104(void* handle, const TMSAssistCfg* c);
int32_t tm_vehicle_set_raw_steering_owner_v62(void*,uint32_t);
int32_t tm_vehicle_configure_pedal_actuation_v312(void*,const TMPedalActuationConfigV312*);
int32_t tm_vehicle_configure_car_powertrain_v104(void* handle, uint32_t id, uint32_t unit, const TMCitySpawnV2* spawn);
}

namespace {
thread_local std::string g_err;
const char* setErr(const std::string& s){ g_err=s; return g_err.c_str(); }
struct World { void* surface=nullptr; };
struct Car { void* v=nullptr; World* w=nullptr; int car=1, tune=0; uint32_t owner=1; };

// DriveChain SURF_* -> Tarmac city semantic role / friction / rolling (mt_default_grip values).
uint16_t roleOf(int s){
  switch(s){case TMS_SURF_TARMAC: case TMS_SURF_LINE: case TMS_SURF_PAD: case TMS_SURF_CONCRETE: return TM_CITY_ROLE_ROAD_V2;
  case TMS_SURF_KERB: case TMS_SURF_SAUSAGE: return TM_CITY_ROLE_ROAD_V2; /* racing kerbs are drivable; city KERB_V2 is an obstacle role */ case TMS_SURF_GRASS: return TM_CITY_ROLE_GRASS_V2;
  case TMS_SURF_ROUGH: return TM_CITY_ROLE_PATH_V2; case TMS_SURF_GRAVEL: return TM_CITY_ROLE_TREE_SOIL_V2;
  case TMS_SURF_PAVE: return TM_CITY_ROLE_SIDEWALK_V2; case TMS_SURF_WALL: return TM_CITY_ROLE_FACADE_V2; default: return TM_CITY_ROLE_HARDSCAPE_V2;}
}
float gripOf(int s){ switch(s){case TMS_SURF_KERB: return .92f; case TMS_SURF_SAUSAGE: case TMS_SURF_PAVE: return .85f; case TMS_SURF_ROUGH: return .75f;
  case TMS_SURF_GRAVEL: return .70f; case TMS_SURF_GRASS: return .62f; default: return 1.f;} }
float rollOf(int s){ switch(s){case TMS_SURF_GRASS: case TMS_SURF_GRAVEL: return 3.f; case TMS_SURF_ROUGH: return 2.f; case TMS_SURF_SAUSAGE: return 1.5f; default: return 1.f;} }

// The shipped Unity standalone (ConnectedReview11.ConfigureStandalone) selects the authoritative
// contact/rack path through process environment before the first vehicle exists. Mirror it exactly.
void shippedMechanicsEnv(){
  static bool done=false; if(done) return; done=true;
  const char* kv[][2]={{"TARMAC_PROVING","1"},{"TARMAC_PROVING_SUBSTEPS","8"},{"TARMAC_PROVING_NORMAL22","1"},{"TARMAC_PROVING_RACK36","1"},
                       {"TARMAC_PROVING_TWIST27","1"},{"TARMAC_PROVING_ROLLING49","1"},{"TARMAC_PROVING_GEOMETRY56","1"}};
  for(auto& e:kv){
#if defined(_WIN32)
    _putenv_s(e[0],e[1]);
#else
    setenv(e[0],e[1],1);
#endif
  }
}
TMCitySpawnV2 spawnOf(double x,double z,double yaw,double yHint){
  TMCitySpawnV2 s{}; s.struct_size=sizeof s; s.flags=TM_CITY_SPAWN_CAST_DOWN_V2; // exactly one mode flag
  s.x=x; s.z=z; s.yaw=yaw; s.support_y_hint=yHint; s.max_support_distance=200; s.support_mask=TM_CITY_COLLISION_PRIMARY_V2|TM_CITY_COLLISION_FALLBACK_V2;
  s.preferred_surface_id=TM_CITY_SURFACE_ID_ANY; return s;
}
void labelDigest(const char* l,uint8_t out[32]){ for(int lane=0;lane<4;lane++){ uint64_t h=1469598103934665603ull^(uint64_t)(lane+1)*0x9E3779B97F4A7C15ull; for(const char* c=l;*c;c++){h^=(uint8_t)*c;h*=1099511628211ull;} for(int k=0;k<8;k++)out[lane*8+k]=(uint8_t)(h>>(8*k)); } }
// OrdinaryPhysicalStandards313.CreatePedals313(vehicleId) ported 1:1 (C# -> TMPedalActuationConfigV312 wire).
// The native step refuses to advance without an active brake actuation (stepCore247), so this is mandatory.
TMPedalActuationConfigV312 pedals313(int id){
  const bool formula=id>=2, bmw=id==1; TMPedalActuationConfigV312 w{}; w.struct_size=sizeof w; w.abi_version=TM_PEDAL_ACTUATION_ABI_V312;
  const double pressure=formula?9.0e6:6.8e6, totalTorque=formula?7200:bmw?4300:2500, frontFraction=formula?.58:bmw?.61:.64;
  const double masterBore=formula?.01905:bmw?.02064:.01905, travel=formula?.105:.120, freeT=formula?.004:.006, pedalRatio=formula?5.1:bmw?4.75:4.6;
  const double masterArea=3.141592653589793*masterBore*masterBore*.25, compliance=masterArea*((travel-freeT)/pedalRatio)/pressure;
  const double fr[4]={formula?.150:bmw?.125:.110,formula?.150:bmw?.125:.110,formula?.135:bmw?.112:.095,formula?.135:bmw?.112:.095};
  auto& b=w.brake; b.activation=TM_PEDAL_ACTUATION_AUTHORED_V312; b.topology=formula?1:0; b.assist=formula?0:1; b.provenance=3; /* AuthoredDesign */
  labelDigest(formula?"formula-vector|inferred-brake":bmw?"bmw-e21-323i|inferred-brake|4300|0.61":"fiat-uno|inferred-brake",b.provenance_sha256);
  b.maximum_pedal_travel_m=travel; b.free_pedal_travel_m=freeT; b.pedal_ratio=pedalRatio; b.booster_force_gain=formula?1:4; b.master_bore_m=masterBore;
  b.circuit_compliance_m3_per_pa[0]=b.circuit_compliance_m3_per_pa[1]=compliance; b.maximum_pressure_pa=formula?18e6:12e6;
  b.compensation_port_pedal_fraction=.005; b.pedal_return_stiffness_n_per_m=formula?320:250; b.pedal_return_preload_n=formula?8:5;
  for(int k=0;k<4;k++){ double corner=totalTorque*(k<2?frontFraction:1-frontFraction)*.5; b.effective_radius_m[k]=fr[k]; b.pad_friction[k]=.38;
    b.effective_clamp_area_m2[k]=corner/(pressure*.38*fr[k]); }
  const uint32_t circ[4]={0u,formula?0u:1u,1u,formula?1u:0u}; for(int k=0;k<4;k++) b.circuit_for_corner[k]=circ[k];
  auto& c=w.clutch; c.activation=TM_PEDAL_ACTUATION_AUTHORED_V312; c.clamp_curve=1; c.provenance=3;
  if(formula||bmw){ const double ct=formula?.100:.130, cf=formula?.004:.010, cr=formula?4.8:4, cm=formula?.015:.016, cs=formula?.024:.028;
    const double clear=(ct-cf)/cr*(cm*cm)/(cs*cs); c.actuation=1;
    labelDigest(formula?"formula-vector|inferred-hydraulic-multiplate-clutch/v1":"bmw-e21-323i|inferred-hydraulic-clutch/v1",c.provenance_sha256);
    c.maximum_pedal_travel_m=ct;c.free_pedal_travel_m=cf;c.pedal_ratio=cr;c.master_bore_m=cm;c.slave_bore_m=cs;c.cable_release_travel_ratio=.30;
    c.bite_release_travel_m=(formula?.12:.18)*clear;c.clear_release_travel_m=clear;c.opening_speed_m_per_s=(formula?18:12)*clear;c.closing_speed_m_per_s=(formula?5:2.8)*clear;
    c.pedal_return_stiffness_n_per_m=formula?160:180;c.pedal_return_preload_n=formula?3:4;
    const double fb[5]={80,1000,1600,1450,1100}, ff[5]={100,1250,1900,1700,1200}; for(int k=0;k<5;k++)c.release_bearing_force_n[k]=formula?ff[k]:fb[k];
  } else { c.actuation=0; labelDigest("fiat-uno|inferred-cable-clutch/v1",c.provenance_sha256);
    c.maximum_pedal_travel_m=.125;c.free_pedal_travel_m=.012;c.pedal_ratio=4.1;c.master_bore_m=.016;c.slave_bore_m=.028;c.cable_release_travel_ratio=.32;
    c.bite_release_travel_m=.0022;c.clear_release_travel_m=.0070;c.opening_speed_m_per_s=.080;c.closing_speed_m_per_s=.025;c.pedal_return_stiffness_n_per_m=155;c.pedal_return_preload_n=4;
    const double fu[5]={60,700,1150,1050,780}; for(int k=0;k<5;k++)c.release_bearing_force_n[k]=fu[k]; }
  return w;
}
// BmwEmpiricalTunes439.Suspension(name) ported 1:1 (effective wheel rates N/m, N s/m).
void bmwTune(int t,TMVehicleSuspensionConfigV8& c){
  double fs=34000,fd=3000,fa=18000,rs=36000,rd=2800,ra=4000;
  switch(t){case TMS_TUNE_CITY: fs=27200;fd=2400;fa=9000;rs=28800;rd=2240;ra=2000;break;
  case TMS_TUNE_STREET: fa=9000;break; case TMS_TUNE_RACE: fd=3600;fa=9000;rs=43200;rd=3360;ra=8000;break; default: break;}
  c.front_spring_rate_n_per_m=fs;c.front_damper_rate_n_s_per_m=fd;c.front_anti_roll_rate_n_per_m=fa;
  c.rear_spring_rate_n_per_m=rs;c.rear_damper_rate_n_s_per_m=rd;c.rear_anti_roll_rate_n_per_m=ra;
}
}

extern "C" {
TMS_API const char* tms_version(void){ return "tms 0.1 over Tarmac NativeCurrent287 (Tarmac-Lab-Beta zip 2026-11-23)"; }
TMS_API const char* tms_last_error(void){ return g_err.c_str(); }
TMS_API double tms_tick_hz(void){ return 240.0; }

TMS_API void* tms_world_create(const float* p,uint32_t nv,const uint32_t* idx,uint32_t nt,const uint8_t* surf){
  if(!p||!idx||!nv||!nt){ setErr("world: empty mesh"); return nullptr; }
  std::vector<uint8_t> cls(nt); std::vector<uint16_t> role(nt); std::vector<float> fr(nt), rr(nt);
  std::vector<uint32_t> chunk(nt,0), chunkTri(nt), sid(nt), oid(nt,0), mid(nt);
  double mn[3]={1e30,1e30,1e30}, mx[3]={-1e30,-1e30,-1e30};
  for(uint32_t i=0;i<nv;i++) for(int k=0;k<3;k++){ double c=p[3*i+k]; if(c<mn[k])mn[k]=c; if(c>mx[k])mx[k]=c; }
  for(uint32_t t=0;t<nt;t++){ int s=surf?surf[t]:0; role[t]=roleOf(s);
    cls[t]= role[t]>=TM_CITY_ROLE_FACADE_V2?TM_CITY_COLLISION_OBSTACLE_V2: role[t]>=TM_CITY_ROLE_GRASS_V2?TM_CITY_COLLISION_FALLBACK_V2:TM_CITY_COLLISION_PRIMARY_V2; fr[t]=gripOf(s); rr[t]=rollOf(s); chunkTri[t]=t; sid[t]=(uint32_t)s; mid[t]=(uint32_t)s; }
  TMCitySurfaceDescV2 d{}; d.struct_size=sizeof d; d.abi_version=TM_CITY_SURFACE_ABI_V2;
  d.positions_xyz=p; d.vertex_count=nv; d.indices=idx; d.triangle_count=nt; d.collision_classes=cls.data(); d.semantic_roles=role.data();
  d.chunk_ids=chunk.data(); d.chunk_triangle_ids=chunkTri.data(); d.surface_ids=sid.data(); d.object_ids=oid.data(); d.material_ids=mid.data();
  d.friction_scales=fr.data(); d.rolling_scales=rr.data();
  for(int k=0;k<3;k++){ d.bounds_min[k]=mn[k]; d.bounds_max[k]=mx[k]; }
  // Provenance digests are required non-zero by the core. Not SHA-256: 4 lanes of FNV-1a over the mesh bytes (identity only).
  auto digest=[](const void* a,size_t n,const void* b,size_t m,uint8_t out[32],uint64_t salt){ for(int lane=0;lane<4;lane++){ uint64_t h=1469598103934665603ull^(salt+lane*0x9E3779B97F4A7C15ull);
      const uint8_t* x=(const uint8_t*)a; for(size_t i=lane;i<n;i+=4){h^=x[i];h*=1099511628211ull;} x=(const uint8_t*)b; for(size_t i=lane;i<m;i+=4){h^=x[i];h*=1099511628211ull;}
      for(int k=0;k<8;k++) out[lane*8+k]=(uint8_t)(h>>(8*k)); } };
  digest(p,(size_t)nv*12,idx,(size_t)nt*12,d.package_sha256,1); digest(p,(size_t)nv*12,surf?(const void*)surf:(const void*)idx,surf?nt:0,d.source_sha256,2);
  void* s=tm_city_surface_create_v2(&d);
  if(!s){ setErr(std::string("tm_city_surface_create_v2: ")+tm_vehicle_last_error()); return nullptr; }
  auto* w=new World; w->surface=s; return w;
}
TMS_API void tms_world_destroy(void* w){ if(!w)return; auto* W=(World*)w; tm_city_surface_destroy_v2(W->surface); delete W; }
TMS_API double tms_world_height(void* w,double x,double yFrom,double z){
  if(!w) return -1e9; TMCityRaycastV2 q{}; q.struct_size=sizeof q; q.collision_mask=TM_CITY_COLLISION_PRIMARY_V2|TM_CITY_COLLISION_FALLBACK_V2;
  q.origin[0]=x;q.origin[1]=yFrom;q.origin[2]=z; q.direction[1]=-1; q.max_distance=1000; q.support_only=1; q.preferred_surface_id=TM_CITY_SURFACE_ID_ANY;
  TMCityHitV2 h{}; h.struct_size=sizeof h; int r=tm_city_surface_raycast_v2(((World*)w)->surface,&q,&h); return r==1?h.point[1]:-1e9;
}

TMS_API void* tms_car_create(void* w,int carId,int tune,double x,double z,double yaw,double yHint){
  if(!w){ setErr("car: null world"); return nullptr; }
  shippedMechanicsEnv();
  auto* W=(World*)w; TMCitySpawnV2 sp=spawnOf(x,z,yaw,yHint); void* v=nullptr;
  if(carId==TMS_CAR_BMW_E21 && tune!=TMS_TUNE_BASELINE && tune!=TMS_TUNE_DRIFT){
    // Custom spring/damper/ARB route (v8) with every other record at native defaults.
    TMVehicleGearboxConfigV4 g{}; g.struct_size=sizeof g; TMVehicleEngineConfigV5 e{}; e.struct_size=sizeof e;
    TMVehicleInstallationConfigV6 in{}; in.struct_size=sizeof in; TMVehicleDrivetrainConfigV7 dt{}; dt.struct_size=sizeof dt;
    TMVehicleSuspensionConfigV8 su{}; su.struct_size=sizeof su;
    if(tm_vehicle_get_default_gearbox_config_v4(&g)==0&&tm_vehicle_get_default_engine_config_v5(&e)==0&&tm_vehicle_get_default_installation_config_v6(&in)==0&&
       tm_vehicle_get_default_drivetrain_config_v7(&dt)==0&&tm_vehicle_get_default_suspension_config_v8(&su)==0){
      bmwTune(tune,su); v=tm_vehicle_create_with_surface_and_config_v8(W->surface,&sp,&g,&e,&in,&dt,&su);
    }
    if(!v) setErr(std::string("tune route rejected, using baseline: ")+tm_vehicle_last_error());
  }
  if(!v) v=tm_vehicle_create_with_surface_v2(W->surface,&sp);
  if(!v){ setErr(std::string("tm_vehicle_create_with_surface_v2: ")+tm_vehicle_last_error()); return nullptr; }
  if(carId>0){ if(tm_vehicle_configure_car_powertrain_v104(v,(uint32_t)carId,1u,&sp)!=0){ setErr(std::string("configure_car_powertrain_v104: ")+tm_vehicle_last_error()); tm_vehicle_destroy(v); return nullptr; } }
  { TMPedalActuationConfigV312 pc=pedals313(carId); if(tm_vehicle_configure_pedal_actuation_v312(v,&pc)!=0){ setErr(std::string("pedal312: ")+tm_vehicle_last_error()); tm_vehicle_destroy(v); return nullptr; } }
  // Raw62 ownership bit0: the finite Driver68 "hands" drive the physical rack toward input.steer (Unity keyboard/wheel path).
  tm_vehicle_set_raw_steering_owner_v62(v,1u);
  TMSAssistCfg a{16,104,1,0}; tm_vehicle_set_assists_v104(v,&a); // ABS on. ESP (bit1) is off by default: in this snapshot it cuts straight-line drive torque ~70% (measured 0-5 s: 6.4 vs 17.2 m/s)
  auto* c=new Car; c->v=v; c->w=W; c->car=carId; c->tune=tune; return c;
}
TMS_API void tms_car_destroy(void* c){ if(!c)return; tm_vehicle_destroy(((Car*)c)->v); delete (Car*)c; }
TMS_API int32_t tms_car_reset(void* c,double x,double z,double yaw,double yHint){
  if(!c) return -1; TMCitySpawnV2 sp=spawnOf(x,z,yaw,yHint); int r=tm_vehicle_reset_on_surface_v2(((Car*)c)->v,&sp);
  if(r) setErr(tm_vehicle_last_error()); return r;
}
// Raw62 ownership is consumed every tick by the core (connected_proving.inc: "Consume per-tick authorization"),
// exactly like Unity re-submits it each FixedUpdate. We store it and re-submit before every tick.
TMS_API int32_t tms_car_set_steer_owner(void* c,uint32_t flags){ if(!c||flags>7) return -1; ((Car*)c)->owner=flags; return 0; }
TMS_API int32_t tms_car_set_assists(void* c,int absOn,int espOn){ if(!c)return -1; TMSAssistCfg a{16,104,(absOn?1u:0u)|(espOn?2u:0u),0}; return tm_vehicle_set_assists_v104(((Car*)c)->v,&a); }
TMS_API int32_t tms_car_step(void* c,const TMSInput* in,int32_t ticks){
  if(!c||!in) return -1; TMInput i{in->throttle,in->brake,in->steer,in->handbrake,in->clutch,in->gear,in->starter};
  int n=0; for(;n<ticks;n++){ tm_vehicle_set_raw_steering_owner_v62(((Car*)c)->v,((Car*)c)->owner); int r=tm_vehicle_step(((Car*)c)->v,&i); if(r<0){ setErr(tm_vehicle_last_error()); return n?n:r; } }
  return n;
}
TMS_API int32_t tms_car_step6(void* c,double th,double br,double st,double hb,double cl,int32_t gear,int32_t ticks){
  TMSInput i{th,br,st,hb,cl,gear,0}; return tms_car_step(c,&i,ticks);
}
TMS_API int32_t tms_car_step7(void* c,double th,double br,double st,double hb,double cl,int32_t gear,int32_t starter,int32_t ticks){
  TMSInput i{th,br,st,hb,cl,gear,starter}; return tms_car_step(c,&i,ticks);
}
TMS_API int32_t tms_car_state(void* c,float* o){
  if(!c||!o) return -1; TMState s{}; if(tm_vehicle_get_state(((Car*)c)->v,&s)) { setErr(tm_vehicle_last_error()); return -1; }
  TMVehicleSurfaceStateV2 ss{}; ss.struct_size=sizeof ss; bool haveSurf=tm_vehicle_get_surface_state_v2(((Car*)c)->v,&ss)==0;
  for(int k=0;k<3;k++){o[k]=(float)s.position[k];o[7+k]=(float)s.velocity[k];} for(int k=0;k<4;k++)o[3+k]=(float)s.quaternion[k];
  o[10]=(float)s.speed;o[11]=(float)s.rpm;o[12]=(float)s.gear;o[13]=(float)s.running;o[14]=(float)s.time;o[15]=(float)s.tick;
  for(int w=0;w<4;w++){ float* b=o+16+w*16; const TMWheelState& h=s.wheels[w];
    for(int k=0;k<3;k++)b[k]=(float)h.center[k]; for(int k=0;k<4;k++)b[3+k]=(float)h.quaternion[k];
    b[7]=(float)h.angle;b[8]=(float)h.omega;b[9]=(float)h.compression;b[10]=(float)h.normal_load;b[11]=(float)h.slip;b[12]=(float)h.fx;b[13]=(float)h.fy;b[14]=(float)h.contact;
    b[15]=haveSurf?(float)ss.wheels[w].semantic_role:0.f; }
  return 0;
}
}
