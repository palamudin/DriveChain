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
// ---- Stock 1990 Mazda MX-5 / Miata (NA6, 1.6 B6ZE, 5-speed, open diff) ---------------------------------------------
// All values below come from public sources or are own engineering estimates (marked EST). Sources:
//  [S1] Moss Miata, 1990 MX-5 Miata specifications: https://mossmiata.com/miata-1990
//       (B6 1.6 DOHC, 116 hp @ 6500, 100 lb-ft/136 N m @ 5500, idle 850, curb 940 kg, 185/60R14, open diff std / viscous LSD option)
//  [S2] Mazda 1990 MX-5 Miata Specifications (press kit, hosted by miata.net):
//       https://www.miata.net/faq/First%2025%20Years/Pricing%20&%20Specs%20-%20Model%20Year/Specifications/1990%20MX-5%20Miata%20Specifications.pdf
//       (5-speed 3.136/1.888/1.330/1.000/0.814, R 3.758, final 4.300; hydraulic single dry plate clutch, 200 mm);
//       cross-checked: https://topofsecond.com/vehicles/1990/mazda/mx-5-miata-1.6-us (brochure: 7000 rpm tach redline)
//  [S3] DriveChain cars/mx5_na.ini (public repo palamudin/DriveChain): OEM spring rates 28.75/16.98 kN/m and motion
//       ratios 0.66/0.77 -> wheel rates 12.5/10.1 kN/m; damping ratios; ARB 19 mm/12 mm wheel-rate estimates; torque curve shape.
//  Disc sizes 235 mm vented front / 231 mm solid rear, 7/8" brake master, 5/8" clutch master and 3/4" slave are the
//  commonly published NA 1.6 part sizes (parts catalogues); treat as EST.
// The core's chassis mass, inertia, tyre and hardpoint geometry are its frozen baseline (not configurable through this ABI);
// only gearbox, engine, drivetrain, springs/dampers/ARBs and pedal actuation are Miata-specific here.
TMPedalActuationConfigV312 miataPedals(){
  TMPedalActuationConfigV312 w{}; w.struct_size=sizeof w; w.abi_version=TM_PEDAL_ACTUATION_ABI_V312;
  const double pressure=7.0e6;                          // EST working line pressure at full braking
  const double frontTorque=950, rearTorque=450;         // N m per wheel at full pedal [S3]
  const double masterBore=0.0222, travel=0.120, freeT=0.006, pedalRatio=4.0; // 7/8" master (EST), EST travel/ratio
  const double masterArea=3.141592653589793*masterBore*masterBore*.25, compliance=masterArea*((travel-freeT)/pedalRatio)/pressure;
  const double fr[4]={0.100,0.100,0.097,0.097};        // EST effective pad radius: 235/231 mm discs minus half pad height
  auto& b=w.brake; b.activation=TM_PEDAL_ACTUATION_AUTHORED_V312; b.topology=0; b.assist=1; b.provenance=3;
  labelDigest("mazda-mx5-na6-1990|public-spec-brake",b.provenance_sha256);
  b.maximum_pedal_travel_m=travel; b.free_pedal_travel_m=freeT; b.pedal_ratio=pedalRatio; b.booster_force_gain=4; b.master_bore_m=masterBore;
  b.circuit_compliance_m3_per_pa[0]=b.circuit_compliance_m3_per_pa[1]=compliance; b.maximum_pressure_pa=12e6;
  b.compensation_port_pedal_fraction=.005; b.pedal_return_stiffness_n_per_m=250; b.pedal_return_preload_n=5;
  for(int k=0;k<4;k++){ double corner=k<2?frontTorque:rearTorque; b.effective_radius_m[k]=fr[k]; b.pad_friction[k]=.38; b.effective_clamp_area_m2[k]=corner/(pressure*.38*fr[k]); }
  const uint32_t circ[4]={0u,1u,1u,0u}; for(int k=0;k<4;k++) b.circuit_for_corner[k]=circ[k]; // diagonal split
  auto& c=w.clutch; c.activation=TM_PEDAL_ACTUATION_AUTHORED_V312; c.clamp_curve=1; c.provenance=3; c.actuation=1; // hydraulic
  labelDigest("mazda-mx5-na6-1990|public-spec-hydraulic-clutch",c.provenance_sha256);
  const double ct=.130, cf=.010, cr=4.0, cm=.015875, cs=.01905; const double clear=(ct-cf)/cr*(cm*cm)/(cs*cs);
  c.maximum_pedal_travel_m=ct;c.free_pedal_travel_m=cf;c.pedal_ratio=cr;c.master_bore_m=cm;c.slave_bore_m=cs;c.cable_release_travel_ratio=.30;
  c.bite_release_travel_m=.18*clear;c.clear_release_travel_m=clear;c.opening_speed_m_per_s=12*clear;c.closing_speed_m_per_s=2.8*clear;
  c.pedal_return_stiffness_n_per_m=170;c.pedal_return_preload_n=4;
  const double rb[5]={70,850,1350,1250,950}; for(int k=0;k<5;k++)c.release_bearing_force_n[k]=rb[k]; // EST diaphragm curve, 200 mm clutch
  return w;
}
void miataConfig(TMVehicleGearboxConfigV4& g,TMVehicleEngineConfigV5& e,TMVehicleDrivetrainConfigV7& dt,TMVehicleSuspensionConfigV8& su){
  const double gr[5]={3.136,1.888,1.330,1.000,0.814}; for(int k=0;k<6;k++) g.forward_ratios[k]=k<5?gr[k]:0; // [S2]
  g.reverse_ratio=3.758; g.final_drive_ratio=4.300; // [S2]
  e.idle_rpm=850; e.limit_rpm=7200; e.shaft_inertia_kg_m2=0.12; // EST fuel cut just past the 7000 rpm redline [S2]; EST crank+flywheel inertia
  const double rpm[13]={0,1000,2000,3000,4000,5000,5500,6000,6500,7000,7200,7500,8000};
  const double tq[13]={60,95,112,121,127,132,136,133,127,115,110,60,0}; // shape [S3], peak 136 N m @ 5500 [S1]
  e.curve_point_count=13; for(int k=0;k<13;k++){ e.torque_curve[k].rpm=rpm[k]; e.torque_curve[k].torque_nm=tq[k]; }
  dt.drive_layout=TM_VEHICLE_DRIVE_RWD_V7; dt.differential_type=TM_VEHICLE_DIFFERENTIAL_OPEN_V7; // base 1990 car: open diff (viscous LSD was optional)
  dt.preload_torque_nm=0; dt.power_lock_ratio=0; dt.coast_lock_ratio=0;
  // Stock Miata wheel rates [S3]: springs 12.5/10.1 kN/m, dampers 1050/900 N s/m (mean of bump/rebound), ARB 9.5/2.5 kN/m.
  // The core validates springs >= 18 kN/m and dampers >= 1.2 kN s/m (modules.hpp), so the soft stock values are clamped to
  // that floor. Its baseline sprung mass is 1052 kg (Miata ~880 kg + driver), so the ride is stiffer than a real NA6.
  auto clampR=[](double x,double lo,double hi){ return x<lo?lo:x>hi?hi:x; };
  su.front_spring_rate_n_per_m=clampR(12500,18000,246153); su.rear_spring_rate_n_per_m=clampR(10100,18000,246153);
  su.front_damper_rate_n_s_per_m=clampR(1050,1200,9000);   su.rear_damper_rate_n_s_per_m=clampR(900,1200,9000);
  su.front_anti_roll_rate_n_per_m=9500; su.rear_anti_roll_rate_n_per_m=2500;
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
  if(carId==TMS_CAR_MIATA_NA){
    TMVehicleGearboxConfigV4 g{}; g.struct_size=sizeof g; g.abi_version=TM_VEHICLE_GEARBOX_CONFIG_ABI_V4; TMVehicleEngineConfigV5 e{}; e.struct_size=sizeof e; e.abi_version=TM_VEHICLE_ENGINE_CONFIG_ABI_V5;
    TMVehicleInstallationConfigV6 in{}; in.struct_size=sizeof in; in.abi_version=TM_VEHICLE_INSTALLATION_CONFIG_ABI_V6; TMVehicleDrivetrainConfigV7 dt{}; dt.struct_size=sizeof dt; dt.abi_version=TM_VEHICLE_DRIVETRAIN_CONFIG_ABI_V7;
    TMVehicleSuspensionConfigV8 su{}; su.struct_size=sizeof su; su.abi_version=TM_VEHICLE_SUSPENSION_CONFIG_ABI_V8;
    if(tm_vehicle_get_default_gearbox_config_v4(&g)==0&&tm_vehicle_get_default_engine_config_v5(&e)==0&&tm_vehicle_get_default_installation_config_v6(&in)==0&&
       tm_vehicle_get_default_drivetrain_config_v7(&dt)==0&&tm_vehicle_get_default_suspension_config_v8(&su)==0){
      miataConfig(g,e,dt,su); v=tm_vehicle_create_with_surface_and_config_v8(W->surface,&sp,&g,&e,&in,&dt,&su);
    }
    if(!v){ setErr(std::string("miata config rejected: ")+tm_vehicle_last_error()); return nullptr; }
  }
  if(!v) v=tm_vehicle_create_with_surface_v2(W->surface,&sp); // TMS_CAR_BASELINE: the core's own default vehicle
  if(!v){ setErr(std::string("tm_vehicle_create_with_surface_v2: ")+tm_vehicle_last_error()); return nullptr; }
  (void)tune; { TMPedalActuationConfigV312 pc=miataPedals(); if(tm_vehicle_configure_pedal_actuation_v312(v,&pc)!=0){ setErr(std::string("pedal312: ")+tm_vehicle_last_error()); tm_vehicle_destroy(v); return nullptr; } }
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
