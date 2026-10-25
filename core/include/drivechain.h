/*
 * drivechain.h - public C ABI of the DriveChain vehicle physics core.
 *
 * Conventions (all SI units, double precision):
 *   World frame:   right-handed, Z up.  Vehicle frame: x forward, y left, z up (ISO 8855).
 *   Steering-wheel angle: positive = counter-clockwise (left turn), radians.
 *   Wheel order: 0 = FL, 1 = FR, 2 = RL, 3 = RR.
 *
 * Threading / allocation: dc_create() allocates one block; dc_step()/dc_substep() never allocate,
 * never read the clock, and are bit-deterministic for identical inputs on one platform/compiler.
 *
 * Unity: all structs are blittable (only doubles and int32). See unity/DriveChain.cs.
 */
#ifndef DRIVECHAIN_H
#define DRIVECHAIN_H
#ifdef __cplusplus
extern "C" {
#endif

#if defined(_WIN32) && defined(DC_BUILD_DLL)
#define DC_API __declspec(dllexport)
#elif defined(_WIN32) && defined(DC_USE_DLL)
#define DC_API __declspec(dllimport)
#else
#define DC_API
#endif

#define DC_MAX_LINKS 5
#define DC_MAX_RAYS 31
#define DC_MAX_ROWS 5
#define DC_MAX_LUT 64
#define DC_MAX_GEARS 8
#define DC_MAX_BODYPTS 12

enum { DC_SUSP_DWB = 0, DC_SUSP_STRUT = 1, DC_SUSP_MULTILINK = 2 };
enum { DC_DRIVE_RWD = 0, DC_DRIVE_FWD = 1, DC_DRIVE_AWD = 2 };
enum { DC_STEER_TORQUE = 0, DC_STEER_ANGLE = 1 };
enum { DC_TYRE_ENVELOPE = 0, DC_TYRE_TARMAC = 1 };

/* ---------------- parameters (all data driven; see cars/ *.ini) ---------------- */
typedef struct {
    double radius, rim_radius, width;      /* m */
    double spin_inertia;                   /* kg m^2 (tyre+rim+disc about axle) */
    double kz_ref, pressure_ref, kz_per_pa;/* vertical rate at reference pressure, d(rate)/d(pressure) */
    double kz_flat;                        /* carcass-only rate at 0 gauge (N/m); 0 = plain linear law.
                                              Below pressure_ref/2 the rate blends linearly to this value. */
    double cz;                             /* radial damping N s/m */
    double pressure;                       /* gauge pressure, Pa (hot/operating) */
    double k_rim, c_rim, rim_pinch;        /* rim bottoming: stiffness, damping, rubber thickness kept under rim */
    double mu_x, mu_y;                     /* peak friction at fz0 */
    double fz0, ls_exp_x, ls_exp_y;        /* load sensitivity: mu = mu_ref*(Fz/fz0)^(exp-1) */
    double falloff_level, falloff_speed;   /* sliding-speed friction decay: mu*(f + (1-f)/(1+(vs/v0)^2)) */
    double patch_factor;                   /* half patch length a = patch_factor*sqrt(2 R delta) */
    double peak_slip_deg;                  /* calibration: brush peak slip angle at fz0 -> tread stiffness */
    double cx_mult;                        /* longitudinal / lateral brush stiffness */
    double carcass_ky, carcass_kx;         /* carcass lateral/longitudinal stiffness N/m -> relaxation lengths */
    double twist_k, twist_relax;           /* patch torsional stiffness N m/rad, relaxation (m) */
    double camber_gain;                    /* camber thrust as equivalent slip ratio (dimensionless) */
    double dcamber0, dcamber1;             /* grip vs camber: mu*(1 - (g*dc0 + g^2*dc1)), g=|camber| rad */
    double rr_factor;                      /* rolling resistance: Fz offset ahead of patch centre = rr_factor*deflection */
    int n_rays, n_rows;                    /* envelope fan rays per row, rows across width */
    double ray_span_deg;                   /* +- span of the fan around straight down */
    double trail_factor;                   /* pneumatic-trail shape factor on the brush trail (1 = parabolic pressure, a/3 at
                                              zero slip; 1.5 gives a/2 = 1/4 contact length as measured, Pacejka 2012 ch.3) */
    /* ---- tyre model selector and TARMAC model (model = 1; see core/src/dc_tyre_tm.c, docs/TYRE_TARMAC.md).
       A parameter left at 0 takes the documented default (derived from the parameters above where noted). */
    int model;                             /* DC_TYRE_ENVELOPE (0, default) or DC_TYRE_TARMAC (1) */
    int tm_lanes;                          /* lanes across the contact width (7) */
    double tm_kz_mode;                     /* 0: Rhyne 2005 Kz(p, size); 1: kz law above (dc_tyre_kz) */
    double tm_contact_ratio;               /* kappa = mean contact pressure / inflation pressure (0.8) */
    double tm_carcass_roll;                /* carcass roll per unit lateral belt shift / section height (0.6) */
    double tm_cz;                          /* radial carcass damping N s/m (300) */
    double tm_kx, tm_ky;                   /* carcass (brush) stiffness N/m (default carcass_kx, carcass_ky) */
    double tm_relax_x, tm_relax_y;         /* relaxation lengths at the reference load, m (default: brush cornering /
                                              slip stiffness of the envelope model at fz_ref divided by tm_ky / tm_kx) */
    double tm_damping;                     /* brush damping N s/m (420) */
    double tm_mu;                          /* peak friction at tm_fz_ref (default mu_y at that load) */
    double tm_load_sens;                   /* mu = tm_mu (fz_ref/N)^ls (default 1 - ls_exp_y) */
    double tm_fz_ref;                      /* reference corner load N (default: static corner load m g / 4) */
    double tm_slide_floor, tm_slide_speed; /* sliding-friction decay: floor (0.7) and e-folding speed m/s (5) */
    double tm_crr;                         /* rolling resistance coefficient (0.012) */
    double tm_k_rim, tm_c_rim;             /* rim-flange pinch stiffness / damping (1.2e6, 1500) */
    double tm_shape;                       /* 0: TARMAC linear-plastic brush (faithful); 1: brush (Fiala) saturation of the tread slip */
    double tm_damp_mode;                   /* 0: TARMAC damping on slip velocity (faithful); 1: damping on the bristle deflection rate (LuGre) */
    double tm_slide_exp;                   /* sliding decay shape: floor + (1-floor) exp(-(v/vs)^exp); 0 or 1: TARMAC exponential (faithful), 2: Stribeck/Gaussian (docs/TYRE_DATA.md) */
} dc_tyre_params;

typedef struct {
    int type;                       /* DC_SUSP_* */
    int steered, driven;
    /* hardpoints of the LEFT corner, vehicle frame (x fwd, y left, z up), origin = CG at design ride height.
       right corner is mirrored (y -> -y). */
    double hub[3];                  /* wheel centre at design position */
    int nlinks;                     /* DWB: 5, STRUT: 3, MULTILINK: 5 */
    double link_c[DC_MAX_LINKS][3]; /* chassis-side point of each rigid link */
    double link_u[DC_MAX_LINKS][3]; /* upright-side point of each rigid link */
    int rack_link;                  /* index of link whose chassis end is on the steering rack (-1 none) */
    int lower_arm_a, lower_arm_b;   /* the two links forming the lower A-arm (spring/ARB carrier), -1 if none */
    double strut_top[3];            /* STRUT: chassis top mount */
    double strut_low[3];            /* STRUT: a point on the strut axis fixed to the upright (design position) */
    double toe_rod_delta;           /* m added to the rack-link length (static toe adjustment) */
    double static_camber_deg;       /* rotation of the spindle about the hub (negative = top in) */
    double static_toe_deg;          /* extra spindle toe (positive = toe-in) */
    /* spring / damper unit */
    int spring_on_upright;          /* 0: lower mount on lower A-arm, 1: on upright */
    double spring_lo[3], spring_hi[3];
    double wheel_rate;              /* N/m at the wheel at design (spring rate derived from motion ratio) */
    double spring_rate;             /* N/m along the spring (if >0 overrides wheel_rate) */
    double preload_travel;          /* m: design position is static position plus this (0 = settle at design) */
    double damp_bump, damp_fast_bump, damp_bump_thr;    /* wheel-rate N s/m, threshold m/s (wheel) */
    double damp_reb, damp_fast_reb, damp_reb_thr;
    double bumpstop_up, bumpstop_dn, bumpstop_rate;     /* wheel travel from design to stops, wheel rate */
    double bumpstop_solid;   /* wheel travel from bump-stop contact to its solid height (metal-to-metal: rigid, inelastic), m */
    double droop_rate;              /* stiffness of the rebound (droop) stop, wheel rate */
    double arb;                     /* anti-roll bar, N/m at wheel for unit L-R travel difference */
    double unsprung_mass;           /* per corner: upright+hub+brake+wheel (arms half-lumped) */
    double upright_inertia[3];      /* about x,y,z (vehicle frame at design), excl. wheel spin */
    double brake_torque;            /* max per wheel N m */
    double handbrake_torque;        /* max per wheel N m */
    dc_tyre_params tyre;
} dc_axle_params;

/* Multibody solver parameters (solver = DC_SOLVER_MULTIBODY). Index [0] front, [1] rear. Lengths m, masses kg.
   Zero bushing rates mean a rigid joint (revolute) at that pivot; kr, ka and kt all > 0 make both arm pivots of that axle
   compliant bushings (5 implicit spring-damper rows, docs/BUSHINGS.md). rack_mount_k/c are parsed but not yet used. */
typedef struct {
    double uca_mass[2], lca_mass[2], knuckle_mass[2], wheel_mass[2];   /* wheel_mass = rim + tyre + hub/rotor */
    double knuckle_inertia[2][3];   /* knuckle (+caliper) about x,y,z, kg m^2 */
    double wheel_inertia_t[2];      /* wheel transverse inertia (spin inertia is tyre_spin_inertia) */
    double bush_kr[2], bush_ka[2], bush_kt[2];   /* per bush (two per arm): radial N/m, axial N/m, conical (tilt) N m/rad */
    double bush_damp;               /* bushing damping as a loss factor: c = bush_damp * k / (2 pi 10 Hz) */
    double rack_mount_k, rack_mount_c;   /* rack housing to body along the rack axis, N/m, N s/m (0 = rigid) */
    double arb_arm[2], arb_link_frac[2], arb_bar_dz[2];  /* bar lever length, drop-link point along the lower arm (0 inner..1 ball joint), bar axis height above the lower-arm axis */
    double col_rim[3], col_u1[3], col_u2[3];  /* steering-wheel centre, upper and lower U-joint centres (vehicle frame) */
    double pinion_y;                /* lateral position of the pinion on the rack */
    double rim_inertia, ishaft_inertia, input_inertia, pinion_inertia;   /* about own axes, kg m^2 */
    double valve_a0, valve_amin, valve_theta_c, valve_cd;   /* rotary valve: open orifice area m^2, leak area of a closed metering edge, twist to close rad, discharge coeff */
    double oil_rho, oil_beta, chamber_vol, supply_vol;     /* oil density, effective bulk modulus incl. hose (Pa), each cylinder chamber + hose m^3, pump/supply volume m^3 */
    double diff_inertia, gb_out_inertia;   /* diff carrier + ring, gearbox output + propshaft, kg m^2 */
    double baumgarte;               /* joint drift correction per substep (0.1..0.3) */
    /* load-dependent joint friction (box rows bounded by mu * |own normal reaction| + preload; docs/JOINT_FRICTION.md).
       jf = 0: only the constant rack_friction / column_friction rows (previous behaviour). */
    double jf;
    double bj_mu, bj_radius, bj_torque;           /* arm ball joints: coefficient, ball radius m, preload torque N m */
    double yoke_preload, yoke_mu, mesh_sep;       /* rack yoke: spring preload N, coefficient, separating/tangential tooth force */
    double rack_bush_mu, rack_bush_span;          /* rack guide bushes: coefficient, bush spacing m */
    double brg_mu, col_brg_radius, col_brg_span;  /* rolling bearings (coefficient as in M = mu F d/2), column bearing radius, span m */
    double pin_brg_radius, pin_brg_span, pin_torque;   /* pinion bearings radius, span m, seal drag N m */
    double strut_mount_mu, strut_mount_radius, strut_mount_torque;   /* MacPherson top-mount bearing (axial load = spring + stops) */
    double strut_mu, strut_lever, strut_seal;     /* damper rod guide + piston under side load: coefficient, contact load / top side load, seal drag N */
    /* hydraulic assist valve (docs/VALVE.md): 0 = algebraic open-centre law (assist_* keys, default);
       1 = rotary-valve bridge with chamber + supply pressure states, metering edges contoured so the steady boost at
           rated flow equals the assist_pmax/assist_tref law; 2 = same bridge with linear-chamfer edges (valve_a0,
           valve_amin, valve_theta_c): boost curve fully emergent. Relief valve on the supply at assist_pmax. */
    double valve_model;
    double hose_compliance;   /* valve_model > 0: supply-hose volumetric expansion, m^3/Pa (adds to supply_vol/oil_beta). PS pressure hose
                                 is built to expand (SAE J188 "high volumetric expansion"): listed 8-17 cc/ft, test pressure not stated;
                                 taken at the 10.3 MPa rating = ~3.8e-12 per m. Plain 2-braid hose ~9e-13 per m (Parker 4660).
                                 Default 2e-12 = ~0.5 m of J188 hose. It is also the damping element: with less, rack break-away
                                 sets off large, lightly damped ~35-40 Hz rack/pressure ringing bursts (docs/VALVE.md). */
} dc_mb_params;

typedef struct {
    double mass;                   /* total vehicle mass kg (incl. unsprung) */
    double sprung_inertia[3];      /* about sprung CG, vehicle axes, kg m^2 */
    double sprung_cg[3];           /* sprung CG in vehicle frame (origin = total CG at design) */
    dc_axle_params axle[2];        /* 0 front, 1 rear */
    /* steering */
    double steer_ratio;            /* overall ratio at centre (sets pinion radius from linkage) */
    double steer_lock_deg;         /* steering wheel lock each side, deg */
    double rack_mass, sw_inertia, column_inertia;
    double torsion_k, torsion_c;   /* torsion bar / column stiffness N m/rad, damping */
    double rack_friction, column_friction; /* Coulomb N, N m */
    double rack_stop_k, rack_stop_c;
    double rack_stop_solid;   /* rack travel past stop contact to steel-on-steel (end-stop ring compressed): rigid, inelastic, m */
    int assist;                    /* 0 manual, 1 hydraulic */
    double assist_area, assist_pmax, assist_tref; /* piston area m^2, max pressure Pa, valve torque for half pressure N m */
    double assist_flow, assist_rpm_full, assist_damping; /* pump flow m^3/s at/above rpm_full, hydraulic damping N s/m */
    double assist_backdrive; /* valve orifice back-drive law gain (0.5 default: two parallel bridge legs; 1 = single orifice dP~Q^2; 0 = off) */
    double assist_torsion_k;       /* torsion bar rate when assisted */
    /* driveline */
    int drive;                     /* DC_DRIVE_* */
    int ngears; double gear[DC_MAX_GEARS]; double gear_reverse, final_drive;
    double engine_inertia, gearbox_inertia;
    int nlut; double lut_rpm[DC_MAX_LUT], lut_torque[DC_MAX_LUT];
    double coast_torque, coast_rpm, idle_rpm, limiter_rpm;
    double clutch_torque;
    double diff_power, diff_coast, diff_preload;     /* clutch-pack LSD: cap = preload + lock*|input| */
    double diff_viscous;           /* viscous-coupling LSD between the axle halves: T = diff_viscous * (w_L - w_R), N m s/rad */
    double awd_front_split, center_lock;
    /* aero */
    double cda, cla_front, cla_rear;  /* m^2; lift areas positive = downforce */
    double aero_front_x, aero_rear_x, aero_z;
    /* body ground contact (sliding) points */
    int nbodypts; double bodypt[DC_MAX_BODYPTS][3]; double body_k, body_c, body_mu;
    double body_fy, body_kh;   /* body crush: yield force N and plastic hardening N/m per point (elastic-plastic, permanent set; body_fy <= 0 = purely elastic) */
    /* integrator */
    double substep_hz;
    double road_resolve;   /* m: tyre/road resolution: the road under the envelope is linearly interpolated between nodes of this pitch, world-anchored along the heading and
                              across the axle, so road discontinuities become ramps that enter the contact continuously (the belt
                              cannot wrap features finer than this anyway; default 25 mm) */
    double gravity;
    int solver;            /* DC_SOLVER_REDUCED (0, legacy reduced-coordinate core) or DC_SOLVER_MULTIBODY (1) */
    int mb_pad;
    dc_mb_params mb;       /* multibody-only parameters (masses of the individual parts, column, valve, bushings) */
} dc_car_params;

/* ---------------- road ---------------- */
/* Return road height at (x,y) and the unit normal (nx,ny,nz). *grip = surface friction multiplier.
   Must be deterministic and side-effect free. */
typedef double (*dc_road_fn)(void* user, double x, double y, double* nx, double* ny, double* nz, double* grip);

typedef struct {   /* regular heightfield helper (bilinear) for engines that bake terrain */
    int nx, ny; double x0, y0, cell; const float* h; const float* grip; /* grip may be NULL */
} dc_heightfield;

/* ---------------- inputs ---------------- */
typedef struct {
    int steer_mode;          /* DC_STEER_TORQUE: steer_torque is the driver's hand torque on the rim.
                                DC_STEER_ANGLE:  "virtual hands": PD servo towards steer_target, strength-limited. */
    double steer_torque;     /* N m */
    double steer_target;     /* rad (steering-wheel angle) */
    double hands_k, hands_d, hands_tmax; /* PD gains N m/rad, N m s/rad, max torque N m */
    double throttle, brake, handbrake, clutch; /* 0..1 ; clutch 1 = pedal fully pressed (disengaged) */
    int gear;                /* -1 reverse, 0 neutral, 1..ngears */
    int pad;
} dc_input;

/* ---------------- outputs ---------------- */
typedef struct { double pos[3]; double quat[4]; /* w,x,y,z */ } dc_xform;

enum { DC_PART_CHASSIS = 0, DC_PART_UPRIGHT0 = 1, DC_PART_WHEEL0 = 5, DC_PART_RACK = 9, DC_PART_STEERWHEEL = 10, DC_PART_COUNT = 11 };
enum { DC_SEG_LINK = 0, DC_SEG_SPRING = 1, DC_SEG_TIEROD = 2, DC_SEG_STRUT = 3 };
typedef struct { double a[3], b[3]; int type, corner; } dc_seg;

typedef struct {
    double fz, fx, fy, mz;             /* contact-frame forces on the tyre from the road (N, N m) */
    double force_world[3];             /* total contact force, world */
    double contact[3], normal[3];      /* effective contact point & road plane normal, world */
    double slip_angle, slip_ratio;     /* transient: kinematic slip relaxed over the relaxation length (does not saturate in a slide) */
    double slip_angle_ss, slip_ratio_ss; /* kinematic steady-state */
    double patch_half_len, deflection, pressure, rim_force;
    double mu, sliding;                /* effective peak mu, 0..1 sliding fraction (brush lambda) */
    double travel, travel_vel;         /* hub travel from design (+ = bump) */
    double spring_force, damper_force, bumpstop_force;
    double camber, toe, steer;         /* rad: inclination to road, toe vs chassis, steer about chassis z */
    double omega, drive_torque, brake_torque;
    double caster, kpi, scrub, trail;  /* instantaneous kingpin geometry (rad, m) */
    double tierod_force;               /* N along rack link (+ tension) */
    double row_fz[DC_MAX_ROWS];
} dc_wheel_telem;

typedef struct {
    double time; long long tick;
    double speed, vx, vy, yaw_rate, ax, ay;   /* vehicle frame */
    double sw_angle, sw_rate, rack_pos, rack_vel;
    double column_torque;                     /* torsion-bar torque = what the driver feels (FFB) */
    double driver_torque, assist_force, rack_force;
    double engine_rpm, clutch_slip; int gear, pad;
    double kinetic, potential;                /* J */
    double momentum[3], ang_momentum[3];      /* total system, world, about world origin */
    double ext_force[3], ext_torque[3];       /* sum of external forces (road, gravity, aero) */
    double dissipated;                        /* cumulative J */
    double resid_force[3], resid_torque[3];   /* Σ m a - ΣF_ext, ΣdL/dt - ΣT_ext over ALL bodies+rotors (Newton-3 audit) */
    int engine_running, pad2;
    double proj_work;           /* cumulative KE change from the momentum projection (J), reported for honesty */
    double rack_fric_force, col_fric_torque;  /* MB core: rack friction force on the rack (N), column bearing friction torque (N m) */
    int pgs_iters, box_rows, box_dropped, pad3; /* MB core: box-LCP sweeps, active box rows, rows dropped by MB_MAXBOX (should be 0) */
    double valve_p[3];                        /* MB core, valve_model > 0: cylinder chamber A (+rack side), chamber B, pump supply gauge pressure, Pa */
    double pump_flow, hyd_energy;             /* pump delivery m^3/s, oil compression energy J (included in potential) */
    double valve_nfail;                       /* cumulative valve Newton non-convergences since reset (health counter, expect 0) */
    dc_wheel_telem w[4];
} dc_telemetry;

typedef struct dc_car dc_car;

DC_API void   dc_default_params(dc_car_params* p);
DC_API int    dc_load_params(const char* path, dc_car_params* p, char* err, int errlen); /* 1 ok */
DC_API int    dc_save_params(const char* path, const dc_car_params* p);  /* writes every parameter as INI, 1 ok */
DC_API int    dc_set_param(dc_car_params* p, const char* section, const char* key, const char* value); /* section "car"/"front"/"rear"; 1 ok */
DC_API int    dc_abi_size(int which); /* sizeof: 0 params, 1 input, 2 telemetry, 3 wheel_telem, 4 xform, 5 seg, 6 driver_params, 7 driver_cmd, 8 driver_state (managed-side layout check) */
DC_API dc_car* dc_create(const dc_car_params* p);
DC_API void   dc_destroy(dc_car* c);
DC_API void   dc_set_road(dc_car* c, dc_road_fn fn, void* user);
#define DC_ROAD_HEIGHT_ONLY 1   /* road callback accepts nx/ny/nz == NULL: return height (and grip) only */
DC_API void   dc_set_road_flags(dc_car* c, int flags); /* opt-in, call after dc_set_road (which resets it); lets the tyre lattice sampler skip the normal. Results are identical. */
DC_API void   dc_set_heightfield(dc_car* c, const dc_heightfield* hf); /* uses built-in bilinear road fn */
DC_API void   dc_reset(dc_car* c, const double pos[3], double yaw, double speed); /* pos = ground point under CG */
DC_API void   dc_set_input(dc_car* c, const dc_input* in);
DC_API int    dc_step(dc_car* c, double frame_dt);   /* runs fixed substeps; returns count */
DC_API void   dc_substep(dc_car* c);
DC_API int    dc_active_solver(const dc_car* c);   /* solver actually running (multibody falls back to reduced for AWD) */
DC_API void   dc_get_telemetry(const dc_car* c, dc_telemetry* t);
DC_API int    dc_get_parts(const dc_car* c, dc_xform* out, int max);
DC_API int    dc_get_segments(const dc_car* c, dc_seg* out, int max);
DC_API void   dc_puncture(dc_car* c, int wheel, double leak_time_const); /* exponential leak, s */
DC_API void   dc_set_pressure(dc_car* c, int wheel, double pa);
/* effective droop/bump-stop travel of a corner (m); returns 1 if the configured stops were pulled in because the
   linkage degenerates (toggle) inside the configured travel. */
DC_API int    dc_travel_limits(const dc_car* c, int corner, double* droop, double* bump);
DC_API double dc_ffb_torque(const dc_car* c);       /* column torque N m (same as telemetry) */
DC_API const dc_car_params* dc_params(const dc_car* c);
DC_API void   dc_debug_flags(dc_car* c, int flags);

/* ---------------- driver model (optional; the "interface plugin") ----------------
   A deterministic, allocation-free-per-step human/robot driver that writes the steering and pedal fields of a
   dc_input. The hands are a finite-strength impedance on the rim (the core's DC_STEER_ANGLE PD with arm_k/arm_d/
   arm_tmax), driven by a voluntary command that passes through a pure transport delay and a bandwidth limit, so
   the road can overpower the arms and self-steer still emerges. Command sources: a held angle / torque, a
   sine (scripted manoeuvres), a closed-loop path follower (preview pure pursuit + yaw damping + slip catch),
   hands-off, or a locked rim. Used by the demo, the headless host (tools/drvhost.c), Python and Unity alike. */
enum { DC_DRV_OFF = 0, DC_DRV_HANDS_OFF = 1, DC_DRV_TORQUE = 2, DC_DRV_ANGLE = 3, DC_DRV_PATH = 4, DC_DRV_LOCKED = 5 };
enum { DC_PED_OFF = 0, DC_PED_DIRECT = 1, DC_PED_SPEED = 2 };
#define DC_DRV_HIST 1024
typedef struct {
    double arm_k, arm_d, arm_tmax;      /* arm impedance on the rim: N m/rad, N m s/rad, max hand torque N m (strong adult 15-25) */
    double delay;                       /* s: transport delay on the voluntary command (human 0.15-0.25; robot 0) */
    double bandwidth_hz;                /* first-order limit on the voluntary command, Hz (0 = none) */
    double rim_rate_max;                /* rad/s cap on the commanded rim-angle rate (0 = none) */
    double preview_time, preview_min;   /* PATH: look-ahead distance L = max(preview_min, preview_time * v) */
    double yaw_damp;                    /* PATH: s; road-wheel target -= yaw_damp * (r - v * kappa_path) */
    double catch_gain;                  /* PATH: 0..1; adds catch_gain * (rear-axle sideslip beyond a 0.03 rad deadband,
                                           continuous) to the road-wheel target (countersteer) */
    double speed_kp, speed_ki, brake_kp;/* SPEED pedals: throttle PI (1/(m/s), 1/m), brake P (1/(m/s)) */
    double lock_frac;                   /* fraction of the steering lock the driver may use */
    double lat_ki;                      /* PATH: slow lateral-error trim, rad road-wheel per (m s); models the driver
                                           re-aiming against arm compliance / understeer (0 = pure pursuit only) */
    double predict;                     /* PATH: internal-model anticipation; aims from the pose predicted
                                           predict*(delay + 1/(2 pi bandwidth)) s ahead (0 = none, 1 = full) */
} dc_driver_params;
typedef struct {
    int steer_mode, pedal_mode;         /* DC_DRV_*, DC_PED_* */
    double steer_target;                /* ANGLE: rim angle, rad (+ = left) */
    double steer_torque;                /* TORQUE: hand torque on the rim, N m (clipped to +-arm_tmax) */
    double wave_amp, wave_freq, wave_t0;/* ANGLE/TORQUE: adds wave_amp*sin(2 pi f (t - t0)) (rad or N m) for t >= t0 */
    double path_offset;                 /* PATH: m, + = left of the path */
    double throttle, brake, handbrake;  /* DIRECT pedals 0..1 */
    double speed;                       /* SPEED: target m/s */
} dc_driver_cmd;
typedef struct {
    double time, rim_cmd, rim_cmd_delayed, lat_err, head_err, path_s, kappa_path, beta, preview_dist;
    int saturated, path_index;          /* saturated: |hand torque| at arm_tmax this update */
} dc_driver_state;
typedef struct dc_driver dc_driver;
DC_API void   dc_driver_default_params(dc_driver_params* p);
DC_API dc_driver* dc_driver_create(const dc_driver_params* p);
DC_API void   dc_driver_destroy(dc_driver* d);
DC_API void   dc_driver_set_params(dc_driver* d, const dc_driver_params* p);
DC_API void   dc_driver_get_params(const dc_driver* d, dc_driver_params* p);
DC_API void   dc_driver_reset(dc_driver* d);                       /* clear delay line / integrators / path search */
DC_API void   dc_driver_set_path(dc_driver* d, const double* xy, int n, int closed); /* xy[2n] world points (caller keeps alive) */
DC_API void   dc_driver_command(dc_driver* d, const dc_driver_cmd* c);
/* call once per host tick before dc_set_input/dc_step: overwrites io's steering fields (unless OFF) and pedals (unless PED_OFF) */
DC_API void   dc_driver_update(dc_driver* d, const dc_car* car, double dt, dc_input* io);
DC_API void   dc_driver_get_state(const dc_driver* d, dc_driver_state* s);
enum { DC_SOLVER_REDUCED = 0, DC_SOLVER_MULTIBODY = 1 };
enum { DC_DBG_NO_DAMPING = 1, DC_DBG_NO_FRICTION = 2, DC_DBG_NO_AERO = 4, DC_DBG_FREEZE_STEER = 8, DC_DBG_ENGINE_OFF = 16, DC_DBG_NO_PROJECTION = 32 };
#ifdef __cplusplus
}
#endif
#endif
