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

/* ---------------- parameters (all data driven; see cars/ *.ini) ---------------- */
typedef struct {
    double radius, rim_radius, width;      /* m */
    double spin_inertia;                   /* kg m^2 (tyre+rim+disc about axle) */
    double kz_ref, pressure_ref, kz_per_pa;/* vertical rate at reference pressure, d(rate)/d(pressure) */
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
    double toe_rod_delta;           /* m added to the rack-link length (AC TOE_OUT) */
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
    double droop_rate;              /* stiffness of the rebound (droop) stop, wheel rate */
    double arb;                     /* anti-roll bar, N/m at wheel for unit L-R travel difference */
    double unsprung_mass;           /* per corner: upright+hub+brake+wheel (arms half-lumped) */
    double upright_inertia[3];      /* about x,y,z (vehicle frame at design), excl. wheel spin */
    double brake_torque;            /* max per wheel N m */
    double handbrake_torque;        /* max per wheel N m */
    dc_tyre_params tyre;
} dc_axle_params;

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
    int assist;                    /* 0 manual, 1 hydraulic */
    double assist_area, assist_pmax, assist_tref; /* piston area m^2, max pressure Pa, valve torque for half pressure N m */
    double assist_flow, assist_rpm_full, assist_damping; /* pump flow m^3/s at/above rpm_full, hydraulic damping N s/m */
    double assist_torsion_k;       /* torsion bar rate when assisted */
    /* driveline */
    int drive;                     /* DC_DRIVE_* */
    int ngears; double gear[DC_MAX_GEARS]; double gear_reverse, final_drive;
    double engine_inertia, gearbox_inertia;
    int nlut; double lut_rpm[DC_MAX_LUT], lut_torque[DC_MAX_LUT];
    double coast_torque, coast_rpm, idle_rpm, limiter_rpm;
    double clutch_torque;
    double diff_power, diff_coast, diff_preload;     /* clutch-pack LSD: cap = preload + lock*|input| */
    double awd_front_split, center_lock;
    /* aero */
    double cda, cla_front, cla_rear;  /* m^2; lift areas positive = downforce */
    double aero_front_x, aero_rear_x, aero_z;
    /* body ground contact (sliding) points */
    int nbodypts; double bodypt[DC_MAX_BODYPTS][3]; double body_k, body_c, body_mu;
    /* integrator */
    double substep_hz;
    double gravity;
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
    double slip_angle, slip_ratio;     /* transient (from carcass deflection) */
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
    dc_wheel_telem w[4];
} dc_telemetry;

typedef struct dc_car dc_car;

DC_API void   dc_default_params(dc_car_params* p);
DC_API int    dc_load_params(const char* path, dc_car_params* p, char* err, int errlen); /* 1 ok */
DC_API dc_car* dc_create(const dc_car_params* p);
DC_API void   dc_destroy(dc_car* c);
DC_API void   dc_set_road(dc_car* c, dc_road_fn fn, void* user);
DC_API void   dc_set_heightfield(dc_car* c, const dc_heightfield* hf); /* uses built-in bilinear road fn */
DC_API void   dc_reset(dc_car* c, const double pos[3], double yaw, double speed); /* pos = ground point under CG */
DC_API void   dc_set_input(dc_car* c, const dc_input* in);
DC_API int    dc_step(dc_car* c, double frame_dt);   /* runs fixed substeps; returns count */
DC_API void   dc_substep(dc_car* c);
DC_API void   dc_get_telemetry(const dc_car* c, dc_telemetry* t);
DC_API int    dc_get_parts(const dc_car* c, dc_xform* out, int max);
DC_API int    dc_get_segments(const dc_car* c, dc_seg* out, int max);
DC_API void   dc_puncture(dc_car* c, int wheel, double leak_time_const); /* exponential leak, s */
DC_API void   dc_set_pressure(dc_car* c, int wheel, double pa);
DC_API double dc_ffb_torque(const dc_car* c);       /* column torque N m (same as telemetry) */
DC_API const dc_car_params* dc_params(const dc_car* c);
DC_API void   dc_debug_flags(dc_car* c, int flags);
enum { DC_DBG_NO_DAMPING = 1, DC_DBG_NO_FRICTION = 2, DC_DBG_NO_AERO = 4, DC_DBG_FREEZE_STEER = 8 };
#ifdef __cplusplus
}
#endif
#endif
