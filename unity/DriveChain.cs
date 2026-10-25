// DriveChain.cs - Unity P/Invoke binding for the DriveChain C core (libdrivechain.so / drivechain.dll / .bundle).
//
// Frames: the core is ISO 8855 / Z-up right-handed (x fwd, y left, z up). Unity is Y-up left-handed (x right, y up, z fwd).
//   unity = ( -c.y, c.z, c.x )        c = ( u.z, -u.x, u.y )
//   quaternion (proper->improper map, so the vector part flips sign):  q_u = ( q.y, -q.z, -q.x, q.w )
// Threading: one dc_car per vehicle; call Step() from FixedUpdate (or Update with the real frame dt - the core substeps
// internally at substep_hz and keeps the remainder, so physics is decoupled from the render rate).
// Road: either bake a heightfield (fast, IL2CPP-safe) or give a static callback marked [MonoPInvokeCallback].
using System;
using System.Runtime.InteropServices;
#if UNITY_5_3_OR_NEWER
using UnityEngine;
#endif

namespace DriveChain
{
    public static class Native
    {
#if UNITY_IOS && !UNITY_EDITOR
        const string Lib = "__Internal";
#else
        const string Lib = "drivechain";
#endif
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        public delegate double RoadFn(IntPtr user, double x, double y, out double nx, out double ny, out double nz, out double grip);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)] public static extern void dc_default_params(IntPtr p);
        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)] public static extern int dc_load_params([MarshalAs(UnmanagedType.LPStr)] string path, IntPtr p, System.Text.StringBuilder err, int errlen);
        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)] public static extern int dc_save_params([MarshalAs(UnmanagedType.LPStr)] string path, IntPtr p);
        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)] public static extern int dc_set_param(IntPtr p, string section, string key, string value);
        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)] public static extern int dc_abi_size(int which);
        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)] public static extern IntPtr dc_create(IntPtr p);
        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)] public static extern void dc_destroy(IntPtr c);
        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)] public static extern void dc_set_road(IntPtr c, RoadFn fn, IntPtr user);
        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)] public static extern void dc_set_heightfield(IntPtr c, ref Heightfield hf);
        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)] public static extern void dc_reset(IntPtr c, double[] pos, double yaw, double speed);
        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)] public static extern void dc_set_input(IntPtr c, ref Input inp);
        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)] public static extern int dc_step(IntPtr c, double frame_dt);
        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)] public static extern void dc_get_telemetry(IntPtr c, ref Telemetry t);
        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)] public static extern int dc_get_parts(IntPtr c, [Out] Xform[] outp, int max);
        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)] public static extern int dc_get_segments(IntPtr c, [Out] Seg[] outp, int max);
        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)] public static extern void dc_puncture(IntPtr c, int wheel, double leak_tc);
        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)] public static extern void dc_set_pressure(IntPtr c, int wheel, double pa);
        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)] public static extern double dc_ffb_torque(IntPtr c);
        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)] public static extern void dc_debug_flags(IntPtr c, int flags);
        // ---- plug-in driver (finite-strength arms + path/angle/torque commands; see docs/DRIVER_INTERFACE.md)
        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)] public static extern void dc_driver_default_params(ref DriverParams p);
        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)] public static extern IntPtr dc_driver_create(ref DriverParams p);
        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)] public static extern void dc_driver_destroy(IntPtr d);
        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)] public static extern void dc_driver_set_params(IntPtr d, ref DriverParams p);
        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)] public static extern void dc_driver_get_params(IntPtr d, ref DriverParams p);
        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)] public static extern void dc_driver_reset(IntPtr d);
        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)] public static extern void dc_driver_set_path(IntPtr d, IntPtr xy, int n, int closed);
        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)] public static extern void dc_driver_command(IntPtr d, ref DriverCmd c);
        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)] public static extern void dc_driver_update(IntPtr d, IntPtr car, double dt, ref Input io);
        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)] public static extern void dc_driver_get_state(IntPtr d, ref DriverState s);
    }

    public enum SteerMode { Torque = 0, Angle = 1 }
    public enum Part { Chassis = 0, Upright0 = 1, Wheel0 = 5, Rack = 9, SteeringWheel = 10, Count = 11 }

    [StructLayout(LayoutKind.Sequential)]
    public struct Heightfield { public int nx, ny; public double x0, y0, cell; public IntPtr h; public IntPtr grip; }

    [StructLayout(LayoutKind.Sequential)]
    public struct Input
    {
        public int steer_mode; public double steer_torque, steer_target, hands_k, hands_d, hands_tmax;
        public double throttle, brake, handbrake, clutch; public int gear, pad;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct Xform { public double px, py, pz, qw, qx, qy, qz; }

    [StructLayout(LayoutKind.Sequential)]
    public struct Seg { public double ax, ay, az, bx, by, bz; public int type, corner; }

    [StructLayout(LayoutKind.Sequential)]
    public struct WheelTelemetry
    {
        public double fz, fx, fy, mz;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)] public double[] force_world;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)] public double[] contact;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)] public double[] normal;
        public double slip_angle, slip_ratio, slip_angle_ss, slip_ratio_ss;
        public double patch_half_len, deflection, pressure, rim_force, mu, sliding, travel, travel_vel;
        public double spring_force, damper_force, bumpstop_force, camber, toe, steer, omega, drive_torque, brake_torque;
        public double caster, kpi, scrub, trail, tierod_force;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 5)] public double[] row_fz;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct Telemetry
    {
        public double time; public long tick;
        public double speed, vx, vy, yaw_rate, ax, ay, sw_angle, sw_rate, rack_pos, rack_vel;
        public double column_torque, driver_torque, assist_force, rack_force, engine_rpm, clutch_slip;
        public int gear, pad; public double kinetic, potential;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)] public double[] momentum;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)] public double[] ang_momentum;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)] public double[] ext_force;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)] public double[] ext_torque;
        public double dissipated;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)] public double[] resid_force;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)] public double[] resid_torque;
        public int engine_running, pad2; public double proj_work;
        public double rack_fric_force, col_fric_torque;
        public int pgs_iters, box_rows, box_dropped, pad3;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)] public WheelTelemetry[] w;
    }

    public enum DriverSteer { Off = 0, HandsOff = 1, Torque = 2, Angle = 3, Path = 4, Locked = 5 }
    public enum DriverPedals { Off = 0, Direct = 1, Speed = 2 }
    /// <summary>Mirror of dc_driver_params (16 doubles; checked against dc_abi_size(6)).</summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct DriverParams
    {
        public double arm_k, arm_d, arm_tmax;      // arm impedance on the rim: N m/rad, N m s/rad, max hand torque N m
        public double delay, bandwidth_hz, rim_rate_max;
        public double preview_time, preview_min, yaw_damp, catch_gain;
        public double speed_kp, speed_ki, brake_kp, lock_frac, lat_ki, predict;
    }
    /// <summary>Mirror of dc_driver_cmd (dc_abi_size(7)).</summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct DriverCmd
    {
        public int steer_mode, pedal_mode;         // DriverSteer, DriverPedals
        public double steer_target, steer_torque;  // rad at the rim (+ left) / N m
        public double wave_amp, wave_freq, wave_t0, path_offset;
        public double throttle, brake, handbrake, speed;   // DIRECT 0..1 / SPEED target m/s
    }
    /// <summary>Mirror of dc_driver_state (dc_abi_size(8)).</summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct DriverState
    {
        public double time, rim_cmd, rim_cmd_delayed, lat_err, head_err, path_s, kappa_path, beta, preview_dist;
        public int saturated, path_index;
    }

    /// <summary>Owning wrapper for dc_driver. Call Update(car, dt, ref input) every tick BEFORE car.Step(ref input, dt).
    /// The path array is pinned for the lifetime of the driver (the core keeps the pointer).</summary>
    public sealed class Driver : IDisposable
    {
        IntPtr d; GCHandle pathPin; public DriverParams Params; public DriverCmd Cmd; public DriverState State;
        public Driver() { Native.dc_driver_default_params(ref Params); d = Native.dc_driver_create(ref Params); }
        public void ApplyParams() => Native.dc_driver_set_params(d, ref Params);
        public void Command() => Native.dc_driver_command(d, ref Cmd);
        public void Reset() => Native.dc_driver_reset(d);
        public void SetPath(double[] xy, bool closed)   // core frame (x fwd, y left, z up) world points, flat x0,y0,x1,y1...
        {
            if (pathPin.IsAllocated) pathPin.Free();
            if (xy == null || xy.Length < 4) { Native.dc_driver_set_path(d, IntPtr.Zero, 0, 0); return; }
            pathPin = GCHandle.Alloc(xy, GCHandleType.Pinned);
            Native.dc_driver_set_path(d, pathPin.AddrOfPinnedObject(), xy.Length / 2, closed ? 1 : 0);
        }
        public void Update(Car car, double dt, ref Input io) { Native.dc_driver_update(d, car.Handle, dt, ref io); Native.dc_driver_get_state(d, ref State); }
        public void Dispose() { if (d != IntPtr.Zero) Native.dc_driver_destroy(d); d = IntPtr.Zero; if (pathPin.IsAllocated) pathPin.Free(); }
    }

    /// <summary>Owning wrapper: params blob + car handle. Parameters are opaque on the managed side
    /// (set them with SetParam / a car .ini), so the binding never drifts from the C struct.</summary>
    public sealed class Car : IDisposable
    {
        IntPtr par, car; public Telemetry T; public readonly Xform[] Parts = new Xform[(int)Part.Count];
        Native.RoadFn keepAlive;  // prevent GC of a managed road callback

        public static void CheckAbi()
        {
            int[] expect = { -1, Marshal.SizeOf(typeof(Input)), Marshal.SizeOf(typeof(Telemetry)), Marshal.SizeOf(typeof(WheelTelemetry)),
                             Marshal.SizeOf(typeof(Xform)), Marshal.SizeOf(typeof(Seg)),
                             Marshal.SizeOf(typeof(DriverParams)), Marshal.SizeOf(typeof(DriverCmd)), Marshal.SizeOf(typeof(DriverState)) };
            for (int i = 1; i < expect.Length; i++)
                if (Native.dc_abi_size(i) != expect[i]) throw new Exception($"DriveChain ABI mismatch for struct {i}: native {Native.dc_abi_size(i)} managed {expect[i]}");
        }
        public Car(string iniPath = null)
        {
            CheckAbi();
            par = Marshal.AllocHGlobal(Native.dc_abi_size(0));
            Native.dc_default_params(par);
            if (!string.IsNullOrEmpty(iniPath))
            {
                var err = new System.Text.StringBuilder(512);
                if (Native.dc_load_params(iniPath, par, err, err.Capacity) == 0) throw new Exception("DriveChain: " + err);
            }
        }
        public bool SetParam(string section, string key, string value) { if (car != IntPtr.Zero) throw new InvalidOperationException("set params before Create()"); return Native.dc_set_param(par, section, key, value) != 0; }
        public void Create() { car = Native.dc_create(par); T.w = new WheelTelemetry[4]; }
        public void SetRoad(Native.RoadFn fn) { keepAlive = fn; Native.dc_set_road(car, fn, IntPtr.Zero); }
        public void SetHeightfield(ref Heightfield hf) { Native.dc_set_heightfield(car, ref hf); }  // caller pins h/grip arrays
        public void Reset(double x, double y, double z, double yaw, double speed) { Native.dc_reset(car, new[] { x, y, z }, yaw, speed); }
        public int Step(ref Input inp, double dt)
        {
            Native.dc_set_input(car, ref inp); int n = Native.dc_step(car, dt);
            Native.dc_get_telemetry(car, ref T); Native.dc_get_parts(car, Parts, Parts.Length); return n;
        }
        public double FfbTorque => Native.dc_ffb_torque(car);
        public IntPtr Handle => car;
        public void Puncture(int wheel, double leakTimeConst) => Native.dc_puncture(car, wheel, leakTimeConst);
        public void Dispose() { if (car != IntPtr.Zero) Native.dc_destroy(car); car = IntPtr.Zero; if (par != IntPtr.Zero) Marshal.FreeHGlobal(par); par = IntPtr.Zero; }

        // ---- frame conversion helpers (core Z-up RH  <->  Unity Y-up LH)
        public static void ToUnity(double cx, double cy, double cz, out float ux, out float uy, out float uz) { ux = (float)-cy; uy = (float)cz; uz = (float)cx; }
        public static void ToCore(float ux, float uy, float uz, out double cx, out double cy, out double cz) { cx = uz; cy = -ux; cz = uy; }
        public static void QuatToUnity(Xform x, out float qx, out float qy, out float qz, out float qw) { qx = (float)x.qy; qy = (float)-x.qz; qz = (float)-x.qx; qw = (float)x.qw; }
    }

#if UNITY_5_3_OR_NEWER
    /// <summary>Minimal driver: drives part transforms from the core; road from Physics.Raycast (static callback, IL2CPP-safe).</summary>
    public class DriveChainVehicle : MonoBehaviour
    {
        public string carIni = "";                         // e.g. StreamingAssets/cars/mx5_na.ini
        public Transform chassis, steeringWheel, rack;
        public Transform[] uprights = new Transform[4], wheels = new Transform[4];   // FL FR RL RR
        public LayerMask roadMask = ~0;
        Car car; Input inp; DriveChainDriverBridge bridge;
        static LayerMask sMask;
        public Car Car => car;

        [AOT.MonoPInvokeCallback(typeof(Native.RoadFn))]
        static double Road(IntPtr user, double x, double y, out double nx, out double ny, out double nz, out double grip)
        {
            Car.ToUnity(x, y, 0, out var ux, out _, out var uz);
            grip = 1; nx = 0; ny = 0; nz = 1;
            if (Physics.Raycast(new Vector3(ux, 1000f, uz), Vector3.down, out var hit, 2000f, sMask))
            {
                Car.ToCore(hit.normal.x, hit.normal.y, hit.normal.z, out nx, out ny, out nz);
                return hit.point.y;
            }
            return -1000;
        }
        // NOTE: the core samples the road several hundred times per substep (envelope stations); Physics.Raycast from
        // the physics thread is fine for a PoC, but for production bake a Heightfield around the car each frame.

        void Start()
        {
            car = new Car(string.IsNullOrEmpty(carIni) ? null : System.IO.Path.Combine(Application.streamingAssetsPath, carIni));
            car.Create(); sMask = roadMask; car.SetRoad(Road);
            Car.ToCore(transform.position.x, transform.position.y, transform.position.z, out var cx, out var cy, out var cz);
            car.Reset(cx, cy, cz, -transform.eulerAngles.y * Mathf.Deg2Rad, 0);
            inp.steer_mode = (int)SteerMode.Angle; inp.hands_k = 25; inp.hands_d = 1.2; inp.hands_tmax = 12; inp.gear = 1;
        }
        void FixedUpdate()
        {
            float h = UnityEngine.Input.GetAxis("Horizontal"), v = UnityEngine.Input.GetAxis("Vertical");
            inp.steer_mode = (int)SteerMode.Angle; inp.hands_k = 25; inp.hands_d = 1.2; inp.hands_tmax = 12;
            inp.steer_target = -h * 7.0;   // rad at the steering wheel (+ = left)
            inp.throttle = Mathf.Max(0, v); inp.brake = Mathf.Max(0, -v);
            if (!bridge) bridge = GetComponent<DriveChainDriverBridge>();
            if (bridge) bridge.Apply(car, Time.fixedDeltaTime, ref inp, h, Mathf.Max(0, v), Mathf.Max(0, -v));   // plug-in driver (overrides inp when engaged)
            car.Step(ref inp, Time.fixedDeltaTime);
            Apply(chassis, car.Parts[(int)Part.Chassis]);
            for (int i = 0; i < 4; i++) { Apply(uprights[i], car.Parts[(int)Part.Upright0 + i]); Apply(wheels[i], car.Parts[(int)Part.Wheel0 + i]); }
            Apply(rack, car.Parts[(int)Part.Rack]); Apply(steeringWheel, car.Parts[(int)Part.SteeringWheel]);
            // FFB: car.FfbTorque (N m at the steering wheel) -> your wheel SDK (Logitech G SDK / DirectInput constant force)
        }
        static void Apply(Transform t, Xform x)
        {
            if (!t) return;
            Car.ToUnity(x.px, x.py, x.pz, out var px, out var py, out var pz);
            Car.QuatToUnity(x, out var qx, out var qy, out var qz, out var qw);
            t.SetPositionAndRotation(new Vector3(px, py, pz), new Quaternion(qx, qy, qz, qw));
        }
        void OnDestroy() { car?.Dispose(); }
    }
#endif
}
