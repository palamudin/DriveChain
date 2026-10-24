// DriveChainDriverBridge.cs - plug-in driver for the Unity frontend: the same JSON-lines protocol as the raylib demo
// (docs/DRIVER_INTERFACE.md), so tools/dcdrive.py --tcp 127.0.0.1:7878 can drive a Unity scene exactly like the demo.
//
//   * dc_driver (C core) runs every FixedUpdate before the car step: finite-strength arms on the rim, so the road can
//     still overpower the "hands" (kickback, self-centring and countersteer stay emergent).
//   * Live human takeover: any deliberate human steer/pedal input releases the driver (takeover flag); "engage" or the
//     engage key re-engages.
//   * Commands (subset of the demo's): hello, drive, engage, path (points | line), get, stream, quit.
//     reset / step / set / log / metrics are host-specific and are answered with an error here.
//   * Networking runs on background threads; every command is executed on the Unity main thread (FixedUpdate).
//   * Binds 127.0.0.1 by default. The port accepts control from any local process: keep it off in shipped builds.
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;

namespace DriveChain
{
    /// <summary>Engine-independent core of the bridge (unit-testable without Unity). Call Pump() then Tick() every physics tick.</summary>
    public sealed class DriverBridge : IDisposable
    {
        public readonly Driver Drv = new Driver();
        public bool Engaged, Takeover;
        public double TakeoverStick = 0.25, TakeoverPedal = 0.15;
        public int Clients => clients.Count;
        public double StreamHz;

        TcpListener listener; Thread acceptThread; volatile bool running;
        readonly List<Client> clients = new List<Client>();
        readonly ConcurrentQueue<KeyValuePair<Client, string>> inbox = new ConcurrentQueue<KeyValuePair<Client, string>>();
        double[] path; double streamNext; Car car;

        sealed class Client
        {
            public TcpClient Tcp; public StreamWriter W; public readonly object Lock = new object(); public volatile bool Dead;
            public void Send(string s) { try { lock (Lock) { W.Write(s); W.Write('\n'); W.Flush(); } } catch { Dead = true; } }
        }

        public void Listen(int port, string bind = "127.0.0.1")
        {
            listener = new TcpListener(IPAddress.Parse(bind), port); listener.Start(); running = true;
            acceptThread = new Thread(() =>
            {
                while (running)
                {
                    TcpClient t; try { t = listener.AcceptTcpClient(); } catch { break; }
                    t.NoDelay = true;
                    var c = new Client { Tcp = t, W = new StreamWriter(t.GetStream(), new UTF8Encoding(false)) };
                    lock (clients) clients.Add(c);
                    var rt = new Thread(() =>
                    {
                        try { using (var r = new StreamReader(t.GetStream(), Encoding.UTF8)) { string line; while (running && (line = r.ReadLine()) != null) inbox.Enqueue(new KeyValuePair<Client, string>(c, line)); } }
                        catch { }
                        c.Dead = true;
                    }) { IsBackground = true };
                    rt.Start();
                }
            }) { IsBackground = true };
            acceptThread.Start();
        }

        /// <summary>Main thread: execute queued commands and drop dead clients. Disengages when the last client leaves.</summary>
        public void Pump(Car c)
        {
            car = c;
            while (inbox.TryDequeue(out var kv)) { string reply; bool quit = Handle(kv.Value, out reply); kv.Key.Send(reply); if (quit) kv.Key.Dead = true; }
            lock (clients)
            {
                bool had = clients.Count > 0;
                for (int i = clients.Count - 1; i >= 0; i--) if (clients[i].Dead) { try { clients[i].Tcp.Close(); } catch { } clients.RemoveAt(i); }
                if (had && clients.Count == 0) Engaged = false;   // controller gone: hands back to the human
            }
        }

        /// <summary>Main thread, every physics tick before car.Step: human input in [-1,1] / [0,1]; overwrites inp when engaged.</summary>
        public void Tick(Car c, double dt, ref Input inp, double humanSteer, double humanThrottle, double humanBrake)
        {
            car = c;
            if (Engaged && (Math.Abs(humanSteer) > TakeoverStick || humanThrottle > TakeoverPedal || humanBrake > TakeoverPedal)) { Engaged = false; Takeover = true; }
            if (Engaged) Drv.Update(c, dt, ref inp);
            if (StreamHz > 0 && c.T.time >= streamNext) { streamNext = c.T.time + 1.0 / StreamHz; var s = Telemetry(); lock (clients) foreach (var k in clients) k.Send(s); }
        }

        // ---------------- protocol ----------------
        public bool Handle(string line, out string reply)
        {
            Dictionary<string, object> o;
            try { o = Json.Parse(line); } catch (Exception e) { reply = Err("bad json: " + e.Message); return false; }
            if (o == null) { reply = "{\"ok\":true,\"note\":\"empty\"}"; return false; }
            string cmd = Json.Str(o, "cmd", null);
            switch (cmd)
            {
                case null: reply = Err("missing cmd"); return false;
                case "hello": case "info":
                    reply = "{\"ok\":true,\"protocol\":1,\"name\":\"drivechain-unity\",\"lockstep\":0,\"spots\":[],\"cones\":[]}"; return false;
                case "drive": reply = Drive(o); return false;
                case "engage": Engaged = Json.Num(o, "on", 1) != 0; if (Engaged) Takeover = false; reply = "{\"ok\":true,\"engaged\":" + (Engaged ? 1 : 0) + "}"; return false;
                case "path": reply = Path(o); return false;
                case "get": reply = Telemetry(); return false;
                case "stream": StreamHz = Json.Num(o, "hz", 0); reply = "{\"ok\":true,\"hz\":" + F(StreamHz) + "}"; return false;
                case "quit": Engaged = false; reply = "{\"ok\":true,\"bye\":1}"; return true;
                default: reply = Err("command '" + cmd + "' is not available on the Unity bridge (use drvhost for lockstep tests)"); return false;
            }
        }

        static readonly string[] SteerNames = { "off", "hands_off", "torque", "angle", "path", "locked" };
        string Drive(Dictionary<string, object> o)
        {
            var c = Drv.Cmd; var p = Drv.Params; const double D = 180.0 / Math.PI;
            var sm = Json.Str(o, "steer", null);
            if (sm != null) { int m = Array.IndexOf(SteerNames, sm.Replace('-', '_')); if (m < 0) return Err("unknown steer mode '" + sm + "'"); c.steer_mode = m; }
            var pm = Json.Str(o, "pedals", null);
            if (pm != null) { if (pm == "off") c.pedal_mode = 0; else if (pm == "direct") c.pedal_mode = 1; else if (pm == "speed") c.pedal_mode = 2; else return Err("unknown pedal mode '" + pm + "'"); }
            c.steer_target = o.ContainsKey("target_deg") ? Json.Num(o, "target_deg", 0) / D : Json.Num(o, "target", c.steer_target);
            c.steer_torque = Json.Num(o, "torque", c.steer_torque);
            c.wave_amp = o.ContainsKey("wave_amp_deg") ? Json.Num(o, "wave_amp_deg", 0) / D : Json.Num(o, "wave_amp", c.wave_amp);
            c.wave_freq = Json.Num(o, "wave_freq", c.wave_freq);
            if (o.ContainsKey("wave_delay")) c.wave_t0 = Drv.State.time + Json.Num(o, "wave_delay", 0);
            c.path_offset = Json.Num(o, "offset", c.path_offset);
            c.throttle = Json.Num(o, "throttle", c.throttle); c.brake = Json.Num(o, "brake", c.brake); c.handbrake = Json.Num(o, "handbrake", c.handbrake);
            c.speed = o.ContainsKey("speed_kmh") ? Json.Num(o, "speed_kmh", 0) / 3.6 : Json.Num(o, "speed", c.speed);
            p.arm_k = Json.Num(o, "arm_k", p.arm_k); p.arm_d = Json.Num(o, "arm_d", p.arm_d); p.arm_tmax = Json.Num(o, "arm_tmax", p.arm_tmax);
            if (o.ContainsKey("delay")) { double dl = Json.Num(o, "delay", 0); if (dl < 0 || dl > 0.5) return Err("delay must be 0..0.5 s"); p.delay = dl; }
            p.bandwidth_hz = Json.Num(o, "bandwidth_hz", p.bandwidth_hz);
            if (o.ContainsKey("rate_max_dps")) p.rim_rate_max = Json.Num(o, "rate_max_dps", 0) / D;
            p.preview_time = Json.Num(o, "preview_time", p.preview_time); p.preview_min = Json.Num(o, "preview_min", p.preview_min);
            p.yaw_damp = Json.Num(o, "yaw_damp", p.yaw_damp); p.catch_gain = Json.Num(o, "catch_gain", p.catch_gain);
            p.lock_frac = Json.Num(o, "lock_frac", p.lock_frac); p.lat_ki = Json.Num(o, "lat_ki", p.lat_ki); p.predict = Json.Num(o, "predict", p.predict);
            p.speed_kp = Json.Num(o, "speed_kp", p.speed_kp); p.speed_ki = Json.Num(o, "speed_ki", p.speed_ki); p.brake_kp = Json.Num(o, "brake_kp", p.brake_kp);
            if (c.steer_mode == 4 && (path == null || path.Length < 4)) return Err("steer=path needs a path first (cmd path, type points|line)");
            Drv.Params = p; Drv.ApplyParams(); Drv.Cmd = c; Drv.Command();
            Engaged = Json.Num(o, "engage", 1) != 0; if (Engaged) Takeover = false;
            return "{\"ok\":true,\"driver\":\"" + SteerNames[Math.Max(0, Math.Min(5, c.steer_mode))] + "\",\"pedals\":" + c.pedal_mode + ",\"engaged\":" + (Engaged ? 1 : 0) + "}";
        }

        string Path(Dictionary<string, object> o)
        {
            string type = Json.Str(o, "type", "points");
            if (type == "points")
            {
                var pts = Json.Arr(o, "pts"); if (pts == null || pts.Length < 4 || pts.Length % 2 != 0) return Err("pts must be a flat [x0,y0,x1,y1,...] array (core frame, metres)");
                path = pts; Drv.SetPath(path, Json.Num(o, "closed", 0) != 0);
            }
            else if (type == "line")
            {
                double x = Json.Num(o, "x", 0), y = Json.Num(o, "y", 0), yaw = Json.Num(o, "yaw_deg", 0) * Math.PI / 180, len = Json.Num(o, "length", 1000);
                int n = Math.Max(2, Math.Min(8192, (int)(len / 2) + 1)); path = new double[2 * n];
                for (int i = 0; i < n; i++) { double s = len * i / (n - 1); path[2 * i] = x + s * Math.Cos(yaw); path[2 * i + 1] = y + s * Math.Sin(yaw); }
                Drv.SetPath(path, false);
            }
            else return Err("path type '" + type + "' not available on the Unity bridge (points | line)");
            Drv.Reset();
            return "{\"ok\":true,\"points\":" + path.Length / 2 + "}";
        }

        public string Telemetry()
        {
            if (car == null) return Err("no car");
            var t = car.T; var ch = car.Parts[(int)Part.Chassis]; var s = Drv.State; const double D = 180.0 / Math.PI;
            double yaw = Math.Atan2(2 * (ch.qw * ch.qz + ch.qx * ch.qy), 1 - 2 * (ch.qy * ch.qy + ch.qz * ch.qz));
            var b = new StringBuilder(512);
            b.Append("{\"ok\":true,\"t\":").Append(F(t.time)).Append(",\"x\":").Append(F(ch.px)).Append(",\"y\":").Append(F(ch.py)).Append(",\"z\":").Append(F(ch.pz))
             .Append(",\"yaw_deg\":").Append(F(yaw * D)).Append(",\"speed_kmh\":").Append(F(t.speed * 3.6)).Append(",\"yaw_rate_dps\":").Append(F(t.yaw_rate * D))
             .Append(",\"ax\":").Append(F(t.ax)).Append(",\"ay\":").Append(F(t.ay)).Append(",\"sw_deg\":").Append(F(t.sw_angle * D)).Append(",\"column_nm\":").Append(F(t.column_torque))
             .Append(",\"rim_cmd_deg\":").Append(F(s.rim_cmd * D)).Append(",\"lat_err\":").Append(F(s.lat_err)).Append(",\"head_err_deg\":").Append(F(s.head_err * D))
             .Append(",\"beta_deg\":").Append(F(s.beta * D)).Append(",\"saturated\":").Append(s.saturated)
             .Append(",\"driver\":\"").Append(SteerNames[Math.Max(0, Math.Min(5, Drv.Cmd.steer_mode))]).Append("\",\"engaged\":").Append(Engaged ? 1 : 0)
             .Append(",\"takeover\":").Append(Takeover ? 1 : 0).Append('}');
            return b.ToString();
        }

        static string F(double v) => double.IsNaN(v) || double.IsInfinity(v) ? "0" : v.ToString("0.######", CultureInfo.InvariantCulture);
        static string Err(string m) => "{\"ok\":false,\"error\":\"" + m.Replace("\\", "\\\\").Replace("\"", "'") + "\"}";

        public void Dispose()
        {
            running = false; try { listener?.Stop(); } catch { }
            lock (clients) { foreach (var c in clients) try { c.Tcp.Close(); } catch { } clients.Clear(); }
            Drv.Dispose();
        }
    }

    /// <summary>Minimal JSON reader for the flat protocol objects (strings, numbers, booleans, null, number arrays).</summary>
    public static class Json
    {
        public static Dictionary<string, object> Parse(string s)
        {
            int i = 0; Ws(s, ref i); if (i >= s.Length || s[i] == '#') return null;
            if (s[i] != '{') throw new FormatException("expected an object");
            var o = new Dictionary<string, object>(); i++;
            for (;;)
            {
                Ws(s, ref i); if (i < s.Length && s[i] == '}') { i++; break; }
                string k = ReadStr(s, ref i); Ws(s, ref i); if (i >= s.Length || s[i] != ':') throw new FormatException("expected ':'"); i++;
                o[k] = Value(s, ref i); Ws(s, ref i);
                if (i < s.Length && s[i] == ',') { i++; continue; }
                if (i < s.Length && s[i] == '}') { i++; break; }
                throw new FormatException("expected ',' or '}'");
            }
            return o;
        }
        static object Value(string s, ref int i)
        {
            Ws(s, ref i); if (i >= s.Length) throw new FormatException("unexpected end");
            char c = s[i];
            if (c == '"') return ReadStr(s, ref i);
            if (c == '[')
            {
                i++; var l = new List<double>();
                for (;;) { Ws(s, ref i); if (i < s.Length && s[i] == ']') { i++; break; } l.Add(Convert.ToDouble(Value(s, ref i), CultureInfo.InvariantCulture)); Ws(s, ref i); if (i < s.Length && s[i] == ',') i++; }
                return l.ToArray();
            }
            if (s.Length - i >= 4 && string.CompareOrdinal(s, i, "true", 0, 4) == 0) { i += 4; return 1.0; }
            if (s.Length - i >= 5 && string.CompareOrdinal(s, i, "false", 0, 5) == 0) { i += 5; return 0.0; }
            if (s.Length - i >= 4 && string.CompareOrdinal(s, i, "null", 0, 4) == 0) { i += 4; return null; }
            int j = i; while (j < s.Length && "+-0123456789.eE".IndexOf(s[j]) >= 0) j++;
            if (j == i) throw new FormatException("bad value at " + i);
            var v = double.Parse(s.Substring(i, j - i), NumberStyles.Float, CultureInfo.InvariantCulture); i = j; return v;
        }
        static string ReadStr(string s, ref int i)
        {
            if (i >= s.Length || s[i] != '"') throw new FormatException("expected a string"); i++;
            var b = new StringBuilder();
            while (i < s.Length && s[i] != '"') { if (s[i] == '\\' && i + 1 < s.Length) { i++; char e = s[i]; b.Append(e == 'n' ? '\n' : e == 't' ? '\t' : e); } else b.Append(s[i]); i++; }
            i++; return b.ToString();
        }
        static void Ws(string s, ref int i) { while (i < s.Length && char.IsWhiteSpace(s[i])) i++; }
        public static string Str(Dictionary<string, object> o, string k, string def) => o.TryGetValue(k, out var v) && v is string sv ? sv : def;
        public static double Num(Dictionary<string, object> o, string k, double def) => o.TryGetValue(k, out var v) && v is double d ? d : def;
        public static double[] Arr(Dictionary<string, object> o, string k) => o.TryGetValue(k, out var v) ? v as double[] : null;
    }

#if UNITY_5_3_OR_NEWER
    /// <summary>Add next to DriveChainVehicle: opens the control port and runs the plug-in driver on that vehicle.</summary>
    [UnityEngine.RequireComponent(typeof(DriveChainVehicle))]
    public class DriveChainDriverBridge : UnityEngine.MonoBehaviour
    {
        public int port = 7878;
        public string bindAddress = "127.0.0.1";
        public UnityEngine.KeyCode engageKey = UnityEngine.KeyCode.G;
        public DriverBridge Bridge { get; private set; }
        bool toggle;
        void Awake() { Bridge = new DriverBridge(); Bridge.Listen(port, bindAddress); UnityEngine.Debug.Log($"[DriveChain] driver control port {bindAddress}:{port}"); }
        void Update() { if (UnityEngine.Input.GetKeyDown(engageKey)) toggle = true; }
        /// <summary>Called by DriveChainVehicle.FixedUpdate before the car step.</summary>
        public void Apply(Car car, double dt, ref Input inp, double humanSteer, double humanThrottle, double humanBrake)
        {
            Bridge.Pump(car);
            if (toggle) { toggle = false; Bridge.Engaged = !Bridge.Engaged; if (Bridge.Engaged) Bridge.Takeover = false; }
            Bridge.Tick(car, dt, ref inp, humanSteer, humanThrottle, humanBrake);
        }
        void OnDestroy() { Bridge?.Dispose(); }
    }
#endif
}
