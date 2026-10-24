// AbiTest.cs - plain .NET/Mono console check of the binding (no Unity): mcs unity/DriveChain.cs unity/DriveChainDriverBridge.cs unity/AbiTest.cs
using System; using DriveChain;
class AbiTest {
    static int Main() {
        using (var car = new Car("cars/mx5_na.ini")) {
            Console.WriteLine("ABI sizes OK; set mass: " + car.SetParam("car", "mass", "1050") + ", front arb: " + car.SetParam("front", "arb", "9500"));
            car.Create(); car.Reset(0, 0, 0, 0, 15);
            var inp = new Input { steer_mode = 1, hands_k = 25, hands_d = 1.2, hands_tmax = 12, gear = 3, throttle = 0.3, steer_target = 0.5 };
            for (int i = 0; i < 300; i++) car.Step(ref inp, 1.0 / 100);
            var t = car.T;
            Console.WriteLine($"t {t.time:F2} speed {t.speed:F2} yaw {t.yaw_rate:F3} ay {t.ay:F2} rpm {t.engine_rpm:F0} ffb {car.FfbTorque:F2} FL fz {t.w[0].fz:F0} slip {t.w[0].slip_angle*57.3:F2} row_fz0 {t.w[0].row_fz[0]:F0}");
            Car.ToUnity(t.w[0].contact[0], t.w[0].contact[1], t.w[0].contact[2], out var ux, out var uy, out var uz);
            Console.WriteLine($"FL contact (unity) {ux:F3} {uy:F3} {uz:F3}; chassis q {car.Parts[0].qw:F4} {car.Parts[0].qz:F4}");
        }
        // plug-in driver: closed-loop path follower on a straight line offset 2 m left of the start, speed-held at 20 m/s
        using (var car = new Car("cars/mx5_na.ini")) using (var drv = new Driver()) {
            car.Create(); car.Reset(0, 0, 0, 0, 20);
            var path = new double[2 * 200]; for (int i = 0; i < 200; i++) { path[2 * i] = i * 2.0; path[2 * i + 1] = 2.0; }
            drv.SetPath(path, false);
            var ev = Environment.GetEnvironmentVariable("DRV_SET"); if (ev != null) { foreach (var kv in ev.Split(',')) { var a = kv.Split('='); var f = typeof(DriverParams).GetField(a[0]); object bx = drv.Params; f.SetValue(bx, double.Parse(a[1])); drv.Params = (DriverParams)bx; } drv.ApplyParams(); }
            drv.Cmd = new DriverCmd { steer_mode = (int)DriverSteer.Path, pedal_mode = (int)DriverPedals.Speed, speed = 20 };
            drv.Command();
            var inp = new Input { gear = 3 };
            double maxLat = 0;
            for (int i = 0; i < 10000; i++) { drv.Update(car, 1e-3, ref inp); car.Step(ref inp, 1e-3); if (i % 1000 == 0) Console.WriteLine($"  t {i*1e-3:F2} y {car.Parts[0].py:F2} lat {drv.State.lat_err:F2} rim {car.T.sw_angle*57.3:F0} beta {drv.State.beta*57.3:F1}"); if (i > 8000) maxLat = Math.Max(maxLat, Math.Abs(drv.State.lat_err)); }
            Console.WriteLine($"driver: arm_k {drv.Params.arm_k} delay {drv.Params.delay} predict {drv.Params.predict}; after 10 s y {car.Parts[0].py:F3} m (target 2.0) speed {car.T.speed:F2} lat_err(8-10 s) max {maxLat:F3} m");
            if (Math.Abs(car.Parts[0].py - 2.0) > 0.4 || maxLat > 0.4) { Console.WriteLine("DRIVER PATH FAIL"); return 1; }
        }
        // Unity driver bridge (engine-independent core): real loopback TCP JSON-lines session + human takeover
        using (var car = new Car("cars/mx5_na.ini")) using (var br = new DriverBridge()) {
            car.Create(); car.Reset(0, 0, 0, 0, 20);
            int port = 17000 + (System.Diagnostics.Process.GetCurrentProcess().Id % 2000);
            br.Listen(port);
            var tc = new System.Net.Sockets.TcpClient("127.0.0.1", port);
            var w = new System.IO.StreamWriter(tc.GetStream()) { AutoFlush = true, NewLine = "\n" }; var r = new System.IO.StreamReader(tc.GetStream());
            var inp = new Input { gear = 3 }; Func<string, string> rpc = line => {
                w.WriteLine(line); var task = r.ReadLineAsync();
                for (int k = 0; k < 2000 && !task.IsCompleted; k++) { br.Pump(car); System.Threading.Thread.Sleep(1); }
                return task.Result; };
            Console.WriteLine("  bridge hello: " + rpc("{\"cmd\":\"hello\"}"));
            Console.WriteLine("  bridge path: " + rpc("{\"cmd\":\"path\",\"type\":\"line\",\"x\":0,\"y\":2,\"yaw_deg\":0,\"length\":600}"));
            var dr = rpc("{\"cmd\":\"drive\",\"steer\":\"path\",\"pedals\":\"speed\",\"speed_kmh\":72,\"delay\":0.18}");
            Console.WriteLine("  bridge drive: " + dr);
            string bad = rpc("{\"cmd\":\"drive\",\"steer\":\"warp\"}");
            for (int i = 0; i < 10000; i++) { br.Pump(car); inp.steer_mode = 1; inp.hands_k = 25; inp.hands_d = 1.2; inp.hands_tmax = 12; inp.steer_target = 0; inp.throttle = 0; inp.brake = 0;
                br.Tick(car, 1e-3, ref inp, 0, 0, 0); car.Step(ref inp, 1e-3); }
            string g = rpc("{\"cmd\":\"get\"}"); Console.WriteLine("  bridge get: " + g);
            double y = car.Parts[0].py; bool eng1 = br.Engaged;
            br.Tick(car, 1e-3, ref inp, 0.6, 0, 0);   // human grabs the wheel
            bool tk = br.Takeover && !br.Engaged;
            Console.WriteLine($"  bridge: y {y:F3} (target 2) engaged {eng1} takeover-on-stick {tk} bad-mode-rejected {bad.Contains("\"ok\":false")}");
            tc.Close();
            if (!dr.Contains("\"engaged\":1") || Math.Abs(y - 2.0) > 0.4 || !eng1 || !tk || !bad.Contains("\"ok\":false") || !g.Contains("\"lat_err\"")) { Console.WriteLine("BRIDGE FAIL"); return 1; }
        }
        Console.WriteLine("ABI TEST PASS");
        return 0;
    }
}
