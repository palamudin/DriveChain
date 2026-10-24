// AbiTest.cs - plain .NET/Mono console check of the binding (no Unity): mcs unity/DriveChain.cs unity/AbiTest.cs
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
        Console.WriteLine("ABI TEST PASS");
        return 0;
    }
}
