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
        return 0;
    }
}
