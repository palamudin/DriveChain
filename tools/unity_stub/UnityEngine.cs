// Minimal UnityEngine API stub: compile-checks the #if UNITY_5_3_OR_NEWER parts of unity/*.cs without Unity
// (make unitycheck). Signatures only; nothing here runs.
namespace UnityEngine {
    public class Object { public static implicit operator bool(Object o) => o != null; }
    public class Component : Object { public Transform transform; public T GetComponent<T>() where T : class => null; }
    public class Behaviour : Component { }
    public class MonoBehaviour : Behaviour { }
    public class Transform : Component { public Vector3 position, localPosition, eulerAngles; public Quaternion rotation, localRotation; public void SetPositionAndRotation(Vector3 p, Quaternion q) { } }
    public struct Vector3 { public float x, y, z; public Vector3(float a, float b, float c) { x = a; y = b; z = c; } public static Vector3 down, up, zero; }
    public struct Quaternion { public float x, y, z, w; public Quaternion(float a, float b, float c, float d) { x = a; y = b; z = c; w = d; } public static Quaternion Euler(float a, float b, float c) => new Quaternion(); public static Quaternion AngleAxis(float a, Vector3 v) => new Quaternion(); public static Quaternion operator *(Quaternion a, Quaternion b) => a; public static Quaternion identity; }
    public struct LayerMask { public int value; public static implicit operator int(LayerMask m) => m.value; public static implicit operator LayerMask(int v) => new LayerMask { value = v }; }
    public struct RaycastHit { public Vector3 point, normal; public float distance; }
    public static class Physics { public static bool Raycast(Vector3 o, Vector3 d, out RaycastHit h, float m, int mask) { h = new RaycastHit(); return false; } }
    public static class Mathf { public const float Deg2Rad = 0.0174533f, Rad2Deg = 57.29578f; public static float Max(float a, float b) => a > b ? a : b; public static float Min(float a, float b) => a < b ? a : b; public static float Clamp(float v, float a, float b) => v; public static float Abs(float v) => v; }
    public static class Time { public static float fixedDeltaTime, deltaTime, time; }
    public static class Application { public static string streamingAssetsPath = ""; }
    public static class Debug { public static void Log(object o) { } public static void LogWarning(object o) { } public static void LogError(object o) { } }
    public enum KeyCode { None, G, R, Space }
    public static class Input { public static float GetAxis(string n) => 0; public static bool GetKeyDown(KeyCode k) => false; public static bool GetKey(KeyCode k) => false; }
    [System.AttributeUsage(System.AttributeTargets.Class, AllowMultiple = true)] public class RequireComponent : System.Attribute { public RequireComponent(System.Type t) { } }
}
namespace AOT { public class MonoPInvokeCallbackAttribute : System.Attribute { public MonoPInvokeCallbackAttribute(System.Type t) { } } }
