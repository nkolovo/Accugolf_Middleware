// ------------------------------------------------------------
// Compat/NetFrameworkCompat.cs
// ------------------------------------------------------------
// .NET Framework 4.8 doesn't ship System.MathF (added in .NET Core 2.0 /
// .NET Standard 2.1). This is net48-only — on net10.0-windows this file
// compiles to nothing and the real System.MathF is used.
//
// Math.Clamp(x, min, max) has the same problem (also added later) but can't
// be polyfilled the same way — System.Math is sealed, so a same-named type
// in the System namespace would collide. Call sites use
// MathF.Max(min, MathF.Min(max, x)) instead, which works unchanged on both
// targets.
//
// Only implements whatever methods call sites have actually needed so far —
// NOT the full real MathF surface. Adding a new MathF.Whatever() call site
// compiles fine on net10.0-windows (real System.MathF has everything) but
// fails on net48 with "MathF does not contain a definition for Whatever"
// unless it's added here too. Found live: Atan/Abs/Round were added to this
// file only after a net48 build (on the actual hardware workstation, which
// this dev environment can't compile-check at all — no Spinnaker SDK
// installed here) failed on calls this session's mock-only testing had no
// way to catch. If you add a new MathF call, add it here in the same pass.
// ------------------------------------------------------------
#if NET48
namespace System
{
    internal static class MathF
    {
        public const float PI = 3.14159265358979323846f;
        public static float Sqrt(float x) => (float)Math.Sqrt(x);
        public static float Atan2(float y, float x) => (float)Math.Atan2(y, x);
        public static float Ceiling(float x) => (float)Math.Ceiling(x);
        public static float Max(float a, float b) => Math.Max(a, b);
        public static float Min(float a, float b) => Math.Min(a, b);
        public static float Cos(float x) => (float)Math.Cos(x);
        public static float Sin(float x) => (float)Math.Sin(x);
        public static float Atan(float x) => (float)Math.Atan(x);
        public static float Abs(float x) => Math.Abs(x);
        public static float Round(float x) => (float)Math.Round(x);
    }
}
#endif
