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
    }
}
#endif
