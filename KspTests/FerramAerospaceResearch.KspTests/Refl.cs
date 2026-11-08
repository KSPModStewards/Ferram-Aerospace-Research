using System;
using System.Reflection;
using KspTest;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>
    /// The tests' one way into FAR's non-public fields: by name, on the object's type or any type it derives from,
    /// instance or static. A missing field fails the test with its name.
    /// </summary>
    internal static class Refl
    {
        public const BindingFlags Members = BindingFlags.Instance | BindingFlags.Static | BindingFlags.Public | BindingFlags.NonPublic;

        /// <summary>The field on the type or a base type; null if there is none.</summary>
        public static FieldInfo Find(Type type, string name)
        {
            for (Type t = type; t != null; t = t.BaseType)
            {
                FieldInfo f = t.GetField(name, Members | BindingFlags.DeclaredOnly);
                if (f != null)
                    return f;
            }

            return null;
        }

        public static FieldInfo FieldOf(object o, string name)
        {
            if (o == null)
                throw new UITestFailure($"no object to read {name} from");
            return Find(o.GetType(), name) ?? throw new UITestFailure($"{o.GetType().Name} has no field {name}");
        }

        public static object Get(object o, string name) => FieldOf(o, name).GetValue(o);

        public static T Get<T>(object o, string name) => (T)Get(o, name);

        public static void Set(object o, string name, object value) => FieldOf(o, name).SetValue(o, value);

        /// <summary>A field's value if the object has it (null object or no such field: null).</summary>
        public static T TryGet<T>(object o, string name) where T : class =>
            o == null ? null : Find(o.GetType(), name)?.GetValue(o) as T;

        public static FieldInfo StaticField(Type t, string name) =>
            Find(t, name) ?? throw new UITestFailure($"{t.Name} has no static field {name}");

        public static object GetStatic(Type t, string name) => StaticField(t, name).GetValue(null);

        public static void SetStatic(Type t, string name, object value) => StaticField(t, name).SetValue(null, value);
    }
}
