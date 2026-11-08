using System;
using System.Reflection;

namespace FarCoverage
{
    /// <summary>
    /// Stands in for AltCover.Recorder.Instance.Visit(moduleId, hitPointId) at every instrumented point (the call sites
    /// are rewritten after instrumenting): the first visit to a point goes on to AltCover, later ones return at once.
    /// AltCover 9 records only the first visit anyway (without --all), but reaches that answer through allocations and
    /// string-keyed dictionaries on every visit, which under Mono's stop-the-world GC made multithreaded sweeps ~100x
    /// slower than uninstrumented.
    /// </summary>
    public static class FastVisit
    {
        private const int Size = 1 << 16;
        private static volatile string _m0, _m1;
        private static volatile bool[] _s0, _s1;
        private static readonly object Gate = new object();
        private static Action<string, int> _visit;

        public static void Visit(string moduleId, int hitPointId)
        {
            bool[] seen = ReferenceEquals(moduleId, _m0) ? _s0 : ReferenceEquals(moduleId, _m1) ? _s1 : Seen(moduleId);
            if (seen != null && (uint)hitPointId < (uint)seen.Length)
            {
                if (seen[hitPointId])
                    return;
                Recorder()(moduleId, hitPointId);
                seen[hitPointId] = true;   // after the visit is recorded: a race only visits twice
                return;
            }

            Recorder()(moduleId, hitPointId);
        }

        // the module's array (two modules, FAR and FAR.Base, each an interned literal), or null past two
        private static bool[] Seen(string moduleId)
        {
            lock (Gate)
            {
                if (ReferenceEquals(moduleId, _m0)) return _s0;
                if (ReferenceEquals(moduleId, _m1)) return _s1;
                if (_m0 == null)
                {
                    _s0 = new bool[Size];
                    _m0 = moduleId;
                    return _s0;
                }

                if (_m1 == null)
                {
                    _s1 = new bool[Size];
                    _m1 = moduleId;
                    return _s1;
                }

                return null;
            }
        }

        private static Action<string, int> Recorder()
        {
            Action<string, int> v = _visit;
            if (v != null)
                return v;
            Type t = Type.GetType("AltCover.Recorder.Instance, AltCover.Recorder.g", true);
            MethodInfo m = t.GetMethod("Visit", BindingFlags.Public | BindingFlags.Static, null, new[] { typeof(string), typeof(int) }, null)
                           ?? throw new MissingMethodException("AltCover.Recorder.Instance", "Visit");
            return _visit = (Action<string, int>)Delegate.CreateDelegate(typeof(Action<string, int>), m);
        }
    }
}
