using System;
using System.Collections;
using System.Collections.Generic;
using JetBrains.Annotations;
using Unity.Collections;
using Unity.Mathematics;
using Object = UnityEngine.Object;

namespace FerramAerospaceResearch.Geometry.Exposure;

public class RenderResult
{
    public float3 position;
    public float3 forward;
    public float centerDistance;
    public NativeSlice<int> pixelCounts;
    public readonly List<double> Areas = new();
    public double areaPerPixel;
}

public class RenderResult<T> : RenderResult, IReadOnlyDictionary<T, double> where T: Object
{
    public Renderer<T> renderer;

    public Enumerator GetEnumerator()
    {
        return new Enumerator(renderer, Areas);
    }

    IEnumerator<KeyValuePair<T, double>> IEnumerable<KeyValuePair<T, double>>.GetEnumerator()
    {
        return GetEnumerator();
    }

    IEnumerator IEnumerable.GetEnumerator()
    {
        return GetEnumerator();
    }

    public int Count
    {
        get { return renderer.Count; }
    }

    public bool ContainsKey(T key)
    {
        return renderer.ContainsKey(key);
    }

    public bool TryGetValue(T key, out double value)
    {
        if (renderer.TryGetValue(key, out int index))
        {
            value = Areas[index];
            return true;
        }

        value = default;
        return false;
    }

    public double this[T key]
    {
        get { return Areas[renderer[key]]; }
    }

    public Dictionary<T, int>.KeyCollection Keys
    {
        get { return renderer.Keys; }
    }

    public List<double> Values
    {
        get { return Areas; }
    }

    IEnumerable<T> IReadOnlyDictionary<T, double>.Keys
    {
        get { return renderer.Keys; }
    }

    IEnumerable<double> IReadOnlyDictionary<T, double>.Values
    {
        get { return Areas; }
    }

    public struct Enumerator : IEnumerator<KeyValuePair<T, double>>, IDictionaryEnumerator
    {
        private readonly Renderer<T> renderer;
        private readonly List<double> areas;
        private Dictionary<T, int>.KeyCollection.Enumerator keys;
        private List<double>.Enumerator values;

        internal Enumerator([NotNull] Renderer<T> renderer, [NotNull] List<double> areas)
        {
            if (renderer is null)
                throw new ArgumentNullException(nameof(renderer));
            if (areas is null)
                throw new ArgumentNullException(nameof(areas));

            this.renderer = renderer;
            this.areas = areas;
            keys = renderer.Keys.GetEnumerator();
            values = areas.GetEnumerator();
        }

        public void Dispose()
        {
            keys.Dispose();
            values.Dispose();
        }

        public bool MoveNext()
        {
            return keys.MoveNext() && values.MoveNext();
        }

        void IEnumerator.Reset()
        {
            // fresh ones: the struct enumerators reset through IEnumerator are boxed copies, leaving these where they were
            keys.Dispose();
            values.Dispose();
            keys = renderer.Keys.GetEnumerator();
            values = areas.GetEnumerator();
        }

        public KeyValuePair<T, double> Current
        {
            get { return new KeyValuePair<T, double>(keys.Current, values.Current); }
        }

        object IEnumerator.Current
        {
            get { return Current; }
        }

        public T Key
        {
            get { return keys.Current; }
        }

        public double Value
        {
            get { return values.Current; }
        }

        object IDictionaryEnumerator.Key
        {
            get { return keys.Current; }
        }

        object IDictionaryEnumerator.Value
        {
            get { return values.Current; }
        }

        DictionaryEntry IDictionaryEnumerator.Entry
        {
            // ReSharper disable once AssignNullToNotNullAttribute
            get { return new DictionaryEntry(Key, Value); }
        }
    }
}
