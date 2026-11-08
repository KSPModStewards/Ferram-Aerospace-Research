using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;

namespace FerramAerospaceResearch.Resources
{
    public struct Disposable<T> : IDisposable
    {
        private readonly Pool<T> pool;
        public T value;

        public Disposable(T value, Pool<T> owner)
        {
            this.value = value;
            pool = owner;
        }

        public void Dispose()
        {
            pool.Release(value);
        }
    }

    public class Pool<T> : IDisposable, IEnumerable<T>
    {
        private readonly Stack<T> objects = new();

        public static readonly Pool<T> Default = new();

        private readonly Func<T> create;
        private readonly Action<T> release;

        public Pool()
        {
            // C# generic constraints are unnecessarily too limiting so can't use new T() in a generic way without
            // imposing constraints, Activator has a large overhead but that shouldn't matter once the pool is filled
            create = () => (T)Activator.CreateInstance(typeof(T));
        }

        public Pool(Func<T> create, Action<T> release)
        {
            this.create = create;
            this.release = release;
        }

        public int Count
        {
            get { return objects.Count; }
        }

        public void Clear()
        {
            objects.Clear();
        }

        public T Acquire()
        {
            return objects.Count == 0 ? create() : objects.Pop();
        }

        public void Release(T obj)
        {
            release?.Invoke(obj);
            objects.Push(obj);
        }

        // Unused: nothing in FAR calls it (commented out after the test suite's coverage run)
        // public Disposable<T> Borrow()
        // {
        //     return new Disposable<T>(Acquire(), this);
        // }

        public Stack<T>.Enumerator GetEnumerator()
        {
            return objects.GetEnumerator();
        }

        IEnumerator<T> IEnumerable<T>.GetEnumerator()
        {
            return objects.GetEnumerator();
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            return ((IEnumerable)objects).GetEnumerator();
        }

        public virtual void Dispose()
        {
            foreach (T o in objects)
            {
                if (o is IDisposable d)
                    d.Dispose();
            }

            objects.Clear();
        }
    }

    public class ListPool<T> : Pool<List<T>>
    {
        private const int DefaultCapacity = 64;

        public ListPool() : base(() => new List<T>(DefaultCapacity),
                                 list =>
                                 {
                                     list.Clear();
                                     // prevent capacity from growing too much consuming memory
                                     list.Capacity = math.min(list.Capacity, DefaultCapacity);
                                 })
        {
        }

        public override void Dispose()
        {
            Clear();
        }
    }
}
