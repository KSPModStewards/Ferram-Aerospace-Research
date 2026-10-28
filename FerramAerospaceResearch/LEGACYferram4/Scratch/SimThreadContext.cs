/*
Ferram Aerospace Research
=========================
Fast per-thread slot assignment for the parallel editor sweep.

   This file is part of Ferram Aerospace Research.

   Ferram Aerospace Research is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */

using System;
using System.Collections.Concurrent;
using System.Threading;

// ReSharper disable once CheckNamespace
namespace ferram4
{
    /// <summary>
    /// Hands each parallel unit of work a small dense slot index so the aero solve can hold per-thread
    /// scratch in plain arrays indexed by <see cref="Slot"/>. This replaces <see cref="System.Threading.ThreadLocal{T}"/>
    /// on the hot path: under KSP's Mono runtime ThreadLocal.Value is slow enough that reading it
    /// millions of times per sweep erased (and reversed) the parallel speedup. A <c>[ThreadStatic]</c>
    /// int plus an array index is effectively free.
    ///
    /// Slots come from a POOL: each row acquires one with <see cref="AcquireSlot"/> on entry and returns
    /// it with <see cref="ReleaseSlot"/> on exit. So the live-slot count is bounded by the CONCURRENCY,
    /// not by how many distinct threads the pool cycles through - which matters because an
    /// allocation-heavy sweep makes the thread pool inject and retire workers over its lifetime. A
    /// per-thread counter would grow past the array size and two threads would end up sharing a slot,
    /// corrupting rows; the pool never hands the same slot to two live workers.
    ///
    /// Serial callers (the stability sweep, the static/stability tabs, flight aero) never acquire and
    /// simply read slot 0 - safe because they run on the main thread. Slot 0 is never in a pool: the main
    /// thread keeps evaluating on it (flight, the editor's center-of-lift marker and tabs) while a sweep runs.
    /// </summary>
    public static class SimThreadContext
    {
        /// <summary>Upper bound the scratch arrays are sized to. The pool only uses ~concurrency of them.</summary>
        public const int MaxSlots = 64;

        [ThreadStatic] private static int slot;
        [ThreadStatic] private static SlotPool heldFrom;

        /// <summary>
        /// A set of slots handed out one per concurrent unit of work. Taking one waits while all are in use,
        /// so two live workers never share a slot.
        /// </summary>
        private sealed class SlotPool
        {
            private readonly ConcurrentStack<int> free = new ConcurrentStack<int>();
            private readonly SemaphoreSlim available;

            public SlotPool(int first, int count)
            {
                for (int i = first + count - 1; i >= first; i--)
                    free.Push(i);
                available = new SemaphoreSlim(count, count);
            }

            public int Take()
            {
                available.Wait();
                free.TryPop(out int s); // the semaphore guarantees one is there
                return s;
            }

            public void Give(int s)
            {
                free.Push(s);
                available.Release();
            }
        }

        private static SlotPool sweepPool = new SlotPool(1, 1);

        /// <summary>The calling thread's current slot; 0 when it holds none (serial contexts).</summary>
        public static int Slot => slot;

        [ThreadStatic] private static bool inSweep;

        /// <summary>
        /// Whether the calling thread holds a sweep slot (between <see cref="AcquireSlot"/> and
        /// <see cref="ReleaseSlot"/>). Only then does the solve read the
        /// sweep's frozen state (a part's SimGeom, the editor CoM and facility, the adiabatic index override); the
        /// main thread's own evaluations during a sweep (the center-of-lift marker, the serial tabs) read the live
        /// game as they do outside one.
        /// </summary>
        public static bool InSweep => inSweep;

        /// <summary>
        /// How many slots a sweep hands out: one per possible concurrent worker plus a small margin. They are
        /// slots 1 to SweepPoolSize, so an array indexed by a sweep worker's slot needs SweepPoolSize + 1 entries.
        /// </summary>
        public static int SweepPoolSize => Math.Max(1, Math.Min(MaxSlots - 1, Environment.ProcessorCount + 2));

        /// <summary>
        /// A new pool for a sweep, slots 1 to <see cref="SweepPoolSize"/>. Main thread, before the parallel loop
        /// and with no worker of an earlier sweep still running.
        /// </summary>
        public static void BeginSweep()
        {
            sweepPool = new SlotPool(1, SweepPoolSize);

            slot = 0;
            heldFrom = null;
            inSweep = false;
        }

        /// <summary>
        /// Claims a slot for this thread for the duration of one parallel iteration, waiting while all of the
        /// sweep's slots are in use (it used to fall back to slot 0, the main thread's).
        /// </summary>
        public static void AcquireSlot()
        {
            SlotPool pool = sweepPool;
            slot = pool.Take();
            heldFrom = pool;
            inSweep = true;
        }

        /// <summary>Returns this thread's slot to the pool. Pair with <see cref="AcquireSlot"/>.</summary>
        public static void ReleaseSlot()
        {
            if (heldFrom != null)
            {
                heldFrom.Give(slot);
                heldFrom = null;
            }

            slot = 0;
            inSweep = false;
        }
    }
}
