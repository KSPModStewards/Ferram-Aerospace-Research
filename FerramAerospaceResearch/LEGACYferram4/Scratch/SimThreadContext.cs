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
using FerramAerospaceResearch.Threading;
using UnityEngine;

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

        /// <summary>
        /// Slots kept for flight predictions (FARVesselAero.SimulateAeroProperties): the top of the range, never slot 0,
        /// and never handed out by the sweep pool, so a prediction cannot touch the flight's state or a sweep's. The
        /// last of them is the main thread's alone, so the main thread never waits for another thread's prediction.
        /// </summary>
        public const int PredictionSlots = 16;

        /// <summary>The prediction slot kept for the main thread.</summary>
        public const int MainThreadPredictionSlot = MaxSlots - 1;

        [ThreadStatic] private static int slot;
        [ThreadStatic] private static SlotPool heldFrom;
        [ThreadStatic] private static PredictionCall prediction;

        // Only the main thread reads or writes this.
        private static bool mainThreadSlotInUse;

        /// <summary>
        /// A set of slots handed out one per concurrent unit of work. Taking one waits while all are in use,
        /// so two live workers never share a slot.
        /// </summary>
        internal sealed class SlotPool
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

            /// <summary>A free slot without waiting, or false when all are in use.</summary>
            public bool TryTake(out int s)
            {
                s = 0;
                if (!available.Wait(0))
                    return false;
                free.TryPop(out s);
                return true;
            }

            public void Give(int s)
            {
                free.Push(s);
                available.Release();
            }
        }

        private static SlotPool sweepPool = new SlotPool(1, 1);
        // Other threads' prediction slots: all of PredictionSlots but the main thread's.
        private static readonly SlotPool predictionPool = new SlotPool(MaxSlots - PredictionSlots, PredictionSlots - 1);

        private static long lastStamp;

        /// <summary>
        /// The flight prediction the calling thread is inside, or null: the airflow it simulates (what a wing's
        /// GetVelocity reports to its neighbours) and the external temperature its skin friction takes.
        /// </summary>
        public sealed class PredictionCall
        {
            public Vector3d Velocity;
            public double? ExternalTemperature;

            /// <summary>The adiabatic index the wing and shock math takes (FARAeroUtil.CurrentAdiabaticIndex), if set.</summary>
            public double? AdiabaticIndex;

            /// <summary>The adiabatic index a wing's skin friction takes (the vessel's, in flight), if set.</summary>
            public double? VesselAdiabaticIndex;

            /// <summary>The body's reference viscosity and temperature (FARAeroUtil.currentBodyData), if set.</summary>
            public FerramAerospaceResearch.Settings.BodySettings? BodyData;

            /// <summary>
            /// Nonzero for a prediction made from a vessel snapshot: the parts' captured state (geometry, masses,
            /// shielding, ...) installed on the slot for this call carries the same stamp, and the solve reads it
            /// instead of the live parts. Zero for a prediction from the live vessel.
            /// </summary>
            public long Stamp;

            internal int Slot;
            internal bool OnMainThreadSlot;
            internal int PreviousSlot;
            internal SlotPool PreviousHeldFrom;
            internal PredictionCall Previous;
        }

        public static PredictionCall Prediction => prediction;

        /// <summary>A new stamp for a snapshot prediction (see <see cref="PredictionCall.Stamp"/>).</summary>
        public static long NewStamp()
        {
            return Interlocked.Increment(ref lastStamp);
        }

        /// <summary>The stamp of the snapshot prediction the calling thread is inside, or 0.</summary>
        public static long SnapshotStamp
        {
            get
            {
                PredictionCall call = prediction;
                return call != null ? call.Stamp : 0;
            }
        }

        /// <summary>
        /// Moves the calling thread onto a prediction slot for the duration of one prediction; pair with
        /// <see cref="EndPrediction"/> in a finally. The main thread takes the slot kept for it, or, inside a
        /// prediction it is already making, a free one of the others without waiting; it gets null rather than wait
        /// for another thread. Another thread waits while all of theirs are in use.
        /// </summary>
        public static PredictionCall BeginPrediction(Vector3d velocity)
        {
            int s;
            bool mainSlot = false;
            if (MainThread.IsMainThread)
            {
                if (!mainThreadSlotInUse)
                {
                    s = MainThreadPredictionSlot;
                    mainThreadSlotInUse = mainSlot = true;
                }
                else if (!predictionPool.TryTake(out s))
                {
                    return null;
                }
            }
            else
            {
                s = predictionPool.Take();
            }

            var call = new PredictionCall
            {
                Velocity = velocity,
                Slot = s,
                OnMainThreadSlot = mainSlot,
                PreviousSlot = slot,
                PreviousHeldFrom = heldFrom,
                Previous = prediction
            };
            slot = s;
            heldFrom = null;
            prediction = call;
            return call;
        }

        /// <summary>Puts the calling thread back on the slot it had before <see cref="BeginPrediction"/>.</summary>
        public static void EndPrediction(PredictionCall call)
        {
            slot = call.PreviousSlot;
            heldFrom = call.PreviousHeldFrom;
            prediction = call.Previous;
            if (call.OnMainThreadSlot)
                mainThreadSlotInUse = false;
            else
                predictionPool.Give(call.Slot);
        }

        /// <summary>The calling thread's current slot; 0 when it holds none (serial contexts).</summary>
        public static int Slot => slot;

        [ThreadStatic] private static bool inSweep;

        /// <summary>
        /// Whether the calling thread holds a sweep slot (between <see cref="AcquireSlot"/> and
        /// <see cref="ReleaseSlot"/>), outside any prediction it makes there. Only then does the solve read the
        /// sweep's frozen state (a part's SimGeom, the editor CoM and facility, the adiabatic index override); the
        /// main thread's own evaluations during a sweep (the center-of-lift marker, the serial tabs) read the live
        /// game as they do outside one.
        /// </summary>
        public static bool InSweep => inSweep && prediction == null;

        /// <summary>
        /// Neither in a prediction nor on a sweep worker: everything the solve reads is the live game state (the
        /// flight, the serial editor tabs). The hot paths check this first, the cheap way to the common case.
        /// </summary>
        public static bool LiveOnly => prediction == null && !inSweep;

        /// <summary>
        /// How many slots a sweep hands out: one per possible concurrent worker plus a small margin. They are
        /// slots 1 to SweepPoolSize, so an array indexed by a sweep worker's slot needs SweepPoolSize + 1 entries.
        /// </summary>
        public static int SweepPoolSize => Math.Max(1, Math.Min(MaxSlots - 1 - PredictionSlots, Environment.ProcessorCount + 2));

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
