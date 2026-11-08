using System.Collections.Generic;
using System.Reflection;
using System.Threading;
using ferram4;
using FerramAerospaceResearch.Threading;
using Xunit;

namespace FerramAerospaceResearch.UnitTests
{
    /// <summary>
    /// SimThreadContext's prediction slots on the main thread (the test's thread, made FAR's main thread for the test):
    /// the first prediction takes the slot kept for the main thread, a prediction made inside it one of the other
    /// threads' slots without waiting, and once those are all in use it gets none (null) rather than wait; ending them
    /// in reverse puts the thread back on the slot it had and frees the slots for the next.
    /// </summary>
    [Collection(Gas.Collection)]
    public class SimThreadContextTests
    {
        private static readonly PropertyInfo mainThread = typeof(MainThread).GetProperty(nameof(MainThread.Thread));

        [SkippableFact]
        public void NestedMainThreadPredictionsTakeFreeSlotsUntilNoneAreLeft()
        {
            UnityRuntime.Require();   // Vector3d
            Thread saved = MainThread.Thread;
            mainThread.SetValue(null, Thread.CurrentThread);
            var calls = new Stack<SimThreadContext.PredictionCall>();
            try
            {
                Assert.True(MainThread.IsMainThread);
                Assert.Null(SimThreadContext.Prediction);
                int before = SimThreadContext.Slot;

                SimThreadContext.PredictionCall first = SimThreadContext.BeginPrediction(new Vector3d(1, 0, 0));
                Assert.NotNull(first);
                calls.Push(first);
                Assert.Equal(SimThreadContext.MainThreadPredictionSlot, SimThreadContext.Slot);
                Assert.Same(first, SimThreadContext.Prediction);
                Assert.Equal(1, SimThreadContext.Prediction.Velocity.x);
                Assert.False(SimThreadContext.LiveOnly);

                // the other threads' slots: the prediction range but the main thread's, each handed out once
                var slots = new HashSet<int> { SimThreadContext.Slot };
                for (int i = 0; i < SimThreadContext.PredictionSlots - 1; i++)
                {
                    SimThreadContext.PredictionCall nested = SimThreadContext.BeginPrediction(new Vector3d(0, i, 0));
                    Assert.NotNull(nested);
                    calls.Push(nested);
                    int s = SimThreadContext.Slot;
                    Assert.InRange(s, SimThreadContext.MaxSlots - SimThreadContext.PredictionSlots, SimThreadContext.MainThreadPredictionSlot - 1);
                    Assert.True(slots.Add(s), $"slot {s} handed out twice");
                    Assert.Same(nested, SimThreadContext.Prediction);
                }

                int last = SimThreadContext.Slot;
                Assert.Null(SimThreadContext.BeginPrediction(Vector3d.zero));
                Assert.Equal(last, SimThreadContext.Slot);

                // unwound in reverse: each end puts back the slot and the prediction from before it
                while (calls.Count > 0)
                {
                    SimThreadContext.PredictionCall call = calls.Pop();
                    SimThreadContext.EndPrediction(call);
                    if (calls.Count > 0)
                        Assert.Same(calls.Peek(), SimThreadContext.Prediction);
                }

                Assert.Null(SimThreadContext.Prediction);
                Assert.Equal(before, SimThreadContext.Slot);
                Assert.True(SimThreadContext.LiveOnly);

                // all freed: the main thread's slot and then a nested one again
                SimThreadContext.PredictionCall again = SimThreadContext.BeginPrediction(Vector3d.zero);
                calls.Push(again);
                Assert.Equal(SimThreadContext.MainThreadPredictionSlot, SimThreadContext.Slot);
                SimThreadContext.PredictionCall againNested = SimThreadContext.BeginPrediction(Vector3d.zero);
                Assert.NotNull(againNested);
                calls.Push(againNested);
            }
            finally
            {
                while (calls.Count > 0)
                    SimThreadContext.EndPrediction(calls.Pop());
                mainThread.SetValue(null, saved);
            }
        }
    }
}
