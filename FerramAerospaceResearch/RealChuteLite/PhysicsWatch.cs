using System;

// ReSharper disable UnusedMember.Global

/* RealChuteLite is the work of Christophe Savard (stupid_chris), and is licensed the same way than the rest of FAR is.
 * If you have any questions about this code, or want to report something, don't annoy ferram about it, ask me
 * directly on GitHub, the forums, or IRC. */

namespace FerramAerospaceResearch.RealChuteLite
{
    /// <summary>
    ///     A generic Stopwatch clone which runs on KSP's internal clock
    /// </summary>
    public sealed class PhysicsWatch
    {
        /// <summary>
        ///     The amount of ticks in a second
        /// </summary>
        private const long ticksPerSecond = 10000000L;

        /// <summary>
        ///     The amount of milliseconds in a second
        /// </summary>
        private const long millisecondPerSecond = 1000L;

        /// <summary>
        ///     UT of the last frame
        /// </summary>
        private double lastCheck;

        /// <summary>
        ///     Total elapsed time calculated by the watch in seconds
        /// </summary>
        private double totalSeconds;

        /// <summary>
        ///     Creates a new PhysicsWatch
        /// </summary>
        public PhysicsWatch()
        {
        }

        /// <summary>
        ///     Creates a new PhysicsWatch starting at a certain amount of time
        /// </summary>
        /// <param name="seconds">Time to start at, in seconds</param>
        public PhysicsWatch(double seconds)
        {
            totalSeconds = seconds;
        }

        /// <summary>
        ///     If the watch is currently counting down time
        /// </summary>
        public bool IsRunning { get; private set; }

        /// <summary>
        ///     The current elapsed time of the watch
        /// </summary>
        public TimeSpan Elapsed
        {
            get { return new TimeSpan(ElapsedTicks); }
        }

        /// <summary>
        ///     The amount of milliseconds elapsed to the current watch
        /// </summary>
        public long ElapsedMilliseconds
        {
            get
            {
                if (IsRunning)
                    UpdateWatch();
                return (long)Math.Round(totalSeconds * millisecondPerSecond);
            }
        }

        /// <summary>
        ///     The amount of ticks elapsed to the current watch
        /// </summary>
        public long ElapsedTicks
        {
            get
            {
                if (IsRunning)
                    UpdateWatch();
                return (long)Math.Round(totalSeconds * ticksPerSecond);
            }
        }

        /// <summary>
        ///     Starts the watch
        /// </summary>
        public void Start()
        {
            if (IsRunning)
                return;
            lastCheck = Planetarium.GetUniversalTime();
            IsRunning = true;
        }

        /// <summary>
        ///     Stops the watch
        /// </summary>
        public void Stop()
        {
            if (!IsRunning)
                return;
            UpdateWatch();
            IsRunning = false;
        }

        /// <summary>
        ///     Resets the watch to zero and starts it
        /// </summary>
        public void Restart()
        {
            totalSeconds = 0;
            lastCheck = Planetarium.GetUniversalTime();
            IsRunning = true;
        }

        /// <summary>
        ///     Stops the watch and resets it to zero
        /// </summary>
        public void Reset()
        {
            totalSeconds = 0;
            lastCheck = 0;
            IsRunning = false;
        }

        /// <summary>
        ///     Updates the time on the watch
        /// </summary>
        private void UpdateWatch()
        {
            double current = Planetarium.GetUniversalTime();
            totalSeconds += current - lastCheck;
            lastCheck = current;
        }

        /// <summary>
        ///     Returns a string representation fo this instance
        /// </summary>
        public override string ToString()
        {
            return Elapsed.ToString();
        }

        /// <summary>
        ///     Creates a new PhysicsWatch, starts it, and returns the current instance
        /// </summary>
        public static PhysicsWatch StartNew()
        {
            var watch = new PhysicsWatch();
            watch.Start();
            return watch;
        }

        // Unused: nothing in FAR calls it (commented out after the test suite's coverage run)
        // /// <summary>
        // ///     Creates a new PhysicsWatch from a certain amount of time, starts it, and returns the current instance
        // /// </summary>
        // /// <param name="seconds">Time to start the watch at, in seconds</param>
        // public static PhysicsWatch StartNewFromTime(double seconds)
        // {
        //     var watch = new PhysicsWatch(seconds);
        //     watch.Start();
        //     return watch;
        // }
    }
}
