using System;
using System.Collections;
using System.Collections.Concurrent;
using System.Threading;
using UnityEngine;

namespace FerramAerospaceResearch.Threading
{
    /// <summary>
    ///     MonoBehaviour object that can be used to run tasks or start coroutines from threads different to the main.
    /// </summary>
    [FARAddon(0, true)]
    public class MainThread : MonoSingleton<MainThread>
    {
        private static readonly ConcurrentQueue<CoroutineTask> routines = new ConcurrentQueue<CoroutineTask>();
        private static readonly ConcurrentQueue<Task> tasks = new ConcurrentQueue<Task>();

        public static bool IsMainThread
        {
            get { return Thread == Thread.CurrentThread; }
        }

        public static Thread Thread { get; private set; }

        protected override void OnAwake()
        {
            // Awake is always called in main thread
            Thread = Thread.CurrentThread;
        }

        private static void CheckStatus()
        {
            FARLogger.Assert(Thread != null, "Calling main thread task before initialization");
        }

        // Unused: nothing in FAR calls it (commented out after the test suite's coverage run)
        // /// <summary>
        // ///     Run action on the main thread and wait for its execution.
        // /// </summary>
        // /// <param name="action">Action to run on the main thread</param>
        // public static Task StartTaskWait(Action action)
        // {
        //     CheckStatus();
        //     Task task = StartTask(action);
        //     Wait(task);
        //     return task;
        // }

        public static CoroutineTask StartCoroutine(Func<IEnumerator> routine)
        {
            CheckStatus();
            var task = new CoroutineTask {Action = routine};
            if (IsMainThread)
                Instance.ExecuteTask(task);
            else
                routines.Enqueue(task);
            return task;
        }

        /// <summary>
        ///     Run action on the main thread. If current thread is main, action is executed immediately, otherwise it is only
        ///     queued and the method returns.
        /// </summary>
        /// <param name="action">Action to run on the main thread</param>
        /// <returns>The queued task</returns>
        public static Task StartTask(Action action)
        {
            CheckStatus();
            var task = new Task {Action = action};

            if (IsMainThread)
            {
                ExecuteTask(task);
                return task;
            }

            tasks.Enqueue(task);
            return task;
        }

        public static void Wait(TaskBase task)
        {
            CheckStatus();
            if (task is null)
                return;

            lock (task)
            {
                while (!task.Executed)
                    Monitor.Wait(task);
            }
        }

        public static void ExecuteTask(Task task)
        {
            try
            {
                task.Action();
            }
            catch (Exception e)
            {
                FARLogger.Exception(e, "Caught exception while executing task");
            }

            CompleteTask(task);
        }

        public void ExecuteTask(CoroutineTask task)
        {
            try
            {
                task.Result = StartCoroutine(task.Action());
            }
            catch (Exception e)
            {
                FARLogger.Exception(e, "Caught exception while starting coroutine");
            }

            CompleteTask(task);
        }

        private static void CompleteTask(TaskBase task)
        {
            lock (task)
            {
                task.Executed = true;
                Monitor.PulseAll(task);
            }
        }

        private void Update()
        {
            while (tasks.TryDequeue(out Task task))
                ExecuteTask(task);
            while (routines.TryDequeue(out CoroutineTask task))
                ExecuteTask(task);
        }

        public class TaskBase
        {
            public volatile bool Executed;
        }

        public class Task : TaskBase
        {
            public Action Action;
        }

        public class CoroutineTask : TaskBase
        {
            public Func<IEnumerator> Action;
            public Coroutine Result;

            public void Cancel()
            {
                if (Result is null)
                    return;

                Instance.StopCoroutine(Result);
            }
        }
    }
}
