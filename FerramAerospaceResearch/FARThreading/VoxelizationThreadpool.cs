/*
Ferram Aerospace Research v0.16.2.0 "Maskell"
=========================
Aerodynamics model for Kerbal Space Program

Copyright 2022, Michael Ferrara, aka Ferram4

   This file is part of Ferram Aerospace Research.

   Ferram Aerospace Research is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Ferram Aerospace Research is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with Ferram Aerospace Research.  If not, see <http://www.gnu.org/licenses/>.

   Serious thanks:		a.g., for tons of bugfixes and code-refactorings
				stupid_chris, for the RealChuteLite implementation
            			Taverius, for correcting a ton of incorrect values
				Tetryds, for finding lots of bugs and issues and not letting me get away with them, and work on example crafts
            			sarbian, for refactoring code for working with MechJeb, and the Module Manager updates
            			ialdabaoth (who is awesome), who originally created Module Manager
                        	Regex, for adding RPM support
				DaMichel, for some ferramGraph updates and some control surface-related features
            			Duxwing, for copy editing the readme

   CompatibilityChecker by Majiir, BSD 2-clause http://opensource.org/licenses/BSD-2-Clause

   Part.cfg changes powered by sarbian & ialdabaoth's ModuleManager plugin; used with permission
	http://forum.kerbalspaceprogram.com/threads/55219

   ModularFLightIntegrator by Sarbian, Starwaster and Ferram4, MIT: http://opensource.org/licenses/MIT
	http://forum.kerbalspaceprogram.com/threads/118088

   Toolbar integration powered by blizzy78's Toolbar plugin; used with permission
	http://forum.kerbalspaceprogram.com/threads/60863
 */

using System;
using System.Collections.Generic;
using System.Threading;

namespace FerramAerospaceResearch.FARThreading
{
    //This class only exists to ensure that the ThreadPool is not choked with requests to start running voxels, which will deadlock the entire voxelization process when the MaxThread limit is reached because they will be unable to start up their various worker threads
    internal class VoxelizationThreadpool
    {
        private const int THREAD_COUNT = 8;

        public static readonly VoxelizationThreadpool Instance = new VoxelizationThreadpool();

        public static bool RunInMainThread = false;

        private readonly Queue<Task> queuedMainThreadTasks;

        private readonly Thread[] _threads;
        private readonly Queue<Action> queuedVoxelizations;

        private Thread _mainThread;

        // Explicit static constructor to tell C# compiler not to mark type as beforefieldinit
        static VoxelizationThreadpool()
        {
        }

        private VoxelizationThreadpool()
        {
            _threads = new Thread[THREAD_COUNT];
            queuedVoxelizations = new Queue<Action>();
            queuedMainThreadTasks = new Queue<Task>();
            for (int i = 0; i < _threads.Length; i++)
            {
                _threads[i] = new Thread(ExecuteQueuedVoxelization);
                _threads[i].Start();
            }

            // make sure we get main thread while in main thread, ctor is not guaranteed to be run in main thread
            queuedMainThreadTasks.Enqueue(new Task(SetupMainThread));
        }

        public bool inMainThread
        {
            get { return _mainThread == Thread.CurrentThread; }
        }

        ~VoxelizationThreadpool()
        {
            for (int i = 0; i < _threads.Length; i++)
                //this will pass a null action to each thread, ending it
                QueueVoxelization(null);
        }

        private void SetupMainThread()
        {
            _mainThread = Thread.CurrentThread;
        }

        private void ExecuteQueuedVoxelization()
        {
            while (true)
            {
                Action task;
                lock (this)
                {
                    while (queuedVoxelizations.Count == 0)
                        Monitor.Wait(this);

                    task = queuedVoxelizations.Dequeue();
                }

                if (task != null)
                    task();
                else
                    break;
            }
        }

        public void QueueVoxelization(Action voxelAction)
        {
            lock (this)
            {
                queuedVoxelizations.Enqueue(voxelAction);
                Monitor.Pulse(this);
            }
        }

        public void RunOnMainThread(Action action)
        {
            if (inMainThread)
            {
                action();
                return;
            }

            var task = new Task(action);
            lock (queuedMainThreadTasks)
            {
                queuedMainThreadTasks.Enqueue(task);
            }

            lock (task)
            {
                while (!task.Executed)
                    Monitor.Wait(task);
            }
        }

        public void ExecuteMainThreadTasks()
        {
            while (true)
            {
                Task task;
                lock (queuedMainThreadTasks)
                {
                    if (queuedMainThreadTasks.Count == 0)
                        break;
                    task = queuedMainThreadTasks.Dequeue();
                }

                task.Action();
                lock (task)
                {
                    task.Executed = true;
                    Monitor.Pulse(task);
                }
            }
        }

        public class Task
        {
            public readonly Action Action;
            public bool Executed;

            public Task(Action action)
            {
                Action = action;
                Executed = false;
            }
        }
    }
}
