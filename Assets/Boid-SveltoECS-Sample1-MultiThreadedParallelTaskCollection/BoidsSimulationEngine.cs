using System;
using System.Collections;
using Svelto.ECS;
using Svelto.Tasks;
using Svelto.Tasks.Parallelism;
using Svelto.Utilities;
using UnityEngine;

namespace Boid.SveltoECS.Sample1
{
    /// <summary>
    /// This Svelto.ECS engine takes care of the running of the Svelto.Tasks to perform the simulation.
    /// has engine doesn't do much, beside setting them up and starting them
    /// </summary>
    public partial class BoidsSimulationEngine : IQueryingEntitiesEngine, IDisposable
    {
        public BoidsSimulationEngine(ThreadSynchronizationSignal synchronizationSignal, Param param, int boidCount)
        {
            this.param = param;
            
            //Svelto.Tasks is based on the concept of tasks and runners to run them. Runners can be many. I may even
            //write a Unity Jobs runner one day. The MultiThreadRunner is the Svelto.Tasks runner to run tasks
            //on other threads. For more information please check my Svelto.Tasks articles.
            //this runner will run the main loop
            _runner = new MultiThreadRunner("BoidSimulationSystem", 1);
            //MultiThreadedParallelTaskCollection enables massive parallelism on the CPU. I will create a Svelto.Tasks
            //job for each boid. These jobs will be then spread on 16 threads on an I7. 
            _parallelJobs = new MultiThreadedParallelTaskCollection("BoidsSimulationJobs", (uint) (System.Environment.ProcessorCount * 2), true);
            //Svelto Tasks Jobs must be struct, as they must be instantiated multiple times, the first one
            //will be used as blueprint fro the other ones
            var jobInstance = new BoidSimulationJob(this, boidCount);
            //Create boidCount jobs and add them
            _parallelJobs.Add(ref jobInstance, boidCount);
            
            //the Synchronization signal is used to send signals between threads and allow synchronizations
            _synchronizationSignal = synchronizationSignal;
            _boidCount = boidCount;
        }

        public IEntitiesDB entitiesDB { get; set; }
        public Param param { get; }

        public void Ready()
        {
            WallJob().RunOnScheduler(_runner);
            FetchDeltaTime().RunOnScheduler(StandardSchedulers.updateScheduler);
        }

        /// <summary>
        /// just fetch Unity delta time
        /// </summary>
        /// <returns></returns>
        IEnumerator FetchDeltaTime()
        {
            while (true)
            {
                ThreadUtility.VolatileWrite(ref _deltaTime, Time.deltaTime);
                yield return null;
            }
        }

        /// <summary>
        /// Svelto.Tasks exploit the power of iteration blocks and the yield instruction. 
        /// </summary>
        /// <returns></returns>
        IEnumerator WallJob()
        {
            int count = 0;
            //wait until all the entities are created
            while (entitiesDB.Exists(GAME_GROUPS.BOIDS_GROUP) == false) yield return null;
            while (count < _boidCount)
            {
                entities = entitiesDB.QueryEntities<BoidEntityStruct>(GAME_GROUPS.BOIDS_GROUP, out count);
                
                yield return null;
            }
            
            //runs as long as the appplication runs
            while (true)
            {
                //wait for the main thread to signal that it's time to process the jobs
                yield return _synchronizationSignal.WaitBack();
                //process the jobs and wait until they are done
                yield return _parallelJobs;
                //signal the main thread that it can now synch the boids and restart the loop                
                _synchronizationSignal.Signal();
            }
        }
        
        public void Dispose()
        {
            _parallelJobs.Dispose();
            _runner.Dispose();
        }

        static BoidEntityStruct[]          entities;
        readonly ThreadSynchronizationSignal _synchronizationSignal;
        
        static float _deltaTime;

        readonly MultiThreadedParallelTaskCollection _parallelJobs;
        readonly MultiThreadRunner                   _runner;
        int _boidCount;
    }
}