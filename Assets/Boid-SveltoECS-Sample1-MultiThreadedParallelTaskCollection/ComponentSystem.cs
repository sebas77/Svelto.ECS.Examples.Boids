using System.Collections;
using Svelto.ECS;
using Svelto.Tasks;
using Svelto.Tasks.Enumerators;
using Svelto.Tasks.Parallelism;
using Svelto.Utilities;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;

namespace Boid.SveltoECS.SampleD
{
    public class ThreadSynchronizationSignal : WaitForSignalEnumerator<ThreadSynchronizationSignal>
    {
        public ThreadSynchronizationSignal(string name, float timeout = 1000, bool autoreset = true) : base(name, timeout, autoreset)
        {
        }
    }
    
    public class BoidsSyncronizationEngine:IQueryingEntitiesEngine
    {
        public IEntitiesDB entitiesDB { get; set; }
        
        public BoidsSyncronizationEngine(ComponentGroup unityEcSgroup, ThreadSynchronizationSignal synchronizationSignal)
        {
            _unityECSgroup = unityEcSgroup;
            _synchronizationSignal = synchronizationSignal;
        }
        
        public void Ready()
        {
            SynchronizeUnityECSEntitiesWithSveltoECSEntities().RunOnScheduler(StandardSchedulers.updateScheduler);
        }

        IEnumerator SynchronizeUnityECSEntitiesWithSveltoECSEntities()
        {
            while (true)
            {
                yield return _synchronizationSignal;
                
                int count;
                var entities = entitiesDB.QueryEntities<BoidEntityStruct>(GAME_GROUPS.BOIDS_GROUP, out count);
                var position = _unityECSgroup.GetComponentDataArray<Position>();
                var rotation = _unityECSgroup.GetComponentDataArray<Rotation>();
                
                for (int i = 0; i < count; ++i)
                {
                    position[i] = new Position()
                        {Value = new float3(entities[i].position.x, entities[i].position.y, entities[i].position.z)};
                    rotation[i] = new Rotation() 
                        { Value = new quaternion(entities[i].rotation.x, entities[i].rotation.y, entities[i].rotation.z, 
                                                 entities[i].rotation.w)};
                }

                _synchronizationSignal.SignalBack();
                
                yield return null;
            }
        }

        readonly ComponentGroup _unityECSgroup;
        readonly ThreadSynchronizationSignal _synchronizationSignal;
    }
    
    public partial class BoidsSimulationSystem : IQueryingEntitiesEngine
    {
        public BoidsSimulationSystem(ThreadSynchronizationSignal synchronizationSignal)
        {
            collection = new MultiThreadedParallelTaskCollection(16, true);
            _synchronizationSignal = synchronizationSignal;
            var jobInstance = new BoidSimulationJob(this);
            collection.Add(ref jobInstance, Bootstrap.Instance.boidCount);
        }
        
        public IEntitiesDB entitiesDB { get; set; }
        public void Ready()
        {
            WallJob().RunOnScheduler(StandardSchedulers.multiThreadScheduler);
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

        IEnumerator WallJob()
        {
            //wait until all the entities are created
            int count = 0;
            while (entitiesDB.Exists(GAME_GROUPS.BOIDS_GROUP) == false) yield return null;
            while (count < Bootstrap.Instance.boidCount)
            {
                entities = entitiesDB.QueryEntities<BoidEntityStruct>(GAME_GROUPS.BOIDS_GROUP, out count);
                
                yield return null;
            }

            while (true)
            {
                _synchronizationSignal.WaitBack().Complete();
                
                yield return collection;
                
                _synchronizationSignal.Signal();
            }
        }

        static BoidEntityStruct[]          entities;
        readonly ThreadSynchronizationSignal _synchronizationSignal;
        
        static float _deltaTime;
        MultiThreadedParallelTaskCollection collection;
    }
}