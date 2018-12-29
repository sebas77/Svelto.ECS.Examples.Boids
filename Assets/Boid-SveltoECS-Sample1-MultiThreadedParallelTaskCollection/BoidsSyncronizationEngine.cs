using System.Collections;
using Svelto.ECS;
using Svelto.Tasks;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;

namespace Boid.SveltoECS.Sample1
{
    public class BoidsSyncronizationEngine:IQueryingEntitiesEngine
    {
        public IEntitiesDB entitiesDB { get; set; }
        
        public BoidsSyncronizationEngine(ComponentGroup unityEcSgroup, ThreadSynchronizationSignal synchronizationSignal)
        {
            _unityECSgroup         = unityEcSgroup;
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
#if !JUST_RUN_IT
                //Lock the main thread and force waiting for the Svelto.Tasks threaded jobs to finish. This is not
                //a normal operation and it's here just to measure the frame rate.
                _synchronizationSignal.Complete();
#else           
                //this is the normal not blocking operation. The frame rate will be as fast as the unity main thread
                //can be, but the boids will still update at the real execution rate
                yield return _synchronizationSignal;
#endif    
                
                int count;
                //fetch the Svelto.ECS entities
                var entities = entitiesDB.QueryEntities<BoidEntityStruct>(GAME_GROUPS.BOIDS_GROUP, out count);
                //fetch the Unity ECS components
                var position = _unityECSgroup.GetComponentDataArray<Position>();
                var rotation = _unityECSgroup.GetComponentDataArray<Rotation>();
                
                //synchronize!
                for (int i = 0; i < count; ++i)
                {
                    position[i] = new Position()
                        {Value = new float3(entities[i].position.x, entities[i].position.y, entities[i].position.z)};
                    rotation[i] = new Rotation() 
                    { Value = new quaternion(entities[i].rotation.x, entities[i].rotation.y, entities[i].rotation.z, 
                                             entities[i].rotation.w)};
                }

                //tell to the Svelto.Tasks threads that they can carry on with the next iteration
                _synchronizationSignal.SignalBack();

                //yield one frame so the while (true) will not enter in an infinite loop
                yield return null;
            }
        }

        readonly ComponentGroup              _unityECSgroup;
        readonly ThreadSynchronizationSignal _synchronizationSignal;
    }
}
