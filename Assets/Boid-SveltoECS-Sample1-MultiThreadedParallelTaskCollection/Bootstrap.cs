using Svelto.ECS;
using Svelto.ECS.Schedulers.Unity;
using UnityEngine;
using UnityEngine.Rendering;
using Unity.Entities;
using Unity.Transforms;
using Unity.Mathematics;
using Unity.Rendering;

namespace Boid.SveltoECS.Sample1
{
    public class Bootstrap : MonoBehaviour 
    {
        [SerializeField]
        public int boidCount = 100;
    
        [SerializeField]
        Vector3 boidScale = new Vector3(0.1f, 0.1f, 0.3f);
    
        [SerializeField]
        Param param;
    
        [SerializeField]
        Mesh mesh;
    
        [SerializeField]
        Material material;
    
        void OnDestroy()
        {
            _enginesRoot.Dispose();
        }

        void Start()
        {
            QualitySettings.vSyncCount = -1;
            Application.targetFrameRate = 120;
            
            UnityECS();
            SveltoECS();
        }

        void SveltoECS()
        {
            //creates the _enginesRoot, for more information read my svelto.ECS articles
            _enginesRoot = new EnginesRoot(new UnityEntitySubmissionScheduler());
            //this will be used to build entities
            var entityFactory = _enginesRoot.GenerateEntityFactory();
            var random = new Unity.Mathematics.Random(853);
            
            for (int i = 0; i < boidCount; ++i)
            {
                var initSpeed = param.initSpeed;

                //build entities in the BOIDS_GROUP exclusive group
                var entityStructInitializer = entityFactory.BuildEntity<BoidEntityDecriptor>(i, GAME_GROUPS.BOIDS_GROUP);
                var nextFloat3 = random.NextFloat3(1f);
                var float3 = random.NextFloat3Direction() * initSpeed;
                //init them with these default values
                entityStructInitializer.Init(new BoidEntityStruct()
                {
                    position = new SVector3 { x = nextFloat3.x, y = nextFloat3.y, z = nextFloat3.z },
                    rotation = new SVector4 { x = 0, y = 0, z = 0, w = 1},
                    velocity = new SVector3 { x = float3.x, y = float3.y, z = float3.z },
                    acceleration = new SVector3()
                });
            }
            
            //create a synchronization enumerator to be used inside the Svelto.Tasks
            ThreadSynchronizationSignal _signal = new ThreadSynchronizationSignal("name");
            
            //add the engines we are going to use
            _enginesRoot.AddEngine(new BoidsSyncronizationEngine(_unityECSGroup, _signal));
            _enginesRoot.AddEngine(new BoidsSimulationEngine(_signal, param, boidCount));
        }

        void UnityECS()
        {
            var manager = World.Active.GetOrCreateManager<EntityManager>();
            var archetype = manager.CreateArchetype(typeof(Position),
                                                    typeof(Rotation),
                                                    typeof(Scale),
                                                    typeof(MeshInstanceRenderer));
            
            _unityECSGroup = manager.CreateComponentGroup(archetype.ComponentTypes);
            
            var renderer = new MeshInstanceRenderer
            {
                castShadows    = ShadowCastingMode.On,
                receiveShadows = true,
                mesh           = mesh,
                material       = material
            };
            
            for (int i = 0; i < boidCount; ++i)
            {
                var entity = manager.CreateEntity(archetype);
                manager.SetComponentData(entity, new Scale { Value = new float3(boidScale.x, boidScale.y, boidScale.z) });
                manager.SetSharedComponentData(entity, renderer);
            }
        }

        void OnDrawGizmos()
        {
            if (!param) return;
            Gizmos.color = Color.green;
            Gizmos.DrawWireCube(Vector3.zero, Vector3.one * param.wallScale);
        }

        EnginesRoot    _enginesRoot;
        ComponentGroup _unityECSGroup;
    }
}