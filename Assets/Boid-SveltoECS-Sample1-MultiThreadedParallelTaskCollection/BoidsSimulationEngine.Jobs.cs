using Svelto.Tasks.Parallelism;
using Svelto.Utilities;
using Unity.Mathematics;
using UnityEngine;

namespace Boid.SveltoECS.Sample1
{
    public partial class BoidsSimulationEngine
    {
        /// <summary>
        /// ISveltoJob is a convenient  way to add jobs even if they are not enumerator. It's inspired by the
        /// Unity Jobs interface. 
        /// </summary>
        struct BoidSimulationJob : ISveltoJob
        {
            static float   _weight;
            static float   _scale;
            static float   _thresh;
            static float   _prodThresh;
            static float   _distThresh;
            static float3  _separationWeight;
            static float3  _alignmentWeight;
            static float3  _cohesionWeight;
            static int[][] _neighbours;
            static int     _boidsCount;
            static float   _minSpeed;
            static float   _maxSpeed;

            /// <summary>
            /// ISveltJob can be only structs, this will be called once and then the values just copied
            /// (although in this case they are static so it won't even happen)
            /// </summary>
            /// <param name="simulation"></param>
            /// <param name="boidCount"></param>
            public BoidSimulationJob(BoidsSimulationEngine simulation, int boidCount)
            {
                _scale  = simulation.param.wallScale * 0.5f;
                _thresh = simulation.param.wallDistance;
                _weight = simulation.param.wallWeight;
                _boidsCount = boidCount;
                
                _prodThresh       = math.cos(math.radians(simulation.param.neighborFov));
                _distThresh       = simulation.param.neighborDistance;
                _separationWeight = simulation.param.separationWeight;
                _alignmentWeight  = simulation.param.alignmentWeight;
                _cohesionWeight   = simulation.param.cohesionWeight;
                
                _minSpeed = simulation.param.minSpeed;
                _maxSpeed = simulation.param.maxSpeed;

                _neighbours = new int[_boidsCount][];
                for (int i = 0; i < _boidsCount; i++)
                    _neighbours[i] = new int[_boidsCount];
            }
            
            /// <summary>
            /// I am not really sure why the original author decided to chain Unity jobs, so I kept it simple
            /// and execute all of them in the same job.
            /// </summary>
            /// <param name="jobIndex"></param>
            public void Update(int jobIndex)
            {
                var volatileRead = ThreadUtility.VolatileRead(ref _deltaTime);
                
                Wall(entities, jobIndex, _scale, _thresh, _weight);
                
                var neighbours = _neighbours[jobIndex];
                var currentCount = DetectNeighbour(entities, jobIndex, _boidsCount, neighbours);
                if (currentCount > 0)
                {
                    Separation(entities, jobIndex, currentCount, neighbours);
                    AlignmentJob(entities, jobIndex, currentCount, neighbours);
                    CohesionJob(entities, jobIndex, currentCount, neighbours);
                }

                Move(entities, jobIndex, volatileRead);
            }

            static void Wall(BoidEntityStruct[] entities, int i, float scale, float thresh, float weight)
            {
                ref var pos = ref entities[i].position;
                ref var acc = ref entities[i].acceleration;

                float3 value = new float3(acc.x, acc.y, acc.z) +
                               GetAccelAgainstWall(-scale - pos.x, new float3(+1, 0, 0), thresh, weight) +
                               GetAccelAgainstWall(-scale - pos.y, new float3(0, +1, 0), thresh, weight) +
                               GetAccelAgainstWall(-scale - pos.z, new float3(0, 0, +1), thresh, weight) +
                               GetAccelAgainstWall(+scale - pos.x, new float3(-1, 0, 0), thresh, weight) +
                               GetAccelAgainstWall(+scale - pos.y, new float3(0, -1, 0), thresh, weight) +
                               GetAccelAgainstWall(+scale - pos.z, new float3(0, 0, -1), thresh, weight);

                acc.x = value.x;
                acc.y = value.y;
                acc.z = value.z;
            }

            static float3 GetAccelAgainstWall(float dist, float3 dir, float thresh, float weight)
            {
                if (dist < thresh)
                {
                    return dir * (weight / math.abs(dist / thresh));
                }

                return float3.zero;
            }

            static void Move(BoidEntityStruct[] entities, int i, float dt)
            {
                ref var lastVelocity     = ref entities[i].velocity;
                ref var accel = ref entities[i].acceleration;
                lastVelocity.x += accel.x * dt;
                lastVelocity.y += accel.y * dt;
                lastVelocity.z += accel.z * dt;
                var velocity = new float3(lastVelocity.x, lastVelocity.y, lastVelocity.z);
                var speed   = math.length(velocity);
                var dir     = velocity / speed;
                var vi      = math.clamp(speed, _minSpeed, _maxSpeed) * dir;
                lastVelocity.x = vi.x;
                lastVelocity.y = vi.y;
                lastVelocity.z = vi.z;
                ref var lastPos = ref entities[i].position;
                var     pos  = new float3(lastPos.x, lastPos.y, lastPos.z);
                var     newPos = pos + vi * dt;

                lastPos.x = newPos.x;
                lastPos.y = newPos.y;
                lastPos.z = newPos.z;
                var     lookRotationSafe = quaternion.LookRotationSafe(dir, new float3(0, 1, 0)).value;
                ref var rotation  = ref entities[i].rotation;
                rotation.x = lookRotationSafe.x;
                rotation.y = lookRotationSafe.y;
                rotation.z = lookRotationSafe.z;
                rotation.w = lookRotationSafe.w;

                accel.x = accel.y = accel.z = 0;
            }

            int DetectNeighbour(BoidEntityStruct[] entities, int iindex, int count, int[] _neighbours)
            {
                int currentCount = 0;
                
                ref var velocity = ref entities[iindex].velocity;
                ref var pos      = ref entities[iindex].position;

                float3 pos0 = pos.ToFloat3();
                float3 fwd0 = math.normalize(velocity.ToFloat3());

                for (int i = 0; i < count; ++i)
                {
                    float3 pos1 = entities[i].position.ToFloat3();
                    var    to   = pos1 - pos0;
                    var    dist = math.length(to);

                    if (dist < _distThresh)
                    {
                        var dir  = math.normalize(to);
                        var prod = Vector3.Dot(dir, fwd0);
                        if (prod > _prodThresh)
                        {
                            _neighbours[currentCount++] = i;
                        }
                    }
                }

                return currentCount;
            }

            void Separation(BoidEntityStruct[] entities, int iindex, int count, int[] _neighbours)
            {
                var pos0 = entities[iindex].position.ToFloat3();

                var force = float3.zero;
                for (int i = 0; i < count; ++i)
                {
                    var pos1 = entities[_neighbours[i]].position.ToFloat3();
                    force += math.normalize(pos0 - pos1);
                }

                force /= count;

                var dAccel = force * _separationWeight;

                ref var accel    = ref entities[iindex].acceleration;
                var     newaccel = accel.ToFloat3() + dAccel;
                accel.x = newaccel.x;
                accel.y = newaccel.y;
                accel.z = newaccel.z;
            }

            void AlignmentJob(BoidEntityStruct[]  entities, int iindex, int count, int[] _neighbours)
            {
                var averageVelocity = float3.zero;
                for (int i = 0; i < count; ++i)
                {
                    averageVelocity += entities[_neighbours[i]].velocity.ToFloat3();
                }

                averageVelocity /= count;

                var dAccel = (averageVelocity - entities[iindex].velocity.ToFloat3()) * _alignmentWeight;

                ref var accel    = ref entities[iindex].acceleration;
                var     newaccel = accel.ToFloat3() + dAccel;
                accel.x = newaccel.x;
                accel.y = newaccel.y;
                accel.z = newaccel.z;
            }

            void CohesionJob(BoidEntityStruct[] entities, int iindex, int count,int[] _neighbours)
            {
                var averagePos = float3.zero;
                for (int i = 0; i < count; ++i)
                {
                    averagePos += entities[_neighbours[i]].position.ToFloat3();
                }

                averagePos /= count;

                var dAccel = (averagePos - entities[iindex].position.ToFloat3()) * _cohesionWeight;

                ref var accel    = ref entities[iindex].acceleration;
                var     newaccel = accel.ToFloat3() + dAccel;
                accel.x = newaccel.x;
                accel.y = newaccel.y;
                accel.z = newaccel.z;
            }
        }
    }
}