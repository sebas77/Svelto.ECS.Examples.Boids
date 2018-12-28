using System.Linq;
using Svelto.DataStructures;
using Svelto.Tasks.Parallelism;
using Svelto.Utilities;
using Unity.Mathematics;
using UnityEngine;

namespace Boid.SveltoECS.SampleD
{
    public partial class BoidsSimulationSystem
    {
        struct BoidSimulationJob : IMultiThreadParallelizable
        {
            readonly float weight;
            readonly float scale;
            readonly float thresh;
            readonly float  _prodThresh;
            readonly float  _distThresh;
            readonly float3 _separationWeight;
            readonly float3 _alignmentWeight;
            readonly float3 _cohesionWeight;

            readonly int[][] _neighbours;
            readonly int count;

            public BoidSimulationJob(BoidsSimulationSystem simulation)
            {
                scale  = Bootstrap.Param.wallScale * 0.5f;
                thresh = Bootstrap.Param.wallDistance;
                weight = Bootstrap.Param.wallWeight;
                count = Bootstrap.Instance.boidCount;
                
                _prodThresh       = math.cos(math.radians(Bootstrap.Param.neighborFov));
                _distThresh       = Bootstrap.Param.neighborDistance;
                _separationWeight = Bootstrap.Param.separationWeight;
                _alignmentWeight  = Bootstrap.Param.alignmentWeight;
                _cohesionWeight   = Bootstrap.Param.cohesionWeight;

                _neighbours = new int[count][];
                for (int i = 0; i < count; i++)
                    _neighbours[i] = new int[count];
            }
            
            public void Update(int i)
            {
                var volatileRead = ThreadUtility.VolatileRead(ref _deltaTime);
                
                Wall(entities, i, scale, thresh, weight);
                
                var neighbours = _neighbours[i];

                var currentCount = DetectNeighbour(entities, i, count, neighbours);

                if (currentCount > 0)
                {
                    Separation(entities, i, currentCount, neighbours);
                    AlignmentJob(entities, i, currentCount, neighbours);
                    CohesionJob(entities, i, currentCount, neighbours);
                }

                Move(entities, i, volatileRead);
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
                var minSpeed = Bootstrap.Param.minSpeed;
                var maxSpeed = Bootstrap.Param.maxSpeed / 3;

                ref var v     = ref entities[i].velocity;
                ref var accel = ref entities[i].acceleration;
                v.x += accel.x * dt;
                v.y += accel.y * dt;
                v.z += accel.z * dt;
                var vector3 = new float3(v.x, v.y, v.z);
                var speed   = math.length(vector3);
                var dir     = vector3 / speed;
                var vi      = math.clamp(speed, minSpeed, maxSpeed) * dir;
                v.x = vi.x;
                v.y = vi.y;
                v.z = vi.z;
                ref var pos3 = ref entities[i].position;
                var     pos  = new float3(pos3.x, pos3.y, pos3.z);
                var     pos2 = pos + vi * dt;

                pos3.x = pos2.x;
                pos3.y = pos2.y;
                pos3.z = pos2.z;
                var     lookRotationSafe = quaternion.LookRotationSafe(dir, new float3(0, 1, 0)).value;
                ref var rot              = ref entities[i].rotation;
                rot.x = lookRotationSafe.x;
                rot.y = lookRotationSafe.y;
                rot.z = lookRotationSafe.z;
                rot.w = lookRotationSafe.w;

                accel.x = accel.y = accel.z = 0;
            }

            int DetectNeighbour(BoidEntityStruct[] entities, int iindex, int count, int[] _neighbours)
            {
                int currentCount = 0;
                
                ref var velocity = ref entities[iindex].velocity;
                ref var pos      = ref entities[iindex].position;

                float3 pos0 = pos.ToFloat3();
                float3 fwd0 = math.normalize(velocity.ToFloat3());

                for (int i = iindex; i < count; ++i)
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

            void AlignmentJob(BoidEntityStruct[]  entities, int iindex, int count,
                                     int[] _neighbours)
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

            void CohesionJob(BoidEntityStruct[]          entities, int iindex, int count,
                                    int[] _neighbours)
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