using Svelto.Tasks.Enumerators;

namespace Boid.SveltoECS.Sample1
{
    public class ThreadSynchronizationSignal : WaitForSignalEnumerator<ThreadSynchronizationSignal>
    {
        public ThreadSynchronizationSignal(string name) : base(name, 1000, true, true)
        {
        }
    }
}