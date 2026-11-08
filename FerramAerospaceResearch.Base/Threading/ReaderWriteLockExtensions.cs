using System;
using System.Threading;

namespace FerramAerospaceResearch.Threading
{
    // Unused: nothing in FAR calls it (commented out after the test suite's coverage run)
    // public static class ReaderWriteLockExtensions
    // {
    //     /// <summary>
    //     /// A simple struct to generate disposable objects for using statements, saves writing try/finally statements everywhere
    //     /// </summary>
    //     /// <typeparam name="T"></typeparam>
    //     public readonly struct Disposable<T> : IDisposable
    //     {
    //         private readonly T obj;
    //         private readonly Action<T> exit;

    //         public Disposable(T obj, Action<T> exit)
    //         {
    //             this.obj = obj;
    //             this.exit = exit;
    //         }

    //         /// <inheritdoc />
    //         public void Dispose()
    //         {
    //             exit(obj);
    //         }
    //     }

    //     public static Disposable<ReaderWriterLockSlim> DisposableWriteLock(this ReaderWriterLockSlim ls)
    //     {
    //         ls.EnterWriteLock();
    //         return new Disposable<ReaderWriterLockSlim>(ls, slim => slim.ExitWriteLock());
    //     }

    //     public static Disposable<ReaderWriterLockSlim> DisposableReadLock(this ReaderWriterLockSlim ls)
    //     {
    //         ls.EnterReadLock();
    //         return new Disposable<ReaderWriterLockSlim>(ls, slim => slim.ExitReadLock());
    //     }

    //     public static Disposable<ReaderWriterLockSlim> DisposableUpgradeableReadLock(this ReaderWriterLockSlim ls)
    //     {
    //         ls.EnterUpgradeableReadLock();
    //         return new Disposable<ReaderWriterLockSlim>(ls, slim => slim.ExitUpgradeableReadLock());
    //     }
    // }
}
