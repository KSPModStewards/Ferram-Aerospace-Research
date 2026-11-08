using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using FerramAerospaceResearch.Resources;
using FerramAerospaceResearch.Threading;
using Xunit;

namespace FerramAerospaceResearch.UnitTests
{
    /// <summary>
    /// FAR's small general-purpose helpers that nothing in FAR happens to call yet: the value equality comparer and Swap,
    /// PathUtil's longer Combine overloads, SourceInfo, the object pools, Observable's untyped value, CsvWriter's Dispose
    /// and blank line, MainThread.Wait, the asset dictionary's Contains and the compute kernel's IsValid.
    /// </summary>
    public class UtilityTests
    {
        /// <summary>
        /// The comparer FAR builds for equatable value types hashes as the values do, so it works as a dictionary's
        /// comparer, and every instance of it equals every other (one per type, by type name).
        /// </summary>
        [Fact]
        public void ValueEqualityComparerHashesLikeTheValues()
        {
            var comparer = new GenericValueEqualityComparer<int>();
            Assert.IsType<GenericValueEqualityComparer<int>>(CustomEqualityComparer<int>.Default);
            Assert.Equal(42.GetHashCode(), comparer.GetHashCode(42));
            Assert.True(comparer.Equals(3, 3));
            Assert.False(comparer.Equals(3, 4));

            var set = new HashSet<int>(comparer) { 1, 2, 2, 3 };
            Assert.Equal(3, set.Count);
            Assert.Contains(2, set);

            Assert.True(comparer.Equals(new GenericValueEqualityComparer<int>()));
            Assert.Equal(new GenericValueEqualityComparer<int>().GetHashCode(), comparer.GetHashCode());
            Assert.False(comparer.Equals(EqualityComparer<int>.Default));
            Assert.False(comparer.Equals(new GenericValueEqualityComparer<long>()));
            Assert.False(comparer.Equals(null));
        }

        [Fact]
        public void SwapExchangesTheValues()
        {
            int a = 1, b = 2;
            FARUtils.Swap(ref a, ref b);
            Assert.Equal((2, 1), (a, b));

            string s = "x", t = null;
            FARUtils.Swap(ref s, ref t);
            Assert.Null(s);
            Assert.Equal("x", t);
        }

        /// <summary>
        /// PathUtil.Combine with more parts joins them as Path.Combine does, unless the file name is absolute, which is
        /// returned as it is.
        /// </summary>
        [Fact]
        public void CombineJoinsPartsUnlessTheFileIsAbsolute()
        {
            Assert.Equal(Path.Combine("root", "a", "b", "file.cfg"), PathUtil.Combine("root", "a", "b", "file.cfg"));
            Assert.Equal("C:/abs/file.cfg", PathUtil.Combine("root", "a", "b", "C:/abs/file.cfg"));

            Assert.Equal(Path.Combine("root", "a", "b", "c", "file.cfg"), PathUtil.Combine("root", "a", "b", "c", "file.cfg"));
            Assert.Equal(Path.Combine("root", "file.cfg"), PathUtil.Combine(new[] { "root", "file.cfg" }));
            Assert.Equal("/abs/file.cfg", PathUtil.Combine(new[] { "root", "a", "b", "c", "/abs/file.cfg" }));
        }

        [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.NoInlining)]
        private static SourceInfo Here(int depth) => SourceInfo.Current(depth);

        [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.NoInlining)]
        private static SourceInfo FromCaller() => Here(1);

        /// <summary>
        /// SourceInfo.Current(depth) names the method that many frames above its caller, and ToString ends with the
        /// member name (file and line before it in Debug builds, where the frames have them).
        /// </summary>
        [Fact]
        public void SourceInfoNamesTheCallingMember()
        {
            Assert.Equal(nameof(Here), Here(0).MemberName);
            Assert.Equal(nameof(FromCaller), FromCaller().MemberName);

            SourceInfo info = SourceInfo.Current("Member", "File.cs", 12);
            Assert.EndsWith("Member", info.ToString());
            Assert.EndsWith(nameof(Here), Here(0).ToString());
        }

        /// <summary>
        /// A pool made without a factory creates its objects with their default constructor; released objects are kept,
        /// counted and handed out again last-in first-out, and both interface enumerators see them.
        /// </summary>
        [Fact]
        public void PoolKeepsReleasedObjectsForReuse()
        {
            var pool = new Pool<List<int>>();
            Assert.Equal(0, pool.Count);
            List<int> first = pool.Acquire(), second = pool.Acquire();
            Assert.NotNull(first);
            Assert.NotSame(first, second);
            Assert.Equal(0, pool.Count);

            pool.Release(first);
            pool.Release(second);
            Assert.Equal(2, pool.Count);
            Assert.Equal(new[] { second, first }, ((IEnumerable<List<int>>)pool).ToArray());
            Assert.Equal(new object[] { second, first }, ((IEnumerable)pool).Cast<object>().ToArray());

            Assert.Same(second, pool.Acquire());
            Assert.Equal(1, pool.Count);
            pool.Clear();
            Assert.Equal(0, pool.Count);
        }

        /// <summary>Disposing a Disposable gives its value back to the pool that owns it.</summary>
        [Fact]
        public void DisposableReturnsItsValueToThePool()
        {
            var released = new List<string>();
            var pool = new Pool<string>(() => "new", released.Add);
            using (var borrowed = new Disposable<string>("borrowed", pool))
            {
                Assert.Equal("borrowed", borrowed.value);
                Assert.Equal(0, pool.Count);
            }

            Assert.Equal(new[] { "borrowed" }, released);
            Assert.Equal(1, pool.Count);
            Assert.Equal("borrowed", pool.Acquire());
        }

        /// <summary>
        /// ListPool hands out lists with room for 64 items, empties a list it is given back and caps its capacity at
        /// that, and its Dispose drops the lists it holds.
        /// </summary>
        [Fact]
        public void ListPoolEmptiesAndShrinksReturnedLists()
        {
            var pool = new ListPool<int>();
            List<int> list = pool.Acquire();
            Assert.Equal(64, list.Capacity);
            list.AddRange(Enumerable.Range(0, 1000));
            Assert.True(list.Capacity > 64);

            pool.Release(list);
            Assert.Empty(list);
            Assert.Equal(64, list.Capacity);

            List<int> small = new List<int>(8) { 1 };
            pool.Release(small);
            Assert.Equal(8, small.Capacity);   // only ever shrunk
            Assert.Equal(2, pool.Count);

            pool.Dispose();
            Assert.Equal(0, pool.Count);
        }

        /// <summary>
        /// Observable's untyped Value reads the value boxed and sets it when given its type (with the change event);
        /// a value of another type is ignored.
        /// </summary>
        [Fact]
        public void UntypedObservableValueSetsOnlyItsType()
        {
            var observable = new Observable<int>(3);
            var untyped = (Interfaces.IObservable)observable;
            var seen = new List<int>();
            observable.OnValueChanged += seen.Add;

            Assert.Equal(3, untyped.Value);
            untyped.Value = 8;
            Assert.Equal(8, observable.Value);
            Assert.Equal(8, untyped.Value);
            Assert.Equal(new[] { 8 }, seen);

            // FAR's Release build compiles the assert in this branch out, so it is just ignored
            untyped.Value = "not an int";
            Assert.Equal(8, observable.Value);
            Assert.Equal(new[] { 8 }, seen);
        }

        /// <summary>
        /// CsvWriter: WriteLine() ends the row with nothing more in it, and Dispose flushes the file and lets it go (it
        /// can be read and deleted after); disposing a writer that never opened a file does nothing.
        /// </summary>
        [Fact]
        public void CsvWriterDisposeFlushesAndReleasesTheFile()
        {
            string path = Path.Combine(Path.GetTempPath(), $"far-csv-{Guid.NewGuid():N}.csv");
            try
            {
                using (var writer = new CsvWriter(path))
                {
                    writer.Open();
                    writer.Write(1);
                    writer.WriteLine();
                    writer.WriteLine();
                    writer.WriteLine(2);
                    Assert.Equal(0, new FileInfo(path).Length);   // not flushed yet
                }

                string nl = Environment.NewLine;
                Assert.Equal($"sep=, {nl}1, {nl}{nl}2, {nl}", File.ReadAllText(path));
                File.Delete(path);   // throws if Dispose left the file open
                Assert.False(File.Exists(path));

                new CsvWriter(path).Dispose();
                Assert.False(File.Exists(path));
            }
            finally
            {
                if (File.Exists(path))
                    File.Delete(path);
            }
        }

        /// <summary>
        /// MainThread.Wait blocks until another thread has run the task, returns at once for a task already run, and
        /// does nothing for no task.
        /// </summary>
        [Fact]
        public void MainThreadWaitBlocksUntilTheTaskRuns()
        {
            MainThread.Wait(null);

            bool ran = false;
            var task = new MainThread.Task { Action = () => ran = true };
            var worker = new Thread(() =>
            {
                Thread.Sleep(50);
                MainThread.ExecuteTask(task);
            });
            worker.Start();
            MainThread.Wait(task);
            Assert.True(ran);
            Assert.True(task.Executed);
            worker.Join();

            MainThread.Wait(task);   // already run: no wait
        }

        [Fact]
        public void AssetDictionaryContainsItsKeys()
        {
            var assets = new FARAssetDictionary<int> { ["line_renderer"] = 1 };
            Assert.True(assets.Contains("line_renderer"));
            Assert.False(assets.Contains("Line_Renderer"));
            Assert.False(assets.Contains("debug_voxel"));
        }

        /// <summary>A kernel is valid with a name and an index found in its shader (0 or more).</summary>
        [Fact]
        public void KernelIsValidWithANameAndAnIndex()
        {
            Assert.True(new Kernel { name = "CountPixels", index = 0 }.IsValid);
            Assert.True(new Kernel { name = "CountPixels", index = 2 }.IsValid);
            Assert.False(new Kernel { name = "CountPixels", index = -1 }.IsValid);
            Assert.False(new Kernel { name = "", index = 0 }.IsValid);
            Assert.False(new Kernel { name = null, index = 0 }.IsValid);
        }
    }
}
