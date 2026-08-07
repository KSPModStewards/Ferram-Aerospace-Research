/*
Ferram Aerospace Research
=========================
On-disk cache for finished heatmap sweeps, keyed by the craft's identity (name + part count, the same
signature HeatmapResult.Matches uses). Lets a swept envelope survive a game restart and be reloaded for
the vessel it was swept for, instead of being recomputed every session. The cache is regenerable — a
miss (absent / stale / format change) just means "re-sweep", so it fails soft.

   This file is part of Ferram Aerospace Research.

   Ferram Aerospace Research is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */

using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Compression;
using System.Reflection;
using System.Text;

namespace FerramAerospaceResearch.FARGUI.FAREditorGUI.Simulation
{
    internal static class HeatmapPersistence
    {
        private const uint Magic = 0x46415248;   // "FARH"
        private const int FormatVersion = 1;

        // Cap on cached sweeps kept per vessel — plenty to compare against, but bounded so a long session
        // (or auto-recalc firing on every re-voxelization) can't grow the cache without limit. Oldest go
        // first; Clear still removes them all at once.
        private const int MaxCachedPerVessel = 20;

        // Public instance fields of CellResult, in a stable (name-sorted) order. Reflected once so adding
        // a field to CellResult does not need a hand-edit here; the stored field count guards the format,
        // so an old cache written with a different set simply misses and the craft is re-swept.
        private static readonly FieldInfo[] CellFields = BuildCellFields();

        private static FieldInfo[] BuildCellFields()
        {
            FieldInfo[] fields = typeof(PerformanceEnvelopeCalculator.CellResult)
                .GetFields(BindingFlags.Public | BindingFlags.Instance);
            Array.Sort(fields, (a, b) => string.CompareOrdinal(a.Name, b.Name));
            return fields;
        }

        private static string CacheDir
        {
            get
            {
                return Path.Combine(KSPUtil.ApplicationRootPath,
                                    "GameData", "FerramAerospaceResearch", "PluginData", "Heatmaps");
            }
        }

        private static string Sanitize(string craftName)
        {
            // Long cap so two distinct crafts don't collide onto one prefix — history is now browsed and
            // bulk-deleted by this prefix, so a collision would mix and could wipe another craft's sweeps.
            // Still well under the ~255-char filesystem limit once ".<parts>.<ticks>.farheat" is appended.
            var sb = new StringBuilder();
            foreach (char c in craftName ?? "")
                sb.Append(char.IsLetterOrDigit(c) || c == '-' || c == '_' ? c : '_');
            if (sb.Length > 180)
                sb.Length = 180;
            return sb.ToString();
        }

        /// <summary>One on-disk sweep in a vessel's browsable history: file path, the part count it was
        /// swept at, and when it was written.</summary>
        public sealed class CacheEntry
        {
            public readonly string Path;
            public readonly int PartCount;
            public readonly DateTime Time;

            public CacheEntry(string path, int partCount, DateTime time)
            {
                Path = path;
                PartCount = partCount;
                Time = time;
            }
        }

        /// <summary>
        /// Writes a finished sweep as its OWN cache file — never overwriting a prior sweep — so a vessel
        /// keeps a browsable history of past heatmaps. Named "&lt;craft&gt;.&lt;parts&gt;.&lt;ticks&gt;.farheat".
        /// Returns the path written, or null on failure.
        /// </summary>
        public static string Save(List<HeatmapResult> results)
        {
            if (results == null || results.Count == 0)
                return null;

            try
            {
                Directory.CreateDirectory(CacheDir);
                string path = Path.Combine(CacheDir,
                    Sanitize(results[0].CraftName) + "." + results[0].PartCount + "." + DateTime.Now.Ticks + ".farheat");
                using (FileStream fs = File.Create(path))
                using (var gz = new GZipStream(fs, CompressionMode.Compress))
                using (var w = new BinaryWriter(gz))
                {
                    w.Write(Magic);
                    w.Write(FormatVersion);
                    w.Write(CellFields.Length);
                    WriteString(w, results[0].CraftName ?? "");
                    w.Write(results[0].PartCount);

                    w.Write(results.Count);
                    foreach (HeatmapResult r in results)
                        WriteResult(w, r);
                }

                FARLogger.Info("Heatmap cache written: " + path);
                Prune(results[0].CraftName);
                return path;
            }
            catch (Exception e)
            {
                FARLogger.Exception(e, "while writing the heatmap cache");
                return null;
            }
        }

        /// <summary>Keeps only the newest <see cref="MaxCachedPerVessel" /> cached sweeps for a vessel,
        /// deleting the oldest. Uses file times only (no header reads).</summary>
        private static void Prune(string craftName)
        {
            try
            {
                if (!Directory.Exists(CacheDir))
                    return;

                string prefix = Sanitize(craftName);
                var files = new List<string>();
                foreach (string path in Directory.GetFiles(CacheDir, prefix + ".*.farheat"))
                {
                    string[] seg = Path.GetFileNameWithoutExtension(path).Split('.');
                    if (seg.Length >= 2 && seg[0] == prefix && int.TryParse(seg[1], out int _))
                        files.Add(path);
                }
                if (files.Count <= MaxCachedPerVessel)
                    return;

                files.Sort((a, b) => File.GetLastWriteTime(a).CompareTo(File.GetLastWriteTime(b))); // oldest first
                for (int i = 0; i < files.Count - MaxCachedPerVessel; i++)
                    try { File.Delete(files[i]); }
                    catch (Exception ex) { FARLogger.Exception(ex, "while pruning a heatmap cache"); }
            }
            catch (Exception e)
            {
                FARLogger.Exception(e, "while pruning the heatmap cache");
            }
        }

        /// <summary>Every cached sweep for a vessel (by craft name, any part count), oldest first.</summary>
        public static List<CacheEntry> ListForVessel(string craftName)
        {
            var entries = new List<CacheEntry>();
            try
            {
                if (!Directory.Exists(CacheDir))
                    return entries;

                string prefix = Sanitize(craftName);
                foreach (string path in Directory.GetFiles(CacheDir, prefix + ".*.farheat"))
                {
                    // "<prefix>.<parts>[.<ticks>]" — the prefix has no dots, so segment 1 is the part count.
                    string[] seg = Path.GetFileNameWithoutExtension(path).Split('.');
                    if (seg.Length < 2 || seg[0] != prefix || !int.TryParse(seg[1], out int part))
                        continue;
                    if (!IsCompatible(path)) // skip an old-format file so the nav never lists a dead entry
                        continue;
                    entries.Add(new CacheEntry(path, part, File.GetLastWriteTime(path)));
                }

                entries.Sort((a, b) => a.Time.CompareTo(b.Time));
            }
            catch (Exception e)
            {
                FARLogger.Exception(e, "while listing heatmap caches");
            }
            return entries;
        }

        /// <summary>Cheap header check: reads only magic/version/field-count so the list can drop a file
        /// written by an older build (different cell layout) without trying to fully load it.</summary>
        private static bool IsCompatible(string path)
        {
            try
            {
                using (FileStream fs = File.OpenRead(path))
                using (var gz = new GZipStream(fs, CompressionMode.Decompress))
                using (var r = new BinaryReader(gz))
                    return r.ReadUInt32() == Magic && r.ReadInt32() == FormatVersion && r.ReadInt32() == CellFields.Length;
            }
            catch
            {
                return false;
            }
        }

        /// <summary>Deletes every cached sweep for a vessel — including old-format files the nav hides.
        /// Returns how many files were removed.</summary>
        public static int DeleteForVessel(string craftName)
        {
            int removed = 0;
            try
            {
                if (!Directory.Exists(CacheDir))
                    return 0;

                string prefix = Sanitize(craftName);
                foreach (string path in Directory.GetFiles(CacheDir, prefix + ".*.farheat"))
                {
                    string[] seg = Path.GetFileNameWithoutExtension(path).Split('.');
                    if (seg.Length < 2 || seg[0] != prefix || !int.TryParse(seg[1], out int _))
                        continue;
                    try
                    {
                        File.Delete(path);
                        removed++;
                    }
                    catch (Exception ex)
                    {
                        FARLogger.Exception(ex, "while deleting a heatmap cache");
                    }
                }
            }
            catch (Exception e)
            {
                FARLogger.Exception(e, "while clearing heatmap caches");
            }
            return removed;
        }

        /// <summary>Deletes one cached sweep file (the currently-viewed history entry). True if removed.</summary>
        public static bool DeletePath(string path)
        {
            try
            {
                if (File.Exists(path))
                {
                    File.Delete(path);
                    return true;
                }
            }
            catch (Exception ex)
            {
                FARLogger.Exception(ex, "while deleting a heatmap cache");
            }
            return false;
        }

        /// <summary>The newest valid cached sweep matching a craft's name AND part count, or null — used
        /// to auto-display a craft's map on opening the editor / flight.</summary>
        public static List<HeatmapResult> Load(string craftName, int partCount)
        {
            List<CacheEntry> all = ListForVessel(craftName);
            for (int i = all.Count - 1; i >= 0; i--) // newest first
                if (all[i].PartCount == partCount)
                {
                    List<HeatmapResult> r = LoadPath(all[i].Path);
                    if (r != null)
                        return r;
                }
            return null;
        }

        /// <summary>
        /// Reads a specific cache file, validating only the format (not name/part) so a historical snapshot
        /// from a different edit still loads. Null on a format/version mismatch or read error. Derived
        /// summary values are recomputed on load, so the result stays self-consistent with the current build.
        /// </summary>
        public static List<HeatmapResult> LoadPath(string path)
        {
            try
            {
                if (!File.Exists(path))
                    return null;

                using (FileStream fs = File.OpenRead(path))
                using (var gz = new GZipStream(fs, CompressionMode.Decompress))
                using (var r = new BinaryReader(gz))
                {
                    if (r.ReadUInt32() != Magic || r.ReadInt32() != FormatVersion || r.ReadInt32() != CellFields.Length)
                        return null;

                    ReadString(r); // craft name (informational)
                    r.ReadInt32(); // part count (informational)

                    int n = r.ReadInt32();
                    var list = new List<HeatmapResult>(n);
                    for (int i = 0; i < n; i++)
                        list.Add(ReadResult(r));
                    return list;
                }
            }
            catch (Exception e)
            {
                FARLogger.Exception(e, "while reading the heatmap cache");
                return null;
            }
        }

        // --- HeatmapResult ---

        private static void WriteResult(BinaryWriter w, HeatmapResult r)
        {
            WriteDoubleArray(w, r.MachAxis);
            WriteDoubleArray(w, r.AltAxis);
            w.Write(r.SpeedAxisMode);
            WriteDoubleArray(w, r.SampleMassesKg);
            w.Write(r.DryMassKg);
            w.Write(r.MassKg);
            w.Write(r.RefAreaM2);
            WriteString(w, r.EngineTypeLabel ?? "");
            WriteString(w, r.BodyName ?? "");
            WriteString(w, r.CraftName ?? "");
            w.Write(r.PartCount);

            // Envelope grids, one per mass sample. Envelope itself is samples[last] by reference, so it is
            // not written separately.
            PerformanceEnvelopeCalculator.CellResult[][,] samples = r.EnvelopeSamples;
            w.Write(samples?.Length ?? 0);
            if (samples != null)
                foreach (PerformanceEnvelopeCalculator.CellResult[,] grid in samples)
                    WriteGrid(w, grid);

            WriteStability(w, r.HasStability ? r.Stability : null);
        }

        private static HeatmapResult ReadResult(BinaryReader r)
        {
            var result = new HeatmapResult
            {
                MachAxis = ReadDoubleArray(r),
                AltAxis = ReadDoubleArray(r)
            };
            result.SpeedAxisMode = r.ReadBoolean();
            result.SampleMassesKg = ReadDoubleArray(r);
            result.DryMassKg = r.ReadDouble();
            result.MassKg = r.ReadDouble();
            result.RefAreaM2 = r.ReadDouble();
            result.EngineTypeLabel = ReadString(r);
            result.BodyName = ReadString(r);
            result.CraftName = ReadString(r);
            result.PartCount = r.ReadInt32();

            int nSamples = r.ReadInt32();
            if (nSamples > 0)
            {
                var samples = new PerformanceEnvelopeCalculator.CellResult[nSamples][,];
                for (int k = 0; k < nSamples; k++)
                    samples[k] = ReadGrid(r);
                result.EnvelopeSamples = samples;
                result.Envelope = samples[nSamples - 1]; // wet sample is the displayed grid
            }

            result.Stability = ReadStability(r);
            result.HasStability = result.Stability != null;

            result.Recompute(); // ceiling / best / max are derived, not stored
            return result;
        }

        // --- grids & cells ---

        private static void WriteGrid(BinaryWriter w, PerformanceEnvelopeCalculator.CellResult[,] grid)
        {
            int nx = grid?.GetLength(0) ?? 0;
            int ny = grid?.GetLength(1) ?? 0;
            w.Write(nx);
            w.Write(ny);
            for (int i = 0; i < nx; i++)
                for (int j = 0; j < ny; j++)
                    WriteCell(w, grid[i, j]);
        }

        private static PerformanceEnvelopeCalculator.CellResult[,] ReadGrid(BinaryReader r)
        {
            int nx = r.ReadInt32();
            int ny = r.ReadInt32();
            var grid = new PerformanceEnvelopeCalculator.CellResult[nx, ny];
            for (int i = 0; i < nx; i++)
                for (int j = 0; j < ny; j++)
                    grid[i, j] = ReadCell(r);
            return grid;
        }

        private static void WriteCell(BinaryWriter w, PerformanceEnvelopeCalculator.CellResult c)
        {
            object boxed = c; // struct fields via reflection need a boxed instance
            foreach (FieldInfo f in CellFields)
                WriteValue(w, f.GetValue(boxed), f.FieldType);
        }

        private static PerformanceEnvelopeCalculator.CellResult ReadCell(BinaryReader r)
        {
            object boxed = new PerformanceEnvelopeCalculator.CellResult();
            foreach (FieldInfo f in CellFields)
                f.SetValue(boxed, ReadValue(r, f.FieldType));
            return (PerformanceEnvelopeCalculator.CellResult)boxed;
        }

        private static void WriteValue(BinaryWriter w, object val, Type t)
        {
            if (t == typeof(double))
                w.Write((double)val);
            else if (t == typeof(bool))
                w.Write((bool)val);
            else if (t == typeof(int))
                w.Write((int)val);
            else if (t == typeof(float))
                w.Write((float)val);
            else if (t.IsEnum)
                w.Write(Convert.ToInt32(val));
            else
                throw new NotSupportedException("HeatmapPersistence: unhandled field type " + t);
        }

        private static object ReadValue(BinaryReader r, Type t)
        {
            if (t == typeof(double))
                return r.ReadDouble();
            if (t == typeof(bool))
                return r.ReadBoolean();
            if (t == typeof(int))
                return r.ReadInt32();
            if (t == typeof(float))
                return r.ReadSingle();
            if (t.IsEnum)
                return Enum.ToObject(t, r.ReadInt32());
            throw new NotSupportedException("HeatmapPersistence: unhandled field type " + t);
        }

        // --- stability ---

        private static void WriteStability(BinaryWriter w, HeatmapResult.StabilityCell[,] stab)
        {
            if (stab == null)
            {
                w.Write(0);
                w.Write(0);
                return;
            }

            int nx = stab.GetLength(0), ny = stab.GetLength(1);
            w.Write(nx);
            w.Write(ny);
            for (int i = 0; i < nx; i++)
                for (int j = 0; j < ny; j++)
                {
                    HeatmapResult.StabilityCell c = stab[i, j];
                    w.Write(c != null);
                    if (c == null)
                        continue;
                    w.Write(c.Score);
                    w.Write(c.OutOfRange.Count);
                    foreach ((string name, double value, int expectedSign) in c.OutOfRange)
                    {
                        WriteString(w, name ?? "");
                        w.Write(value);
                        w.Write(expectedSign);
                    }
                }
        }

        private static HeatmapResult.StabilityCell[,] ReadStability(BinaryReader r)
        {
            int nx = r.ReadInt32(), ny = r.ReadInt32();
            if (nx == 0 || ny == 0)
                return null;

            var stab = new HeatmapResult.StabilityCell[nx, ny];
            for (int i = 0; i < nx; i++)
                for (int j = 0; j < ny; j++)
                {
                    if (!r.ReadBoolean())
                        continue;
                    var c = new HeatmapResult.StabilityCell { Score = r.ReadInt32() };
                    int m = r.ReadInt32();
                    for (int k = 0; k < m; k++)
                    {
                        string name = ReadString(r);
                        double value = r.ReadDouble();
                        int sign = r.ReadInt32();
                        c.OutOfRange.Add((name, value, sign));
                    }

                    stab[i, j] = c;
                }

            return stab;
        }

        // --- primitives ---

        private static void WriteDoubleArray(BinaryWriter w, double[] a)
        {
            w.Write(a?.Length ?? -1);
            if (a != null)
                foreach (double v in a)
                    w.Write(v);
        }

        private static double[] ReadDoubleArray(BinaryReader r)
        {
            int n = r.ReadInt32();
            if (n < 0)
                return null;
            var a = new double[n];
            for (int i = 0; i < n; i++)
                a[i] = r.ReadDouble();
            return a;
        }

        private static void WriteString(BinaryWriter w, string s)
        {
            w.Write(s ?? "");
        }

        private static string ReadString(BinaryReader r)
        {
            return r.ReadString();
        }
    }
}
