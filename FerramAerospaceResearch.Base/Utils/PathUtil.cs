using System;
using System.IO;
using FerramAerospaceResearch.Reflection;

namespace FerramAerospaceResearch
{
    public static class PathUtil
    {
        public const string ModDirectoryName = "FerramAerospaceResearch";

        public static string PluginsDir { get; } =
            AbsPath(Path.Combine(ReflectionUtils.ExecutingAssembly.Location, ".."));

        public static string RootDir { get; } = AbsPath(Path.Combine(PluginsDir, ".."));
        public static string AssetsDir { get; } = AbsPath(Path.Combine(RootDir, "Assets"));
        public static string TexturesDir { get; } = AbsPath(Path.Combine(RootDir, "Textures"));
        public static string ParentDir { get; } = AbsPath(Path.Combine(RootDir, ".."));
        public static string PParentDir { get; } = AbsPath(Path.Combine(ParentDir, ".."));

        public static string AbsPath(string path)
        {
            // C# works with either '/' or '\' but KSP will serialise '\\' as '\' and parsing will fail
            return Path.GetFullPath(path).Replace("\\", "/");
        }

        public static string Combine(string path, string filename)
        {
            return IsAbsolute(filename) ? filename : Path.Combine(path, filename);
        }

        public static string Combine(string path, string dir1, string filename)
        {
            return IsAbsolute(filename) ? filename : Path.Combine(path, dir1, filename);
        }

        public static string Combine(string path, string dir1, string dir2, string filename)
        {
            return IsAbsolute(filename) ? filename : Path.Combine(path, dir1, dir2, filename);
        }

        public static string Combine(params string[] paths)
        {
            return IsAbsolute(paths[paths.Length - 1]) ? paths[paths.Length - 1] : Path.Combine(paths);
        }

        public static Func<string, string> CombineDelegate(string root)
        {
            return s => AbsPath(Combine(root, s));
        }

        // Unused: nothing in FAR calls it (commented out after the test suite's coverage run)
        // public static Func<string, string, string> CombineDelegate2(string root)
        // {
        //     return (d, s) => AbsPath(Combine(root, d, s));
        // }

        // Unused: nothing in FAR calls it (commented out after the test suite's coverage run)
        // public static Func<string, string, string, string> CombineDelegate3(string root)
        // {
        //     return (d1, d2, s) => AbsPath(Combine(root, d1, d2, s));
        // }

        public static bool IsAbsolute(string path, bool retry = true)
        {
            try
            {
                return Path.IsPathRooted(path);
            }
            catch (ArgumentException e)
            {
                if (retry)
                {
                    FARLogger.Warning($"Invalid path: {path}, retrying");
                    return IsAbsolute(path.Replace("\\", "/"), false);
                }

                FARLogger.Exception(e, $"Exception in path: {path}");
                throw;
            }
        }
    }
}
