using System.IO;

namespace FerramAerospaceResearch
{
    public static class KSPUtils
    {
        public static string FARRelativePath { get; } = Path.Combine("GameData", PathUtil.ModDirectoryName);
        public static string GameDataPath { get; } = Path.GetFullPath(Path.Combine(PathUtil.RootDir, ".."));
        public static string KSPRootPath { get; } = Path.GetFullPath(Path.Combine(GameDataPath, ".."));

        // Unused: nothing in FAR calls it (commented out after the test suite's coverage run)
        // public static string CombineKSPRoot(string filename)
        // {
        //     return PathUtil.Combine(KSPRootPath, filename);
        // }

        // Unused: nothing in FAR calls it (commented out after the test suite's coverage run)
        // public static string CombineKSPRoot(string dir1, string filename)
        // {
        //     return PathUtil.Combine(KSPRootPath, dir1, filename);
        // }

        // Unused: nothing in FAR calls it (commented out after the test suite's coverage run)
        // public static string CombineKSPRoot(string dir1, string dir2, string filename)
        // {
        //     return PathUtil.Combine(KSPRootPath, dir1, dir2, filename);
        // }

        // Unused: nothing in FAR calls it (commented out after the test suite's coverage run)
        // public static string CombineGameData(string filename)
        // {
        //     return PathUtil.Combine(GameDataPath, filename);
        // }

        // Unused: nothing in FAR calls it (commented out after the test suite's coverage run)
        // public static string CombineGameData(string dir1, string filename)
        // {
        //     return PathUtil.Combine(GameDataPath, dir1, filename);
        // }

        // Unused: nothing in FAR calls it (commented out after the test suite's coverage run)
        // public static string CombineGameData(string dir1, string dir2, string filename)
        // {
        //     return PathUtil.Combine(GameDataPath, dir1, dir2, filename);
        // }
    }
}
