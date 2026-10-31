using System;
using System.Linq;

namespace FerramAerospaceResearch
{
    public static class ModUtils
    {
        private static bool ajeInstalled = false;
        private static bool needFindAJE = true;
        private static bool bdArmoryInstalled = false;
        private static bool needFindBDArmory = true;

        public static bool IsAJEInstalled
        {
            get
            {
                if (needFindAJE)
                {
                    needFindAJE = false;
                    ajeInstalled = AssemblyLoader.loadedAssemblies.Any(a => a.name.Equals("AJE", StringComparison.OrdinalIgnoreCase));
                }

                return ajeInstalled;
            }
        }

        public static bool IsBDArmoryInstalled
        {
            get
            {
                if (needFindBDArmory)
                {
                    needFindBDArmory = false;
                    bdArmoryInstalled = AssemblyLoader.loadedAssemblies.Any(a => a.name.Equals("BDArmory", StringComparison.OrdinalIgnoreCase));
                }

                return bdArmoryInstalled;
            }
        }
    }
}
