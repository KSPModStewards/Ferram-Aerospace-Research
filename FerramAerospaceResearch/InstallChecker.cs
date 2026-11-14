/*
 * InstallChecker, originally by Majiir
 * Released into the public domain using a CC0 Public Domain Dedication: http://creativecommons.org/publicdomain/zero/1.0/
 */

using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using UnityEngine;

// ReSharper disable PossibleMultipleEnumeration

namespace FerramAerospaceResearch
{
    // persistent: FARAddonLoader destroys a non-persistent addon in the frame it makes it, before Unity calls its Start
    [FARAddon(800, true)]
    internal class InstallChecker : MonoBehaviour
    {
        protected void Start()
        {
            IEnumerable<AssemblyLoader.LoadedAssembly> assemblies = AssemblyLoader
                                                                    .loadedAssemblies
                                                                    .Where(a => a.assembly.GetName().Name ==
                                                                                Assembly.GetExecutingAssembly()
                                                                                        .GetName()
                                                                                        .Name)
                                                                    .Where(a => a.url !=
                                                                                "FerramAerospaceResearch/Plugins");

            if (!assemblies.Any())
                return;
            IEnumerable<string> badPaths = assemblies
                                           .Select(a => a.path)
                                           .Select(p =>
                                                       Uri.UnescapeDataString(new Uri(Path.GetFullPath(KSPUtil
                                                                                                           .ApplicationRootPath))
                                                                              .MakeRelativeUri(new Uri(p))
                                                                              .ToString()
                                                                              .Replace('/',
                                                                                       Path
                                                                                           .DirectorySeparatorChar)));
            PopupDialog.SpawnPopupDialog(new Vector2(0, 0),
                                         new Vector2(0, 0),
                                         "IncorrectFARInstall",
                                         "Incorrect FAR Installation",
                                         "FAR has been installed incorrectly and will not function properly. All FAR files should be located in KSP/GameData/FerramAerospaceResearch. Do not move any files from inside the FAR folder.\n\nIncorrect path(s):\n" +
                                         string.Join("\n", badPaths.ToArray()),
                                         "OK",
                                         true,
                                         HighLogic.UISkin);
        }
    }
}
