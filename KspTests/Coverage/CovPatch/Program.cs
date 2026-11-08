// Rewrites the AltCover visit calls in instrumented assemblies to FarCoverage.FastVisit.Visit (same signature).
//   CovPatch <dll>...    each rewritten in place; prints the calls redirected per assembly
using System;
using System.IO;
using System.Linq;
using Mono.Cecil;
using Mono.Cecil.Cil;

internal static class Program
{
    private static int Main(string[] args)
    {
        foreach (string path in args)
        {
            var resolver = new DefaultAssemblyResolver();
            resolver.AddSearchDirectory(Path.GetDirectoryName(Path.GetFullPath(path)));
            string tmp = path + ".patched";
            int n = 0;
            using (ModuleDefinition module = ModuleDefinition.ReadModule(path, new ReaderParameters { AssemblyResolver = resolver }))
            {
                var asm = new AssemblyNameReference("FarCoverage.FastVisit", new Version(1, 0, 0, 0));
                var type = new TypeReference("FarCoverage", "FastVisit", module, asm);
                var visit = new MethodReference("Visit", module.TypeSystem.Void, type) { HasThis = false };
                visit.Parameters.Add(new ParameterDefinition(module.TypeSystem.String));
                visit.Parameters.Add(new ParameterDefinition(module.TypeSystem.Int32));
                bool added = false;
                foreach (TypeDefinition t in module.GetTypes())
                    foreach (MethodDefinition m in t.Methods.Where(m => m.HasBody))
                        foreach (Instruction i in m.Body.Instructions)
                            if (i.OpCode == OpCodes.Call && i.Operand is MethodReference r && r.Name == "Visit" &&
                                r.DeclaringType.FullName == "AltCover.Recorder.Instance" && r.Parameters.Count == 2 &&
                                r.Parameters[0].ParameterType.FullName == "System.String" && r.Parameters[1].ParameterType.FullName == "System.Int32")
                            {
                                if (!added)
                                {
                                    module.AssemblyReferences.Add(asm);
                                    added = true;
                                }

                                i.Operand = visit;
                                n++;
                            }

                module.Write(tmp);
            }

            File.Copy(tmp, path, true);
            File.Delete(tmp);
            Console.WriteLine($"{Path.GetFileName(path)}: {n} visit calls redirected");
            if (n == 0)
                return 1;
        }

        return 0;
    }
}
