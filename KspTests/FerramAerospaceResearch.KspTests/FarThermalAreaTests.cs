using System;
using System.Collections;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text;
using KspTest;
using UnityEngine;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>The areas FAR hands KSP's thermal model.</summary>
    public static class FarThermalAreaTests
    {
        /// <summary>
        /// Which model each thermal area comes from, per part, on the runway and in a 200 km orbit (Aeris 4A). For
        /// every part: part.radiativeArea against FAR's voxel area (FARAeroPartModule.ProjectedAreas.totalArea) and
        /// stock's (DragCubes.PostOcclusionArea), part.exposedArea, the flight integrator's sun and body areas, and
        /// skinThermalMass. Records how many parts take their radiative area from each model and the sums; writes
        /// thermal_areas.csv. Params: orbitKm (200).
        /// </summary>
        [UITest("Far_ThermalAreas", Explicit = true, Kind = "Measure", StartsAt = StartState.Flight, Craft = "Aeris 4A", CraftDir = "SPH", LaunchSite = "Runway", TimeoutSec = 900)]
        public static IEnumerator Far_ThermalAreas(TestContext ctx)
        {
            ctx.Record("farBuild", FarSim.FarVersion());
            Type aeroType = Reflect.TryFindType("FerramAerospaceResearch.FARAeroComponents.FARAeroPartModule")
                            ?? throw new UITestFailure("FARAeroPartModule not found");
            Type api = Reflect.TryFindType("FerramAerospaceResearch.FARAPI") ?? throw new UITestFailure("FARAPI not found");
            PropertyInfo projected = aeroType.GetProperty("ProjectedAreas") ?? throw new UITestFailure("ProjectedAreas not found");
            MethodInfo voxelValid = api.GetMethod("VesselVoxelizationCompletedAndValid", BindingFlags.Public | BindingFlags.Static);
            var csv = new StringBuilder("condition,part,radiativeArea,farTotalArea,stockPostOcclusionArea,exposedArea,stockExposedArea,sunArea,bodyArea,skinThermalMass\n");
            var inv = CultureInfo.InvariantCulture;

            IEnumerator Measure(string cond)
            {
                Vessel v = FlightGlobals.ActiveVessel;
                yield return Wait.Until(() => voxelValid == null || (bool)voxelValid.Invoke(null, new object[] { v }), 120, "FAR voxelization");
                yield return Wait.Seconds(3);
                var fi = v.GetComponent<FlightIntegrator>() ?? throw new UITestFailure("no FlightIntegrator");
                int far = 0, stock = 0, neither = 0;
                double sRad = 0, sFar = 0, sStock = 0, sExp = 0, sSun = 0, sBody = 0, sSkin = 0;
                foreach (PartThermalData ptd in fi.partThermalDataList)
                {
                    Part p = ptd.part;
                    PartModule m = p.Modules.Cast<PartModule>().FirstOrDefault(x => aeroType.IsInstanceOfType(x));
                    if (m == null)
                        continue;
                    object pa = projected.GetValue(m);
                    double farArea = Convert.ToDouble(pa.GetType().GetField("totalArea").GetValue(pa));
                    double stockArea = p.DragCubes.None ? double.NaN : p.DragCubes.PostOcclusionArea;
                    double sun = fi.GetSunArea(ptd), body = fi.GetBodyArea(ptd);
                    bool Near(double a, double b) => Math.Abs(a - b) <= 1e-6 * Math.Max(1e-9, Math.Abs(b));
                    if (Near(p.radiativeArea, farArea)) far++;
                    else if (Near(p.radiativeArea, stockArea)) stock++;
                    else neither++;
                    sRad += p.radiativeArea; sFar += farArea; sStock += double.IsNaN(stockArea) ? 0 : stockArea;
                    sExp += p.exposedArea; sSun += sun; sBody += body; sSkin += p.skinThermalMass;
                    csv.Append(string.Join(",", cond, p.partInfo.name,
                        p.radiativeArea.ToString("R", inv), farArea.ToString("R", inv), stockArea.ToString("R", inv),
                        p.exposedArea.ToString("R", inv), (p.DragCubes.None ? double.NaN : p.DragCubes.ExposedArea).ToString("R", inv),
                        sun.ToString("R", inv), body.ToString("R", inv), p.skinThermalMass.ToString("R", inv))).Append('\n');
                }
                ctx.Record(cond + ".altitudeKm", (v.altitude / 1000).ToString("F1", inv));
                ctx.Record(cond + ".radiativeFrom", $"FAR voxel {far}, stock drag cube {stock}, neither {neither}");
                ctx.Record(cond + ".sums", $"radiative {sRad:F3}, FAR voxel {sFar:F3}, stock {sStock:F3}, exposed {sExp:F3}, sun {sSun:F3}, body {sBody:F3} m2; skinThermalMass {sSkin:F3}");
            }

            ctx.Step("On the runway");
            yield return Wait.FlightReady();
            yield return Measure("runway");

            double orbitKm = ctx.ParamDouble("orbitKm", 200);
            ctx.Step($"In a {orbitKm} km orbit");
            Flight.SetOrbit(orbitKm * 1000, orbitKm * 1000);
            yield return Wait.Seconds(5);
            yield return Wait.Until(() => !FlightGlobals.ActiveVessel.packed, 60, "off rails");
            yield return Measure("orbit");

            File.WriteAllText(Path.Combine(ctx.ResultsDir, "thermal_areas.csv"), csv.ToString());
        }
    }
}
