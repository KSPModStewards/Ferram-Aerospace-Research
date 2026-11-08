using System.Collections;
using FerramAerospaceResearch.FARGUI.FAREditorGUI;
using KSP.UI.Screens;
using KspTest;
using UnityEngine;

namespace FerramAerospaceResearch.KspTests
{
    /// <summary>The game's own editor overlays, fed by FAR.</summary>
    public static class EditorOverlayTests
    {
        /// <summary>
        /// The stock center-of-lift overlay in the SPH, on the Aeris 4A: its marker sits where FAR puts the craft's
        /// aerodynamic center (every FAR part module answers the game's lift query with it), behind the center of
        /// mass, as on a plane stable in pitch.
        /// </summary>
        [UITest(Kind = "Check", StartsAt = StartState.Sph, Craft = Far.Plane, TimeoutSec = 900)]
        public static IEnumerator Editor_StockLiftMarkerShowsFarsCenter(TestContext ctx)
        {
            yield return Far.WaitForEditorVoxels();
            EditorVesselOverlays overlays = EditorVesselOverlays.fetch;
            Assert.NotNull(overlays, "the editor's overlays");
            EditorMarker_CoL marker = overlays.CoLmarker;
            bool wasShown = marker.gameObject.activeSelf, comWasShown = overlays.CoMmarker.gameObject.activeSelf;
            try
            {
                if (!wasShown)
                    overlays.toggleCoLbtn.onClick.Invoke();
                if (!comWasShown)
                    overlays.toggleCoMbtn.onClick.Invoke();
                yield return Wait.Until(() => marker.gameObject.activeInHierarchy, 10, "the center of lift marker shown", shot: false);
                yield return Wait.Seconds(1);
                Transform root = EditorLogic.RootPart.partTransform;
                Vector3 expected = root.localToWorldMatrix.MultiplyPoint3x4(EditorAeroCenter.VesselRootLocalAeroCenter);
                Vector3 shown = marker.transform.position;
                Vector3 com = overlays.CoMmarker.transform.position;
                ctx.Record("marker", $"shown {shown:F3}, FAR's {expected:F3}, CoM {com:F3}");
                Assert.True(EditorAeroCenter.VesselRootLocalAeroCenter != Vector3.zero, "FAR has an aerodynamic center for the craft");
                Assert.Near(0, (shown - expected).magnitude, 0.05, "the stock marker at FAR's aerodynamic center");
                // the plane's nose is along the root part's up
                Assert.True(Vector3.Dot(shown - com, root.up) < 0, "the center of lift behind the center of mass");
            }
            finally
            {
                if (!wasShown && marker.gameObject.activeSelf)
                    overlays.toggleCoLbtn.onClick.Invoke();
                if (!comWasShown && overlays.CoMmarker.gameObject.activeSelf)
                    overlays.toggleCoMbtn.onClick.Invoke();
            }
        }
    }
}
