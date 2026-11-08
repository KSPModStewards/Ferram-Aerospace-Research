using System.Collections.Generic;
using FerramAerospaceResearch.Geometry.Exposure;
using Unity.Mathematics;
using UnityEngine;
using Object = UnityEngine.Object;

namespace FerramAerospaceResearch.FARGUI;

public class ExposureDebugger : Debugger
{
    private Vector2 labelScrollPosition = Vector2.zero;
    private ArrowPointer debugArrow;
    public RenderResult<Part> LatestResult { get; private set; }
    private static readonly GUILayoutOption[] partListOptions = { GUILayout.Height(250) };
    private static GUIStyle partLabelStyle;

    public bool EnableDebugTexture { get; set; }

    public bool DisplayArrow
    {
        get { return debugArrow.gameObject.activeSelf; }
        set { debugArrow.gameObject.SetActive(value); }
    }

    public Color ArrowColor
    {
        get { return debugArrow.Color; }
        set { debugArrow.Color = value; }
    }

    public ExposureDebugger(Transform parent = null)
    {
        debugArrow = ArrowPointer.Create(parent, Vector3.zero, Vector3.forward, 10f, Color.red, false);
    }

    protected override void Dispose(bool disposing)
    {
        base.Dispose(disposing);

        // the arrow's GameObject (ArrowPointer.Create makes one for it), not only its component
        if (debugArrow != null)
            Object.Destroy(debugArrow.gameObject);
        debugArrow = null;
    }

    public void SetArrowDirection(in float3 position, in float3 forward, float? length = null)
    {
        Transform tr = debugArrow.transform;
        tr.position = position;
        tr.forward = forward;
        if (length is not null)
            debugArrow.Length = length.Value;
    }

    public void SetArrowDirection(RenderResult result)
    {
        SetArrowDirection(result.position, result.forward, result.centerDistance);
    }

    protected override void OnReceiveResult(in RenderRequest request, RenderResult result, RenderTexture tex)
    {
        if (EnableDebugTexture)
            base.OnReceiveResult(request, result, tex);

        LatestResult = result as RenderResult<Part>;

        if (!DisplayArrow)
            return;

        SetArrowDirection(result);
    }

    public void Display(float width, bool partLabels = false)
    {
        if (Texture is null)
            return;

        GUILayout.BeginVertical();

        DrawTexture(width, width * Texture.height / Texture.width);

        if (partLabels && LatestResult is not null)
        {
            partLabelStyle ??= new GUIStyle(GUI.skin.label) { richText = true };
            labelScrollPosition = GUILayout.BeginScrollView(labelScrollPosition, partListOptions);
            string m2 = LocalizerExtensions.Get("FARUnitMSq");
            Color[] colors = ObjectColorArray;
            foreach (KeyValuePair<Part, double> pair in LatestResult)
            {
                Color debugColor = colors[LatestResult.renderer[pair.Key]];
                string partColor = ColorUtility.ToHtmlStringRGBA(debugColor);
                GUILayout.Label($"<color=#{partColor}>■</color> {pair.Key.partInfo.name}: {pair.Value:F3} {m2}",
                                partLabelStyle);
            }

            GUILayout.EndScrollView();
        }

        GUILayout.EndVertical();
    }
}
