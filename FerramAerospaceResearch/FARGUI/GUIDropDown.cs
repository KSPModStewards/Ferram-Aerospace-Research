/*
Ferram Aerospace Research v0.16.2.0 "Maskell"
=========================
Aerodynamics model for Kerbal Space Program

Copyright 2022, Michael Ferrara, aka Ferram4

   This file is part of Ferram Aerospace Research.

   Ferram Aerospace Research is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Ferram Aerospace Research is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with Ferram Aerospace Research.  If not, see <http://www.gnu.org/licenses/>.

   Serious thanks:		a.g., for tons of bugfixes and code-refactorings
				stupid_chris, for the RealChuteLite implementation
            			Taverius, for correcting a ton of incorrect values
				Tetryds, for finding lots of bugs and issues and not letting me get away with them, and work on example crafts
            			sarbian, for refactoring code for working with MechJeb, and the Module Manager updates
            			ialdabaoth (who is awesome), who originally created Module Manager
                        	Regex, for adding RPM support
				DaMichel, for some ferramGraph updates and some control surface-related features
            			Duxwing, for copy editing the readme

   CompatibilityChecker by Majiir, BSD 2-clause http://opensource.org/licenses/BSD-2-Clause

   Part.cfg changes powered by sarbian & ialdabaoth's ModuleManager plugin; used with permission
	http://forum.kerbalspaceprogram.com/threads/55219

   ModularFLightIntegrator by Sarbian, Starwaster and Ferram4, MIT: http://opensource.org/licenses/MIT
	http://forum.kerbalspaceprogram.com/threads/118088

   Toolbar integration powered by blizzy78's Toolbar plugin; used with permission
	http://forum.kerbalspaceprogram.com/threads/60863
 */


using UnityEngine;

namespace FerramAerospaceResearch.FARGUI
{
    internal static class GUIDropDownStyles
    {
        public static readonly GUIStyle List;
        public static readonly GUIStyle ToggleButton;
        public static readonly GUIStyle DropDownItem;

        public static readonly GUIStyle SelectedItem;

        static GUIDropDownStyles()
        {
            List = new GUIStyle(GUI.skin.window) { padding = new RectOffset(1, 1, 1, 1) };
            ToggleButton = new GUIStyle(GUI.skin.button);
            ToggleButton.normal.textColor = ToggleButton.focused.textColor = Color.white;
            ToggleButton.hover.textColor =
                ToggleButton.active.textColor = ToggleButton.onActive.textColor = Color.yellow;
            ToggleButton.onNormal.textColor =
                ToggleButton.onFocused.textColor = ToggleButton.onHover.textColor = Color.green;

            DropDownItem = new GUIStyle(GUI.skin.button)
            {
                padding = new RectOffset(2, 2, 2, 2),
                margin =
                {
                    top = 1,
                    bottom = 1
                }
            };

            SelectedItem = new GUIStyle(GUI.skin.button)
            {
                padding = new RectOffset(2, 2, 2, 2),
                margin =
                {
                    top = 1,
                    bottom = 1
                }
            };
            SelectedItem.normal.textColor = SelectedItem.focused.textColor =
                                                SelectedItem.hover.textColor =
                                                    SelectedItem.active.textColor =
                                                        SelectedItem.onActive.textColor =
                                                            SelectedItem.onNormal.textColor =
                                                                SelectedItem.onFocused.textColor =
                                                                    SelectedItem.onHover.textColor =
                                                                        XKCDColors.KSPNotSoGoodOrange;
        }
    }

    public class GUIDropDown<T>
    {
        private readonly string[] stringOptions;
        private readonly T[] typeOptions;
        private int selectedOption;
        private bool isActive;
        private bool toggleBtnState;
        private Vector2 scrollPos;

        public GUIDropDown(string[] stringOptions, T[] typeOptions, int defaultOption = 0)
        {
            this.stringOptions = stringOptions;
            this.typeOptions = typeOptions;

            selectedOption = defaultOption;
        }

        public T ActiveSelection
        {
            get { return typeOptions[selectedOption]; }
        }

        public void GUIDropDownDisplay(params GUILayoutOption[] guiOptions)
        {
            FARGUIDropDownDisplay display = FARGUIDropDownDisplay.Instance;
            toggleBtnState = GUILayout.Toggle(toggleBtnState,
                                              "▼ " + stringOptions[selectedOption] + " ▼",
                                              GUIDropDownStyles.ToggleButton,
                                              guiOptions);

            // Calculate absolute regions for the button and dropdown list, this only works when
            // Event.current.type == EventType.Repaint
            Vector2 relativePos = GUIUtility.GUIToScreenPoint(new Vector2(0, 0));
            Rect btnRect = GUILayoutUtility.GetLastRect();
            btnRect.x += relativePos.x;
            btnRect.y += relativePos.y;
            var dropdownRect = new Rect(btnRect.x, btnRect.y + btnRect.height, btnRect.width, 150);

            switch (isActive)
            {
                // User activated the dropdown
                case false when toggleBtnState && Event.current.type == EventType.Repaint:
                    ShowList(btnRect, dropdownRect);
                    break;
                // User deactivated the dropdown or moved the mouse cursor away
                case true when (!toggleBtnState || !display.ContainsMouse()):
                    HideList();
                    break;
            }
        }

        private void ShowList(Rect btnRect, Rect dropdownRect)
        {
            if (isActive)
                return;
            toggleBtnState = isActive = true;
            FARGUIDropDownDisplay.Instance.ActivateDisplay(GetHashCode(),
                                                           btnRect,
                                                           dropdownRect,
                                                           OnDisplayList,
                                                           GUIDropDownStyles.List);
            InputLockManager.SetControlLock(ControlTypes.All, "DropdownScrollLock");
        }

        private void HideList()
        {
            if (!isActive)
                return;
            toggleBtnState = isActive = false;
            FARGUIDropDownDisplay.Instance.DisableDisplay();
            InputLockManager.RemoveControlLock("DropdownScrollLock");
        }

        private void OnDisplayList(int id)
        {
            GUI.BringWindowToFront(id);
            scrollPos = GUILayout.BeginScrollView(scrollPos, GUIDropDownStyles.List);
            for (int i = 0; i < stringOptions.Length; i++)
            {
                // Highlight the selected item
                GUIStyle tmpStyle =
                    selectedOption == i ? GUIDropDownStyles.SelectedItem : GUIDropDownStyles.DropDownItem;
                if (!GUILayout.Button(stringOptions[i], tmpStyle))
                    continue;
                FARLogger.Info("Selected " + stringOptions[i]);
                selectedOption = i;
                HideList();
            }

            GUILayout.EndScrollView();
        }
    }

    [KSPAddon(KSPAddon.Startup.MainMenu, true)]
    public class FARGUIDropDownDisplay : MonoBehaviour
    {
        private Rect btnRect;
        private Rect displayRect;
        private int windowId;
        private GUI.WindowFunction windowFunction;
        private GUIStyle listStyle;
        public static FARGUIDropDownDisplay Instance { get; private set; }

        private void Awake()
        {
            Instance = this;
        }

        private void Start()
        {
            enabled = true;
            DontDestroyOnLoad(this);
        }


        private void OnGUI()
        {
            GUI.skin = HighLogic.Skin;
            if (windowFunction != null)
                displayRect = GUILayout.Window(windowId,
                                               displayRect,
                                               windowFunction,
                                               "",
                                               listStyle,
                                               GUILayout.Height(0));
        }

        public bool ContainsMouse()
        {
            return btnRect.Contains(GUIUtils.GetMousePos()) || displayRect.Contains(GUIUtils.GetMousePos());
        }

        public void ActivateDisplay(int id, Rect buttonRect, Rect rect, GUI.WindowFunction func, GUIStyle style)
        {
            windowId = id;
            btnRect = buttonRect;
            displayRect = rect;
            windowFunction = func;
            listStyle = style;
        }

        public void DisableDisplay()
        {
            windowFunction = null;
        }
    }
}
