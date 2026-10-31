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

using System.Collections.Generic;
using UnityEngine;

namespace FerramAerospaceResearch
{
    namespace PartExtensions
    {
        public static class FARPartExtensions
        {
            // ReSharper disable once UnusedMember.Global
            public static Collider[] GetPartColliders(this Part part)
            {
                Collider[] colliders;
                try
                {
                    if (HighLogic.LoadedSceneIsEditor)
                    {
                        //In the editor, this returns all the colliders of this part AND all the colliders of its children, recursively
                        Collider[] tmpColliderArray = part.GetComponentsInChildren<Collider>();
                        //However, this can also be called on its child parts to get their colliders, so we can exclude the child colliders
                        //Also, fortunately, parent colliders are at the beginning of this; we can take advantage of this to reduce the time iterating through lists
                        var partColliders = new List<Collider>();
                        //We'll use a hash to make this fast
                        var excludedCollidersHash = new HashSet<Collider>();

                        foreach (Part p in part.children)
                        {
                            //All the colliders associated with the immediate child of this part AND their children
                            Collider[] excludedColliders = p.GetComponentsInChildren<Collider>();

                            //The first collider _must_ be part of the immediate child; because it is closer to the parent, it will appear earlier in tmpColliderArray
                            if (!excludedCollidersHash.Contains(excludedColliders[0]))
                                //That means we only ever need the first collider for our purposes
                                excludedCollidersHash.Add(excludedColliders[0]);
                        }

                        foreach (Collider collider in tmpColliderArray)
                            //If the collider isn't in the hash, that means that it must belong to _this_ part, because it doesn't belong to any child parts
                            if (!excludedCollidersHash.Contains(collider))
                                partColliders.Add(collider);
                            else
                                //Once we find something that is in the hash, we're out of the colliders associated with the parent part and can escape
                                break;

                        colliders = partColliders.ToArray();
                    }
                    else
                    {
                        colliders = part.GetComponentsInChildren<Collider>();
                    }
                }
                catch
                {
                    //FIXME
                    //Fail silently because it's the only way to avoid issues with pWings
                    colliders = new[] {part.collider};
                }

                return colliders;
            }

            // ReSharper disable once UnusedMember.Global
            public static Bounds[] GetPartMeshBoundsInPartSpace(this Part part, int excessiveVerts = 2500)
            {
                List<Transform> transforms = part.FindModelComponents<Transform>();
                var bounds = new Bounds[transforms.Count];
                Matrix4x4 partMatrix = part.partTransform.worldToLocalMatrix;
                for (int i = 0; i < transforms.Count; i++)
                {
                    var newBounds = new Bounds();
                    Transform t = transforms[i];

                    var mf = t.GetComponent<MeshFilter>();
                    if (mf == null)
                        continue;
                    Mesh m = mf.sharedMesh;

                    if (m == null)
                        continue;
                    Matrix4x4 matrix = partMatrix * t.localToWorldMatrix;

                    if (m.vertices.Length < excessiveVerts)
                        foreach (Vector3 vertex in m.vertices)
                            newBounds.Encapsulate(matrix.MultiplyPoint(vertex));
                    else
                        newBounds.SetMinMax(matrix.MultiplyPoint(m.bounds.min), matrix.MultiplyPoint(m.bounds.max));

                    bounds[i] = newBounds;
                }

                return bounds;
            }

            /// <summary>
            ///     Returns the total mass of the part
            /// </summary>
            public static float TotalMass(this Part part)
            {
                return part.physicalSignificance != Part.PhysicalSignificance.NONE
                           ? part.mass + part.GetResourceMass()
                           : 0;
            }

            /// <summary>
            ///     Initiates an animation for later use
            /// </summary>
            /// <param name="part">Animated part</param>
            /// <param name="animationName">Name of the animation</param>
            public static void InitiateAnimation(this Part part, string animationName)
            {
                foreach (Animation animation in part.FindModelAnimators(animationName))
                {
                    AnimationState state = animation[animationName];
                    state.normalizedTime = 0;
                    state.normalizedSpeed = 0;
                    state.enabled = false;
                    state.wrapMode = WrapMode.Clamp;
                    state.layer = 1;
                }
            }

            /// <summary>
            ///     Plays an animation at a given speed
            /// </summary>
            /// <param name="part">Animated part</param>
            /// <param name="animationName">Name of the animation</param>
            /// <param name="animationSpeed">Speed to play the animation at</param>
            public static void PlayAnimation(this Part part, string animationName, float animationSpeed)
            {
                foreach (Animation animation in part.FindModelAnimators(animationName))
                {
                    AnimationState state = animation[animationName];
                    state.normalizedTime = 0;
                    state.normalizedSpeed = animationSpeed;
                    state.enabled = true;
                    animation.Play(animationName);
                }
            }

            /// <summary>
            ///     Skips directly to the given time of the animation
            /// </summary>
            /// <param name="part">Animated part</param>
            /// <param name="animationName">Name of the animation to skip to</param>
            /// <param name="animationSpeed">Speed of the animation after the skip</param>
            /// <param name="animationTime">Normalized time skip</param>
            public static void SkipToAnimationTime(
                this Part part,
                string animationName,
                float animationSpeed,
                float animationTime
            )
            {
                foreach (Animation animation in part.FindModelAnimators(animationName))
                {
                    AnimationState state = animation[animationName];
                    state.normalizedTime = animationTime;
                    state.normalizedSpeed = animationSpeed;
                    state.enabled = true;
                    animation.Play(animationName);
                }
            }
        }
    }
}
