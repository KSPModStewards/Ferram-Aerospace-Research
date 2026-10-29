# Ferram Aerospace Research Continued v0.16.1.2 "Marangoni"

[![Build](https://github.com/dkavolis/Ferram-Aerospace-Research/actions/workflows/package.yml/badge.svg?branch=master)](https://github.com/dkavolis/Ferram-Aerospace-Research/actions/workflows/package.yml)  
Download latest build [here](https://nightly.link/dkavolis/Ferram-Aerospace-Research/workflows/package/master/FerramAerospaceResearch.zip)  

=========================  
Aerodynamics model for Kerbal Space Program

Serious thanks:  

* a.g., for tons of bugfixes and code-refactorings  
* stupid_chris, for the RealChuteLite implementation  
* Taverius, for correcting a ton of incorrect values  
* Tetryds, for finding lots of bugs and issues and not letting me get away with them, and work on example crafts  
* sarbian, for refactoring code for working with MechJeb, and the Module Manager updates  
* ialdabaoth (who is awesome), who originally created Module Manager  
* Regex, for adding RPM support  
* DaMichel, for some ferramGraph updates and some control surface-related features  
* Duxwing, for copy editing the readme  

CompatibilityChecker by Majiir, BSD 2-clause [http://opensource.org/licenses/BSD-2-Clause](http://opensource.org/licenses/BSD-2-Clause)

Part.cfg changes powered by sarbian & ialdabaoth's ModuleManager plugin; used with permission  
[http://forum.kerbalspaceprogram.com/threads/55219](http://forum.kerbalspaceprogram.com/threads/55219)

ModularFLightIntegrator by Sarbian, Starwaster and Ferram4, MIT: [http://opensource.org/licenses/MIT](http://opensource.org/licenses/MIT)  
[http://forum.kerbalspaceprogram.com/threads/118088](http://forum.kerbalspaceprogram.com/threads/118088)

Toolbar integration powered by blizzy78's Toolbar plugin; used with permission  
[http://forum.kerbalspaceprogram.com/threads/60863](http://forum.kerbalspaceprogram.com/threads/60863)

Source available at: [https://github.com/dkavolis/Ferram-Aerospace-Research](https://github.com/dkavolis/Ferram-Aerospace-Research)

## Installation

Install by merging the GameData folder in the zip with the GameData folder in your KSP install

ModuleManager and ModularFlightIntegrator are REQUIRED for FAR to work properly.  Failing to copy this over will result in strange errors.

## Legacy wing configs

Sample Part.cfg:

For wings

```cfg
MODULE  
{  
    name = FARControllableSurface / FARWingAerodynamicModel  
    b_2 = 0.5                       // distance from wing root to tip; semi-span  
    MAC = 0.5                       // Mean Aerodynamic Chord  
    nonSideAttach = 0               // 0 for canard-like / normal wing pieces, 1 for ctrlsurfaces attached to the back of other wing parts  
    TaperRatio = 0.7                // Ratio of tip chord to root chord generally < 1, must be > 0  
    MidChordSweep = 25              // Sweep angle in degrees; measured down the center of the span / midchord position  
    maxdeflect = 15                 // Default maximum deflection value; only used by FARControlableSurface  
    controlSurfacePivot = 1, 0, 0;  // Local vector that obj_ctrlSrf pivots about; defaults to 1, 0, 0 (right)  
    ctrlSurfFrac = 0.2              // Value from 0-1, percentage of the part that is a flap; only used by FARControlableSurface  
}
```

For control surfaces, use above but replace `FARWingAerodynamicModel` with `FARControllableSurface` and add `maxdeflect` value

Set all the other winglet/control surface values to zero

## Development on Linux

If developing on Linux, you will need the following tools:

- [Mono 6](https://www.mono-project.com/download/stable/)
- [msbuild](https://learn.microsoft.com/en-us/visualstudio/msbuild/msbuild?view=visualstudio)
- Python 3

You can also use the included [flake.nix](./flake.nix) to manage your development environment, either using direnv or by running `nix develop`.

You will need an installation of Kerbal Space Program with the following addons:

- [Harmony](https://github.com/KSPModdingLibs/HarmonyKSP)
- [Burst](https://github.com/KSPModdingLibs/KSPBurst)
    - On Linux, only the non-compiler version works
- [ModuleManager](https://ksp.sarbian.com/jenkins/job/ModuleManager/)
- [ModularFlightIntegrator](https://ksp.sarbian.com/jenkins/job/ModularFlightIntegrator/)

You can also install these via CKAN.

Create a file called `Directory.Build.props.user` with the following content:

```
<Project>
  <PropertyGroup>
    <KSP_DIR_BUILD>your/path/to/Kerbal Space Program</KSP_DIR_BUILD>
    <KSP_DATA_DIRNAME>KSP_Data</KSP_DATA_DIRNAME>
  </PropertyGroup>
</Project>
```

and replace `your/path/to/Kerbal Space Program`.

Download the git submodules:

```bash
git submodule update --init --recursive
```

Now, simply run:

```bash
msbuild
```

It should result in FAR being installed into your `GameData`.

## CHANGELOG

0.16.1.2V "Marangoni"------------------------------------  

Fixed `submergedDynamicPressurekPa` and `dynamicPressurekPa` not getting set to 0 when part becomes shielded, thanks [@siimav](https://github.com/siimav) [#145](https://github.com/dkavolis/Ferram-Aerospace-Research/pull/145)  

0.16.1.1V "Marangoni"------------------------------------  

Fixed chute deployment when shielded and the original chute module defined `shieldedCanDeploy = true`, now shielding is respected only for parts with `shieldedCanDeploy = false` or not set  
Fixed invalid aero simulation values in editor  

0.16.1.0V "Marangoni"------------------------------------  

[Experimental, WIP] Rendering based exposed surface computation approach. At the moment, has no effect on physics computations and doesn't work with OpenGL, and thus is disabled by default. Can be enabled from the debug window in flight GUI and contains settings and visualizations only. Has a low-to-moderate performance cost. Requires KSPBurst as dependency and KSP 1.12.  
Respect shielding for chute deployment [4121b20](https://github.com/dkavolis/Ferram-Aerospace-Research/commit/4121b2036c20cf75a4eaac781a5d33bf186f3972)  
Unsubscribe from `onVesselStandardModification` on destruction as `OUnloadVessel` may not be called [efec236](https://github.com/dkavolis/Ferram-Aerospace-Research/commit/efec236f46d7677a4076fbb9235f18bdae2dae01)  
Fixed inconsistent aerodynamic forces between regular launches and reverting when using `GeometryPartModule` with non-default values [2324be4](https://github.com/dkavolis/Ferram-Aerospace-Research/commit/2324be41cd309b4d5957eaa4dd0f52271363899d)  
Added API function to get/set a modifier function for part local forces before applying them [476e99b](https://github.com/dkavolis/Ferram-Aerospace-Research/commit/476e99b84a5bea015de9a71555988234d81dff85)  
Fixed rare NRE in `FARControllableSurface.get_MovableSection` when no control surface transforms where found [#136](https://github.com/dkavolis/Ferram-Aerospace-Research/pull/136). In addition, control surface animations will be disabled for such parts.  
Zero PID error integral if there is any roll input, thanks [@tanihwa](https://github.com/tanihwa) [#134](https://github.com/dkavolis/Ferram-Aerospace-Research/pull/134)  
Find latest binaries in [GitHub Actions](https://github.com/dkavolis/Ferram-Aerospace-Research/actions/workflows/package.yml)  
NRE fixes, thanks [@BrettRyland](https://github.com/BrettRyland) [#133](https://github.com/dkavolis/Ferram-Aerospace-Research/pull/133)  

0.16.0.5V "Mader"------------------------------------  

Fix cfg syntax errors, thanks [@HebaruSan](https://github.com/HebaruSan) [#132](https://github.com/dkavolis/Ferram-Aerospace-Research/issues/132)  
Sanitize vessel filenames, thanks [@HebaruSan](https://github.com/HebaruSan) [#131](https://github.com/dkavolis/Ferram-Aerospace-Research/issues/131)  
FAR aerodynamic are now presets MM-configurable [#130](https://github.com/dkavolis/Ferram-Aerospace-Research/issues/130)  
`_FARPartModule` also recognizes `ModuleRestockLaunchClamp`, thanks [@Gordon-Dry](https://github.com/Gordon-Dry) [#129](https://github.com/dkavolis/Ferram-Aerospace-Research/issues/129)  
Improve `KSPWheel` support for the toggle gear button, thanks [@simmav](https://github.com/siimav) [#128](https://github.com/dkavolis/Ferram-Aerospace-Research/issues/128)  
Fix crash due to log messages from voxelization worker threads  

0.16.0.4V "Mader"------------------------------------  

Fixed NREs with potatoroids, thanks [@simmav](https://github.com/siimav) [#127](https://github.com/dkavolis/Ferram-Aerospace-Research/issues/127)  
Added `massOverride` to `FARWingAerodynamicModel` that fixes the module mass to the set value, with thanks to [@StoneSmile](https://github.com/StonesmileGit) [#126](https://github.com/dkavolis/Ferram-Aerospace-Research/issues/126)  

0.16.0.3V "Mader"------------------------------------  

Updated version file for KSP 1.12 max  
Fixed picking up fairing would invalidate voxelization until reopening VAB/SPH  
Fixed aerodynamic simulations  
Fixed navball displaying wrong speed after switching back and forth from IVA view [#112](https://github.com/dkavolis/Ferram-Aerospace-Research/issues/112)  
Fixed voxelization error when a part is placed in the editor but not attached to a ship  

0.16.0.2V "Mader"------------------------------------  

Updated KSP version 1.11  
Update to MFI 1.2.10

Added Dynamic Deflection for Control Surfaces Option [#110](https://github.com/dkavolis/Ferram-Aerospace-Research/pull/110), [@Alioth81](https://github.com/Alioth81)  
Fixed Mk1pod.v2 voxelization by switching to mesh voxelization, colliders were missing top and bottom planes resulting in shell-only voxelization  
Rotor blades are no longer voxelized  
Reduced submerged lift and drag, multipliers are controlled by `FARConfig/Settings/submerged[Drag|Lift]Multiplier` with default value of 0.25  
Renamed FARWind to FARAtmosphere and added overrides for gas properties  

0.16.0.1V "Mader"------------------------------------  

Resized (again) toolbar icon to 64x64  
All meshes in `TransparentFX` layer are ignored for voxelization, previously only visible non-skinned meshes had this check ([f6a005d](https://github.com/dkavolis/Ferram-Aerospace-Research/commit/f6a005d6759e9de97434ef75d2da336c45931aad))  
Improve async config loading and s


aving ([9e67b32](https://github.com/dkavolis/Ferram-Aerospace-Research/commit/9e67b32878c9fb872515b3f8ea41135ac23a5718))  
Enable debug voxels in flight mode ([5560079](https://github.com/dkavolis/Ferram-Aerospace-Research/commit/5560079e6682e762dfff53ee4422979b90998b69)) with:  

```cfg
@FARConfig:FINAL {
    @Voxelization {
        %debugInFlight = true
    }
}
```

0.16.0.0V "Mader"------------------------------------  

Update to MM 4.1.4  
Update to KSP 1.10  

Added more config options (see `FARConfig.cfg`):

* Additional log messages can be enabled  
* Log FAR flight variables to text file  
* Debug voxels are now colored by part (customizable)  
* Removed `FARGUIColors.cfg` and integrated it into `FARConfig.cfg`  

Added new DLL `FerramAerospaceResearch.Base.dll` which is independent of KSP  
Unchanged configs should no longer be saved to new files  
Update Chinese localization (thanks [@tinygrox](https://github.com/tinygrox), [#88](https://github.com/dkavolis/Ferram-Aerospace-Research/pull/88))  
Workaround for `KSPUpgradePipeline` with removed stock control surface modules  
Fix KSP resetting shielded state on decouple  
Heating should now be closer (and higher) to stock values, KSP applies magic scaling to areas which made FAR calculated exposed areas significantly smaller than stock (can be reverted with MM patch)  
Meshes used for voxelization are now displayed in log with `debug` FAR log level  
Fix inconsistent voxelization when changing from VAB/SPH to Flight scene and reverting ([#89](https://github.com/dkavolis/Ferram-Aerospace-Research/issues/89))  
Fix voxelization starting before part is set up in editor  
Ignore launch clamps in editor simulations  
Fix NRE with MechJeb on destruction ([#93](https://github.com/dkavolis/Ferram-Aerospace-Research/issues/93))  
Resized stock button image to 36x36 px  
Simulation buttons are now disabled while voxelization is active  
Invisible meshes are ignored for voxelization by default  

0.15.11.4V "Mach"------------------------------------  

Update to MM 4.1.3  
Update to KSP 1.9  

Removed CompatibilityChecker  
Debug voxel mesh is now built as a single mesh, with possible submeshes. Choose between 16 (default) and 32 bit vertex indices from the Space Centre settings menu. ([#86](https://github.com/dkavolis/Ferram-Aerospace-Research/issues/86))  

0.15.11.3V "Mach"------------------------------------

Update to MM 4.1.0  
Update to MFI 1.2.7.0  

Update to KSP 1.8  (thanks [@lukecologne](https://github.com/lukecologne))  
Update shaders to Unity 2019.2  
Update .NET to 4.8  

InstallChecker now runs instantly instead of waiting for main menu  

0.15.11.2V "Mach"------------------------------------

Update to MM 4.0.3

Revert NaN stability derivatives if no stable AoA is found [#75](https://github.com/dkavolis/Ferram-Aerospace-Research/issues/75) ([#81](https://github.com/dkavolis/Ferram-Aerospace-Research/pull/81))  
Fix [#74](https://github.com/dkavolis/Ferram-Aerospace-Research/issues/74) ([#76](https://github.com/dkavolis/Ferram-Aerospace-Research/pull/76), [@parachutingturtle](https://github.com/dkavolis/Ferram-Aerospace-Research/issues?q=is%3Apr+author%3Aparachutingturtle))  

0.15.11.1V "Mach"------------------------------------

Fix [#72](https://github.com/dkavolis/Ferram-Aerospace-Research/issues/72) ([#73](https://github.com/dkavolis/Ferram-Aerospace-Research/pull/73))  

0.15.11.0V "Mach"------------------------------------

Chute staging now works with stage lock ([#68](https://github.com/dkavolis/Ferram-Aerospace-Research/pull/68))  
Fix German localization formatting ([#70](https://github.com/dkavolis/Ferram-Aerospace-Research/pull/70), [@HebaruSan](https://github.com/HebaruSan))  
Greatly improve stable angle of attack solver in stability derivative calculation, now works for all cases where stable angle of attack exists and converges faster ([#65](https://github.com/dkavolis/Ferram-Aerospace-Research/pull/65))  
Fix aerodynamic torque simulation and expose total aerodynamic force and torque through API ([#22](https://github.com/dkavolis/Ferram-Aerospace-Research/pull/22), [@BenChung](https://github.com/BenChung))  
Shaders are now platform specific ([#60](https://github.com/dkavolis/Ferram-Aerospace-Research/pull/60))  
Now really fixed ocassional NRE when cleaning up debug voxels ([#59](https://github.com/dkavolis/Ferram-Aerospace-Research/pull/59))  
Fix unsubsribing correct method from `GameEvents.onGUIEngineersReportDestroy` in `EditorGUI` ([#58](https://github.com/dkavolis/Ferram-Aerospace-Research/pull/58))  
Fix assymetrical voxelization on some Mk2/Mk3 adapters [#56](https://github.com/dkavolis/Ferram-Aerospace-Research/pull/56) ([#57](https://github.com/dkavolis/Ferram-Aerospace-Research/pull/57))  
Cache some Unity properties for performance reasons ([#53](https://github.com/dkavolis/Ferram-Aerospace-Research/pull/53))  
Fix Runge Kutta method in transient simulation ([#50](https://github.com/dkavolis/Ferram-Aerospace-Research/pull/50))  
Fix part tinting not cleared after last tint option is disabled ([#49](https://github.com/dkavolis/Ferram-Aerospace-Research/pull/49))  

0.15.10.1V "Lundgren"------------------------------------  

Fix config values reset on load in `GeometryPartModule` [#47](https://github.com/dkavolis/Ferram-Aerospace-Research/pull/47)  
Fix NRE spam on EVA [#43](https://github.com/dkavolis/Ferram-Aerospace-Research/issues/43) ([#44](https://github.com/dkavolis/Ferram-Aerospace-Research/pull/44))  

0.15.10.0V "Lundgren"------------------------------------

**Note for Kopernicus users: DO NOT overwrite MFI that comes with Kopernicus since it is locked to that particular version.**

Recompiled for KSP 1.7

Fix principal axes in simulation and other bugfixes ([#23](https://github.com/dkavolis/Ferram-Aerospace-Research/pull/23), [@Rodhern](https://github.com/Rodhern))  
Fix voxelization of most stock engines ([#39](https://github.com/dkavolis/Ferram-Aerospace-Research/pull/39))  
Voxelization correctly handles jettison transforms with part variants ([#39](https://github.com/dkavolis/Ferram-Aerospace-Research/pull/39))  
Fix voxelization of `InflatableHeatShield` ([#39](https://github.com/dkavolis/Ferram-Aerospace-Research/pull/39))  
Fixed voxelization of simple heat shields ([#37](https://github.com/dkavolis/Ferram-Aerospace-Research/pull/37))  
Added additional nodes to `GeometryPartModule` ([#37](https://github.com/dkavolis/Ferram-Aerospace-Research/pull/37), [#39](https://github.com/dkavolis/Ferram-Aerospace-Research/pull/39), see PRs for more details). The new nodes should allow parts to be voxelized much closer to what is visible and handle animations better:

* `ignoreTransform` (string)
* `ignoreIfNoRenderer` (bool)
* `unignoreTransform` (string)
* `rebuildOnAnimation` (bool)

NRE fixes ([#36](https://github.com/dkavolis/Ferram-Aerospace-Research/pull/36)):  

* Trying to display debug voxels with no parts in the editor
* Occasional  NRE when voxelizing vehicle (due to race conditions?)
* When force closing the game from the editor with debug voxels displayed

Voxelization respects part variant changes in the editor ([#35](https://github.com/dkavolis/Ferram-Aerospace-Research/pull/35))  
Expose IAS and EAS used by FAR in FARAPI ([#34](https://github.com/dkavolis/Ferram-Aerospace-Research/pull/34))  
Enable/disable FAR speed display on the navball from FARAPI ([#33](https://github.com/dkavolis/Ferram-Aerospace-Research/pull/33))  

0.15.9.7V "Lumley"------------------------------------

**Note for Kopernicus users: DO NOT overwrite MFI that comes with Kopernicus since it is locked to that particular version.**

Update to MM 4.0.2

Fix skinned meshes having scale applied twice in voxelization ([#30](https://github.com/dkavolis/Ferram-Aerospace-Research/pull/30))  
Fix new stock parts ([#24](https://github.com/dkavolis/Ferram-Aerospace-Research/pull/24), thanks [@HSJasperism](https://github.com/HSJasperism))  
Greatly improved debug visual voxels framerate ([#18](https://github.com/dkavolis/Ferram-Aerospace-Research/pull/18), [#29](https://github.com/dkavolis/Ferram-Aerospace-Research/pull/29))  
Enlarged debug voxel texture for higher fidelity ([#18](https://github.com/dkavolis/Ferram-Aerospace-Research/pull/18))  

0.15.9.6V "Lin"------------------------------------

**Note for Kopernicus users: DO NOT overwrite MFI that comes with Kopernicus since it is locked to that particular version.**

Update for KSP 1.6  
Update to MM 3.1.2  
Update to MFI 1.2.6  

Mainly a release for RO  
Trying out enabling FAR for KSP 1.4-1.6  

Much nicer looking anti-aliased line plots  
Fix NRE when Trajectories tried to access simulation before the vessel was initialized  
Changed icon to "FARc" to avoid confusion with the original FAR  
Fix MM pass in a config that is only used on first start of FAR  
Fix NRE when trying to save stability augmentation settings on closing KSP  

0.15.9.5V "Lighthill"------------------------------------

Update for KSP 1.5.1  
Update to MM 3.1.0 for KSP 1.5.1  
Update to MFI 1.2.5  

Dealt with NullReferenceException when trying to access part colliders which do
not have any (e.g. fuelLine)  
Dealt with NullReferenceException when trying to determine if an engine has
fairing which can be jettisoned by defaulting to no fairing  
Added '[FAR]' tags to all log messages  
Replaced farshaders.ksp with farassets.ksp which only contains a single
material (source is in Assets)  
Fixed curve colors in transonic design defaulting to purple  
Moved asset bundle from shaders to Assets  
Replaced icons with new ones (source is in icons, feel free to submit better ones)

Removed all All Rights Reserved files  

0.15.9.1V "Liepmann"------------------------------------  

Update for KSP 1.3.1 (though not strictly necessary)  
Update to MM 3.0.4 for KSP 1.3.1  

Added ability to override structural stress values for aerodynamic failures on a per-part basis  
Switch to applying forces through part.AddForce rather than rb.AddForce to allow Principia to handle gravity within atmospheres  
Added functions to KSPAPI to check the status of any vessel's voxelization  

Fix issues with all RealChuteLite chutes having the same exact drag properties  
Fix RealChuteLite GUI not displaying any information  
Remove unnecessary stock lifting body effects on pods  

0.15.9V "Liebe"------------------------------------  

Update for KSP 1.3  
Update to MM 2.8.1  

Include support for localization  
Include German (by terorie), Russian (by pand5461), and Chinese (by Nigh) translations

Fix NaN errors with Trajectories  
Fix some issues with identifying KSPWheel Adjustable Landing Gear as gear  

0.15.8.1V "Lewis"------------------------------------  

Bugfix patch for KSP 1.2.2  

Fix Flight GUI button activated/not activated being backwards  
Don't revoxelize for several B9 and AJE animation modules to reduce lag, thanks blowfish  
Fix game crashing when a vessel landed in water is loaded  

0.15.8V "de Laval"------------------------------------  

Compatibility for KSP 1.2.2 (finally)  
Update to MFI 1.2.4  
Update to MM 2.7.6  

Lots of compatibility changes thanks to Alexander Abramov  
Reduce memory use and garbage production in GUI thanks to soulsource and Virindi-AC  

Fix GUI button multiplication  
Fix stock drag arrows to be useful again  
Fix voxelization errors with some intake parts  
Fix FARAction group settings not saving  
Fix landing gear main axis dtermination  
Fix voxel errors with some stock parts  

Made ignorable transforms for voxelization customizable via config  

0.15.7.2V "Lanchester"------------------------------------  

Fix a serious bug in v0.15.7 and v0.15.7.1 where chutes would not provide any drag  

0.15.7.1V "Kutta"------------------------------------  

Update to MFI 1.1.6 to fix an incompatibility with Kopernicus and the earlier version  
Update CompatibilityChecker version  
Update license  

Fix an issue where voxels could be incredibly asymmetric on symmetric crafts  

0.15.7V "Küchemann"------------------------------------  

Update to ModuleManager 2.6.25  
Update for KSP 1.1.3 compatibility  

Implement higher resolution sub-voxel voxelization method  
Allow switching between high and low res sub-voxel methods  
Optimize voxel shell generation, particularly for high triangle count meshes  
Increase the resistance to sideways aerostructural failures for many fuselage and rocket parts  

Fix voxelization error that would lead to transparent mesh objects being voxelized  
Fix voxelization errors that could lead to incomplete voxelization of some stock procedural fairing shapes  

0.15.6.5V "Knudsen"------------------------------------  

Update to ModularFlightIntegrator 1.1.4  
Fix a serious issue where wings would provide no forces and forces would be distributed incorrectly across vehicles  
Fix an issue where wing symmetry counterparts would not have equal masses  
Fix non-zero convective heat flux on shielded parts  

0.15.6.4V "Kleinhans"------------------------------------  

Fix a no-drag issue with asteroids  
Fix a physics breaking issue with Tweakscaled wing parts, thanks pellinor  
Fix GUI window positions not loading on vessel spawn  
Fix distribution of forces on parts; no change in total force and torque applied to vessel, just to which parts  
Fix slightly negative drag on rearward-facing vehicles at high Knudsen numbers  

0.15.6.3V "Kindelberger"------------------------------------

Recompile for KSP 1.1.2 compatibility  
Bundle ModuleManager 2.6.24 for 1.1.2 compatibility  

Fix a critical error that would cause KerbalEVAs to have no aerodynamic forces applied to them  

0.15.6.2V "Kartveli"------------------------------------

Ensure KSP 1.1.1 compatiblity  
Upgrade to ModuleManager 2.6.23  

Fix new landing gear interfering with main axis determination  
Fix RealChute / RealChuteLite interaction breaking stock chute behavior, thanks to stupid_chris  
Fix mass-calc error for wing-mass-strength that resulted in all planes gaining unhealthy amounts of weight  
Attempt to make debug-compatibility actually work, thanks to NathanKell  

0.15.6.1V "von Kármán"------------------------------------

Fix a critical CPU usage bug that resulted in voxelization threads SpinWaiting forever, monopolizing the processor  
Fix parachutes without RealChute configs not applying forces when FAR + RC are installed, thanks to stupid_chris  
Fix ModuleManager database reload function hanging halfway through, breaking the game, thanks to stupid_chris  

0.15.6V "Jones"------------------------------------  

Update to KSP 1.1  
Update to bundle ModuleManager 2.6.22  
Update to bundle ModularFlightIntegrator 1.1.3  

Updates to RealChuteLite, thanks to stupid_chris  
Compatibility changes for use of KSP debuggers, thanks to neouy  

Increase aerodynamic damping for fuselages to somewhat more realistic levels  
Fix a serious issue that disabled the majority of conduction between parts  

Disable win64 locking

0.15.5.7V "Johnson"------------------------------------  

Tweak pitch and roll damping of fuselages to make more logical sense; excessive roll damping at high dynamic pressures for wingless vehicles has been fixed  
Change units for specific excess power in the Flight Data readout to be W/kg on the basis that it makes more logical sense than m^2/s^3  

Fix a critical error that prevented voxelizations of Kerbals or any vehicles that had Kerbals riding in a command seat  

0.15.5.6V "Jacobs"------------------------------------  

Update to MM 2.6.18

Fix more negative sonic drag issues  
Fix unrealistically low sonic drag  
Fix failure to load saved FAR data in flight  
Fix unrealistically high numbers in indicated airspeed at higher Mach numbers  

Lower critical Mach number for slender vehicles with sudden bulges and waviness in their cross-section  

0.15.5.5V "Hugoniot"------------------------------------  

Fix an inconsistency in calculations of sonic drag  
Fix possibility of sonic drag resulting in negative drag coefficients on very blunt shapes  
Generally increase sonic drag of blunt objects, generally decrease drag of slender objects  

Fix water drag failing to function under complete submersion  
Fix rare error where Procedural Fairings will not properly voxelize  
Fix GetCurrentDensity method (for external mods) to return result consistent with simulation  
Fix overheat interaction on load with ModuleCoreHeat  
Fix FAR breaking on attempts to load Training or Scenario scenes  
Fix spoilers and flaps not updating with settings in the editor  

0.15.5.4V "Hoerner"------------------------------------  

Adjust water drag for better splashdown performance  
Fix a serious voxelization issue with ModuleJettison, most notable in leading to no-drag reentries  
Fix an issue where 3rd-party voxelization updates could sometimes break the editor GUI and CoL  
Fix a serious issue that could lead to spontaneous crashes on aero initialization (either VAB / SPH CoL, editor GUI, or going to flight)  

0.15.5.3V "von Helmholtz"------------------------------------  

Upgrade to MM 2.6.13  

RealChuteLite consistency with RealChute calcs and optimizations thanks to stupid_chris  
Implement dynamic smoothing calculations based on relative "filledness" of voxel; should help reduce effect of voxel-resolution-induced smoothing on larger vehicles  
Tweaks to critical Mach calculations  

Fix "silent" KSP update breaking hydrodynamic drag  
Fix some voxelization irregularities  
Fix control surface flap settings not appearing if the settings are turned on in flight  
Fix some other control surface in-flight changes oddities  

Fix Firehound MS example craft action groups not acting in symmetry  
Added E42 example craft by tetryds  

0.15.5.2V "Helmbold"------------------------------------  

Compatibility with KSP 1.0.5  
Upgrade to MFI 1.1.2  

Optimizations of runtime aerodynamic calculations  
Cut background memory usage for voxels to ~60% of previous value  
Increases in consistency of properties with similar voxel shapes  

Full support for new stock hydrodynamic drag  
Voxel model used in calculating radiative influx from celestial bodies  
Reduction in first-load inconsistency in editor  
More varied support for intake ducting setups, including support for stock Goliath engine  
Tweaks to intake drag at low airbreather throttles

Editor GUI header cleanup  
Dropdowns notated with down triangles for clarity  
Added AoA Arrow to make AoAs for static analysis sweeps and stability deriv sims clearer  

Fix for voxelization issues with degenerate triangles  
Fix for voxelization issues with meshes with 0 triangles  
Fix for B9 pWings not solidifying properly  
Fix editor race condition in displaying sonic drag for vehicles  
Fix for multiple vehicle aerodynamic NREs that could break aero  
Fix for vehicle aerodynamics breaking under certain vessel-part configurations  

Updated FAR Firehound MS, FAR SkyEye, FAR Montauk Shuttle to be more useful in KSP 1.0.5

0.15.5.1V "Hayes"------------------------------------  

Upgrade to MM 2.6.8  

Fix some legacy wing interaction issues  
Fix drag properties drifting slowly over multiple voxelization events due to numerical errors  
Fix parts being occluded when main axis is in a strange orientation  
Fix in-flight control surface tweaks not applying to symmetry counterparts  
Fix KerbalEVAs working with Vanguard Parachutes  

Fix for a critical error where detached boosters, weapons, debris, etc. would not have drag properties  

0.15.5V "Haack"------------------------------------  

Upgrade to MM 2.6.7  
Fix for some RealChute issues by DaMichel  
Animation ignoring for voxelization by Blowfish  
Addition of air brakes for yaw control (RudderBrakes) adopted from original code contributed by HoneyFox  

Reduction in memory garbage created during voxelization; this should reduce the impact of voxelization somewhat, especially hitching.  Note: some hitching may still occur with vehicles with many wing parts due to legacy wing code  
Runtime performance optimizations for ram drag and general aero calculations  
Highly optimized bounds checking for voxelization  

Kerbals handled by voxel model now (using a simple primitive shape, presence or absence of helmet doesn't matter)  
Control surface parameters available for tweaking in flight  
Control surface tweakables given open/close buttons for sections to reduce clutter  
Revert BDArmory bombs and missiles to stock model after launch to improve tracking, stability and predictions  

Fixed main axis issue with BDArmory parts and similarly designed parts  
Fixed a long-standing issue in wing aspect ratio calcs (they were double what they should have been)  
Fixed ram drag variation with throttle not accepting AJE jets as valid jets for that purpose  
Fixed drag not using the calculated critical Mach number for the beginning of the drag transonic rise  
Fixed possible NRE issues during steady destruction of vessel with parts being destroyed  
Fixed issue where increased number of wing parts would cause wing mass adjustment to increase mass without bound; mass is now bounded, but more concentrated in root parts than wingtip parts  

Tweaked subsonic drag downwards to get more accurate results with fixed AJE props
Tweaked wing mass downwards slightly  
Tweaked wing strength power to result in greater strength from lower-mass wings  

0.15.4.1V "Goldstein"------------------------------------  

Re-implementation of aero viz coloration, thanks to mjn33  

Reduction in garbage produced by voxelization, prep for further garbage reductions  

Fixed NaN issue with KAX electric props  
Fixed drag-breaking NRE during rapid disintegrations  
Fixed some issues with Blizzy Toolbar icons  
Fixed exacerbation of stock heating bug  
Fixed control surfaces not updating direction during staging-related CoM shifts  

0.15.4V "Glauert"------------------------------------  

Update to MM 2.6.6  
Update to MFI 1.1.1, fixes gimbaling bug below 750 m/s on short vehicles  
Update win64 check code to MM method  

Added internal ducted area feature:  

* Area ducted through a vehicle from intakes to airbreathing engines will be removed from cross-section  
* Adjusts area ruling to properly model air that flows through the vehicle as opposed to around it  
* Does not count for airflow through switch-backing or reversing ducts; no benefits for intakes that feed upstream engines  
* Supports stock intake part + airbreathing engine part setups, AJE intake part + airbreathing engine part setups, and combined intake + engine part setups  

Slight improvement to Flight Data readouts from mjn33  
Toggle gear button now states "Raise" or "Lower" gear for clarity  

Fixed serious issue where exposed area was not updated for thermal calculations  
Fixed some blunt shapes having NaN drag in the transonic regime  
Fixed some blunt, thin-plate shapes having negative drag in the transonic regime  
Fixed serious issue where long, skinny vehicles would have incorrect 2nd derivatives and incorrect transonic drag as a result  
Fixed NRE with Launch Clamps  
Fixed NRE with animations that are removed from a part (for whatever reason)  

Cleaned up unused values from FARAeroData.cfg  
Added support for planets to be identified by planet name, not just index; combining these in a FARAeroData MM patch is likely to cause overwrites, don't do it  
Added ability to read and set flap and spoiler states from FARAPI  
Fixed Firespitter gear not responding to Toggle Gear button  
Added support for adjustable landing gear in Toggle Gear button  
Stopgap fix to unintended voxelization of USI Warp Drive bubbles  

0.15.3.1V "Garabedian"------------------------------------  

Compatibility with KSP v1.0.3 and thermal changes  
Fix one last error with voxelization breaking due to reverts

Add ability to make parts not count for main axis determination; fix structural panels interfering with proper main axis determination  

0.15.3V "Froude"------------------------------------  

Update to MM 2.6.5 for greater nyan nyan  
Allow display of pressure coefficient (under assumption of axisymmetric flow) over the vehicle  
Tweak subsonic drag to be lower for slender shapes  

Fixed voxelization breaking due to combined memory leak + hard memory limit for voxelization after many editor -> flight cycles  
Fixed some race conditions in voxelization that could break aero properties  
Fixed deadlock in threadpool if many voxelization events triggered simultaneously  
Fixed possibility of deadlock if voxelization settings were updated  

Fixed voxelization errors for some cargo bays and other parts  
Fixed voxelization errors for pWings; includes support for any parts making use of mirrorAxis  

Fixed some longstanding wing interaction issues, including permanent stalled wings  
Fixed a newer issue with wing shielding on symmetry counterparts  
  
Some main axis determination improvements  
Fixed an where certain user atmospheric settings would not take  

0.15.2V "Ferri"------------------------------------  

Improved voxelization accuracy  
Changed CoL code again to try and make it more useful  
Cleaned up some unnecessary calculations  

Fixed voxelization breaking after many voxelization events; this fixes no-drag situations  
Fixed deployed spoilers not producing drag if mounted flush with vehicle  
Fixed some main axis issues  
Fixed improper heating area for atmospheric heat  

Fixed interaction with KIS breaking things  
Fixed some data not saving  
Fixed exceptions during EVA  

0.15.1V "Fanno"------------------------------------  

Fixed improper voxelization of debris and vehicles dropped from existing vessel, including effects on stock "occlusion" system  
Fixed improper determination of vehicle main axis  
Fixed Kerbal EVAs having no drag  
Fixed exceptions where outirght disintegration could prevent some vehicles from having aerodynamics applied  

Added upper cap on memory allocated for voxelization  

Changed calculation of CoL to make more sense  
Fixed error in determining AoA for nominal flight in Stability Derivative GUI  
Hid yellow aero moment arrows by default in aero overlay to reduce user confusion  
Fixed lift / drag arrows remaining on wings that become shielded when aero overlay is open  

Switched to a cleaner method of setting internal speedometers  
Disable control surfaces auto-response below 5 m/s to prevent wacky flailing during load / when stopped  

Change compatibility settings to reject KSP 1.0.0, which is not compatible with RealChuteLite  
Updated save-load method to save more reliably and not throw exceptions  

0.15V "Euler"------------------------------------  

Compatibility with KSP 1.0, 1.0.1, and 1.0.2  
Upgraded to MM 2.6.3  
Introduction of ModularFlightIntegrator for interfacing with KSP drag / heating systems without interference with other mods

Replaced previous part-based drag model with new vessel-centered, voxel-powered model:  

* Generates voxel model of vehicle using part meshes, accounting for part clipping  
* Drag is calculated for vehicle as a whole, rather than linear combination of parts  
* Payload fairings and cargo bays are emergent from code and do not require special treatment with configs  
* Area ruling of vehicles is accounted for; unsmooth area distributions will result in very high drag at and above Mach 1  
* Body lift accounts for vehicle shape in determining potential and viscous flow contributions  
* Areas exposed to outside used for stock heating calculations  

Performance optimizations in legacy wing model  
Jet engine windmilling drag accounted for at intakes  

Editor GUI improvements including:  

* Greater clarity in AoA / Mach sweep tab  
* Stability deriv GUI math modified for improved accuracy  
* Stability deriv simulation tweaked to fix some minor issues in displaying and calculating response  
* Addition of a Transonic Design tab that displays cross-section distribution and drag at Mach 1 for area ruling purposes  

Parachute methods have been replaced with RealChuteLite implementation by stupid_chris:  
    * Less severe parachute deployment  
    * Parachutes melt / break in high Mach number flows  
    * No interference with RealChute  

Changes to FARAPI to get information faster  

FARBasicDragModel, FARPayloadFairingModule, FARCargoBayModule are now obsolete and removed from the codebase  
Extensive reorganizing of source to reduce spaghetti and improve maintainability  

Modifications to Firehound and Colibri to function with new flight model  
Addition of Blitzableiter and SkyEye example crafts  

A 1.5x increase to all stock gimbal ranges  

0.14.7V------------------------------------  
Features:  
Raised stalled-wing drag up to proper maximum levels  
Adjusted intake drag to be lower  
Improved method of dealing with very high vertex count parts for geometry purposes  
Upgraded to MM 2.5.13  
Included FAR Colibri, a VTOL by Tetryds as an example craft  

Bugfixes:  
Fixed an issue preventing loading custom-defined FARBasicDragModels  

0.14.7V------------------------------------  
Features:  
Raised stalled-wing drag up to proper maximum levels  
Adjusted intake drag to be lower  
Improved method of dealing with very high vertex count parts for geometry purposes  
Upgraded to MM 2.5.13  
Included FAR Colibri, a VTOL by Tetryds as an example craft  

Bugfixes:  
Fixed an issue preventing loading custom-defined FARBasicDragModels

0.14.6V------------------------------------  
Features:  
Modified skin friction variation with M and Re to closer to that expected by using the Knudsen number  
Changed saving and loading method to allow better behavior when settings need to be cleaned during updates, especially for automated installs  
Modified aerodynamic failures for water landings for compatibility with upcoming BetterBuoyancy  
Option for aerodynamic failures to result in explosions at the joint during failure.  
Serious reworking to handle edge cases with lightly-clipped parts and their effects on blunt body drag (read: when people clip heatshields into the bottom of Mk1 pods and cause problems)  
Upgrade to MM 2.5.6

Bugfixes:  
Fixed an issue that prevented Trajectories from functioning  
Fixed blunt body drag errors with AJE  
Fixed issues involving editor GUI and control surface deflections  
Fixed edge cases involving attach-node blunt body drag being applied when it shouldn't have  
Fixed issues with command pods containing intakes

0.14.5.1V------------------------------------  
Features:  
Add Reynolds Number readout to main flight GUI

Tweaks:  
Adjust skin friction drag for rarefied atmosphere

Bugfixes:  
Fix Stab Deriv GUI from breaking for altitudes above atmosphere  
Fix flaps and spoilers not functioning with negative deflections

0.14.5V------------------------------------  
Features:  
Skin friction drag now varies with Reynolds number; this means much higher skin friction drags at higher altitudes  
Added simple attempt at handling hydrodynamic effects; not detailed, but objects in oceans move much less  
Added color changing options for colorblind users  
Tweak flap and spoiler deflection functions  
Give spoilers faster deflection coefficients  
Update to ModuleManager 2.5.4

Bugfixes:  
Removed spontaneous aero-spline warp drive in some Linux64 versions

0.14.4.1v------------------------------------  
Features:  
Added changes to blunt body drag to make command pods more stable on reentry  
Attempt to account for most inaccurate effects of part clipping  

0.14.4v------------------------------------
Features:  
Default ActionGroups now controlled throuhg dropdown menus rather than string entry  
Stability Deriv tab now takes entry in terms of planet, altitude and Mach Number, not density, temperature and Mach number  
Stability Deriv tab now accounts for reduced gravity due to high speeds

Contributed by HoneyFox:  
    Pitch damper now has an additional gain for greater tuning  
    Control surfaces can now be set to deflect in response to local AoA changes  
    Control surfaces are not On/Off for a given control direction; can be scaled from -100% to 100% for each  

Contributed by Bitronic:  
    Full Tweakscale Support

BugFixes:  
Fixed no shielding with some payload fairings (particularly resized procedural fairings)  
Fixed aero tinting blocking tinting from other mods

0.14.3.2v------------------------------------  
Features:  
Contributed by Da Michel:  
    Airspeed settings change readouts in cockpits  

Bugfixes:  
Fixed serious issues with the wing interaction code  
Fixed an issue where wind velocity was applied in the opposite direction that was expected  

0.14.3.1v------------------------------------  
Features:  
Improved performance in editor and flight for vessel configuration changes  
Fliht GUI appears in mapview  

Bugfixes:  
Fixed neverending stall resulting from wing interactions with sudden changes in velocity vector direction  
Fixed flight GUI issues when passing another vehicle  

0.14.3v------------------------------------
Features:
Refactored wing interaction code:  
    Wing interactions should be smoother  
    Code should be less processor intensive  

Upgrade to ModuleManager v2.5.1  
Added stall visualization to aero force visualization  
Added ability to scale wing mass up or down for additional strength / weight savings (addedby NathanKell)  
Improved cargo bay and payload fairing detection algorithm  

Tweaks:  
Reduced intake drag  
Decreased wing mass per area slightly  

Bugfixes:
Fixed aero visualization leaving parachutes glowing brightly  
Fixed some critical errors for when config files do not have values listed  
Fixed an issue with AppLauncher buttons multiplying when KSP fails at loading a particular vessel

0.14.2v------------------------------------
Features:
0.25 compatibility, with stock support for SP+ parts  
Upgrade CompatibilityChecker  
Disable functions on CompatibilityChecker warnings

Prototype aero force visualization in flight  
Removed vector from CoL indicator to reduce confusion  
More Get functions for the FARAPI  
Estimated range and endurance readouts in the Flight Data UI  
See and dump FAR module data in the VAB / SPH using the Editor GUI  
Some runtime optimizations  

Contributed by Da Michel:  
    Implement separate deflection speeds for flaps / spoilers  
    Allow preferred default action groups for spoilers / flaps  
Contributed by regex:  
    Add some RPM integration  
Contributed by Ippo:  
    FARWind class for 3rd-party wind implementation

Bugfixes:
Fixed some vessel-switching FAR GUI issues  
Fixed control surface reversal on undocking or backwards root part selection  
Fixed some issues involving CoL position with wings when dealing with parts that have multiple colliders  
Fixed some payload fairing and cargo bay part detection issues  
