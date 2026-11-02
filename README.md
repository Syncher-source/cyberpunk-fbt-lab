# cyberpunk-fbt-lab

The lab for Cyberpunk 2077's full-body tracking in VR: V's full-body IK outside the game. A desktop and VR tool to
check the tracking (waist and foot trackers, controllers or gloves, headset) on V's real skeleton without launching the
game, and to record a session so that what you saw can be replayed and analysed offline.

[![The lab in action (YouTube)](https://img.youtube.com/vi/OzhYR3ugzLk/maxresdefault.jpg)](https://www.youtube.com/watch?v=OzhYR3ugzLk)

**A recording plus your rig file is a complete bug report.** The replay tells, frame by frame, whether a
jump came from a device, from a limb at the end of its reach, or from the solver itself.

The IK is [wrpuppets-fbt-ik](https://github.com/Syncher-source/wrpuppets-fbt-ik), the submodule
`external/wrpuppets-fbt-ik`: a generic library, so the lab runs the same code as the game, and it works on other
humanoid skeletons too (see [Other skeletons](#other-skeletons)); `samples/humanoid_rig.txt` lets it start without the
game.

## Build

Windows, Visual Studio 2022 or later, CMake 3.24 or later, git (CMake fetches ImGui, ImGuizmo, cgltf and
the OpenXR SDK).

```
git clone --recursive https://github.com/Syncher-source/cyberpunk-fbt-lab.git
cd cyberpunk-fbt-lab
cmake -S . -B build -A x64
cmake --build build --config Release
```

Cloned without `--recursive`: `git submodule update --init`. A checkout of the library elsewhere:
`-DWRPUPPETS_FBT_IK_DIR=<path>`.

Produces `build\Release\fbt_ik_lab.exe` and `fbt_ik_test.exe`.

## Files it needs

- **The rig file**, the lab's first argument (default: `fbt_rig_male.txt` in the working folder). The Cyberpunk VR
  mod's plugin writes `fbt_rig_male.txt` / `fbt_rig_female.txt` into the game's `bin\x64` when V's rig loads (once
  per body); copy it anywhere, e.g. `data\rigs\`. Without the game: `fbt_ik_lab.exe samples\humanoid_rig.txt`, a
  generic 1.75 m humanoid.
- **V's body mesh (optional).** The game's player body, exported with WolvenKit to glTF binary (`.glb`), expected
  under `<rig folder>\..\mesh\base\characters\common\player_base_bodies\`; the lab picks the body by the rig file's
  name (`female` in it: the female body). Male: `player_man_average\t0_000_pma_base__full.glb`,
  `arms_hq\a0_000_pma_base_hq__l.glb`, `..._r.glb`, `i0_000_pma_base_full_censored.glb`; female: the same in
  `player_female_average` with `pwa` for `pma`. With the WolvenKit console (both bodies):
  ```
  WolvenKit.CLI.exe uncook "<game>\archive\pc\content" -o <rig folder>\..\mesh
      -r "player_(man|female)_average\\(t0_000_pw?ma_base__full|arms_hq\\a0_000_pw?ma_base_hq__[lr]|i0_000_pw?ma_base_full_censored)\.mesh$"
      --mesh-export-type MeshOnly --mesh-export-lod-filter
  ```
  These are game files: export your own, do not share them. Without them the lab shows the robot of boxes.
  Female V's default body has the lifted feet the game shows in shoes (the heel 5 cm off the floor); barefoot the game
  swaps in `l0_000_pwa_base__cs_flat`, flat feet around the same bones, so the skeleton -- and the IK -- is the same.
- **Recordings** are written to `<rig folder>\..\recordings\fbt_<date>_<time>.txt`.

## Without VR

The six goals (head, hands, hips, feet) sit on V's reference pose; drag them with the gizmo (left click
near a goal selects it; W move, E rotate, R reset; "local"/"world"). Right drag orbits, middle drag pans,
the wheel zooms. "robot" draws one box per bone segment, each face tinted by its axis, so a bone that rolls
or flips shows at once; "skeleton" and "helper bones" draw the stick figure.

The yellow arrows are the knee planes, the orange ones the elbow directions. The "stack" section switches
each solver on and off and shows what it decided (reach, spine passes, arm scale, clavicle swing).

## In VR

1. SteamVR running, trackers on (waist, left foot, right foot roles assigned in SteamVR), controllers or
   gloves on. "Start VR".
2. Calibration (V is tinted green): stand at the room's centre, straight, arms relaxed along the body,
   look down, pull **both triggers**. That captures the hips, feet and head.
3. Then raise both arms straight out to the sides and pull both triggers once more: that measures your
   arms (V's arms are scaled to them). Only the instant counts.
4. V follows you ("live"). The mirror is 1.5 m in front (slider). The flight recorder runs from the capture.
5. Both triggers again start a new calibration; "Stop VR" saves the recording.

The "devices" boxes show each device in its own frame (controllers yellow, trackers green, headset white);
"puck up" picks which axis comes out of the trackers' large faces, so a tracker mounted oddly is obvious.

## Replay and tests

```
fbt_ik_test.exe --replay <recording> <rig file> [--nofilter]
```
Runs the recording through the same calibration, filter and IK and prints: devices lost and their biggest
steps, the arms' wrist bend and twist, V's elbow bend where your arm was straight, the elbow's height with
the hands in front, out-of-reach counts, how far the hips gave way, the count of bone steps over 30° and
over 90° (flips), then the 20 worst frames, each classed `INPUT` (a device jumped), `REACH` (a limb at the
end of its length) or `SOLVER` (the IK jumped on smooth input). `FBT_DUMP_FRAMES=from-to` prints per-frame
detail for a range; other `FBT_*` environment variables override solver and filter settings for tuning
(see the top of `ik_test.cpp`).

```
fbt_ik_test.exe <rig file>
```
The stress test: bone lengths kept, goals reached, hinges aligned, knees forward, continuity over sweeps
(feet, crouch, steps, hips yaw, head leans and slouches, arm arcs), 20,000 random poses, a synthetic
calibration. Every line should say PASS.

```
fbt_ik_test.exe --mesh <rig file> <glb>...
```
Checks an exported body against the rig: rigid parts keep their distance to their joint, the trunk's flesh
sits front/back of the spine, the feet's forward of the ankles.

## Reporting a problem

Start VR, calibrate, watch yourself in the mirror and make the moves that go wrong in the game; then "Stop VR", which saves the
recording. Attach the recording (`fbt_<date>_<time>.txt`) and your rig file, say what you did at the time it looked
wrong (the replay prints the time of each bad frame) and which devices you used. A recording holds only device poses
and your calibration: no images, no audio.

## Other skeletons

The lab is tailored to Cyberpunk 2077's V, but forking it for another skeleton is welcome: the IK only needs a rig
file, whose format, axes, T-pose and roles are in the library's README,
[The rig file](https://github.com/Syncher-source/wrpuppets-fbt-ik#the-rig-file). `samples/make_humanoid_rig.py`
writes `samples/humanoid_rig.txt` and is the template for an exporter. To check a rig: `fbt_ik_test.exe <rig file>`
(every line should say PASS), then the lab with a headset.

What in the lab is generic and what is V's:

| Part | Generic | Notes |
|---|---|---|
| The IK, calibration, filters (the library), recordings, replay and stress tests | yes | any skeleton through its rig file |
| Devices | yes | plain OpenXR: headset, controllers (grip pose), Vive trackers (`XR_HTCX_vive_tracker_interaction`), STAGE space |
| The robot of boxes | yes | fingers and heels/toes are drawn when bones with Cyberpunk 2077's names exist; otherwise a palm box and an estimated foot |
| V's body mesh (`VMesh`) | no | WolvenKit's glTF export of the player body (its axes and file names); optional |

## Coordinates

Everything is in V's model space: x right, y forward, z up, metres. SteamVR's STAGE space (x right, y up,
−z forward) maps onto it as (x, −z, y).

## Licence

MIT (see LICENSE). The library in the submodule is MIT as well.
