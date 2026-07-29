# radarays_gazebo_plugins Migration Notes

For the current cross-package session handoff, read:

- [`../../MIGRATION_HANDOFF.md`](../../MIGRATION_HANDOFF.md) (at the workspace root, two levels up from this package)

This file tracks the next migration phase after the Harmonic Embree prototype was
validated in `rmagine_gazebo_plugins`.

## Current State

`radarays_gazebo_plugins` has a working ROS 2 / Gazebo Harmonic path with
real radar-specific runtime: `/radar/scan` and `/radar/points` delegated to
`rmagine_gazebo_plugins`, and `/radar/image` generated natively by this
package's own `radarays_embree_sensor_system` (real wave-propagation
physics, not a bridge), including per-visual `<radarays_material>` parsing,
live ROS 2 parameter tuning (the `dynamic_reconfigure` equivalent), and
dynamic (moving-geometry) scenarios — single-object translation
(`dynamic_cpu`) and multi-object translation+rotation with per-object
materials (`dynamic_multi_cpu`).

- `package.xml` uses `ament_cmake`
- `CMakeLists.txt` builds/installs the Harmonic asset/launch/plugin path
- the physics itself now comes from `radarays_ros`'s exported
  `radarays_core` library, not a local copy — this package used to carry
  its own from-scratch `radar_algorithms.hpp` (needed while `radarays_ros`
  was still ROS1), now deleted; see `radarays_ros/MIGRATION.md`, "Two
  independent physics copies", for the how/why
- legacy Classic source files (`radarays_embree_gzplugin.*`,
  `radarays_optix_*`) remain in-repo for reference but are not part of the
  ROS 2 build
- GPU/OptiX radar path: done (`radarays_optix_sensor_system`), see below

This package is no longer the blocker for a real Harmonic radar scenario —
it produces genuine `/radar/scan`, `/radar/points`, and material-aware,
live-tunable, dynamic-geometry-capable `/radar/image` output, on both the
CPU/Embree and GPU/OptiX backends.

### GPU/OptiX radar path (`radarays_optix_sensor_system`)

Mirrors `radarays_embree_sensor_system` almost exactly -- same SDF/ROS 2
parameter surface, same per-visual `<radarays_material>` lookup via the
`generate_world_sdf` service, same denoising/ambient-noise/normalization
math. The physics loop is the real difference: instead of a host-side
per-wave loop against Embree, it runs radarays_ros's GPU CUDA kernels
(`move_waves`/`signal_shader`/`fresnel_split`, from the new
`radarays_gpu_core` target -- see `radarays_ros/MIGRATION.md`) in a
multi-pass, doubling-buffer loop generalized from radarays_ros's `RadarGPU`
(which hardcodes exactly 3 passes; this system supports the full
`n_reflections` SDF range, 1-20, with a debug-log warning above 6 passes
since VRAM cost doubles per pass with no compaction of dropped-below-
threshold waves).

Real per-visual materials needed a GPU-buffer translation: the SDF-parsed
materials (keyed by `object_id + 1`, air is `0`) get uploaded as a dense
`rm::Memory<radarays_ros::RadarMaterial, VRAM_CUDA>` array plus a matching
`object_materials` index array, rebuilt whenever the map's object count
changes (not every frame).

**Two real bugs found and fixed, both verified with a standalone repro
built outside gz-sim entirely** (rules out any gz-sim/cross-.so
involvement, and iterates far faster than a full sim launch):

1. The materials-not-ready-yet fallback built a **zero-length**
   `object_materials` array while the map already had real geometry (a
   valid object id 0) -- the GPU kernels then read `object_materials[0]`
   out of an empty device buffer, an illegal memory access. Fixed by always
   sizing the GPU material buffers from the map's actual object count
   (via `rmagine_gazebo_plugins`'s `OptixMapRegistry::GetObjectEntities()`),
   independent of whether the `<radarays_material>` SDF fetch has succeeded
   yet -- that fetch still gates *when materials stop being all-default*,
   just not *whether the buffers are validly sized*.
2. A real bug in `rmagine_optix_map_system.cpp` itself (not this package):
   see `rmagine_gazebo_plugins/README.md`, "OptiX / GPU Harmonic port",
   bug #3 -- its MESH case added the `OptixMesh` directly to the scene
   instead of wrapping it in an `OptixInst` like the primitive-shape cases
   already do, and this was the *first* time that MESH case had ever
   actually been exercised (every earlier GPU milestone only used `<box>`
   geometry). Fixed there, not here, but discovered via this package's
   testing.

**Runtime-verified against the real GPU**, not just a compile check: new
world `worlds/gz_static_radar_gpu.sdf` (straight port of
`gz_static_radar_cpu.sdf`), launched under
`gz sim -s -r --headless-rendering` against the same `avz_no_roof.stl`
mesh with a real `<radarays_material>` tag. Subscribed to `/radar/image`
directly (matching QoS -- the publisher uses `SensorDataQoS`, a plain
reliable subscription silently receives nothing) and computed real pixel
statistics: 1024x400 `mono8`, 100% nonzero, min 3 / max 255 / mean ~20.6 --
physically plausible, non-degenerate output. Also verified
`rmagine_optix_sensor_system` and `radarays_optix_sensor_system` running
**together** in the same world (matching `gz_static_radar_cpu.sdf`'s own
CPU-side precedent of running both the generic and radar-specific sensor
systems at once) -- `/radar/scan`, `/radar/points`, and `/radar/image` all
publish real data simultaneously with no crash.

## What Is Already Available Upstream

From `rmagine_gazebo_plugins`, the workspace now has a Harmonic Embree baseline for:

- building an Embree map from gz-sim visuals
- publishing `sensor_msgs/msg/LaserScan`
- publishing `sensor_msgs/msg/PointCloud2`
- reacting to dynamic obstacle updates in controlled validation worlds
- validating the backend with:
  - `gz_embree_baseline.sdf`
  - `gz_embree_dynamic.sdf`
  - fixture capture / compare tooling

That means `radarays_gazebo_plugins` should not reimplement the whole scene / map
management stack unless there is a clear technical reason to do so.

## First Harmonic Build Path

The repository now includes a first ROS 2 / Harmonic integration scenario:

- `worlds/gz_static_radar_cpu.sdf`
- `launch/gz_static_radar_cpu.launch.py`

This is deliberately thin:

- the world and launch assets live in `radarays_gazebo_plugins`
- the runtime sensor/map implementation is delegated to
  `rmagine_gazebo_plugins`
- the published outputs are the generic baseline topics:
  - `/radar/scan`
  - `/radar/points`

This is the initial bridge from the old package structure into the new Harmonic stack.

It now also has a committed regression guard:

- script entrypoints:
  - `capture_radarays_fixture static_cpu`
  - `compare_radarays_fixture static_cpu`
- fixture:
  - `testdata/radarays_harmonic/static_cpu_fixture.json`
- measured output:
  - `/tmp/radarays_harmonic_static_cpu_capture.json`
- wired into `colcon test` (`ament_add_test`, see `CMakeLists.txt`) for all
  three fixtures (`static_cpu`, `dynamic_cpu`, `dynamic_multi_cpu`) — real,
  aggregated tests via `colcon test-result`, not just a hand-run or
  CI-script invocation. See `MIGRATION_HANDOFF.md`, "Wired fixtures into
  colcon test".

And it now exposes a real radar-specific ROS 2 output:

- `/radar/image`

Current implementation status:

- produced by the `radarays_embree_sensor_system` gz-sim system
  (`src/gz/radarays_embree_sensor_system.cpp`), not a scan-derived bridge
- runs the actual radar wave-propagation physics ported from Classic
  `radarays_ros` (multi-bounce Fresnel reflection/refraction, cone-sampled
  beams, signal denoising, ambient noise) against the delegated Embree map
- the old `scripts/radarays_scan_image_node.py` bridge is retired (kept
  in-repo, no longer built/launched)
- per-visual `<radarays_material>` tags are parsed and applied (velocity,
  ambient, diffuse, specular) via `rmagine_gazebo_plugins`'s object id ->
  Entity mapping plus gz-sim's `generate_world_sdf` service; objects
  without a tag fall back to a shared default material
- 24 radar-physics params (`n_reflections`, `signal_denoising`,
  `ambient_noise`, ...) are live-tunable ROS 2 parameters with range
  validation — the `dynamic_reconfigure` equivalent. Structural params
  (frame/topic names, angular sampling, map key) stay SDF-only.
- GPU/OptiX path (`radarays_optix_sensor_system`) done, same parameter
  surface — see above

## Migration Direction

Recommended direction:

1. Port this package from `catkin` to `ament_cmake`.
2. Replace Classic Gazebo integration with Gazebo Harmonic (`gz-sim`) systems or
   plugins.
3. Reuse the validated Harmonic Embree backend from `rmagine_gazebo_plugins`.
4. Keep this package as thin as possible:
   - radar-specific integration
   - radar-specific message / image publication
   - radar-specific material / parameter handling
5. Avoid duplicating:
   - map registry logic
   - Embree map rebuild logic
   - Embree sensor refresh lifecycle

## Responsibility Audit

Current Classic source split:

- `radarays_embree_gzplugin.*`
  - radar-specific simulation core
  - dynamic reconfigure
  - radar material lookup from SDF
  - production of the polar radar image
- `radarays_embree_ros_gzplugin.*`
  - ROS 1 image publication
- `radarays_embree_gzregister.cpp`
  - Classic static sensor registration
- `radarays_optix_*`
  - GPU/OptiX variants of the same responsibilities

Migration implication:

- generic map + scan/point publishing should stay delegated to
  `rmagine_gazebo_plugins`
- radar image generation and radar material semantics both now have a real
  Harmonic-native implementation (`radarays_embree_sensor_system`); dynamic
  parameter reconfiguration is the remaining piece from this list

## Immediate Engineering Milestone

The next practical milestone is not “full migration”.

It is:

- build `rmagine`
- build `rmagine_gazebo_plugins`
- build the first Harmonic-ready version of `radarays_gazebo_plugins`
- launch one real target scenario in the workspace
- verify that the integrated radar layer produces plausible ROS 2 outputs

Use one real scenario from this package, not a synthetic toy world, as the first
integration target.

Recommended default:

- CPU / Embree path only
- one real radar model mounting
- one real world / launch path
- one downstream consumer or topic check

## Main Work Items

### 1. Packaging and build migration

- convert `package.xml` to ROS 2 / `ament_cmake`
- replace `gazebo_ros` / Classic dependencies with Harmonic dependencies
- define a Harmonic-only build path first
- keep Classic support only if it is explicitly required

Status:

- first step done
- package now builds and installs the first Harmonic asset/launch path
- the first Harmonic scenario now has a stored capture/compare regression fixture
- Classic source files remain in-repo but are not part of the ROS 2 build

### 2. Plugin responsibility inventory

Current Classic package responsibilities include:

- radar sensor generation
- ROS image publication
- registration behavior for custom sensors
- radar-specific materials and parameter handling

For each responsibility, decide whether it should:

- move into a new Harmonic system inside this package, or
- delegate to the already migrated systems in `rmagine_gazebo_plugins`

Recommended default:

- delegate map and generic range-sensor lifecycle to `rmagine_gazebo_plugins`
- keep only radar-specific logic here

### 3. Real integration target

Choose one concrete scenario from this package and make it the first migration
target:

- one URDF/Xacro
- one world
- one launch path
- one expected ROS topic or output artifact

That scenario becomes the first end-to-end proof that the Harmonic prototype is
usable in the actual radar simulation stack.

### 4. Follow-on package dependency

`radarays_ros` is still ROS 1 / catkin as well.

It should be treated as the next package after this one unless the Harmonic Gazebo
port is blocked by a missing ROS 2-side interface and forces earlier ROS-side work.

## Not the Immediate Next Focus

These still matter, but they should remain secondary while this package is being
ported:

- project-grade TF ownership strategy
- broader Classic parity beyond the chosen real integration scenario
- performance optimization beyond correctness
- CI-grade automation across the full migrated stack

## Practical Conclusion

At this point:

- `rmagine_gazebo_plugins` is the validated Harmonic Embree prototype baseline
- `radarays_gazebo_plugins` is the next migration target
- the best next step is one real Harmonic integration scenario through this package

## Status update — everything above is now historical planning

Everything above this line was written before the actual Harmonic port
happened and is now stale in places (e.g. "radarays_ros is still ROS
1/catkin", "the next migration target" — both long since done). Kept for
history; see the top of this file / README.md for current state
(CPU+Embree and GPU+OptiX radar sensor systems both built, verified, and
covered by `colcon test` fixtures).

## Known Gaps vs Gazebo Classic (systematic audit)

A dedicated audit (comparing this package's current `src/gz/*` Harmonic
System code against the Classic/ROS1 plugin code still sitting unbuilt in
this same repo — `radarays_embree_gzplugin.*`,
`radarays_embree_ros_gzplugin.*`, `radarays_embree_gzregister.cpp`, and
their OptiX equivalents) found this package genuinely did have a working
Classic/ROS1 release (contrary to an initial guess that it might not
have) — that code is real, complete, and just unbuilt/dead now. One
concrete, real gap surfaced:

- ~~**No moving-sensor (egomotion) scenario has ever been ported or
  tested.**~~ — done: `worlds/gz_egomotion_radar_{cpu,gpu}.sdf`, new. The
  sensor's own model (`sensor_model`) is non-static with
  `test_box_mover_system` attached directly to it (oscillating translation
  + rotation), while the map (`avz_no_roof.stl`) stays static — the
  inverse of the existing "dynamic" worlds, which move an obstacle while
  the sensor stays put. Deliberately generic/world-independent (not tied
  to any specific real vehicle), per the project's current scope: build
  and test the migration on its own, without needing the eventual real
  integration target's own world files.

  Turned out to need **zero changes** to
  `radarays_embree_sensor_system`/`radarays_optix_sensor_system` or
  `rmagine_embree_sensor_system`/`rmagine_optix_sensor_system` — both
  already query the sensor's world pose fresh from the ECS every tick
  (`gz::sim::worldPose(sensor_entity_, _ecm)`), so a physics- or
  plugin-driven moving model is handled correctly with no code path
  assuming a static sensor. The gap really was "never tested," not
  "broken."

  One real bug surfaced building the test world itself (not in the
  migrated code): the `radar` link, rigidly attached to `base_link` via a
  fixed joint, had no `<gravity>` tag of its own -- only `base_link` did.
  With `sensor_model` changed from `<static>true</static>` to
  `<static>false</static>` (needed so `test_box_mover_system`'s
  `WorldPoseCmd` has a real physics body to act on), the `radar` link fell
  under gravity (observed sinking to z=-4.2m within a few seconds on the
  GPU world) despite `base_link`'s own gravity being disabled -- fixed by
  adding `<gravity>false</gravity>` (and matching `<velocity_decay>`) to
  the `radar` link too.

  **Runtime-verified**, both CPU and GPU: `base`/`sensor` pose tracked the
  commanded oscillation exactly (`x` swept the full commanded -1.0..1.0m,
  confirmed via live `/tf`), height stayed stable at the commanded 0.5m
  (no drift, post-fix), and both `/radar/scan`+`/radar/points`
  (`rmagine_*_sensor_system`) and `/radar/image`
  (`radarays_*_sensor_system`) tracked the motion smoothly and physically
  plausibly the whole time (min/max lidar-style range swinging
  0.2m-14.7m as the sensor swept toward/away from walls; radar image mean
  intensity continuously varying ~18.8-19.9 in sync with the oscillation
  period) -- not a single crash, NaN, or discontinuity across multiple
  full oscillation periods on either backend.

  Not yet wired into `colcon test` as a committed regression fixture
  (unlike the static/dynamic-obstacle scenarios) -- manually verified only
  so far; worth doing if this scenario matters enough to guard against
  future regression.
- No dynamic-scenario world exists for the GPU/OptiX path at all (only
  `worlds/gz_static_radar_gpu.sdf`) — the per-visual-material rebuild
  logic on a changing map has never been runtime-verified on GPU.
- `worlds/avz_collada.world` (used by the old Classic launch files) has
  no Harmonic port — minor, the raw `.stl` mesh is reused directly in the
  new SDF worlds instead, not a capability loss.

See MIGRATION_HANDOFF.md's cross-package gap list for how this fits
against the other 3 packages' own gaps and a proposed tackle order.
