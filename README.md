# radarays_gazebo_plugins

[radarays_ros](https://github.com/uos/radarays_ros) Gazebo plugin.

## Migration Status

This package has a **working ROS 2 / Gazebo Harmonic build path**, with real
radar-specific runtime for `/radar/scan`, `/radar/points`, and — as of the
`radarays_embree_sensor_system` gz-sim system — genuine physics-based
`/radar/image` generation (not a bridge).

Current state:

- build system: `ament_cmake`
- generic scan/point sensing: delegated to `rmagine_gazebo_plugins`
- radar image generation: native, in this package, via
  `radarays_embree_sensor_system`. The physics itself (multi-bounce
  Fresnel reflection/refraction, cone-sampled beams, denoising, ambient
  noise) is not a local copy — it's `radarays_ros`'s exported
  `radarays_core` library (no ROS dependency, just `rmagine::core`), the
  same canonical implementation `radar_simulator` uses. This package adds
  only what's specific to it: the gz-sim integration, per-visual materials
  resolved from SDF, and live ROS 2 parameter tuning.
- per-visual `radarays_material` parsing: ported (see "Materials" below) —
  objects without a tag fall back to a shared default material
- dynamic parameter reconfiguration: ported as live-tunable ROS 2
  parameters (see "Radar Parameters" below) — the `dynamic_reconfigure`
  equivalent
- moving-geometry scenarios with their own regression fixtures, confirming
  all of the above keeps working as the map changes every frame:
  `gz_dynamic_radar_cpu.sdf` (single translating object) and
  `gz_dynamic_radar_cpu_multi.sdf` (two simultaneous objects — one
  translating, one rotating in place — each with its own material)
- legacy Classic sources still present in-repo for reference, not built:
  - Classic static sensor registration
  - Classic radar sensor plugin
  - ROS 1 image publication plugin
- still missing: OptiX/GPU path

The intended migration direction is:

- keep `rmagine_gazebo_plugins` as the validated Harmonic Embree backend baseline
- port `radarays_gazebo_plugins` next
- treat this package as a **thin Harmonic integration layer** on top of the migrated
  `rmagine_gazebo_plugins` systems wherever possible
- avoid duplicating map / scene / sensor lifecycle logic that already exists in the
  Harmonic Embree prototype

Important boundary:

- this package is **not yet fully migrated**
- the first Harmonic path currently delegates generic sensing to
  `rmagine_gazebo_plugins`
- the legacy XML launch files below still describe the Classic path
- the next real migration milestone is still one end-to-end Harmonic scenario using
  the CPU / Embree path first

See [MIGRATION.md](MIGRATION.md) for the current migration action points and target
integration shape.

## First Harmonic Path

The package now installs a first ROS 2 / Gazebo Harmonic scenario:

- world: `worlds/gz_static_radar_cpu.sdf`
- launch: `launch/gz_static_radar_cpu.launch.py`

This scenario intentionally delegates the generic Embree map and sensor lifecycle to
`rmagine_gazebo_plugins`:

- `librmagine_embree_map_system.so`
- `librmagine_embree_sensor_system.so`

That keeps this package thin while the real radar-specific functionality is migrated.

Build:

```console
colcon build --packages-select rmagine_gazebo_plugins radarays_gazebo_plugins --cmake-args -DRMAGINE_GZSIM_PORT=ON
```

Run:

```console
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch radarays_gazebo_plugins gz_static_radar_cpu.launch.py
```

Expected topics from the delegated baseline:

- `/radar/scan`
- `/radar/points`
- `/radar/image`

Current image path note:

- `/radar/image` is generated directly by `radarays_embree_sensor_system`,
  ray-tracing against the delegated Embree map with real radar wave physics
- it is published with `rclcpp::SensorDataQoS()` (best-effort/volatile) —
  subscribers need a compatible QoS or they will silently receive nothing
- per-visual `radarays_material` tags (see "Materials" below) are read and
  applied; objects without one fall back to a shared default material

Regression capture and comparison:

```console
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run radarays_gazebo_plugins capture_radarays_fixture static_cpu
ros2 run radarays_gazebo_plugins compare_radarays_fixture static_cpu
```

This writes a compact measured result to:

- `/tmp/radarays_harmonic_static_cpu_capture.json`

and compares it against the committed reference fixture in:

- `testdata/radarays_harmonic/static_cpu_fixture.json`

The current `static_cpu` fixture now covers:

- `/radar/scan`
- `/radar/points`
- `/radar/image`

### Dynamic Scenario

A second scenario with a moving target, mirroring
`rmagine_gazebo_plugins`'s dynamic lidar validation world:

- world: `worlds/gz_dynamic_radar_cpu.sdf` — a 1m box oscillates ±1m along
  x (period 6s, via `test_box_mover_system`) directly ahead of the radar
- launch: `launch/gz_dynamic_radar_cpu.launch.py`

Run and validate the same way, with `dynamic_cpu` in place of `static_cpu`:

```console
ros2 launch radarays_gazebo_plugins gz_dynamic_radar_cpu.launch.py
ros2 run radarays_gazebo_plugins capture_radarays_fixture dynamic_cpu
ros2 run radarays_gazebo_plugins compare_radarays_fixture dynamic_cpu
```

Note: this scenario's `/radar/image` is only ~18% nonzero, unlike
`static_cpu`'s ~100%. That's expected, not a regression — the office mesh
in the static world fills most of the field of view, while this scene is
just one narrow box in open space, so most beam angles get no reflection
at all, and this port's ambient-noise model scales with each column's own
peak signal (no signal → no noise floor either). See
`testdata/radarays_harmonic/dynamic_cpu_fixture.json`'s `reference_summary`
for the measured numbers this fixture is calibrated against.

### Dynamic Scenario — Multiple Objects, Rotation, Materials

A third scenario, `worlds/gz_dynamic_radar_cpu_multi.sdf`, covers what the
single-box scenario above can't: two objects moving *differently* at the
same time, one of them *rotating* rather than translating, each with its
own `radarays_material`.

- `target_box_translate`: same on-boresight oscillation as `dynamic_cpu`'s
  box, now with a rough/diffuse material
- `target_box_rotate`: off-boresight (~31°), rotating in place (zero
  translation) with a specular material — this needed
  `test_box_mover_system` (from `rmagine_gazebo_plugins`) to gain rotation
  support (`angular_axis`/`angular_amplitude` SDF params, additive and
  backward-compatible — existing worlds using only `axis`/`amplitude` are
  unaffected)

This is a real physics test, not just a geometry-visibility one: a
specular material's return strength depends on incidence angle
(`back_reflection_shader`), so as `target_box_rotate` yaws, its return
should swing measurably — confirmed in the fixture via the whole-image
nonzero pixel count swinging across a much larger span than the
single-box scenario, purely from that rotation.

```console
ros2 launch radarays_gazebo_plugins gz_dynamic_radar_cpu_multi.launch.py
ros2 run radarays_gazebo_plugins capture_radarays_fixture dynamic_multi_cpu
ros2 run radarays_gazebo_plugins compare_radarays_fixture dynamic_multi_cpu
```

## GPU / OptiX Status

GPU / OptiX migration is still intended, but it is explicitly deferred.

Current priority order:

1. CPU / Embree path
2. real `radarays` Harmonic integration
3. radar-specific runtime parity
4. then GPU / OptiX parity

So GPU support remains part of the migration plan, but it is not the active front
right now.

## CI

`scripts/ci_build_and_test.sh` builds the full four-package workspace
(this package plus `rmagine`, `rmagine_gazebo_plugins`, `radarays_ros`)
and runs all three regression fixtures (`static_cpu`, `dynamic_cpu`,
`dynamic_multi_cpu`), failing loudly if any of them regress. It's the
single source of truth for both local and CI verification — run it
directly from the workspace root instead of reaching for the individual
`capture_radarays_fixture`/`compare_radarays_fixture` commands unless
you're isolating a specific failure:

```console
src/radarays_gazebo_plugins/scripts/ci_build_and_test.sh
```

`.github/workflows/build-and-test.yml` (and the equivalent in the other
three repos) calls this same script from GitHub Actions. That workflow's
own provisioning (exact package versions, `rosdep` coverage) hasn't been
exercised against a live runner yet — see MIGRATION_HANDOFF.md, "What Has
Already Been Done" #15 for exactly what has and hasn't been verified.

## Dependencies

- rmagine (embree / optix backend)
- rmagine_gazebo_plugins
- radarays_ros

## Raytracing acceleration structure - World file

For constructing and continuously updating the acceleration structure for ray tracing, add the following lines to your world files:

```xml
<sdf version="1.4">
<world name="default">

...

<!-- CPU: Embree Map Plugin -->
<plugin name='rmagine_embree_map' filename='librmagine_embree_map_gzplugin.so'>
  <update>
    <delta_trans>0.001</delta_trans>
    <delta_rot>0.001</delta_rot>
    <delta_scale>0.001</delta_scale>
    <rate_limit>200</rate_limit>
  </update>
</plugin>

<!-- Optix Map Plugin -->
<plugin name='rmagine_optix_map' filename='librmagine_optix_map_gzplugin.so'>
  <update>
    <delta_trans>0.001</delta_trans>
    <delta_rot>0.001</delta_rot>
    <delta_scale>0.001</delta_scale>
    <rate_limit>500</rate_limit>
  </update>
</plugin>

...

</world>
</sdf>
```

See [rmagine_gazebo_plugins](https://github.com/uos/rmagine_gazebo_plugins) for further explanations on that.

## Materials

You can assign so called `radarays_materials` to visuals of a model. This
works in the Harmonic (`radarays_embree_sensor_system`) path too — see the
`avz_map_visual` in `worlds/gz_static_radar_cpu.sdf` for a working example.
An example on the Classic side is in `worlds/avz_collada.world`:

```xml
...
<model name="avz_map">
  <static>true</static>
  <link name="avz_map_link">
    <pose>0 0 0 0 0 0</pose>
    <visual name="avz_map_visual">
      <cast_shadows>false</cast_shadows>
      <geometry>
        <mesh>
            <uri>./avz_no_roof.stl</uri>
            <scale>1.0 1.0 1.0</scale>
        </mesh>
      </geometry>
      <radarays_material>
        <velocity>0.0</velocity>
        <ambient>1.0</ambient>
        <diffuse>0.0</diffuse>
        <specular>3000.0</specular>
      </radarays_material>
    </visual>
...
```

## Examples

Important files are:
- urdf/robot_radar_cpu.urdf -> URDF for a robot with a radar sensor simulated on CPU
- urdf/robot_radar_gpu.urdf -> URDF for a robot with a radar sensor simulated on GPU
- worlds/avz_collada.world -> Gazebo world-file that contains a mesh of an office with `radarays_materials` attached to it.

### example_robot.launch

Example for a robot that is spawned in a office-like environment, with `radarays_materials` attached to it. 

Run CPU version of radarays with Gazebo by calling

```console
roslaunch radarays_gazebo_plugins example_robot.launch rmagine:=embree
```

Run GPU version of radarays with Gazebo by calling

```console
roslaunch radarays_gazebo_plugins example_robot.launch rmagine:=optix
```

![Teaser](media/radarays_gazebo_plugin.png)


## Radar Parameters

**Harmonic path** (`radarays_embree_sensor_system`): the radar-physics
parameters are live-tunable ROS 2 parameters, the direct equivalent of
Classic's dynamic reconfigure:

```console
ros2 param list /radarays_embree_sensor_system
ros2 param set /radarays_embree_sensor_system n_reflections 5
```

Any generic ROS 2 parameter UI works too (the numeric ones declare
min/max ranges, so a UI can render sliders/steppers the same way
`rqt_reconfigure` did). Structural params — frame/topic names, angular
sampling (`samples`/`min_angle`/`max_angle`), `map_key` — are SDF-only,
since those change the sensor's shape rather than tune its physics.

**Classic path**: change the radar parameters of the Gazebo simulation using dynamic reconfigure:

```console
rosrun rqt_reconfigure rqt_reconfigure
```

Note: We are using the same set of parameters (configuration file) we used for the experiments in `radarays_ros`:
- Some parameters are only used in the experiments but not in the Gazebo simulation
- For the sake of performance, the Gazebo plugin does not consider the robot's motion while simulating a polar image. (we will add this in the future)

## Known Issues
- At some point I got a segmentation fault when using rmagine's CPU simulators inside of the Gazebo threads. Unfortunately, I couldn't reproduce those errors.
