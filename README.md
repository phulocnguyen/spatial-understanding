# Benchmark

> **Current MOU v1:** Start with [configs/mou.yaml](configs/mou.yaml) and
> [the redesign guide](reports/mou_v1_design.md). Default generation is an
> 756-scene, seven-attribute diversity plan using separate files under `configs/mou/attributes/`. The completed 18-scene
> baseline is preserved via `configs/mou_pilot.yaml`; the expanded plan is not collected. The broader eleven-case documentation
> below describes experimental capabilities, not qualified v1 coverage.

Benchmark is the project name. Its current hierarchy is:

```text
Benchmark
└── Metric Spatial Understanding                 # Category
    └── Metric Object Understanding (MOU)        # One task
        ├── Physical Dimension Identification
        ├── Geometric Property Identification
        └── Structured Object Identification
```

The previous DI/MTD/MAI task definitions and manifests have been removed from the active project. The directory is now `Benchmark/`, not `metric-grounding/`. Existing datasets were retained; external datasets on Seagate were not changed or relabeled. Old commands/manifests refer to retired tasks and should not be used for MOU collection.

## File organization

```text
Benchmark/
├── configs/
│   ├── settings.yaml                  # Table, robot, cameras, timing, grasp/success
│   ├── domain_randomization.yaml      # Shared physics/scene reset factors
│   ├── difficulty.yaml                # Preliminary gaps/counts/appearance
│   ├── object_sizes.yaml              # Object templates, size/grasp limits, radius gap scale
│   └── metric_spatial_understanding/
│       └── metric_object_understanding/
│           ├── physical_dimensions/   # length, width, height, radius, diameter
│           ├── geometric_properties/  # area, volume, capacity
│           └── structured_objects/    # multiple, component, nested dimensions
├── tasks/
│   ├── registry.py                    # Category/task IDs -> adapters
│   ├── common/success.py              # Manipulation success proxy
│   └── metric_spatial_understanding/
│       └── metric_object_understanding/
│           ├── catalog.py             # Case IDs, families, categories, queries
│           ├── scenarios.py           # Conditions, exact truth, controls, layout
│           ├── manifest.py            # Balanced MOU scene/control generation
│           └── task.py                # One adapter for all MOU cases
├── assets/
│   ├── geometry.py                    # Measurements, cavities, components, AABBs
│   └── spawners.py                    # Primitive and compound USD collision assets
├── core/
│   ├── units.py                       # Dimension-checked SI conversions
│   └── timing.py                      # Independent camera/control cadence
├── environments/
│   ├── settings.py                    # YAML -> runtime settings
│   ├── tabletop.py                    # Panda/table/legs/objects/sensor definitions
│   └── cameras.py                     # Fixed poses, wrist synchronization, RGB-D capture
├── controllers/grasp_lift.py           # Feedback-driven oracle grasp/lift
├── data_collection/
│   ├── config.py                      # Collection YAML validation and overrides
│   ├── runner.py                      # CLI and task dispatch
│   ├── manifest.py                    # Plans, retries, logs, resume
│   ├── simulation.py                  # Reset, control, capture, success loop
│   ├── metadata.py                    # Compact outcomes and shared schema readers
│   ├── recorder.py                    # Incremental causal HDF5 recording
│   ├── calibration.py                 # Reference-format camera calibration
│   ├── video.py                       # Recorded RGB -> MP4
│   ├── validate.py                    # Dataset contract checks
│   ├── finalize.py                    # Validation and exports before Kit shutdown
│   └── export_lerobot.py               # Optional training-format export
├── evaluation/
│   ├── mou.py                         # Policy-selection metrics and controls
│   └── review.py                      # Data statistics and camera contact sheets
├── tests/                             # Geometry, recording, calibration contracts
├── manifests/                         # Planned scenes; not demonstrations
├── reports/                           # Specification and actual verification status
├── datasets/                          # Retained/generated data
├── collect_data.py
├── generate_manifest.py
├── collect_manifest.py
├── evaluate.py
├── review_dataset.py
├── validate_dataset.py
└── export_dataset.py
```

The per-case defaults and formal rules are listed in [reports/MOU_TASKS.md](reports/MOU_TASKS.md).

This organization keeps one adapter for MOU. Adding a future task under Metric Spatial Understanding means adding a sibling task folder and one registry entry. Its asset and recording code can reuse the shared modules. Adding another MOU case generally means extending the catalog/geometry and adding a YAML, rather than duplicating the collector.

### Where to make changes

| Change | Owner |
|---|---|
| Table, robot pose, cameras, lighting, materials | `configs/settings.yaml` |
| Physics/control rate, settling, timeout | `settings.yaml` → `simulation` |
| Approach/lift offsets and convergence tolerances | `settings.yaml` → `controller` |
| Lift height, dwell, distractor preservation | `settings.yaml` → `success` |
| Randomization ranges and switches | `configs/domain_randomization.yaml` |
| Difficulty, object count, measurement gaps | `configs/difficulty.yaml` |
| Object dimensions and permitted size ranges | `configs/object_sizes.yaml` |
| Case, target measurement, seed, output, videos | Case YAML under `configs/metric_spatial_understanding/` |
| Task selection and generated instructions | MOU `catalog.py` and `scenarios.py` |
| New geometry or physical shape | `assets/geometry.py` and `assets/spawners.py` |
| Episode summary fields or dataset readers | `data_collection/metadata.py` |

The pipeline is **case YAML → task/scenario → randomized scene → controller →
causal HDF5 recorder → validation, calibration, videos → visibility review →
optional LeRobot export**. Policy identification evaluation is a separate path
through `evaluate.py`; the collection controller uses exact simulator geometry.
Root Python scripts remain short command entry points; their implementation lives
in the owning package. Package `__init__.py` files are needed for imports.

## Implemented cases

| Case | ID | Query convention |
|---|---|---|
| `length` | MOU-1.1 | Rectangular bar or box, designated local X |
| `width` | MOU-1.2 | Designated local lateral Y; container outside width |
| `height` | MOU-1.3 | Upright intrinsic vertical/axial dimension |
| `radius` | MOU-1.4 | Cylinder cross-section or sphere center-to-surface radius |
| `diameter` | MOU-1.5 | Twice radius |
| `area` | MOU-2.1 | Local reference base/footprint area, not surface area |
| `volume` | MOU-2.2 | Solid geometric volume, not capacity |
| `capacity` | MOU-2.3 | Represented usable interior volume |
| `multi_dimension` | MOU-3.1 | Shape and all specified local dimensions |
| `component_dimension` | MOU-3.2 | Parent identity plus semantic handle length |
| `nested_dimension` | MOU-3.3 | Actual cavity inner width and outside width |

There are **no depth or thickness identification cases**. Wall/floor dimensions and usable cavity depth remain geometry parameters needed to construct real containers; image-plane depth remains an observation channel. Neither is a physical-depth identification task.

## Conditions and controls

- `same_shape`: shared category, different numerical measurements, exactly one full match.
- `cross_shape`: different categories, exactly one numerical match.
- `shape_conditioned`: all four combinations of correct/incorrect shape and correct/incorrect measurements, with exactly one complete instruction match.

Matched `shape_only`, `measurement_only`, and `counterfactual_joint` identification controls use the **same scene** with changed instructions and explicit acceptable-ID sets. These are identification diagnostics, not separate multi-object robot collection instructions. The normal joint instruction always has one target. The counterfactual control changes the numerical request in the same scene so selecting the original object regardless of language fails. Model evaluation receives only the instruction and permitted observations; acceptable IDs and object measurements remain evaluator-private ground truth.

Five levels use preliminary linear gaps of 4, 3, 2, 1.5, and 1 cm, with unit-correct relative gaps for area/volume/capacity. Counts are minimum design targets. A four-quadrant scene requires at least four objects; multi-attribute scenes may require five or six for independent hard negatives. Metadata records the effective count instead of falsely describing a four-object scene as two objects. Impossible gap/grasp combinations are listed explicitly in manifest feasibility results.

For new radius scenarios, the gap is multiplied by 0.5 so separation is measured
on the same diameter scale as width. The factor is explicit in
`object_sizes.yaml` and saved in the scenario.

Manifest generation varies numerical targets using balanced 0.8/1.0/1.2 reference-value scales; `--fixed_queries` is available for fixed-value diagnostics. Direct task YAMLs keep their explicitly requested values. Target object IDs are assigned after permutation. Color is not derived from shape or target status. Target-relative rank varies where geometry permits; manifests report target-rank counts because impossible ranges can still limit rank counterbalancing. Do not infer absolute metric competence from a rank-biased subset.

## Procedural geometry

Supported geometry includes boxes, upright cylinders, spheres, disks/pucks, cones, hollow rectangular containers/trays, cylindrical polygonal cups, and handled tools/paddles. Cylindrical rods are excluded. Length uses rectangular bars and boxes. Solid cylinders appear only in radius, height, and diameter cases, standing on their circular base with the Z axis vertical. Circular disks and hollow capacity containers remain separate categories. Regenerate manifests after changing this catalog; saved recordings retain their original scene definitions.

Hollow boxes use a floor plus four disjoint wall colliders. Cups use a convex polygonal floor and 64 independent convex wall sectors; a single convex hull does not fill their cavities. Their capacity is the actual regular-64-gon cavity area times usable interior height, not an unqualified ideal-circle estimate. Component objects have separate handle/head geometry, semantic labels, local positions, and component dimensions. Different handle lengths can share the same overall object size.

Objects now use larger physical dimensions, with limits checked before launch:
each bounding extent is at least 2.5 cm, the larger planar extent is at least
4.5 cm, no extent exceeds 22 cm, and grasp width is at most 7.4 cm. Components
also have a 2.5 cm minimum extent. The tool and paddle are 20 cm long; the paddle
has a flat elliptical head instead of a cylinder pointing along its handle.
Query values, prompts, and annotations use the new physical measurements.
Camera resolution remains 256 × 256. Old videos and plans retain their authored
geometry; use `manifests/mou/larger30/plan.yaml` for the enlarged objects.

Cones are excluded from autonomous generation and collection because the current
overhead Panda grasp slips under the allowed mass randomization. Geometry and
historical recordings remain readable. The eleven default presets use other
shapes; exclusions are reported by `check_tasks.py`.

Grasp offsets target the handle near the head rather than the parent's bounding-box center; hollow containers are pinched nearer the upper rim so the palm can clear tall walls. Wide rectangular containers can close along their shorter local axis while retaining the original length/width annotations. These are implemented geometry/grasp configurations, not proof of reliable demonstrations; inspect simulation outcomes and videos, especially for cavities, thin plates, cones, spheres, and compound objects.

## Run commands

Collection presets use a readable plan plus separate scene files. Browse
[`manifests/mou/first30/slots.csv`](manifests/mou/first30/slots.csv) for one row per
slot; see [the manifest organization guide](manifests/README.md) for editing,
generation, reports, and collection commands. Exact object geometry no longer
fills a single large collection-plan JSON file.

From the Isaac Lab workspace root:

```bash
cd /home/locnp/IsaacLab

# List the registered task (MOU is one task).
uv run python Benchmark/collect_data.py --list_tasks

# Validate without launching simulation.
uv run python Benchmark/collect_data.py \
  --config Benchmark/configs/metric_spatial_understanding/metric_object_understanding/physical_dimensions/length.yaml \
  --check_config

# Preview, without collecting.
uv run python Benchmark/collect_data.py \
  --config Benchmark/configs/metric_spatial_understanding/metric_object_understanding/physical_dimensions/length.yaml \
  --preview

# Collect one episode and save four videos.
uv run python Benchmark/collect_data.py \
  --config Benchmark/configs/metric_spatial_understanding/metric_object_understanding/physical_dimensions/length.yaml \
  --video --visualizer none
```

Change `--case`, `--condition`, or `--level`, or edit the relevant YAML. Scalar and conjunction targets are defined by the YAML `measurements` mapping. Units are dimension-checked. One geometry scene is instantiated per process; manifest collection starts the next process for each new scene.

### First 30 slots on Seagate

```bash
uv run python Benchmark/generate_manifest.py \
  --count 30 --seed 712 --output Benchmark/manifests/mou/first30

uv run python Benchmark/collect_manifest.py Benchmark/manifests/mou/first30/plan.yaml \
  --output /media/Seagate16T/locnp/Benchmark/mou_first30 \
  --max_slots 30 --attempts_per_slot 3
```

Videos are enabled for every attempt. The scheduler visits all eleven cases before repeating them and includes all three conditions/five difficulty levels where feasible. Failed attempts are retained. This is a scene plan, not a claim of 30 successful or visually accepted demonstrations.

The runner prints slot/attempt start, heartbeat messages, and outcomes. Simulator logs are saved in each slot. `--resume` continues a compatible existing output folder; it does not relabel an old benchmark. `--attempt_timeout_s` defaults to 300 wall-clock seconds (five minutes) and terminates only its own stalled child process. Use `--start_slot 10` to begin from slot 10, or generate a 300-slot plan with `--count 300` after representative checks pass.

## Cameras and timing

The 1.6 × 1.0 m table has its long axis along world X. In `configs/settings.yaml`,
`robot.base_position: [0, -0.4, 0.75]` places the arm at the midpoint of the long
−Y side, inset 10 cm from the edge. `robot.base_yaw_deg: 90` points its local
forward axis into the table along +Y. `objects.layout` defines row distances
forward of the base and column offsets to its left; both placement and initial
object yaw follow the base orientation. Domain yaw randomization is an offset
from that initial yaw. Local measurement definitions do not change.

`cameras.scene.views` contains world-space positions and look-at targets. The
`top` camera is above the far side at `[0, 0.65, 1.85]`, aimed at `[0, 0.12, 0.78]`,
looking downward at approximately 64° so the long table edge is horizontal in the image.
It frames the object workspace closely; the outer table edges can be cropped.
It is an oblique view, not
a vertical overhead camera. `left` is on −X and `right` on +X relative to the
robot facing +Y. The interactive viewport starts at the `top` pose. The wrist
camera follows `panda_hand` automatically.

| View | World position (m) | Look-at target (m) | Vertical FOV |
|---|---|---|---|
| `top` | `[0, 0.65, 1.85]` | `[0, 0.12, 0.78]` | 45° |
| `left` | `[-0.8, 0.1, 1.4]` | `[-0.08, 0.1, 0.78]` | 45° |
| `right` | `[0.8, 0.1, 1.4]` | `[0.08, 0.1, 0.78]` | 45° |
| `wrist` | Hand-local offset `[0.05, 0, 0]` | Hand-local ROS quaternion XYZW `[0, 0, 0.70710678, 0.70710678]` | 75° |

These settings use LIBERO's
[tabletop agent camera](https://github.com/Lifelong-Robot-Learning/LIBERO/blob/master/libero/libero/envs/problems/libero_tabletop_manipulation.py)
as a framing reference, adapted to this table height, rotated robot, and wider
object grid. The wrist mount and 75° FOV follow the
[robosuite 1.4.0 Panda camera](https://github.com/ARISE-Initiative/robosuite/blob/v1.4.0/robosuite/models/assets/robots/panda/robot.xml),
with its MuJoCo/OpenGL rotation converted to ROS coordinates.
LIBERO's [dataset collector](https://github.com/Lifelong-Robot-Learning/LIBERO/blob/master/scripts/create_dataset.py)
records `agentview` and `robot0_eye_in_hand`; our `left` and `right` views are
additional cameras, not a standard LIBERO four-camera preset. Closer fixed
cameras enlarge objects in the image without changing their physical measurements.
The broader wrist view reduces cropping during approach and grasp.

If you later change the base pose, update the fixed camera poses here too.
The IK controller uses world-space hand poses, targets, and Jacobians consistently.
These settings apply to new collection processes; saved runs retain their poses.

`rendering.shadows: false` disables cast shadows in all views through the realtime
renderer switch and each light's empty USD shadow-link collection for path tracing. Object
materials still retain directional surface shading so their shapes remain visible.

Your shared settings were retained: four views (`top`, `left`, `right`, `wrist`), 256 × 256 RGB-D, and the currently configured 0.05 s update period (20 FPS). Physics currently runs at 120 Hz and control/actions at 30 Hz. The new physics-step capture scheduler supports 20 FPS with 30 Hz actions; cameras no longer have to divide the control rate. They must divide the physics rate and cannot exceed the recorded action rate.

Visual contrast is controlled in `configs/settings.yaml`: `appearance` defines
the dark matte table, object palette, and material roughness; `lighting` defines
dome fill (350) and a directional key light (650). The lower light levels and
deeply saturated linear-RGB palette preserve object/table contrast;
`rendering.mode: path_traced` selects
64-sample denoised native-resolution rendering. This costs more rendering time;
`mode: realtime` restores the faster renderer with the configured antialiasing.
Resolution stays 256 × 256. Similar-appearance levels
use shades of one seeded saturated hue, while distinct-appearance levels use the
palette without replacement. Colors are unrelated to target identity or size.
The shared light randomization multiplier is limited to [0.85, 1.15]. Episode
videos use H.264 CRF 16 to retain more detail than the encoder default; this can
increase file sizes. These changes apply to new captures, not existing videos.

Tabletop color is sampled once per episode by the `table` group in
`configs/domain_randomization.yaml`. The options are solid-color approximations
of visible tabletop materials in LIBERO's official scenes:

- `wood_table`: [standard tabletop wood](https://github.com/Lifelong-Robot-Learning/LIBERO/blob/master/libero/libero/assets/scenes/libero_tabletop_base_style.xml).
- `living_room_wood`: [the living-room mesh table material](https://github.com/Lifelong-Robot-Learning/LIBERO/blob/master/libero/libero/assets/scenes/libero_living_room_tabletop_base_style.xml).
- `coffee_table_marble`: [the short coffee-table marble material](https://github.com/Lifelong-Robot-Learning/LIBERO/blob/master/libero/libero/assets/scenes/libero_coffee_table_base_style.xml).

LIBERO uses textures, not a standard tabletop RGB palette. These are derived
representative colors: source pixels are decoded from sRGB to linear RGB and
averaged (mesh-table textures were reduced to 512 × 512 before averaging).
No grain, veins, or mesh UV mapping is imported. The floor/wall style names are
not used as tabletop colors, and unused `tex-table` declarations in mesh scenes
are not treated as the visible table material. Selection is uniform and seeded
independently of object/robot/light randomization. It changes only the table's
visual material, stays fixed within the episode, and is saved under
`randomization_samples.table` with its name, linear RGB, and source filename.
Disable `table.enabled` (or global `enabled`) to use `appearance.table_color`
from shared settings. No LIBERO installation or network access is required
during collection.

Images carry their actual timestamps and causal indices; an action uses the latest image at or before that action. Camera streams stay uniformly sampled for video export. Rendering occurs on capture steps rather than every physics step during collection; preview still renders interactively.

`objects.start_asleep: true` in `configs/settings.yaml` places objects directly on the
table and initializes them as sleeping dynamic bodies. Contact can wake them for
grasping; they are not kinematic. This initialization intentionally suppresses
pre-contact rolling, including motion from randomized centre-of-mass offsets.
It selects CPU PhysX because direct GPU PhysX disables sleeping; camera rendering
still uses the GPU. Set this option to `false` to use the requested CLI device and
ordinary free settling. Collection requires all objects to remain below 1 mm/s
linear speed and 0.01 rad/s angular speed for 0.5 seconds, with a 10-second limit.
The HDF5 scene record identifies the initialization mode. Other dynamic objects
can also wake an object through contact; sleeping is not restricted to robot contact.

## Success, dataset, and visual acceptance

Every current MOU instruction is a unique-target **pick/lift-and-hold** task. No old threshold sorting or multi-pick behavior remains. Manipulation success uses the retained geometric proxy: target raised ≥5 cm, near the hand, gripper sufficiently closed, non-targets displaced <4 cm, sustained for the configured dwell. It is not contact-force-verified grasp success.

Each run contains `dataset.hdf5`, resolved settings, episode metadata, summary/validation files, and optional videos. Geometry parameters, local/inner/outer measurements, components, pose, target-match labels, condition, query, target/distractor IDs, randomization, trajectory, calibration, and success are retained. The observed selected-object field is a peak-lift proxy, separate from the commanded oracle target.

### Compact episode output (schema 3)

```text
<run-id>/
├── run_config.yaml                 # One resolved config; no duplicate config.json
├── dataset.hdf5                    # RGB-D, calibration, actions, states, exact scene truth
├── episode_0000/metadata.json      # Short identity/instruction/outcome summary
├── meta/camera_calibration.json    # Reference camera snapshot, saved once
├── videos/episode_0000_<view>.mp4  # When video is enabled
├── summary.json                    # Collection counts
└── validation.json                 # Checks performed on this recording
```

`metadata.json` contains the task/scenario ID, case/condition/level/split, prompt,
seed, commanded target, observed selection, success/termination, transition count,
duration, finalization, and visibility/acceptance labels. It does not repeat object
geometry, arrays of poses, camera settings, or the randomization configuration.
This follows Extrinsic Dexterity's separation of small episode annotations from
calibration and trajectory data; the native HDF5 schema remains Benchmark-specific.

Exact dimensions and sampled physical parameters are essential ground truth:
they live in the episode's HDF5 `scene_json` attribute, alongside the original
scenario and measured initial object poses. Shared action/sensor conventions are
in the file's `recording_json` attribute. A `run_config_json` attribute preserves
the configuration inside HDF5 as well, so copying the HDF5 alone does not lose it.
Only the small episode summary is repeated as the `metadata_json` attribute.
These ground-truth records are evaluator data, not policy observations.

```python
import json
import h5py

with h5py.File("path/to/dataset.hdf5", "r") as data:
    episode = data["data/episode_0000"]
    outcome = json.loads(episode.attrs["metadata_json"])
    scene = json.loads(episode.attrs["scene_json"])
    objects = scene["objects"]  # Exact geometry and initial poses
    query = scene["scenario"]["query"]  # Numerical selection rule
    reset = scene["randomization_samples"]  # Actual sampled physics/appearance
```

Existing schema-2 datasets remain readable by validation, review, calibration,
and export tools. They are not automatically rewritten. The new output layout
applies to newly started collections.

### Camera calibration in the Extrinsic Dexterity format

Newly finalized runs write one reference-format snapshot:

```text
meta/camera_calibration.json
```

The JSON matches the reference's camera fields: sensor name, image dimensions,
intrinsic matrix, optical settings, prim path, robot mounting flag, and world
position with WXYZ quaternions in world, ROS, and OpenGL camera conventions.
Optical settings come from the saved run configuration; intrinsics come from
the recorded sensor. The wrist mounting flag uses its configured body, since
its synchronized camera prim is not a child of the robot.

Per-frame calibration is already stored in native HDF5. Explicit calibration
conversion (command below) additionally writes `meta/camera_poses.hdf5`, which contains
`data/episode_0000/obs/camera_pose_camera_<view>` as
`[x, y, z, qw, qx, qy, qz]`, using the reference's **+X forward, +Z up** camera
convention. Camera timestamps accompany these poses. Repeated images at the
30 Hz control rate also repeat their matching poses; no future camera pose is
paired with an earlier image. The snapshot alone must not be used as a constant
wrist extrinsic.

Native `dataset.hdf5` retains its ROS/XYZW camera streams. The LeRobot exporter
now writes reference-convention WXYZ `observation.camera_pose.<view>` features
and the same calibration sidecars. Older LeRobot exports used ROS/XYZW; export
them again from native recordings into a new directory instead of mixing formats.
This matches the camera calibration contract, not the entire reference dataset's
robot-action or observation schema.

Convert an existing finalized recording without launching Isaac Sim or installing LeRobot:

```bash
uv run python Benchmark/export_dataset.py /path/to/run/dataset.hdf5 --calibration_only
```

This writes `meta/` beside that dataset. Add `--output /path/to/converted` to use
another destination. Schema-3 files contain their own saved settings; legacy
files need the original `config.json` or resolved `run_config.yaml`.
For multiple nonempty episodes, conversion also writes
`camera_calibration_episodes.json` to preserve distinct calibration snapshots.
A single-episode run does not need that duplicate JSON. Conversion does not
modify the native HDF5 file.

Observability starts `unverified`, especially for ambiguous local axes, cavities, similar projected sizes, and sub-resolution features. Verify actual rendered views; local coordinate axes are not automatically visible to a model. A successful but unreviewed episode is not an accepted demonstration.

```bash
uv run python Benchmark/review_dataset.py /media/Seagate16T/locnp/Benchmark/mou_first30 \
  --dataset /absolute/path/to/dataset.hdf5 --episode episode_0000 \
  --contact_sheet Benchmark/reports/episode_0000.png

uv run python Benchmark/review_dataset.py /media/Seagate16T/locnp/Benchmark/mou_first30 \
  --dataset /absolute/path/to/dataset.hdf5 --episode episode_0000 \
  --visibility verified --reason "Reviewed the queried dimensions/cavity in permitted views"
```

Use `rejected` when features are hidden or indistinguishable. Statistics distinguish manipulation success from identification accuracy. Native HDF5 remains authoritative; optional LeRobot export selects accepted successful records and excludes object-pose/geometry oracle features from policy observations. Ground-truth sidecars remain private evaluation data.

## Evaluate a VLA

Collection uses simulator geometry to generate oracle demonstrations. It does not demonstrate that a VLA recognizes metric dimensions. Run a policy on the permitted observations/instructions and supply selections separately:

```json
[
  {
    "scenario_id": "<scenario_id from the manifest>",
    "selected_object_id": "object_2",
    "source": "policy",
    "control": "joint",
    "manipulation_success": false
  }
]
```

```bash
uv run python Benchmark/evaluate.py Benchmark/manifests/mou/first30/plan.yaml \
  predictions.json --output Benchmark/reports/policy_metrics.json
```

Reports include joint target accuracy, shape-conditioned accuracy, numerical-match accuracy, shape correctness, selected-object metric error in SI, manipulation success, matched-control accuracy, and breakdowns by case/level. Invalid selections count as identification failures; metric error coverage is reported separately. Oracle results are explicitly separated and excluded from policy accuracy. Missing manipulation results remain unknown rather than being invented as failures or successes.

## Validation

```bash
uv run python -m unittest discover -s Benchmark/tests -v
uv run python Benchmark/check_tasks.py --seeds 5 \
  --manifest Benchmark/manifests/mou/larger30/plan.yaml
uv run python Benchmark/validate_dataset.py /absolute/path/to/dataset.hdf5
```

`check_tasks.py` checks every preset, supported geometry/condition/difficulty
combination, placement, and manifest split. It reports infeasible combinations
separately from errors, and flags constant query values, target ranks, or target
categories. Results are saved in `reports/collection_preflight.json`.
The enlarged-object audit is recorded in
[reports/object_size_validation.json](reports/object_size_validation.json), with
its geometry sweep in [reports/object_size_preflight.json](reports/object_size_preflight.json).
The earlier audit remains in
[reports/collection_readiness.json](reports/collection_readiness.json).
Diagnostic runs use 8 path-tracing samples; the enlarged component preset is also
recorded at the normal 64 samples. Resolution stays at 256 × 256 pixels.

Use `manifests/mou/larger30/plan.yaml` for the enlarged-object 30-slot pilot.
Older saved plans may contain constant per-case target values from a sampling
bug; regenerating a plan now rotates query values across case occurrences.
Existing recordings and active plans are not rewritten. A 30-slot pilot is too
small to establish coverage or grasp reliability for every combination.

```bash
uv run python Benchmark/collect_manifest.py Benchmark/manifests/mou/larger30/plan.yaml \
  --output /media/Seagate16T/locnp/Benchmark/mou_larger30 \
  --max_slots 30 --attempts_per_slot 3
```

Keep diagnostic and evaluation episodes out of training exports. In particular,
successful oracle grasping does not establish that the requested measurement is
visible: cavity dimensions and local length/width axes require visual review.
Unreviewed episodes remain `accepted_demonstration: false`. The default plans
also use one target category per case, so they do not establish generalization
to unseen target shapes.

The architecture is documented here; task conventions live in
[reports/MOU_TASKS.md](reports/MOU_TASKS.md). Historical verification evidence
remains in [reports/validation.json](reports/validation.json) and
[reports/runtime_smoke.json](reports/runtime_smoke.json); each new run saves its
own validation result. Geometry correctness, runtime validity, grasp reliability,
visual observability, and VLA accuracy are separate gates.
