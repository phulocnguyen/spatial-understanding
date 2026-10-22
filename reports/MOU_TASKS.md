# Benchmark: Metric Spatial Understanding / Metric Object Understanding

MOU is one registered task with three families and eleven query cases. All examples below are generated from the new task YAMLs. The old DI/MTD/MAI definitions are not used.

## Conditions

- Same shape: shared category, different metric values, one unique target.
- Cross shape: different categories and one unique numerical match.
- Shape-conditioned: correct/incorrect shape crossed with correct/incorrect measurement; the instruction requires both and has one target.
- Structured conjunctions add independent hard negatives for each query where feasible. Required controls can raise the effective object count above the preliminary level count; the effective count is stored explicitly.

## Measurement conventions

Length is the authored local X extent of a box or rectangular bar. Cylindrical rods are excluded; cylinders stand upright on their circular bases. Width follows the designated lateral axis; container width means outside width unless the query explicitly says inner width. Height is the authored upright extent. Radius and diameter follow their ordinary primitive definitions.

Footprint/base area is the local reference area (L×W for rectangular bases, πr² for upright circular primitives), not total surface area and not a new projection after tilting. Solid volume excludes cavities; capacity is usable cavity volume. Cup cavities are represented as regular 64-gon prisms and measured using that actual polygon area.

Components have parent object IDs, local component IDs, semantic labels, dimensions, and poses. Their overall bounding dimensions do not substitute for queried component dimensions. Hollow containers have physically separate floors/walls with measured inner and outer dimensions.

No depth or thickness identification case is registered. Geometric wall/floor sizes and usable interior height are construction annotations, while camera image-plane depth is an observation.

## Cases

### MOU-1.1 — length

Family: `physical_dimensions`. Config: [length.yaml](../configs/metric_spatial_understanding/metric_object_understanding/physical_dimensions/length.yaml).

> Pick the rectangular bar with local length 14 cm. Lift and hold it; leave all other objects untouched.

**Supported categories:** rectangular_bar, box.

**Query:** length = 14 cm.

Default scene: shape_conditioned, L3, 4 objects; exactly one complete match.

### MOU-1.2 — width

Family: `physical_dimensions`. Config: [width.yaml](../configs/metric_spatial_understanding/metric_object_understanding/physical_dimensions/width.yaml).

> Pick the rectangular plate with local width 6 cm. Lift and hold it; leave all other objects untouched.

**Supported categories:** rectangular_plate, box, rectangular_container.

**Query:** width = 6 cm.

Default scene: shape_conditioned, L3, 4 objects; exactly one complete match.

### MOU-1.3 — height

Family: `physical_dimensions`. Config: [height.yaml](../configs/metric_spatial_understanding/metric_object_understanding/physical_dimensions/height.yaml).

> Pick the cylinder with local height 12 cm. Lift and hold it; leave all other objects untouched.

**Active categories:** cylinder, box. Cone geometry is retained but excluded from autonomous collection.

**Query:** height = 12 cm.

Default scene: shape_conditioned, L3, 4 objects; exactly one complete match.

### MOU-1.4 — radius

Family: `physical_dimensions`. Config: [radius.yaml](../configs/metric_spatial_understanding/metric_object_understanding/physical_dimensions/radius.yaml).

> Pick the cylinder with local radius 2.9 cm. Lift and hold it; leave all other objects untouched.

**Supported categories:** cylinder, sphere, circular_disk, puck.

**Query:** radius = 2.9 cm.

Default scene: shape_conditioned, L5, 6 objects; exactly one complete match.

### MOU-1.5 — diameter

Family: `physical_dimensions`. Config: [diameter.yaml](../configs/metric_spatial_understanding/metric_object_understanding/physical_dimensions/diameter.yaml).

> Pick the sphere with local diameter 5.8 cm. Lift and hold it; leave all other objects untouched.

**Supported categories:** sphere, cylinder, circular_disk.

**Query:** diameter = 5.8 cm.

Default scene: shape_conditioned, L5, 6 objects; exactly one complete match.

### MOU-2.1 — area

Family: `geometric_properties`. Config: [area.yaml](../configs/metric_spatial_understanding/metric_object_understanding/geometric_properties/area.yaml).

> Pick the circular disk with local base/footprint area 35 cm2. Lift and hold it; leave all other objects untouched.

**Supported categories:** circular_disk, rectangular_plate, box.

**Query:** footprint_area = 35 cm2.

Default scene: shape_conditioned, L3, 4 objects; exactly one complete match.

### MOU-2.2 — volume

Family: `geometric_properties`. Config: [volume.yaml](../configs/metric_spatial_understanding/metric_object_understanding/geometric_properties/volume.yaml).

> Pick the box with solid volume 160 cm3. Lift and hold it; leave all other objects untouched.

**Active categories:** box, sphere. Cone geometry is retained but excluded from autonomous collection.

**Query:** volume = 160 cm3.

Default scene: shape_conditioned, L3, 4 objects; exactly one complete match.

### MOU-2.3 — capacity

Family: `geometric_properties`. Config: [capacity.yaml](../configs/metric_spatial_understanding/metric_object_understanding/geometric_properties/capacity.yaml).

> Pick the cylindrical container with internal usable capacity 250 cm3. Lift and hold it; leave all other objects untouched.

**Supported categories:** cylindrical_container, rectangular_container.

**Query:** capacity = 250 cm3.

Default scene: shape_conditioned, L3, 4 objects; exactly one complete match.

### MOU-3.1 — multi dimension

Family: `structured_objects`. Config: [multi_dimension.yaml](../configs/metric_spatial_understanding/metric_object_understanding/structured_objects/multi_dimension.yaml).

> Pick the box with local length 14 cm and local width 5 cm and local height 4 cm. Lift and hold it; leave all other objects untouched.

**Supported categories:** box, handled_tool.

**Query:** length = 14 cm, width = 5 cm, height = 4 cm.

Default scene: shape_conditioned, L3, 6 objects; exactly one complete match.

### MOU-3.2 — component dimension

Family: `structured_objects`. Config: [component_dimension.yaml](../configs/metric_spatial_understanding/metric_object_understanding/structured_objects/component_dimension.yaml).

> Pick the handled tool with handle length 9 cm. Lift and hold it; leave all other objects untouched.

**Supported categories:** handled_tool, handled_paddle.

**Query:** handle_length = 9 cm.

Default scene: shape_conditioned, L5, 6 objects; exactly one complete match.

### MOU-3.3 — nested dimension

Family: `structured_objects`. Config: [nested_dimension.yaml](../configs/metric_spatial_understanding/metric_object_understanding/structured_objects/nested_dimension.yaml).

> Pick the rectangular container with inner width 7 cm and outer width 10 cm. Lift and hold it; leave all other objects untouched.

**Supported categories:** rectangular_container, rectangular_tray.

**Query:** inner_width = 7 cm, outer_width = 10 cm.

Default scene: shape_conditioned, L5, 6 objects; exactly one complete match.

## Object sizes and component shapes

Shared construction defaults and limits are in [object_sizes.yaml](../configs/object_sizes.yaml).
Every object must have all bounding extents at least 2.5 cm, a planar span of at
least 4.5 cm, and no extent above 22 cm. Grasp width is limited to 7.4 cm for the
8 cm Panda gripper. Handle/head extents must each be at least 2.5 cm. These limits
apply to targets and distractors, including objects loaded from an older manifest
when starting a new collection. Geometry annotations in existing data remain readable.

Component objects are 20 cm long with a 3 × 3 cm handle cross-section. The
handled tool has a rectangular head; the paddle has a flat elliptical head on
its broad face, represented by a 64-sided convex prism. Both are single rigid
assemblies with separate handle/head colliders. The handle query changes the
partition between handle and head while preserving total length. These are
procedural tool shapes, not imported photorealistic assets.

Cone grasp qualification failed under the permitted mass randomization. One
height-cone trial succeeded after adjusting its grasp, but the heavier volume
cone continued to slip. Cones are therefore blocked in new autonomous scenes
and when loading old scenes for collection. Historical cone recordings remain
readable and are retained as diagnostic evidence. The eleven default task
presets do not require cones.

The radius difficulty gap is half the listed linear gap, giving the same diameter
change as a width query. This factor is saved in each new scenario's difficulty
configuration; older saved scenarios retain their original gap semantics.

## Success and evaluation

Current collection uses an oracle pick/lift-and-hold controller. Success requires the correct parent object raised by at least 8 cm, within 16 cm of the hand origin, total finger displacement below 7.5 cm, and non-target displacement below 4 cm, sustained for 0.5 simulated seconds. It is a state-based proxy, not force-verified grasp success.

Shape correctness, numerical correctness, complete target identification, and manipulation success are separate evaluation outputs. Selected-object errors are reported by attribute in SI. Invalid selections fail identification and are excluded from metric-error averages with explicit coverage. Oracle generation is excluded from VLA identification accuracy.

Matched shape-only and measurement-only controls operate on the same observations. A counterfactual joint control changes the requested metric value while keeping the scene fixed. A policy that always selects the original target therefore fails that control.

No component/cavity dimensions, oracle IDs, or target labels may be used as policy inputs. They are evaluator-private annotations. RGB-D, calibration, instruction, and declared proprioception are the permitted observation contract.

## Difficulty and remaining qualification

Linear gaps are preliminary and can be infeasible with the gripper or represented cavity/component dimensions. Derived measurements use unit-correct relative thresholds. The generated manifest records excluded cells, actual gaps, effective object counts, and target-rank distributions. Numerical target values vary in the default manifest; direct YAML queries remain explicit fixed examples.

Target IDs are assigned after shuffling. Appearance does not encode target status or category. These controls reduce shortcuts but do not prove their absence; inspect rank and appearance balance in each actual split.

Local axes can be visually ambiguous, and cavities/components may be occluded. All scenarios start observability-unverified; the four shared cameras and 256×256 resolution are preserved. Review actual frames before accepting demonstrations.

Runtime validation and successful seeds do not establish reliability over all shapes/conditions/levels. See [runtime smoke evidence](runtime_smoke.json) and [offline validation](validation.json).
