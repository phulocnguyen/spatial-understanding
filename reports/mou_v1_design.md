# MOU v1 redesign

Current configuration: seven active attributes, owned by separate files in
`configs/mou/attributes/` and selected by `configs/mou.yaml`. See
[the attribute guide](../configs/mou/README.md) for values, semantics, and limits.
The generated 756-slot plan passes offline checks; it has not been collected.
Earlier pilot and 216-slot design details below are historical context.


## Current expansion

The completed 18-scene baseline is preserved in `configs/mou_pilot.yaml`.
`configs/mou.yaml` now selects a 216-scene diversity plan:

| Case | Shapes | Queries |
|---|---|---|
| Length | Rectangular bar, box | 10, 12, 14, 16 cm |
| Height | Box, upright cylinder | 6, 8, 10, 12 cm |
| Diameter | Upright cylinder, circular disk | 45, 50 mm |

All three conditions are enabled: same-shape, cross-shape (unique numerical
match), and shape-conditioned (shape and measurement must both match). There
are 72 scenarios per case, condition, and level. Each case/condition/level cell
covers every configured shape/query pair. Shape-conditioned scenes have four
objects to represent all four combinations of correct/incorrect shape and
measurement; other scenes have three. Only one instruction is recorded.
The generator restricts distractor shapes to the configured pool.

These new combinations have offline validation, not physical qualification.
The remaining document describes the controlled baseline and validation gates.


## Objective

Given a metric instruction and calibrated RGB-D observations, identify the exact
matching object and pick/lift/hold it while leaving distractors in place.
MOU is one task within Metric Spatial Understanding. Current collection uses
oracle geometry to generate demonstrations; it does not demonstrate a VLA reading
measurements from images. Evaluate a trained policy without object geometry or
target IDs in its inputs.

## Configuration ownership

| File | Edit here for |
|---|---|
| `configs/mou.yaml` | Active cases, shapes, integer query values/units, pilot size, conditions, levels |
| `configs/difficulty.yaml` | Measurement gaps; object count and appearance held constant |
| `configs/settings.yaml` | Table, robot, calibrated cameras, timing, lighting, success thresholds |
| `configs/domain_randomization.yaml` | Reset variation |
| `configs/object_sizes.yaml` | General geometry safety limits and default primitive sizes |
| `tasks/.../catalog.py` | Experimental task definitions and measurement semantics |
| `manifests/mou/` | Generated disposable plans; never the source configuration |

The profile is consumed by `generate_manifest.py --profile ...`. The CLI defaults
to `configs/mou.yaml`; `--count`, `--seed`, `--case`, `--condition`, and `--level`
can override selection for diagnostics. Individual task YAMLs remain experimental
entry points and do not define the new pilot.

## Active first version

| Case | Shape | Query choices | Meaning |
|---|---|---|---|
| Length | Rectangular bar | 12, 16 cm | Long local axis, with fixed 4 cm grasp width |
| Height | Box | 8, 12 cm | Upright vertical dimension |
| Diameter | Upright cylinder | 45, 50 mm | Diameter of circular base, not radius |

Each scene has three same-shape objects and exactly one instruction. Query values
are authored explicitly; no scaling factors and no counterfactual prompts.
Distractor measurements are exact geometry annotations but may be fractional;
only the requested measurement must be a clean number. Shapes remain upright.
The pilot rejects any object with grasp width above 6.5 cm. General experimental
geometry retains its older 7.4 cm upper bound; that is not the pilot qualification limit.

Deferred: width (axis ambiguity), radius (redundant with diameter initially),
area/volume (derived measurements), capacity/interior dimensions (cavity visibility
and repeated collection failures), compound dimensions and components (more grasp
and feature-visibility qualification needed). Their source implementations remain
available; they are excluded from default generation.

## Difficulty and balance

Pilot: 18 scenarios = three cases × three levels × two scenarios per cell.
L1/L2/L3 minimum gaps are 20/15/10 mm for length/height and 10/7.5/5 mm for diameter.
Diameter uses a factor of 0.5 because of its smaller usable grasp range.
Three objects and similar appearance remain fixed across levels. L4/L5 remain
experimental and require visibility qualification at 256 × 256.

Scheduling balances case-by-level combinations, not just separate totals. Target
rank is balanced per case across smallest/middle/largest, preventing an always-
choose-the-middle shortcut. Placement order and color shade are independent of
identity. These checks do not replace auditing correlations on a larger dataset.
Seed 712 pilot checks: zero excluded cells, two scenes per case/level cell, two
examples of each target rank per case. Train/eval assignment is geometry-grouped;
a small pilot is for qualification, not a statistical benchmark. Before production,
create explicit held-out target-value and geometry splits; a new seed alone does
not establish unseen-size generalization.

## Controlled collection settings

- Four views retained: top, left, right, wrist; 256 × 256 RGB-D, 20 Hz.
- Control 30 Hz; physics 120 Hz. Recorded calibration follows actual poses.
- Path tracing reduced to 16 samples; inspect quality before production.
- Simulation budget 8 seconds; worker deadline 300 wall-clock seconds, including export.
- Three attempts per slot; failed/incomplete attempts do not count as demonstrations.
- Fixed table color, lighting, camera poses, robot reset, yaw, 50 g mass and nominal CoM.
- Object XY jitter ±2.5 cm, with existing overlap/table checks.
- Existing sleep-until-contact reset retained. No claim of verified contact forces.

Success requires a 5 cm lift from the settled initial object center, hand proximity and closed-gripper proxy, distractor
movement under 4 cm, held for 0.5 seconds. This is pick/lift/hold, not pick-and-place.
State-based success does not prove a valid two-finger grasp. Review videos and
contact behavior before treating the resulting data as qualified demonstrations.

## Required gates before a new collection

1. Generate the pilot and check every case/level/query and target-rank distribution.
2. Preview all three cases; inspect the actual visible measured dimension in each
   camera, especially cylinder diameter at the smallest gap. A projected bounding
   box or successful oracle grasp is not sufficient evidence of observability.
3. Execute the pilot and review successful and failed trials. Confirm no pre-contact
   movement, clipping, slips, collisions, or target leakage through placement/color.
4. Validate HDF5 timestamps, action/image correspondence, depth, camera calibration,
   episode finalization, and four videos. Timeouts can leave partial recordings.
5. Only then expand to production and enable one nuisance randomization at a time.

This redesign has offline geometry/configuration checks. It has NOT yet received
new physical rollout or rendered-view validation. All collection was stopped as
requested; no production run was restarted.
