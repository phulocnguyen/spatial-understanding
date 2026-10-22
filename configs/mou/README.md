# Attribute configuration

`../mou.yaml` selects active attribute files, conditions, levels, count, seed and
maximum grasp width. Paths in `attribute_files` are relative to that profile file.
Each file in `attributes/` owns one measurement's shapes and requested values.

```yaml
case: diameter
shapes: [cylinder, circular_disk]
measurement: diameter
unit: mm
values: [42, 45, 48, 50, 52, 54]
```

Edit values here rather than editing generated scenario JSON. Dimensions and
instructions are generated from the same query. `area` means base footprint area;
`volume` means solid volume. Radius is half diameter. All cylinders remain upright.
A compound case uses an explicit `queries` list instead of `measurement/unit/values`.

| Attribute | Values | Unit | Active |
|---|---|---|---|
| Length | 8, 10, 12, 14, 16, 18 | cm | Yes |
| Width | 35, 40, 45, 50, 55, 60 | mm | Yes |
| Height | 6, 8, 10, 12, 14, 16 | cm | Yes |
| Radius | 21, 22, 23, 24, 25, 26 | mm | Yes |
| Diameter | 42, 45, 48, 50, 52, 54 | mm | Yes |
| Footprint area | 15, 18, 20, 25, 30 | cm2 | Yes |
| Solid volume | 100, 110, 120, 130 | cm3 | Yes |
| Capacity | 250, 300, 350 | cm3 | No: cavity visibility/grasp qualification pending |
| Component dimension | handle length 8, 9, 10 | cm | No: physical qualification pending |
| Multi-dimension | See file | cm | No: physical qualification pending |
| Nested dimension | See file | cm | No: cavity qualification pending |

Inactive files are editable specifications, not qualified collection presets.
They are not loaded unless added to `attribute_files`. Configs preserve the earlier
pilot/diverse plans as `../mou_pilot.yaml` and `../mou_diverse216.yaml`.

The default 756 slots provide 108 per attribute and 252 per condition/level.
Each attribute/condition/level cell has 12 scenes, enough to cover every configured
shape/value pair (two shapes and at most six values). Reducing count may leave
some combinations unrepresented. Increasing the number of shapes or values also
requires revisiting count and checking the generated coverage.

Width and diameter use half the global linear gap, radius one quarter, because
these dimensions directly constrain grasp clearance. All generated objects must
fit the configured 65 mm grasp-width limit. Extreme values may restrict feasible
target ranks. The scheduler prefers underrepresented ranks, but falls back to a
feasible rank instead of fabricating unsafe geometry. Inspect `target_rank_counts`
for residual shortcuts before using a plan for evaluation.

From IsaacLab:

```bash
uv run python Benchmark/generate_manifest.py --profile Benchmark/configs/mou.yaml --output Benchmark/manifests/mou/attributes756
```

To generate just one attribute, select its case and a separate output directory:

```bash
uv run python Benchmark/generate_manifest.py --case diameter --count 108 --output Benchmark/manifests/mou/diameter108
```

Generation is offline. New shapes/values, especially sphere volume and plate
width/area, still need rendered and physical rollout validation before production.
