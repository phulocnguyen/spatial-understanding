# MOU plans

Generated plans and old MOU recordings were cleared at the user's request.
Collection is stopped. This folder stores generated outputs, not task configuration.

Edit `../configs/mou.yaml` first. It selects attribute files, conditions, levels, count, seed, and safe grasp-width limit.
Edit shapes and values in the selected files under `configs/mou/attributes/`.
The default is a 756-scenario diversity plan. Use `--profile Benchmark/configs/mou_pilot.yaml` for the preserved 18-scenario baseline.

From the IsaacLab root:

```bash
uv run python Benchmark/generate_manifest.py --output Benchmark/manifests/mou/pilot
```

Outputs: `plan.yaml`, readable `slots.csv`, detailed `scenarios/`, and
`reports/coverage.yaml` plus `reports/excluded_cells.yaml`.

Read `../reports/mou_v1_design.md` before collection. Generation checks geometry;
it does not establish grasp reliability or visibility. Do not scale to 300 until
the pilot passes those checks. The collection deadline remains 300 wall-clock
seconds per attempt, including startup and video export.

The diverse plan is in `mou/diverse216/plan.yaml`. It is generated, not collected.

Current plan: `mou/attributes756/plan.yaml`. See `../configs/mou/README.md` for per-attribute settings.
