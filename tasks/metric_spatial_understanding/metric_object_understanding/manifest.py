# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""MOU-specific balanced scenario generation; collection remains task independent."""

import argparse
import json
from collections import Counter
from pathlib import Path

import yaml
from assets.geometry import describe

from .catalog import CASES, CONDITIONS, QUERY_VALUES
from .scenarios import LEVELS, InfeasibleScenario, generate, validate


def load_profile(path):
    """Resolve attribute files relative to the profile and embed their exact values."""
    path = Path(path)
    profile = yaml.safe_load(path.read_text())
    if "attribute_files" not in profile:
        return profile
    if "cases" in profile:
        raise ValueError("Use either inline cases or attribute_files, not both")
    profile["cases"] = {}
    for relative in profile["attribute_files"]:
        spec = yaml.safe_load((path.parent / relative).read_text())
        name = spec.pop("case")
        if name in profile["cases"] or name not in CASES:
            raise ValueError(f"Duplicate or unknown attribute: {name}")
        if "values" in spec:
            measurement = spec.pop("measurement")
            unit = spec.pop("unit")
            values = spec.pop("values")
            if not values or len(values) != len(set(values)):
                raise ValueError(f"Empty or repeated values for {name}")
            spec["queries"] = [{measurement: {"value": value, "unit": unit}} for value in values]
        profile["cases"][name] = spec
    if not profile["cases"]:
        raise ValueError("No active attribute files")
    return profile


def create(count=300, seed=712, case=None, condition=None, level=None, vary_queries=True, profile=None):
    selected_cases = list(profile["cases"]) if profile else list(CASES)
    selected_conditions = profile["conditions"] if profile else CONDITIONS
    selected_levels = profile["levels"] if profile else list(LEVELS)
    queries = dict(QUERY_VALUES)
    shapes = {}
    if profile:
        for name, spec in profile["cases"].items():
            pool = spec.get("shapes", [spec.get("shape")])
            if (
                name not in CASES
                or not spec["queries"]
                or not pool
                or any(shape not in CASES[name]["shapes"] for shape in pool)
            ):
                raise ValueError(f"Invalid MOU profile case: {name}")
            queries[name] = [{key: (v["value"], v["unit"]) for key, v in q.items()} for q in spec["queries"]]
            shapes[name] = pool
        if any(c not in CONDITIONS for c in selected_conditions) or any(
            level_name not in LEVELS for level_name in selected_levels
        ):
            raise ValueError("Invalid profile conditions or levels")
    candidates = []
    infeasible = []
    for name in [case] if case else selected_cases:
        for mode in [condition] if condition else selected_conditions:
            for difficulty in [level] if level else selected_levels:
                try:
                    # A scheduled cell must support every authored query it will receive.
                    # Retrying seeds cannot repair an impossible size/gap combination.
                    for values in queries[name] if vary_queries else [CASES[name]["query"]]:
                        query = {attr: {"value": value, "unit": unit} for attr, (value, unit) in values.items()}
                        for shape in shapes.get(name, [None]):
                            generate(
                                name,
                                mode,
                                difficulty,
                                seed,
                                measurements=query,
                                requested_shape=shape,
                                allowed_shapes=shapes[name] if len(shapes.get(name, [])) > 1 else None,
                            )
                    candidates.append((name, mode, difficulty))
                except InfeasibleScenario as error:
                    infeasible.append(dict(case=name, condition=mode, level=difficulty, reason=str(error)))
    if not candidates:
        raise ValueError("No feasible task/condition/level cells")
    # Balance difficulty first; restricted cases must not overpopulate L5.
    by_level = {difficulty: [cell for cell in candidates if cell[2] == difficulty] for difficulty in LEVELS}
    available_levels = [difficulty for difficulty, cells in by_level.items() if cells]
    cell_occurrences = Counter()
    level_case_occurrences = Counter()
    rank_occurrences = Counter()
    level_occurrences = Counter()
    condition_occurrences = Counter()
    episodes = []
    split_by_geometry = {}
    case_occurrences = Counter()
    for index in range(count):
        difficulty = available_levels[index % len(available_levels)]
        name, mode, difficulty = min(
            by_level[difficulty],
            key=lambda cell: (
                level_case_occurrences[(cell[2], cell[0])],
                case_occurrences[cell[0]],
                cell_occurrences[cell],
                condition_occurrences[cell[1]],
            ),
        )
        cell_occurrences[(name, mode, difficulty)] += 1
        level_case_occurrences[(difficulty, name)] += 1
        condition_occurrences[mode] += 1
        # Split within each level, avoiding correlation with the five-level cycle.
        split = "eval" if level_occurrences[difficulty] % 5 == 0 else "train"
        level_occurrences[difficulty] += 1
        occurrence = case_occurrences[name]
        case_occurrences[name] += 1
        options = queries[name] if vary_queries else [CASES[name]["query"]]
        values = options[(cell_occurrences[(name, mode, difficulty)] - 1 if profile else occurrence) % len(options)]
        pool = shapes.get(name, [None])
        shape = pool[((cell_occurrences[(name, mode, difficulty)] - 1) // len(options)) % len(pool)]
        desired_rank = min(range(3), key=lambda rank: rank_occurrences[(name, rank)]) if profile else None
        feasible_rank_candidates = []
        for retry in range(100):
            try:
                query = {attr: {"value": value, "unit": unit} for attr, (value, unit) in values.items()}
                scenario = generate(
                    name,
                    mode,
                    difficulty,
                    seed + 101 * index + retry,
                    measurements=query,
                    requested_shape=shape,
                    allowed_shapes=pool if len(pool) > 1 else None,
                    split=split,
                )
                if profile and any(
                    describe(obj["geometry"])["grasp_width_m"] > profile["maximum_grasp_width_m"]
                    for obj in scenario["objects"]
                ):
                    continue
                if profile:
                    feasible_rank_candidates.append(scenario)
                    if next(iter(scenario["target_measurement_ranks"].values())) != desired_rank:
                        continue
                break
            except InfeasibleScenario:
                continue
        else:
            if not feasible_rank_candidates:
                raise ValueError(f"Could not generate {name}/{mode}/{difficulty}: {values}, shape={shape}")
            scenario = min(
                feasible_rank_candidates,
                key=lambda s: rank_occurrences[(name, next(iter(s["target_measurement_ranks"].values())))],
            )
        if profile:
            rank_occurrences[(name, next(iter(scenario["target_measurement_ranks"].values())))] += 1
        scenario["split"] = split_by_geometry.setdefault(scenario["control_group_id"], scenario["split"])
        episodes.append(dict(slot=index, scenario=scenario))
    result = dict(
        schema_version=1,
        benchmark="Benchmark",
        category="metric_spatial_understanding",
        task="metric_object_understanding",
        seed=seed,
        query_sampling="explicit_round_values" if vary_queries else "fixed_catalog_values",
        planned_slots=count,
        generation_profile=profile,
        accepted_demonstrations=0,
        infeasible_cells=infeasible,
        episodes=episodes,
    )
    validate_manifest(result)
    return result


def validate_manifest(manifest):
    seen = {}
    slots = set()
    for entry in manifest["episodes"]:
        if entry["slot"] in slots:
            raise ValueError("Duplicate manifest slot")
        slots.add(entry["slot"])
        s = entry["scenario"]
        info = validate(s)
        old = seen.setdefault(info["control_group_id"], s["split"])
        if old != s["split"]:
            raise ValueError("Matched geometry leaks across evaluation splits")
    ranks = Counter(
        f"{e['scenario']['case']}:{attr}:rank_{rank}"
        for e in manifest["episodes"]
        for attr, rank in e["scenario"]["target_measurement_ranks"].items()
    )
    return dict(
        slots=len(slots),
        target_rank_counts=dict(ranks),
        by_case=dict(Counter(e["scenario"]["case"] for e in manifest["episodes"])),
        by_condition=dict(Counter(e["scenario"]["condition"] for e in manifest["episodes"])),
        by_level=dict(Counter(e["scenario"]["level"] for e in manifest["episodes"])),
        infeasible_cells=len(manifest.get("infeasible_cells", [])),
    )


def generate_main():
    parser = argparse.ArgumentParser(description="Generate Benchmark MOU scene manifests; does not collect episodes")
    parser.add_argument(
        "--fixed_queries", action="store_true", help="Disable target-value variation for controlled diagnostics"
    )
    parser.add_argument("--profile", type=Path, default=Path(__file__).resolve().parents[3] / "configs/mou.yaml")
    parser.add_argument("--count", type=int)
    parser.add_argument("--seed", type=int)
    parser.add_argument("--case", choices=list(CASES))
    parser.add_argument("--condition", choices=CONDITIONS)
    parser.add_argument("--level", choices=list(LEVELS))
    parser.add_argument("--output", type=Path, default=Path("Benchmark/manifests/mou/pilot"))
    args = parser.parse_args()
    profile = load_profile(args.profile)
    count = args.count if args.count is not None else profile["count"]
    seed = args.seed if args.seed is not None else profile["seed"]
    if count < 1:
        parser.error("count must be positive")
    manifest = create(count, seed, args.case, args.condition, args.level, not args.fixed_queries, profile)
    from data_collection.manifest import write_manifest

    report = validate_manifest(manifest)
    if args.output.suffix == ".json":
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(manifest, indent=2) + "\n")
    else:
        write_manifest(manifest, args.output, report)
    print(json.dumps(report, indent=2))
