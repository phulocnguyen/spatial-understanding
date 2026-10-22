# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Offline task/configuration checks and manifest shortcut diagnostics."""

import argparse
import json
from collections import Counter, defaultdict
from pathlib import Path

from assets.geometry import describe
from data_collection.config import load_config
from data_collection.manifest import load_manifest, validate_manifest
from data_collection.randomization import sample_episode
from tasks.metric_spatial_understanding.metric_object_understanding.catalog import (
    AUTONOMOUS_EXCLUSIONS,
    CASES,
    CONDITIONS,
)
from tasks.metric_spatial_understanding.metric_object_understanding.scenarios import (
    LEVELS,
    InfeasibleScenario,
    generate,
    object_specs,
)
from tasks.metric_spatial_understanding.metric_object_understanding.task import (
    MOUConfig,
    configure,
    resolve_settings,
)

ROOT = Path(__file__).resolve().parents[1]


def audit_tasks(seeds=3):
    """Exercise every declared shape/condition/level and each shipped collection YAML."""
    errors = []
    presets = []
    counts = Counter()
    infeasible = Counter()
    missing_controls = Counter()
    for path in sorted((ROOT / "configs/metric_spatial_understanding").rglob("*.yaml")):
        try:
            config = load_config(path, {}, MOUConfig)
            configure(config)
            presets.append(str(path.relative_to(ROOT)))
        except (ValueError, TypeError, KeyError) as error:
            errors.append({"config": str(path.relative_to(ROOT)), "error": str(error)})
    config = load_config(
        ROOT / "configs/metric_spatial_understanding/metric_object_understanding/physical_dimensions/length.yaml",
        {},
        MOUConfig,
    )
    settings = resolve_settings(config)
    for case, definition in CASES.items():
        for shape in definition["shapes"]:
            for condition in CONDITIONS:
                for level in LEVELS:
                    cell = f"{case}/{shape}/{condition}/{level}"
                    for seed in range(712, 712 + seeds):
                        try:
                            scene = generate(case, condition, level, seed, requested_shape=shape)
                            specs = object_specs(scene, settings)
                            if not all(describe(obj["geometry"])["grasp_feasible"] for obj in specs):
                                raise ValueError("Scene contains unsupported geometry")
                            sample_episode(specs, config.randomization, seed=seed, table_size=settings.TABLE_SIZE)
                            counts["valid_scenes"] += 1
                            if "counterfactual_joint" not in scene["matched_controls"]:
                                missing_controls[cell] += 1
                        except InfeasibleScenario:
                            counts["excluded_scenes"] += 1
                            infeasible[cell] += 1
                        except (ValueError, TypeError, KeyError) as error:
                            errors.append({"cell": cell, "seed": seed, "error": str(error)})
    return {
        "passed": not errors,
        "autonomous_exclusions": AUTONOMOUS_EXCLUSIONS,
        "presets_checked": presets,
        "seeds_per_cell": seeds,
        "counts": dict(counts),
        "errors": errors,
        "excluded_cells": dict(infeasible),
        "cells_without_counterfactual_control": dict(missing_controls),
    }


def audit_plan(path):
    """Report query/rank/shape coverage without treating geometry as visual evidence."""
    manifest = load_manifest(path)
    validate_manifest(manifest)
    queries, ranks, shapes, splits = (defaultdict(Counter) for _ in range(4))
    for entry in manifest["episodes"]:
        scene = entry["scenario"]
        case = scene["case"]
        queries[case][json.dumps(scene["query"], sort_keys=True)] += 1
        ranks[case][str(scene["target_measurement_ranks"])] += 1
        shapes[case][scene["objects"][scene["target_index"]]["category"]] += 1
        splits[case][scene["split"]] += 1
    warnings = []
    for case in queries:
        if len(queries[case]) == 1:
            warnings.append(f"{case}: only one target value; unsuitable as a varied-query training set")
        if len(ranks[case]) == 1:
            warnings.append(f"{case}: target always has the same measurement rank")
        if len(shapes[case]) == 1:
            warnings.append(f"{case}: only one target category; no target-shape generalization coverage")
    return {
        "path": str(path),
        "slots": len(manifest["episodes"]),
        "distinct_queries_by_case": {key: len(value) for key, value in queries.items()},
        "target_rank_counts": dict(ranks),
        "target_shape_counts": dict(shapes),
        "split_counts": dict(splits),
        "warnings": warnings,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", type=int, default=3)
    parser.add_argument("--manifest", type=Path)
    parser.add_argument("--output", type=Path, default=ROOT / "reports/collection_preflight.json")
    args = parser.parse_args()
    if args.seeds < 1:
        parser.error("--seeds must be positive")
    report = {"tasks": audit_tasks(args.seeds)}
    if args.manifest:
        report["manifest"] = audit_plan(args.manifest)
    report["scope"] = (
        "Offline geometry, configuration, placement and split checks only. "
        "Physical success and visible metric evidence require recorded episode review. "
        "A passing result is not a VLA accuracy or large-scale collection certification."
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(
        json.dumps(
            {
                "passed": report["tasks"]["passed"],
                **report["tasks"]["counts"],
                "errors": len(report["tasks"]["errors"]),
                "report": str(args.output),
            },
            indent=2,
        )
    )
    if not report["tasks"]["passed"]:
        raise SystemExit(1)
