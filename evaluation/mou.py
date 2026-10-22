# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Score explicit policy selections, separate from oracle manipulation collection."""

import argparse
import json
from collections import defaultdict
from pathlib import Path

import numpy as np
from tasks.metric_spatial_understanding.metric_object_understanding.scenarios import numeric_match, validate


def score(scenario, prediction):
    validate(scenario)
    control = prediction.get("control", "joint")
    if control not in ("joint", *scenario.get("matched_controls", {})):
        raise ValueError("Unknown matched control")
    object_id = prediction.get("selected_object_id")
    selected = next((o for o in scenario["objects"] if o["object_id"] == object_id), None)
    acceptable = (
        [scenario["target_id"]] if control == "joint" else scenario["matched_controls"][control]["acceptable_ids"]
    )
    query = (
        scenario["matched_controls"][control].get("query", scenario["query"])
        if control != "joint"
        else scenario["query"]
    )
    errors = (
        {}
        if selected is None
        else {
            attr: {
                "absolute_si": abs(selected["physical_properties_si"][attr] - q["value_si"]),
                "relative": abs(selected["physical_properties_si"][attr] - q["value_si"]) / q["value_si"],
            }
            for attr, q in query.items()
            if selected["physical_properties_si"].get(attr) is not None
        }
    )
    success = prediction.get("manipulation_success")
    if success is not None and not isinstance(success, bool):
        raise ValueError("manipulation_success must be boolean or absent")
    source = prediction.get("source", "policy")
    if source not in ("policy", "oracle"):
        raise ValueError("Prediction source must be policy or oracle")
    return dict(
        scenario_id=scenario["scenario_id"],
        case=scenario["case"],
        condition=scenario["condition"],
        level=scenario["level"],
        source=source,
        control=control,
        valid_selection=selected is not None,
        identification_correct=object_id in acceptable,
        shape_correct=bool(selected and selected["category"] == scenario["requested_shape"])
        if scenario["requested_shape"]
        else None,
        measurement_correct=bool(selected and numeric_match(selected, query)),
        metric_error=errors,
        manipulation_success=success,
    )


def summarize(rows):
    def mean(values):
        return float(np.mean(values)) if values else None

    policy = [r for r in rows if r["source"] == "policy"]
    joint = [r for r in policy if r["control"] == "joint"]
    shape = [r for r in joint if r["condition"] == "shape_conditioned"]
    groups = defaultdict(list)
    for row in joint:
        groups[row["level"]].append(row)
    by_case = defaultdict(list)
    for row in joint:
        by_case[row["case"]].append(row)
    errors = defaultdict(list)
    for row in joint:
        for attr, error in row["metric_error"].items():
            errors[attr].append(error["absolute_si"])
    return dict(
        policy_predictions=len(policy),
        oracle_predictions=len(rows) - len(policy),
        joint_predictions=len(joint),
        target_identification_accuracy=mean([r["identification_correct"] for r in joint]),
        shape_conditioned_identification_accuracy=mean([r["identification_correct"] for r in shape]),
        measurement_conditioned_identification_accuracy=mean([r["measurement_correct"] for r in joint]),
        shape_accuracy=mean([r["shape_correct"] for r in joint if r["shape_correct"] is not None]),
        selected_object_metric_error_si={attr: mean(v) for attr, v in errors.items()},
        metric_error_valid_selection_coverage=sum(bool(r["metric_error"]) for r in joint),
        manipulation_success=mean([r["manipulation_success"] for r in joint if r["manipulation_success"] is not None]),
        by_level={
            k: dict(count=len(v), identification_accuracy=mean([r["identification_correct"] for r in v]))
            for k, v in groups.items()
        },
        by_case={
            k: dict(count=len(v), identification_accuracy=mean([r["identification_correct"] for r in v]))
            for k, v in by_case.items()
        },
        matched_control_accuracy={
            mode: mean([r["identification_correct"] for r in policy if r["control"] == mode])
            for mode in ("shape_only", "measurement_only", "counterfactual_joint", "joint")
        },
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path)
    parser.add_argument("predictions", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    from data_collection.manifest import load_manifest

    manifest = load_manifest(args.manifest)
    scenarios = {entry["scenario"]["scenario_id"]: entry["scenario"] for entry in manifest["episodes"]}
    predictions = json.loads(args.predictions.read_text())
    rows = []
    seen = set()
    for prediction in predictions:
        key = (prediction["scenario_id"], prediction.get("control", "joint"), prediction.get("source", "policy"))
        if key in seen:
            raise ValueError("Duplicate prediction; aggregate repeated trials explicitly")
        seen.add(key)
        rows.append(score(scenarios[prediction["scenario_id"]], prediction))
    report = dict(summary=summarize(rows), per_prediction=rows)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report["summary"], indent=2))
