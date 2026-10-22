# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Controlled MOU episodes with exact local geometry and separate shape/numerical truth."""

import hashlib
import json
from copy import deepcopy
from pathlib import Path

import numpy as np
import yaml
from assets.geometry import OBJECT_SIZES, describe, make
from core.units import DIMENSIONS, equal, to_si

from .catalog import AUTONOMOUS_EXCLUSIONS, CASES, CATEGORY, CONDITIONS, TASK

ROOT = Path(__file__).resolve().parents[3]
LEVELS = yaml.safe_load((ROOT / "configs/difficulty.yaml").read_text())


class InfeasibleScenario(ValueError):
    """A requested task/condition/level cannot satisfy its exact design constraints."""


def query_for(case, overrides=None):
    spec = CASES[case]
    raw = overrides or {key: {"value": value, "unit": unit} for key, (value, unit) in spec["query"].items()}
    if set(raw) != set(spec["query"]):
        raise ValueError(
            "Query must specify exactly the measurements defined by the case; depth/thickness are out of scope"
        )
    return {
        key: {"value": v["value"], "unit": v["unit"], "value_si": to_si(v["value"], v["unit"], key)}
        for key, v in raw.items()
    }


def numeric_match(obj, query):
    return all(equal(obj["physical_properties_si"].get(key), item["value_si"]) for key, item in query.items())


def full_match(obj, query, requested_shape):
    return numeric_match(obj, query) and (requested_shape is None or obj["category"] == requested_shape)


def construct(category, case, values, variant):
    first = next(iter(values))
    g = make(category, first, values[first], variant)
    if case == "multi_dimension":
        if category == "handled_tool":
            length, width, height = [values[k] for k in ("length", "width", "height")]
            g = {
                "kind": "handled",
                "handle_size_m": [length * 0.65, width * 0.65, height * 0.8],
                "head_size_m": [length * 0.35, width, height],
                "head_kind": "box",
            }
        else:
            g["size_m"] = [values[k] for k in ("length", "width", "height")]
    elif case == "nested_dimension":
        g["outer_size_m"][1] = values["outer_width"]
        g["inner_size_xy_m"][1] = values["inner_width"]
    info = describe(g)
    return dict(
        category=category,
        geometry=g,
        physical_properties_si=info["measurements"],
        internal_geometry=info["interior"],
        components=info["components"],
    )


def instruction(case, query, shape):
    noun = shape.replace("_", " ") if shape else "object"
    terms = []
    for attr, q in query.items():
        label = {
            "footprint_area": "local base/footprint area",
            "volume": "solid volume",
            "capacity": "internal usable capacity",
            "handle_length": "handle length",
            "inner_width": "inner width",
            "outer_width": "outer width",
        }.get(attr, "local " + attr)
        terms.append(f"{label} {q['value']:g} {q['unit']}")
    return f"Pick the {noun} with " + " and ".join(terms) + ". Lift and hold it; leave all other objects untouched."


def fingerprint(scenario):
    payload = {key: scenario[key] for key in ("case", "condition", "query", "requested_shape", "objects")}
    return hashlib.sha256(json.dumps(payload, sort_keys=True).encode()).hexdigest()


def geometry_group_id(objects, query=None):
    """Physical scene split identity, independent of object IDs and query phrasing."""
    canonical = sorted(
        json.dumps({"category": o["category"], "geometry": o["geometry"]}, sort_keys=True) for o in objects
    )
    payload = {"geometry": canonical}
    if query is not None:
        payload["query"] = query  # Read pre-finalization smoke records.
    return hashlib.sha256(json.dumps(payload, sort_keys=True).encode()).hexdigest()


def generate(
    case, condition, level, seed, *, requested_shape=None, measurements=None, split="train", allowed_shapes=None
):
    if (
        case not in CASES
        or condition not in CONDITIONS
        or level not in LEVELS
        or split not in ("train", "eval", "diagnostic")
    ):
        raise ValueError("Invalid MOU case, condition, difficulty, or split")
    cfg = CASES[case]
    category = requested_shape or cfg["shapes"][0]
    if category not in cfg["shapes"]:
        raise ValueError("Requested shape is not defined for this measurement")
    if category in AUTONOMOUS_EXCLUSIONS:
        raise InfeasibleScenario(
            f"{category} is excluded from autonomous collection: {AUTONOMOUS_EXCLUSIONS[category]}"
        )
    query = query_for(case, measurements)
    values = {k: q["value_si"] for k, q in query.items()}
    rng = np.random.default_rng(seed)
    difficulty = deepcopy(LEVELS[level])
    difficulty["linear_gap_scale"] = deepcopy(OBJECT_SIZES["linear_gap_scale"])
    minimum = 4 if condition == "shape_conditioned" else 2
    if len(values) > 1:
        minimum = max(
            minimum,
            1 + len(values) + (2 if condition == "shape_conditioned" else 1 if condition == "cross_shape" else 0),
        )
    n = max(difficulty["objects"], minimum)
    difficulty["effective_objects"] = n
    alternative = next(c for c in (allowed_shapes or cfg["shapes"]) if c != category and c not in AUTONOMOUS_EXCLUSIONS)
    objects = [construct(category, case, values, seed % 4 if case == "volume" else int(rng.integers(0, 4)))]
    if not describe(objects[0]["geometry"])["grasp_feasible"]:
        raise InfeasibleScenario(f"{case}/{category}: target exceeds supported grasp envelope")
    roles = ["correct_shape_correct_measurement"]
    gaps = []
    for index in range(1, n):
        wrong_shape = (condition == "cross_shape" and index % 2 == 1) or (
            condition == "shape_conditioned" and (index == 1 or index >= n - 1)
        )
        if len(values) > 1 and condition == "cross_shape":
            wrong_shape = index > len(values)
        obj_category = alternative if wrong_shape else category
        same_number = condition == "shape_conditioned" and index == 1
        # Shape-conditioned roles always include all four quadrants.
        if condition == "shape_conditioned" and index == 2:
            obj_category = category
        if condition == "shape_conditioned" and index == 3 and len(values) == 1:
            obj_category = alternative
        candidate = None
        if same_number:
            candidate = construct(
                obj_category, case, values, (seed + index) % 4 if case == "volume" else int(rng.integers(0, 4))
            )
        else:
            attr = list(values)[
                (index - 2 if condition == "shape_conditioned" and len(values) > 1 else index - 1) % len(values)
            ]
            dimension = DIMENSIONS[attr]
            gap = (
                difficulty["minimum_linear_gap_m"] * difficulty["linear_gap_scale"].get(attr, 1.0)
                if dimension == 1
                else values[attr] * difficulty["relative_derived_gap"]
            )
            # Deterministic but varied signs/near-miss values, bounded by represented geometry.
            signs = [-1, 1] if rng.random() < 0.5 else [1, -1]
            for sign in signs:
                changed = dict(values)
                changed[attr] += sign * gap * (1.02 + float(rng.uniform(0, 0.1)))
                if changed[attr] <= 0:
                    continue
                try:
                    trial = construct(
                        obj_category, case, changed, (seed + index) % 4 if case == "volume" else int(rng.integers(0, 4))
                    )
                    info = describe(trial["geometry"])
                    if not info["grasp_feasible"]:
                        continue
                    candidate = trial
                    gaps.append(
                        {
                            "attribute": attr,
                            "minimum_si": gap,
                            "actual_si": abs(changed[attr] - values[attr]),
                            "dimension": dimension,
                        }
                    )
                    break
                except ValueError:
                    continue
        if candidate is None:
            raise InfeasibleScenario(f"{case}/{condition}/{level}: exact gap/geometry exceeds supported grasp envelope")
        if not describe(candidate["geometry"])["grasp_feasible"]:
            raise InfeasibleScenario(f"{case}/{obj_category}: distractor exceeds supported grasp envelope")
        objects.append(candidate)
        roles.append(
            ("incorrect_shape" if obj_category != category else "correct_shape")
            + "_"
            + ("correct_measurement" if same_number else "incorrect_measurement")
        )
    selected_shape = category if condition in ("same_shape", "shape_conditioned") or len(query) > 1 else None
    for i, obj in enumerate(objects):
        obj.update(object_id=f"object_{i}", distractor_role=roles[i])
    rng.shuffle(objects)
    for i, obj in enumerate(objects):
        obj["object_id"] = f"object_{i}"
    matches = [i for i, o in enumerate(objects) if full_match(o, query, selected_shape)]
    if len(matches) != 1:
        raise InfeasibleScenario("Scene does not have a unique complete instruction match")
    for i, obj in enumerate(objects):
        obj["target_match"] = i == matches[0]
        obj["shape_match"] = obj["category"] == category
        obj["measurement_match"] = numeric_match(obj, query)
    result = dict(
        schema_version=1,
        benchmark="Benchmark",
        category=CATEGORY,
        task=TASK,
        case=case,
        case_id=cfg["id"],
        family=cfg["family"],
        condition=condition,
        level=level,
        seed=seed,
        split=split,
        query=query,
        requested_shape=selected_shape,
        reference_shape=category,
        objects=objects,
        target_index=matches[0],
        target_id=objects[matches[0]]["object_id"],
        distractor_ids=[o["object_id"] for i, o in enumerate(objects) if i != matches[0]],
        difficulty=difficulty,
        separation=gaps,
        observability_status="unverified",
        observability_reason=(
            "Requires calibrated rendered-view review; local axes and cavities may be ambiguous/occluded"
        ),
        manipulation_protocol="oracle_pick_lift_hold_proxy",
        geometry_qualification="analytic_and_procedural; physics validation pending",
    )
    result["control_group_id"] = geometry_group_id(objects)
    necessity = {
        attr: any(
            o["category"] == category
            and not equal(o["physical_properties_si"].get(attr), q["value_si"])
            and all(
                equal(o["physical_properties_si"].get(other), oq["value_si"])
                for other, oq in query.items()
                if other != attr
            )
            for o in objects
        )
        for attr, q in query.items()
    }
    result["constraint_necessity"] = necessity
    result["target_measurement_ranks"] = {
        attr: sum(
            o["physical_properties_si"].get(attr) is not None
            and o["physical_properties_si"][attr] < q["value_si"] - 1e-12
            for o in objects
        )
        for attr, q in query.items()
    }
    result["rank_balance_status"] = (
        "Requires manifest-level distribution review; feasibility can limit rank counterbalancing"
    )
    result["instruction"] = instruction(case, query, selected_shape)
    result["fingerprint"] = fingerprint(result)
    result["scenario_id"] = f"MOU_{case}_{condition}_{level}_{seed}_{result['fingerprint'][:10]}"
    validate(result)
    return result


def validate(s):
    if s["benchmark"] != "Benchmark" or s["category"] != CATEGORY or s["task"] != TASK or s["case"] not in CASES:
        raise ValueError("Not a Benchmark Metric Spatial Understanding / MOU scenario")
    if (
        s["condition"] not in CONDITIONS
        or s["level"] not in LEVELS
        or s["split"] not in ("train", "eval", "diagnostic")
    ):
        raise ValueError("Invalid condition/level/split")
    if any(s["difficulty"].get(key) != value for key, value in LEVELS[s["level"]].items()):
        raise ValueError("Difficulty factors disagree with the declared level")
    if s["difficulty"]["effective_objects"] != len(s["objects"]):
        raise ValueError("Incorrect effective object count")
    if s["fingerprint"] != fingerprint(s):
        raise ValueError("Scenario fingerprint mismatch")
    if s["observability_status"] not in ("unverified", "verified", "rejected"):
        raise ValueError("Invalid visibility status")
    expected = query_for(s["case"], {k: {"value": q["value"], "unit": q["unit"]} for k, q in s["query"].items()})
    if expected != s["query"]:
        raise ValueError("Inconsistent SI query")
    identities = [o["object_id"] for o in s["objects"]]
    if len(set(identities)) != len(identities):
        raise ValueError("Duplicate object IDs")
    matches = []
    for i, obj in enumerate(s["objects"]):
        if obj["category"] not in CASES[s["case"]]["shapes"]:
            raise ValueError("Category outside the case definition")
        if obj["category"] == "cylinder" and obj["geometry"].get("axis", "Z") != "Z":
            raise ValueError("Cylinder measurement cases require an upright cylinder on its circular base")
        info = describe(obj["geometry"])
        if (
            info["measurements"] != obj["physical_properties_si"]
            or info["interior"] != obj["internal_geometry"]
            or info["components"] != obj["components"]
        ):
            raise ValueError("Geometry annotations disagree with represented object")
        match = full_match(obj, s["query"], s["requested_shape"])
        if (
            obj["target_match"] != match
            or obj["measurement_match"] != numeric_match(obj, s["query"])
            or obj["shape_match"] != (obj["category"] == s["reference_shape"])
        ):
            raise ValueError("Incorrect object match annotations")
        if match:
            matches.append(i)
    if matches != [s["target_index"]] or s["target_id"] != s["objects"][matches[0]]["object_id"]:
        raise ValueError("Incorrect unique target annotation")
    if s["instruction"] != instruction(s["case"], s["query"], s["requested_shape"]):
        raise ValueError("Instruction/query mismatch")
    if s["condition"] == "shape_conditioned":
        quadrants = {(o["category"] == s["requested_shape"], numeric_match(o, s["query"])) for o in s["objects"]}
        if quadrants != {(True, True), (True, False), (False, True), (False, False)}:
            raise ValueError("Shape-conditioned scene must contain all four distractor quadrants")
    if s["condition"] == "same_shape" and len({o["category"] for o in s["objects"]}) != 1:
        raise ValueError("Same-shape condition contains different categories")
    if s["condition"] == "cross_shape" and (
        len({o["category"] for o in s["objects"]}) < 2 or sum(numeric_match(o, s["query"]) for o in s["objects"]) != 1
    ):
        raise ValueError("Cross-shape condition must have multiple categories and one numerical match")
    for mode, expected_ids in (
        ("shape_only", [o["object_id"] for o in s["objects"] if o["category"] == s["reference_shape"]]),
        ("measurement_only", [o["object_id"] for o in s["objects"] if numeric_match(o, s["query"])]),
    ):
        if "matched_controls" in s and s["matched_controls"][mode]["acceptable_ids"] != expected_ids:
            raise ValueError("Matched-control annotations inconsistent")
    for gap in s["separation"]:
        attr = gap["attribute"]
        q = s["query"][attr]
        expected_min = (
            LEVELS[s["level"]]["minimum_linear_gap_m"] * s["difficulty"].get("linear_gap_scale", {}).get(attr, 1.0)
            if DIMENSIONS[attr] == 1
            else q["value_si"] * LEVELS[s["level"]]["relative_derived_gap"]
        )
        if not equal(gap["minimum_si"], expected_min) or gap["dimension"] != DIMENSIONS[attr]:
            raise ValueError("Incorrect separation threshold/units")
        actual = [
            abs(o["physical_properties_si"][attr] - q["value_si"])
            for o in s["objects"]
            if o["physical_properties_si"].get(attr) is not None
        ]
        if not any(equal(value, gap["actual_si"]) for value in actual):
            raise ValueError("Claimed distractor gap does not match actual geometry")
        if gap["actual_si"] + 1e-12 < gap["minimum_si"]:
            raise ValueError("Distractor gap below declared minimum")
    group = geometry_group_id(s["objects"])
    if s["control_group_id"] not in (group, geometry_group_id(s["objects"], s["query"])):
        raise ValueError("Invalid physical scene split identity")
    return {"objects": len(s["objects"]), "targets": 1, "condition": s["condition"], "control_group_id": group}


def object_specs(s, settings):
    import colorsys

    specs = []
    n = len(s["objects"])
    palette = list(dict.fromkeys(tuple(c) for c in settings.COLORS))
    for i in range(n):
        if len(palette) >= n:
            break
        color = colorsys.hsv_to_rgb((i + 0.5) / n, 0.75, 0.85)
        if color not in palette:
            palette.append(color)
    color_order = np.random.default_rng(s["seed"]).permutation(len(palette))[:n]
    # Similar-color levels share a vivid hue, rather than blending into the table.
    # A seeded permutation keeps small shade differences unrelated to object rank.
    shade_order = np.random.default_rng(s["seed"] + 1).permutation(n)
    similar_base = np.asarray(palette[int(color_order[0])])
    # Place the grid in the robot's horizontal frame so a base move preserves reach.
    forward = settings.OBJECT_FORWARD_OFFSETS
    lateral = settings.OBJECT_LATERAL_OFFSETS
    if n > len(forward) * len(lateral):
        raise ValueError("Object layout has fewer positions than this scenario needs")
    yaw = np.deg2rad(settings.ROBOT_YAW_DEG)
    c, s_yaw = np.cos(yaw), np.sin(yaw)
    for i, obj in enumerate(s["objects"]):
        if obj["category"] not in CASES[s["case"]]["shapes"]:
            raise ValueError("Category outside the case definition")
        info = describe(obj["geometry"])
        color = (
            palette[int(color_order[i])]
            if s["difficulty"]["appearance"] == "distinct"
            else tuple(np.clip(similar_base * (0.95 + 0.05 * shade_order[i] / max(1, n - 1)), 0.0, 1.0))
        )
        row, column = divmod(i, len(lateral))
        position = [
            settings.ROBOT_POSITION[0] + c * forward[row] - s_yaw * lateral[column],
            settings.ROBOT_POSITION[1] + s_yaw * forward[row] + c * lateral[column],
            settings.TABLE_HEIGHT + info["size_m"][2] / 2 + 0.002,
        ]
        specs.append(
            dict(
                obj,
                name=obj["object_id"],
                shape=obj["geometry"]["kind"],
                size_m=info["size_m"],
                radius_m=obj["physical_properties_si"]["radius"],
                position_m=position,
                rotation_xyzw=list(settings.ROBOT_ROTATION),
                color_rgb=list(color),
                mass_kg=0.05,
                grasp_width_m=info["grasp_width_m"],
                grasp_yaw_offset_rad=info["grasp_yaw_offset_rad"],
                grasp_offset_local_m=info["grasp_offset_local_m"],
            )
        )
    return specs
