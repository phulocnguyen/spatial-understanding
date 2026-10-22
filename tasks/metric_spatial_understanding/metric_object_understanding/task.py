# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Single MOU adapter: geometry truth and unique-target collection."""

import json
from dataclasses import dataclass, field
from pathlib import Path
from types import SimpleNamespace

from controllers import grasp_lift
from data_collection.config import CollectionConfig, apply_task_settings
from environments import settings as defaults

from tasks.common.success import check_success

from .catalog import AUTONOMOUS_EXCLUSIONS, CASES, CONDITIONS, REGISTRY_ID
from .scenarios import generate, object_specs, validate


@dataclass
class MOUConfig(CollectionConfig):
    task: str = REGISTRY_ID
    case: str = "length"
    condition: str = "shape_conditioned"
    level: str = "L3"
    requested_shape: str | None = None
    measurements: dict = field(default_factory=dict)
    split: str = "train"
    scenario_file: str | None = None
    rollout_seed_offset: int = 0
    num_episodes: int = 1
    max_attempts: int = 1
    output_dir: str = "datasets/metric_spatial_understanding/mou"
    scenario: dict = field(default_factory=dict, init=False)


CONFIG_TYPE = MOUConfig
CLI_FIELDS = ("case", "condition", "level", "requested_shape", "split", "scenario_file")


def add_cli_args(parser):
    parser.add_argument("--case", choices=list(CASES))
    parser.add_argument("--condition", choices=CONDITIONS)
    parser.add_argument("--level", choices=["L1", "L2", "L3", "L4", "L5"])
    parser.add_argument("--requested_shape")
    parser.add_argument("--split", choices=["train", "eval", "diagnostic"])
    parser.add_argument("--scenario_file")


def resolve_settings(cfg):
    settings = SimpleNamespace(
        **{key: value for key, value in vars(defaults).items() if key.isupper() and not key.startswith("_")}
    )
    apply_task_settings(cfg.task_settings, settings)
    defaults.validate_settings(settings)
    return settings


def configure(cfg):
    if cfg.num_episodes != 1 or cfg.success_target is not None:
        raise ValueError("Generate one geometry scene per process; use collect_manifest.py for batches")
    if cfg.task_prompt is not None:
        raise ValueError("Instructions are generated from the query; task_prompt overrides are not supported")
    if (
        isinstance(cfg.rollout_seed_offset, bool)
        or not isinstance(cfg.rollout_seed_offset, int)
        or cfg.rollout_seed_offset < 0
    ):
        raise ValueError("rollout_seed_offset must be a nonnegative integer")
    cfg.scenario = (
        json.loads(Path(cfg.scenario_file).read_text())
        if cfg.scenario_file
        else generate(
            cfg.case,
            cfg.condition,
            cfg.level,
            cfg.seed,
            requested_shape=cfg.requested_shape,
            measurements=cfg.measurements,
            split=cfg.split,
        )
    )
    validate(cfg.scenario)
    if cfg.requested_shape is not None and cfg.requested_shape != cfg.scenario["reference_shape"]:
        raise ValueError("Scenario shape disagrees with configuration")
    if cfg.measurements:
        from .scenarios import query_for

        if query_for(cfg.case, cfg.measurements) != cfg.scenario["query"]:
            raise ValueError("Scenario query disagrees with configuration")
    if (
        cfg.scenario["case"],
        cfg.scenario["condition"],
        cfg.scenario["level"],
        cfg.scenario["seed"],
        cfg.scenario["split"],
    ) != (cfg.case, cfg.condition, cfg.level, cfg.seed, cfg.split):
        raise ValueError("Scenario provenance disagrees with run configuration")
    if cfg.scenario["observability_status"] == "rejected":
        raise ValueError("Visibility-rejected scenario")
    settings = resolve_settings(cfg)
    specs = object_specs(cfg.scenario, settings)
    from assets.geometry import describe

    for obj in specs:
        if obj["geometry"]["kind"] in AUTONOMOUS_EXCLUSIONS:
            raise ValueError(f"{obj['name']}: {AUTONOMOUS_EXCLUSIONS[obj['geometry']['kind']]}")
        if not describe(obj["geometry"])["grasp_feasible"]:
            raise ValueError(f"{obj['name']} violates object size limits or the supported Panda grasp envelope")


def run(args, cfg):
    from copy import copy

    from data_collection.simulation import run as collect

    args = copy(args)
    args.seed += cfg.rollout_seed_offset
    s = cfg.scenario
    settings = resolve_settings(cfg)
    specs = object_specs(s, settings)
    return collect(
        args,
        cfg,
        config=settings,
        specs=specs,
        target=s["target_index"],
        prompt=s["instruction"],
        controller=grasp_lift,
        check_success=check_success,
        scenario=s,
    )


def validate_recorded_scenario(scenario):
    """Registry hook used by native validation and task-independent collection."""
    return validate(scenario)


def collection_settings(scenario):
    """Registry hook translating a task-owned scene into collector inputs."""
    return dict(
        task=REGISTRY_ID,
        case=scenario["case"],
        condition=scenario["condition"],
        level=scenario["level"],
        seed=scenario["seed"],
        split=scenario["split"],
    )
