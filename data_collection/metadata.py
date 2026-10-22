# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Compact episode outcomes, shared recording information, and private scene truth.

Human-readable metadata describes the attempt. Geometry and sampled physics live
in HDF5 ``scene_json``; shared recording conventions live in ``recording_json``.
Readers also support the earlier, expanded metadata layout.
"""

import json
from dataclasses import asdict

import yaml


def save_run_config(directory, config, settings, output_dir):
    """Save one resolved configuration, excluding the separately recorded scenario."""
    resolved = asdict(config)
    resolved.pop("scenario", None)
    resolved["output_dir"] = str(output_dir)
    resolved["task_settings"] = {
        key: value for key, value in vars(settings).items() if key.isupper() and not key.startswith("_")
    }
    (directory / "run_config.yaml").write_text(yaml.safe_dump(resolved, sort_keys=False))
    return resolved


def recording_info(settings, joint_names, capture_profile):
    """Describe shared sensor/action contracts once per dataset."""
    return {
        "robot": "franka",
        "joint_names": list(joint_names),
        "control_dt_s": settings.PHYSICS_DT * settings.CONTROL_STEPS,
        "camera_hz": settings.CAMERA_HZ,
        "camera_views": list(settings.CAMERAS),
        "image_size_hw": [settings.IMAGE_HEIGHT, settings.IMAGE_WIDTH],
        "capture_profile": capture_profile,
        "collection_mode": "scripted",
        "identification_source": "oracle_authored_geometry",
        "success_protocol": "mou_pick_lift_hold_proxy",
        "action_schema": "joint_position_target; joint_names order; radians, finger metres",
        "pose_schema": "world XYZ metres; quaternion XYZW",
        "camera_pose_schema": "world XYZ metres; ROS optical quaternion XYZW",
        "depth_schema": "distance_to_image_plane metres; depth_valid mask",
        "transition_schema": "obs=state_t; actions=command_t; states=state_t+1",
        "image_alignment": "obs/camera_index selects latest camera frame at or before action",
    }


def episode_info(task, scenario, seed, target_index):
    """Keep the episode sidecar limited to identity, instruction, and outcome."""
    return {
        "benchmark": "Benchmark",
        "task": task,
        **{key: scenario[key] for key in ("scenario_id", "case", "condition", "level", "split")},
        "prompt": scenario["instruction"],
        "seed": seed,
        "target_id": scenario["target_id"],
        "target_index": target_index,
        "observability_status": scenario["observability_status"],
        "accepted_demonstration": False,
    }


def read_metadata(episode):
    """Read either layout, adding shared recording information for consumers."""
    shared = json.loads(episode.file.attrs.get("recording_json", "{}"))
    return shared | json.loads(episode.attrs["metadata_json"])


def read_scene(episode):
    """Return exact scene truth without adding it to the public episode summary."""
    if "scene_json" in episode.attrs:
        return json.loads(episode.attrs["scene_json"])
    metadata = json.loads(episode.attrs["metadata_json"])
    return {
        "scenario": metadata.get("query", {}).get("scenario"),
        "objects": metadata.get("objects", []),
        "randomization_samples": metadata.get("randomization_samples", {}),
    }


def read_settings(dataset):
    """Use the captured settings, including legacy datasets; never current defaults."""
    if "run_config_json" in dataset.attrs:
        return json.loads(dataset.attrs["run_config_json"])["task_settings"]
    from pathlib import Path

    root = Path(dataset.filename).parent
    legacy = root / "config.json"
    if legacy.exists():
        return json.loads(legacy.read_text())
    return yaml.safe_load((root / "run_config.yaml").read_text())["task_settings"]
