# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Validate native episode lengths, physics states, and causal RGB-D alignment."""

import argparse
import json
from pathlib import Path

import h5py
import numpy as np

from data_collection.metadata import read_metadata, read_scene


def validate_dataset(path: Path) -> dict:
    """Raise on malformed data, incomplete episodes, or future camera observations."""
    report = {"episodes": 0, "successes": 0, "transitions": 0}
    with h5py.File(path, "r") as dataset:
        if "data" not in dataset:
            raise ValueError("No episode data")
        for name, episode in dataset["data"].items():
            if not episode.attrs.get("finalized", False):
                raise ValueError(f"{name}: incomplete episode")
            n = int(episode.attrs["num_samples"])
            metadata = read_metadata(episode)
            _validate_metadata(episode, name, metadata)
            if n == 0:
                if episode.attrs["success"]:
                    raise ValueError(f"{name}: empty successful episode")
                report["episodes"] += 1
                continue
            if len(episode["actions"]) != n or len(episode["timestamps"]) != n:
                raise ValueError(f"{name}: inconsistent transition lengths")
            times = episode["timestamps"][:]
            if not np.isfinite(times).all() or times[0] != 0 or np.any(np.diff(times) <= 0):
                raise ValueError(f"{name}: invalid timestamps")
            if not np.allclose(np.diff(times), metadata["control_dt_s"]):
                raise ValueError(f"{name}: unexpected control rate")
            for group in ("obs", "states"):

                def check(key, value):
                    if isinstance(value, h5py.Dataset):
                        if len(value) != n or not np.isfinite(value[:]).all():
                            raise ValueError(f"{name}/{group}/{key}: invalid state data")

                episode[group].visititems(check)
            if len(episode["success"]) != n or not np.isfinite(episode["actions"][:]).all():
                raise ValueError(f"{name}: invalid actions/success")
            if bool(episode["success"][-1]) != bool(episode.attrs["success"]):
                raise ValueError(f"{name}: inconsistent success label")
            for view in metadata["camera_views"]:
                stream = episode[f"sensors/{view}"]
                clock = stream["timestamps"][:]
                indices = episode[f"obs/camera_index/{view}"][:]
                if not len(clock) or not np.isfinite(clock).all() or np.any(np.diff(clock) <= 0):
                    raise ValueError(f"{name}/{view}: invalid camera clock")
                expected = np.searchsorted(clock, times, side="right") - 1
                if np.any(expected < 0) or not np.array_equal(indices, expected):
                    raise ValueError(f"{name}/{view}: invalid causal frame alignment")
                rgb = stream["rgb"]
                if rgb.dtype != np.uint8 or rgb.ndim != 4 or rgb.shape[-1] != 3:
                    raise ValueError(f"{name}/{view}: invalid RGB")
                for key in stream:
                    if len(stream[key]) != len(clock):
                        raise ValueError(f"{name}/{view}/{key}: camera stream lengths differ")
                if "distance_to_image_plane" in stream:
                    depth = stream["distance_to_image_plane"]
                    for i in range(len(clock)):
                        valid = np.isfinite(depth[i]) & (depth[i] > 0)
                        if not np.array_equal(valid, stream["depth_valid"][i]):
                            raise ValueError(f"{name}/{view}: invalid depth mask")
            report["episodes"] += 1
            report["successes"] += int(episode.attrs["success"])
            report["transitions"] += n
    return report


def _validate_metadata(episode, name, metadata):
    """Keep episode labels consistent with immutable task truth and review status."""
    scenario = read_scene(episode)["scenario"]
    if scenario and scenario.get("benchmark") == "Benchmark":
        from tasks.registry import get_task

        get_task(metadata["task"]).validate_recorded_scenario(scenario)
        from evaluation.review import accepted

        if metadata.get("accepted_demonstration", False) != accepted(metadata):
            raise ValueError(f"{name}: inconsistent acceptance label")
        if (
            "scene_json" not in episode.attrs
            and metadata.get("observability_status") != scenario["observability_status"]
        ):
            raise ValueError(f"{name}: inconsistent visibility labels")
        if "scene_json" in episode.attrs:
            for key in ("scenario_id", "case", "condition", "level", "split", "target_id"):
                if metadata[key] != scenario[key]:
                    raise ValueError(f"{name}: metadata {key} disagrees with scene")
            if metadata["target_index"] != scenario["target_index"] or metadata["prompt"] != scenario["instruction"]:
                raise ValueError(f"{name}: target or instruction disagrees with scene")


def main() -> None:
    """Run the dataset tool from the command line."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", type=Path)
    print(json.dumps(validate_dataset(parser.parse_args().dataset), indent=2))


if __name__ == "__main__":
    main()
