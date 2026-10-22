# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Export successful native episodes to LeRobot using causal image alignment.

Requires LeRobot with the create/add_frame/save_episode/finalize writer API.
No upload is performed. Depth and full ground truth remain in native HDF5.
"""

import argparse
import json
from pathlib import Path

import h5py
import numpy as np

from data_collection.calibration import camera_pose_wxyz, export_calibration
from data_collection.metadata import read_metadata, read_scene
from data_collection.validate import validate_dataset


def export_dataset(path: Path, output: Path, fps: int = 30) -> int:
    """Export successful episodes at an exact integer decimation of the control rate."""
    validate_dataset(path)
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    writer = None
    sources = []
    try:
        with h5py.File(path, "r") as native:
            for name, episode in native["data"].items():
                if not episode.attrs["success"]:
                    continue
                metadata = read_metadata(episode)
                if metadata.get("benchmark") == "Benchmark" and not metadata.get("accepted_demonstration", False):
                    continue
                ratio = 1 / (metadata["control_dt_s"] * fps)
                if fps < 1 or not np.isclose(ratio, round(ratio)) or ratio < 1:
                    raise ValueError("LeRobot FPS must divide the recorded control rate")
                stride = round(ratio)
                views = metadata["camera_views"]
                if writer is None:
                    joints = episode["actions"].shape[1]
                    features = {
                        "observation.state": {"dtype": "float32", "shape": (2 * joints,), "names": None},
                        "action": {"dtype": "float32", "shape": (joints,), "names": metadata["joint_names"]},
                    }
                    for view in views:
                        shape = episode[f"sensors/{view}/rgb"].shape[1:]
                        features[f"observation.images.{view}"] = {
                            "dtype": "image",
                            "shape": tuple(shape),
                            "names": ["height", "width", "channels"],
                        }
                        features[f"observation.intrinsics.{view}"] = {"dtype": "float32", "shape": (9,), "names": None}
                        features[f"observation.camera_pose.{view}"] = {
                            "dtype": "float32",
                            "shape": (7,),
                            "names": ["x", "y", "z", "qw", "qx", "qy", "qz"],
                        }
                        features[f"observation.camera_timestamp.{view}"] = {
                            "dtype": "float32",
                            "shape": (1,),
                            "names": None,
                        }
                    writer = LeRobotDataset.create(
                        repo_id="local/Benchmark-mou",
                        root=output,
                        fps=fps,
                        robot_type="franka_panda",
                        features=features,
                        use_videos=False,
                        image_writer_threads=4,
                    )
                for i in range(0, len(episode["actions"]), stride):
                    frame = {
                        "task": metadata["prompt"],
                        "observation.state": np.concatenate(
                            (episode["obs/joint_position"][i], episode["obs/joint_velocity"][i])
                        ).astype(np.float32),
                        "action": episode["actions"][i].astype(np.float32),
                    }
                    for view in views:
                        sensor = episode[f"sensors/{view}"]
                        index = int(episode[f"obs/camera_index/{view}"][i])
                        frame[f"observation.images.{view}"] = sensor["rgb"][index]
                        frame[f"observation.intrinsics.{view}"] = (
                            sensor["intrinsics"][index].reshape(-1).astype(np.float32)
                        )
                        frame[f"observation.camera_pose.{view}"] = camera_pose_wxyz(
                            sensor["position"][index], sensor["quaternion_ros_xyzw"][index]
                        )
                        frame[f"observation.camera_timestamp.{view}"] = np.array(
                            [sensor["timestamps"][index]], np.float32
                        )
                    writer.add_frame(frame)
                writer.save_episode()
                sources.append({"source_episode": name, "metadata": metadata, "scene": read_scene(episode)})
    finally:
        if writer is not None:
            writer.finalize()
    if writer is not None:
        (output / "ground_truth.json").write_text(json.dumps(sources, indent=2) + "\n")
        export_calibration(path, output, [item["source_episode"] for item in sources])
    return len(sources)


def main() -> None:
    """Run the dataset tool from the command line."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument(
        "--calibration_only",
        action="store_true",
        help="Write reference-format calibration and poses without simulation or LeRobot",
    )
    args = parser.parse_args()
    if args.calibration_only:
        output = args.output or args.dataset.parent
        count = export_calibration(args.dataset, output)
        print(f"Exported calibration for {count} episodes to {output / 'meta'}")
        return
    if args.output is None:
        parser.error("--output is required for LeRobot export")
    print(f"Exported {export_dataset(args.dataset, args.output, args.fps)} successful episodes")


if __name__ == "__main__":
    main()
