# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Finalize dataset checks before the simulator launcher closes its process."""

import json

from data_collection.calibration import export_calibration
from data_collection.validate import validate_dataset


def finalize_run(run_dir, summary, cfg):
    report = validate_dataset(run_dir / "dataset.hdf5")
    (run_dir / "validation.json").write_text(json.dumps(report, indent=2) + "\n")
    export_calibration(run_dir / "dataset.hdf5", run_dir, pose_streams=False)
    if cfg.export_lerobot:
        from data_collection.export_lerobot import export_dataset

        export_dataset(run_dir / "dataset.hdf5", run_dir / "lerobot", cfg.lerobot_hz)
    print(f"Collection summary: {summary}", flush=True)
    if not summary["completed"]:
        raise SystemExit("Collection stopped before the requested count; attempts retained")
