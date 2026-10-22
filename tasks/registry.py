# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Benchmark categories own tasks; MOU is one task, not the benchmark name."""

from importlib import import_module

TASKS = {
    "metric_spatial_understanding/metric_object_understanding": (
        "tasks.metric_spatial_understanding.metric_object_understanding.task"
    )
}


def get_task(name):
    if name not in TASKS:
        raise ValueError(f"Unknown task {name!r}; choose from {', '.join(TASKS)}")
    return import_module(TASKS[name])
