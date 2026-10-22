# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""State-based grasp-and-lift success checks."""

import numpy as np
from environments import settings as config


def check_success(centres, initial_centres, target: int, hand_position, finger_positions, *, settings=config) -> bool:
    """Require target lift near a closed gripper and undisturbed distractors."""
    raised = centres[target, 2] - initial_centres[target, 2] >= settings.LIFT_HEIGHT
    near_hand = np.linalg.norm(centres[target] - hand_position) < settings.MAX_HAND_DISTANCE
    closed = float(np.sum(finger_positions)) < settings.GRIPPER_CLOSED_WIDTH
    distractors = np.arange(len(centres)) != target
    movement = np.linalg.norm(centres[distractors] - initial_centres[distractors], axis=1)
    return bool(raised and near_hand and closed and np.all(movement < settings.MAX_DISTRACTOR_MOVEMENT))
