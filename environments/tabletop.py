# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Build a physically tracked Franka/table/ruler scene for data collection."""

from assets.spawners import spawn_cfg
from isaaclab_physx.renderers import IsaacRtxRendererCfg, IsaacRtxRendererGlobalSettingsCfg

import isaaclab.sim as sim
from isaaclab.assets import AssetBaseCfg, RigidObjectCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import CameraCfg
from isaaclab.utils import configclass, replace

from isaaclab_assets import FRANKA_PANDA_HIGH_PD_CFG

from . import settings as config


@configclass
class TabletopSceneCfg(InteractiveSceneCfg):
    """Shared table and robot; individual rulers are added from the task config."""

    ground = AssetBaseCfg(prim_path="/World/Ground", spawn=sim.GroundPlaneCfg())
    light = AssetBaseCfg(prim_path="/World/Light", spawn=sim.DomeLightCfg(intensity=config.DOME_INTENSITY))
    key_light = AssetBaseCfg(
        prim_path="/World/KeyLight",
        spawn=sim.DistantLightCfg(intensity=config.KEY_INTENSITY, angle=5.0),
        init_state=AssetBaseCfg.InitialStateCfg(rot=(0.258819, 0.0, 0.0, 0.965926)),
    )
    table = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        spawn=sim.CuboidCfg(
            size=config.TABLE_SIZE,
            collision_props=sim.CollisionPropertiesCfg(),
            visual_material=sim.PreviewSurfaceCfg(
                diffuse_color=config.TABLE_COLOR, roughness=config.MATERIAL_ROUGHNESS
            ),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, config.TABLE_HEIGHT - config.TABLE_SIZE[2] / 2)),
    )
    robot = replace(FRANKA_PANDA_HIGH_PD_CFG, prim_path="{ENV_REGEX_NS}/Robot")


def scene_cfg(specs: list[dict], record_depth: bool = True, settings=config) -> TabletopSceneCfg:
    """Spawn each ruler as a rigid object with known size and center."""
    cfg = TabletopSceneCfg(num_envs=1, env_spacing=2.0)
    cfg.table.spawn.size = tuple(settings.TABLE_SIZE)
    cfg.table.init_state.pos = (0.0, 0.0, settings.TABLE_HEIGHT - settings.TABLE_SIZE[2] / 2)
    cfg.robot.init_state.pos = tuple(settings.ROBOT_POSITION)
    cfg.robot.init_state.rot = tuple(settings.ROBOT_ROTATION)
    cfg.robot.spawn.variants["Physics"] = "physx"
    width, depth, thickness = settings.TABLE_SIZE
    leg_width = settings.TABLE_LEG_WIDTH
    leg_height = settings.TABLE_HEIGHT - thickness
    if leg_height <= 0 or not 0 < leg_width < min(width, depth):
        raise ValueError("Table legs must have positive height and fit beneath the tabletop")
    for index, (x_sign, y_sign) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1))):
        setattr(
            cfg,
            f"table_leg_{index}",
            AssetBaseCfg(
                prim_path=f"{{ENV_REGEX_NS}}/TableLeg{index}",
                spawn=sim.CuboidCfg(
                    size=(leg_width, leg_width, leg_height),
                    collision_props=sim.CollisionPropertiesCfg(),
                    visual_material=sim.PreviewSurfaceCfg(
                        diffuse_color=settings.TABLE_COLOR, roughness=settings.MATERIAL_ROUGHNESS
                    ),
                ),
                init_state=AssetBaseCfg.InitialStateCfg(
                    pos=(
                        x_sign * (width / 2 - leg_width),
                        y_sign * (depth / 2 - leg_width),
                        leg_height / 2,
                    ),
                ),
            ),
        )
    for view in settings.CAMERAS:
        wrist = "body" in settings.CAMERAS[view]
        setattr(
            cfg,
            f"camera_{view}",
            CameraCfg(
                prim_path="{ENV_REGEX_NS}/Camera_" + view,
                update_period=0,
                update_latest_camera_pose=True,
                width=settings.IMAGE_WIDTH,
                height=settings.IMAGE_HEIGHT,
                data_types=settings.CAMERA_DATA_TYPES if record_depth else ["rgb"],
                renderer_cfg=IsaacRtxRendererCfg(
                    global_settings=IsaacRtxRendererGlobalSettingsCfg(
                        antialiasing_mode=(
                            settings.RENDER_SETTINGS["antialiasing_mode"]
                            if settings.RENDER_SETTINGS["mode"] == "realtime"
                            else None
                        ),
                        enable_dlssg=False,
                        samples_per_pixel=settings.RENDER_SETTINGS["samples_per_pixel"],
                        enable_shadows=settings.RENDER_SETTINGS["shadows"],
                        enable_direct_lighting=True,
                        enable_dl_denoiser=True,
                        enable_global_illumination=False,
                        enable_reflections=False,
                        carb_settings=(
                            {
                                "/rtx/rendermode": "PathTracing",
                                "/rtx/pathtracing/spp": settings.RENDER_SETTINGS["path_tracing_samples"],
                                "/rtx/pathtracing/totalSpp": settings.RENDER_SETTINGS["path_tracing_samples"],
                                "/rtx/pathtracing/optixDenoiser/enabled": True,
                            }
                            if settings.RENDER_SETTINGS["mode"] == "path_traced"
                            else None
                        ),
                    )
                ),
                spawn=sim.PinholeCameraCfg(
                    focal_length=settings.WRIST_CAMERA_FOCAL_LENGTH if wrist else settings.CAMERA_FOCAL_LENGTH,
                    horizontal_aperture=(
                        settings.WRIST_CAMERA_HORIZONTAL_APERTURE if wrist else settings.CAMERA_HORIZONTAL_APERTURE
                    ),
                    clipping_range=tuple(
                        settings.WRIST_CAMERA_CLIPPING_RANGE if wrist else settings.CAMERA_CLIPPING_RANGE
                    ),
                ),
            ),
        )
    for obj in specs:
        setattr(
            cfg,
            obj["name"],
            RigidObjectCfg(
                prim_path="{ENV_REGEX_NS}/" + obj["name"],
                spawn=spawn_cfg(obj, roughness=settings.MATERIAL_ROUGHNESS),
                init_state=RigidObjectCfg.InitialStateCfg(
                    pos=tuple(obj["position_m"]), rot=tuple(obj["rotation_xyzw"])
                ),
            ),
        )
    return cfg
