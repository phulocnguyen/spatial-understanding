# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Shared Franka grasp-and-lift collection runtime."""

import json
import logging
from datetime import datetime, timezone
from types import SimpleNamespace

import numpy as np

from isaaclab.app import launch_simulation

from data_collection.metadata import episode_info, recording_info, save_run_config


def run(args, run_config, *, config, specs, target, prompt, controller, check_success, scenario):
    """Run the task and return its output directory and collection summary."""
    import torch
    from environments.tabletop import scene_cfg
    from isaaclab_physx.physics import PhysxCfg

    import isaaclab.sim as sim_utils
    from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg
    from isaaclab.utils import instantiate

    from data_collection.randomization import EpisodeRandomizer, sample_episode
    from data_collection.recorder import EpisodeRecorder

    if args.capture_profile == "full" and "distance_to_image_plane" not in config.CAMERA_DATA_TYPES:
        raise ValueError("Full capture requires distance_to_image_plane in configs/settings.yaml")
    # Direct GPU PhysX forcibly disables sleeping. Use CPU physics for this
    # single-scene collector; RTX camera rendering still uses the GPU.
    if config.OBJECT_START_ASLEEP:
        args.device = "cpu"
        print("Objects start asleep: using CPU PhysX with GPU camera rendering.", flush=True)
    sim_cfg = sim_utils.SimulationCfg(dt=config.PHYSICS_DT, device=args.device, physics=PhysxCfg())
    definition = scene_cfg(specs, record_depth=args.capture_profile == "full", settings=config)
    from core.timing import capture_stride

    capture_physics_steps = capture_stride(config.PHYSICS_DT, config.CONTROL_STEPS, config.CAMERA_HZ)
    print(f"Prompt: {prompt}", flush=True)
    run_dir = None
    if not args.preview:
        run_dir = args.output_dir / datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
        run_dir.mkdir(parents=True, exist_ok=False)
        resolved = save_run_config(run_dir, run_config, config, args.output_dir)
        print(f"Dataset: {run_dir}", flush=True)

    with launch_simulation(SimpleNamespace(sim=sim_cfg, scene=definition), args):
        sim = sim_utils.SimulationContext(sim_cfg)
        scene = instantiate(definition)
        _configure_stage(config)
        sim.set_camera_view(config.CAMERA_POSITION, config.CAMERA_TARGET)
        sim.reset()
        robot = scene["robot"]
        from environments.cameras import CameraRig

        camera_rig = CameraRig(scene, robot, sim, config, record_depth=args.capture_profile == "full")
        objects = [scene[obj["name"]] for obj in specs]
        arm_ids, _ = robot.find_joints("panda_joint[1-7]")
        randomizer = EpisodeRandomizer(run_config.randomization, objects, camera_rig.cameras, config.CAMERAS)
        finger_ids, _ = robot.find_joints("panda_finger_joint.*")
        hand_ids, _ = robot.find_bodies("panda_hand")
        hand_id = hand_ids[0]
        jacobian_id = hand_id - 1 if robot.is_fixed_base else hand_id
        jacobian_columns = [j + robot.num_base_dofs for j in arm_ids]
        ik = DifferentialIKController(
            DifferentialIKControllerCfg(command_type="pose", use_relative_mode=False, ik_method=config.IK_METHOD),
            num_envs=1,
            device=sim.device,
        )

        def observe():
            poses = torch.stack([obj.data.root_pose_w.torch[0] for obj in objects])
            from assets.geometry import world_aabb

            poses_np = poses.cpu().numpy().copy()
            bounds = [world_aabb(obj["geometry"], pose[:3], pose[3:]) for obj, pose in zip(specs, poses_np)]
            return {
                "robot_root_pose_xyzw": robot.data.root_pose_w.torch[0].cpu().numpy().copy(),
                "joint_position": robot.data.joint_pos.torch[0].cpu().numpy().copy(),
                "joint_velocity": robot.data.joint_vel.torch[0].cpu().numpy().copy(),
                "object_pose_xyzw": poses.cpu().numpy().copy(),
                "object_velocity": torch.stack([obj.data.root_vel_w.torch[0] for obj in objects]).cpu().numpy().copy(),
                "object_aabb_min": np.asarray([v[0] for v in bounds]),
                "object_aabb_max": np.asarray([v[1] for v in bounds]),
                "hand_pose_xyzw": robot.data.body_pose_w.torch[0, hand_id].cpu().numpy().copy(),
            }

        def step():
            scene.write_data_to_sim()
            sim.step(render=args.preview)
            scene.update(config.PHYSICS_DT)
            camera_rig.sync()

        summary = {"attempts": 0, "successes": 0, "failures": 0, "completed": False}
        attempt_limit = args.max_attempts if args.success_target is not None else min(args.episodes, args.max_attempts)
        for episode in range(attempt_limit):
            episode_seed = args.seed + episode
            episode_specs = sample_episode(
                specs,
                run_config.randomization,
                seed=episode_seed,
                table_size=config.TABLE_SIZE,
                colors=config.COLORS,
            )
            for i, obj in enumerate(objects):
                pose = obj.data.default_root_pose.torch.clone()
                pose[:, :3] = torch.tensor(episode_specs[i]["position_m"], device=sim.device)
                if config.OBJECT_START_ASLEEP:
                    pose[:, 2] = config.TABLE_HEIGHT + episode_specs[i]["size_m"][2] / 2
                pose[:, 3:] = torch.tensor(episode_specs[i]["rotation_xyzw"], device=sim.device)
                obj.write_root_pose_to_sim_index(root_pose=pose)
                obj.write_root_velocity_to_sim_index(root_velocity=obj.data.default_root_vel.torch)
            positions, velocities, randomization_samples = randomizer.apply(episode_specs, robot, arm_ids, episode_seed)
            robot.write_joint_position_to_sim_index(position=positions)
            robot.write_joint_velocity_to_sim_index(velocity=velocities)
            robot.set_joint_position_target_index(target=positions)
            scene.reset()
            ik.reset()
            # Start supported objects asleep. PhysX wakes dynamic bodies on contact;
            # this is an initialization constraint, not a kinematic grasp override.
            import omni.usd
            from omni.physx import get_physx_simulation_interface
            from pxr import PhysicsSchemaTools, Sdf

            physics_interface = get_physx_simulation_interface()
            stage_id = omni.usd.get_context().get_stage_id()
            scene.write_data_to_sim()
            if config.OBJECT_START_ASLEEP:
                for spec in episode_specs:
                    path = Sdf.Path("/World/envs/env_0/" + spec["name"])
                    physics_interface.put_to_sleep(stage_id, PhysicsSchemaTools.sdfPathToInt(path))
            settling_seconds = _settle_objects(sim, step, objects, episode_specs, config)
            if settling_seconds is None:
                return None
            print(f"Objects stable after {settling_seconds:.2f}s", flush=True)
            if args.preview:
                while sim.is_running():
                    step()
                return None

            # Warm up rendering after reset; the first recorded action must have a valid image.
            for _ in range(4):
                sim.render()
            before = observe()
            initial_centres = before["object_pose_xyzw"][:, :3].copy()
            from assets.geometry import rotate_vector

            target_pose = before["object_pose_xyzw"][target]
            grasp_centre = initial_centres[target] + rotate_vector(
                specs[target].get("grasp_offset_local_m", [0.0, 0.0, 0.0]), target_pose[3:]
            )
            motion = controller.GraspLiftController(
                grasp_centre, specs[target].get("grasp_width_m", specs[target]["size_m"][1]), settings=config
            )
            target_quat = before["object_pose_xyzw"][target, 3:]
            yaw = np.arctan2(
                2 * (target_quat[3] * target_quat[2] + target_quat[0] * target_quat[1]),
                1 - 2 * (target_quat[1] ** 2 + target_quat[2] ** 2),
            )
            yaw += specs[target].get("grasp_yaw_offset_rad", 0.0)
            # Choose the equivalent grasp closest to the robot's forward axis.
            base_yaw = np.deg2rad(config.ROBOT_YAW_DEG)
            yaw = base_yaw + (yaw - base_yaw + np.pi / 2) % np.pi - np.pi / 2
            motion.rotation = np.array([np.cos(yaw / 2), np.sin(yaw / 2), 0.0, 0.0])
            metadata = episode_info(run_config.task, scenario, episode_seed, target)
            scene_record = {
                "scenario": scenario,
                "objects": [
                    dict(
                        obj,
                        position_m=initial_centres[i].tolist(),
                        rotation_xyzw=before["object_pose_xyzw"][i, 3:].tolist(),
                    )
                    for i, obj in enumerate(episode_specs)
                ],
                "randomization_samples": randomization_samples,
                "settling_seconds": settling_seconds,
                "object_initialization": (
                    "supported_pose_asleep_until_contact" if config.OBJECT_START_ASLEEP else "free_settling"
                ),
            }
            recorder = EpisodeRecorder(
                run_dir / f"episode_{episode:04d}",
                metadata,
                run_dir / "dataset.hdf5",
                scene=scene_record,
                recording=recording_info(config, robot.joint_names, args.capture_profile),
                run_config=resolved,
            )
            pending_images = {}
            physics_tick = 0
            peak_lifts = np.zeros(len(specs))
            success, hold = False, 0.0
            reason = "timeout"
            dt = config.PHYSICS_DT * config.CONTROL_STEPS
            try:
                for tick in range(round(config.TIMEOUT_SECONDS / dt)):
                    if not sim.is_running():
                        reason = "viewer_closed"
                        break
                    goal, finger, phase = motion.command(
                        before["hand_pose_xyzw"],
                        before["joint_position"][finger_ids],
                        before["joint_velocity"][finger_ids],
                    )
                    goal_w = torch.tensor(goal, dtype=torch.float32, device=sim.device).unsqueeze(0)
                    # Keep goal, hand pose, and link Jacobian in the same world frame.
                    # Mixing root-frame errors with this Jacobian fails for a rotated base.
                    ik.set_command(goal_w)
                    hand = robot.data.body_pose_w.torch[:, hand_id]
                    jacobian = robot.data.body_link_jacobian_w.torch[:, jacobian_id, :, jacobian_columns]
                    desired = ik.compute(hand[:, :3], hand[:, 3:], jacobian, robot.data.joint_pos.torch[:, arm_ids])
                    limits = robot.data.soft_joint_pos_limits.torch[:, arm_ids]
                    desired = torch.maximum(torch.minimum(desired, limits[..., 1]), limits[..., 0])
                    positions[:, arm_ids] = desired
                    positions[:, finger_ids] = finger
                    images = camera_rig.capture(0.0) if tick == 0 else pending_images
                    pending_images = {}
                    recorder.before(
                        action=positions[0].cpu().numpy(),
                        phase=phase,
                        time_s=tick * dt,
                        observation=dict(before, hand_goal_xyzw=goal),
                        cameras=images,
                    )
                    if tick % 300 == 0:
                        print(f"Episode {episode}: {tick * dt:.1f}s simulated, phase={phase}", flush=True)
                    robot.set_joint_position_target_index(target=positions)
                    for _ in range(config.CONTROL_STEPS):
                        step()
                        physics_tick += 1
                        if physics_tick % capture_physics_steps == 0:
                            pending_images = camera_rig.capture(physics_tick * config.PHYSICS_DT)
                    after = observe()
                    if not all(np.isfinite(value).all() for value in after.values()):
                        reason = "nonfinite_state"
                        break
                    peak_lifts = np.maximum(peak_lifts, after["object_pose_xyzw"][:, 2] - initial_centres[:, 2])
                    condition = check_success(
                        after["object_pose_xyzw"][:, :3],
                        initial_centres,
                        target,
                        after["hand_pose_xyzw"][:3],
                        after["joint_position"][finger_ids],
                        settings=config,
                    )
                    hold = hold + dt if condition else 0.0
                    success = hold >= config.SUCCESS_HOLD_SECONDS
                    recorder.after(after, success)
                    before = after
                    if success:
                        reason = "success"
                        break
            except KeyboardInterrupt:
                reason = "interrupted"
            except Exception:
                reason = "error"
                success = False
                raise
            finally:
                selected = int(np.argmax(peak_lifts)) if np.max(peak_lifts) > 0.02 else None
                recorder.metadata["observed_selected_object_id"] = (
                    specs[selected]["object_id"] if selected is not None else None
                )
                recorder.scene["peak_object_lifts_m"] = peak_lifts.tolist()
                recorder.metadata["accepted_demonstration"] = bool(
                    success and recorder.metadata["observability_status"] == "verified"
                )
                recorder.finish(success, reason)
                if run_config.video:
                    from data_collection.video import export_episode_videos

                    try:
                        export_episode_videos(run_dir / "dataset.hdf5", recorder.directory.name, run_dir / "videos")
                    except (OSError, ValueError, RuntimeError) as error:
                        logging.getLogger(__name__).warning("Video export failed; native episode is saved: %s", error)
            summary["attempts"] += 1
            summary["successes"] += int(success)
            summary["failures"] += int(not success)
            summary["completed"] = (
                summary["successes"] >= args.success_target
                if args.success_target is not None
                else summary["attempts"] >= args.episodes
            )
            (run_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
            print(f"Episode {episode}: {reason}; saved {recorder.directory}", flush=True)
            if summary["completed"]:
                break
            if reason in ("viewer_closed", "interrupted"):
                break

        if run_dir is not None:
            from data_collection.finalize import finalize_run

            finalize_run(run_dir, summary, run_config)

    return run_dir, summary


def _settle_objects(sim, step, objects, specs, config):
    """Wait for supported objects to remain still, or report physical reset failure."""
    import torch

    stable_steps = 0
    required_stable_steps = round(config.SETTLE_HOLD_SECONDS / config.PHYSICS_DT)
    for settle_step in range(round(config.SETTLE_MAX_SECONDS / config.PHYSICS_DT)):
        if not sim.is_running():
            return None
        step()
        object_velocities = torch.stack([obj.data.root_vel_w.torch[0] for obj in objects])
        stable = (
            torch.isfinite(object_velocities).all()
            and (torch.linalg.vector_norm(object_velocities[:, :3], dim=-1) < config.SETTLE_LINEAR_SPEED).all()
            and (torch.linalg.vector_norm(object_velocities[:, 3:], dim=-1) < config.SETTLE_ANGULAR_SPEED).all()
        )
        stable_steps = stable_steps + 1 if stable else 0
        if (settle_step + 1) * config.PHYSICS_DT >= config.SETTLE_SECONDS and stable_steps >= required_stable_steps:
            break
    else:
        diagnostics = []
        for spec, velocity in zip(specs, object_velocities.cpu().tolist()):
            diagnostics.append(
                {
                    "object": spec.get("name", spec.get("object_id", "unknown")),
                    "linear_speed_m_s": float(np.linalg.norm(velocity[:3])),
                    "angular_speed_rad_s": float(np.linalg.norm(velocity[3:])),
                    "velocity": velocity,
                }
            )
        print("Settling diagnostics: " + json.dumps(diagnostics), flush=True)
        raise RuntimeError(
            "Objects did not settle before collection. Check support geometry, "
            "reset collisions, and centre-of-mass randomization. No episode was recorded."
        )
    return (settle_step + 1) * config.PHYSICS_DT


def _configure_stage(config):
    """Enable object sleeping and apply cast-shadow settings to the spawned lights."""
    from pxr import PhysxSchema, UsdLux

    from isaaclab.sim.utils.stage import get_current_stage

    for prim in get_current_stage().Traverse():
        if prim.HasAPI(PhysxSchema.PhysxSceneAPI):
            PhysxSchema.PhysxSceneAPI(prim).CreateDisableSleepingAttr().Set(False)
        if prim.HasAPI(UsdLux.LightAPI):
            # RTX path tracing uses shadow linking, not the realtime shadow
            # switch or UsdLux.ShadowAPI. An empty collection casts no shadows.
            shadow_link = UsdLux.LightAPI(prim).GetShadowLinkCollectionAPI()
            shadow_link.CreateIncludeRootAttr().Set(config.RENDER_SETTINGS["shadows"])
