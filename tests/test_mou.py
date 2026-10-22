# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Geometry, task controls, policy scoring, and native recording contracts without GPU."""

import json
import math
import sys
import unittest
from copy import deepcopy
from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from assets.geometry import describe, make, polygon_area, world_aabb
from core.units import to_si
from evaluation.mou import score, summarize
from tasks.metric_spatial_understanding.metric_object_understanding.catalog import CASES, CONDITIONS
from tasks.metric_spatial_understanding.metric_object_understanding.manifest import create, validate_manifest
from tasks.metric_spatial_understanding.metric_object_understanding.scenarios import (
    InfeasibleScenario,
    generate,
    query_for,
    validate,
)


class GeometryContracts(unittest.TestCase):
    def test_primitive_measurements_are_physical_not_image_dimensions(self):
        examples = [
            ({"kind": "box", "size_m": [0.1, 0.04, 0.03]}, 0.004, 0.00012),
            ({"kind": "cylinder", "radius_m": 0.02, "height_m": 0.08}, math.pi * 0.0004, math.pi * 0.000032),
            ({"kind": "sphere", "radius_m": 0.02}, math.pi * 0.0004, 4 * math.pi * 0.000008 / 3),
            ({"kind": "cone", "radius_m": 0.02, "height_m": 0.09}, math.pi * 0.0004, math.pi * 0.0004 * 0.09 / 3),
        ]
        for g, area, volume in examples:
            with self.subTest(kind=g["kind"]):
                info = describe(g)
                self.assertAlmostEqual(info["measurements"]["footprint_area"], area)
                self.assertAlmostEqual(info["measurements"]["volume"], volume)
                self.assertIsNone(info["measurements"]["capacity"])

    def test_upright_round_grasp_and_rotated_bounds(self):
        g = make("cylinder", "height", 0.1)
        info = describe(g)
        self.assertAlmostEqual(info["measurements"]["height"], 0.1)
        self.assertEqual(g["axis"], "Z")
        self.assertAlmostEqual(info["size_m"][2], 0.1)
        self.assertAlmostEqual(info["grasp_width_m"], 0.06)
        self.assertAlmostEqual(info["grasp_offset_local_m"][2], 0.03)
        disk = describe({"kind": "cylinder", "radius_m": 0.02, "height_m": 0.02})
        self.assertEqual(disk["grasp_offset_local_m"], [0.0, 0.0, 0.0])
        low, high = world_aabb(g, [0, 0, 0], [0, 0, math.sqrt(0.5), math.sqrt(0.5)])
        np.testing.assert_allclose(high, [0.03, 0.03, 0.05], atol=1e-8)
        np.testing.assert_allclose(low, -high)
        cone = describe({"kind": "cone", "radius_m": 0.03, "height_m": 0.12})
        self.assertAlmostEqual(cone["grasp_offset_local_m"][2], 0.03)
        self.assertAlmostEqual(cone["grasp_width_m"], 0.06)

    def test_capacity_uses_the_represented_cavity_and_fixed_exteriors(self):
        a = make("cylindrical_container", "capacity", 0.0002)
        b = make("cylindrical_container", "capacity", 0.00024)
        self.assertEqual(a["outer_radius_m"], b["outer_radius_m"])
        self.assertEqual(a["height_m"], b["height_m"])
        self.assertAlmostEqual(describe(a)["measurements"]["capacity"], 0.0002)
        self.assertAlmostEqual(describe(b)["measurements"]["capacity"], 0.00024)
        # A regular inscribed polygon cavity is strictly smaller than an ideal circle.
        self.assertLess(polygon_area(a["inner_radius_m"]), math.pi * a["inner_radius_m"] ** 2)
        self.assertNotAlmostEqual(describe(a)["measurements"]["capacity"], describe(a)["measurements"]["volume"])
        box = make("rectangular_container", "capacity", 0.0002)
        self.assertAlmostEqual(describe(box)["measurements"]["capacity"], 0.0002)

    def test_nested_dimensions_are_actual_inner_and_outer_geometry(self):
        g = make("rectangular_container", "inner_width", 0.06)
        info = describe(g)
        self.assertAlmostEqual(info["measurements"]["inner_width"], 0.06)
        self.assertAlmostEqual(info["measurements"]["outer_width"], 0.10)
        self.assertAlmostEqual(info["measurements"]["capacity"], 0.05 * 0.06 * (0.11 - 0.008))
        invalid = deepcopy(g)
        invalid["inner_size_xy_m"][1] = 0.11
        with self.assertRaisesRegex(ValueError, "Cavity"):
            describe(invalid)

    def test_component_size_cannot_be_inferred_from_overall_length(self):
        a = describe(make("handled_tool", "handle_length", 0.06))
        b = describe(make("handled_tool", "handle_length", 0.08))
        self.assertAlmostEqual(a["measurements"]["length"], b["measurements"]["length"])
        self.assertNotEqual(a["components"][0]["dimensions_m"], b["components"][0]["dimensions_m"])
        self.assertEqual(a["components"][0]["semantic_label"], "handle")
        self.assertAlmostEqual(a["measurements"]["handle_length"], 0.06)
        self.assertLess(a["grasp_offset_local_m"][0], 0)
        self.assertNotEqual(a["components"][0]["position_local_m"], a["grasp_offset_local_m"])
        lo, hi = world_aabb(make("handled_tool", "handle_length", 0.06), [0, 0, 0], [0, 0, 0, 1])
        self.assertAlmostEqual(lo[0], -0.10)
        self.assertAlmostEqual(hi[0], 0.10)
        paddle = make("handled_paddle", "handle_length", 0.09)
        detail = describe(paddle)
        self.assertLess(paddle["head_size_m"][2], paddle["head_size_m"][1] / 2)
        self.assertEqual(detail["components"][1]["geometry"]["kind"], "elliptic_disk")
        # A represented elliptical head has less volume than its bounding box.
        self.assertLess(detail["measurements"]["volume"], 0.09 * 0.03**2 + 0.11 * 0.085 * 0.03)
        low, high = world_aabb(paddle, [0, 0, 0], [0, 0, 0, 1])
        np.testing.assert_allclose(high - low, [0.20, 0.085, 0.03])
        paddle["head_kind"] = "unknown"
        with self.assertRaisesRegex(ValueError, "head geometry"):
            describe(paddle)

    def test_units_are_checked_and_excluded_queries_are_rejected(self):
        self.assertAlmostEqual(to_si(200, "cm3", "capacity"), 0.0002)
        self.assertAlmostEqual(to_si(30, "cm2", "footprint_area"), 0.003)
        with self.assertRaises(ValueError):
            to_si(10, "cm", "volume")
        with self.assertRaises(ValueError):
            query_for("width", {"depth": {"value": 6, "unit": "cm"}})
        with self.assertRaises(ValueError):
            describe({"kind": "box", "size_m": [0.1, -0.1, 0.1]})


class ScenarioContracts(unittest.TestCase):
    def test_every_case_and_condition_has_correct_cardinality_and_control_design(self):
        for case in CASES:
            for condition in CONDITIONS:
                with self.subTest(case=case, condition=condition):
                    s = generate(case, condition, "L5", 712)
                    self.assertEqual(validate(s)["targets"], 1)
                    self.assertEqual(sum(o["target_match"] for o in s["objects"]), 1)
                    if condition == "shape_conditioned":
                        self.assertEqual(
                            {(o["shape_match"], o["measurement_match"]) for o in s["objects"]},
                            {(True, True), (True, False), (False, True), (False, False)},
                        )
                    if condition == "cross_shape":
                        self.assertGreater(len({o["category"] for o in s["objects"]}), 1)
                        self.assertEqual(sum(o["measurement_match"] for o in s["objects"]), 1)

    def test_target_ids_do_not_encode_the_answer(self):
        ids = {generate("length", "shape_conditioned", "L5", seed)["target_id"] for seed in range(20)}
        self.assertGreater(len(ids), 2)

    def test_multi_attribute_distractors_require_every_attribute(self):
        for case in ("multi_dimension", "nested_dimension"):
            s = generate(case, "shape_conditioned", "L5", 712)
            self.assertTrue(all(s["constraint_necessity"].values()))

    def test_gaps_and_effective_counts_are_honest(self):
        validate(generate("radius", "shape_conditioned", "L1", 712))
        s = generate("length", "shape_conditioned", "L1", 712)
        self.assertEqual(s["difficulty"]["objects"], 3)
        self.assertEqual(len(s["objects"]), 4)
        self.assertTrue(all(g["actual_si"] >= g["minimum_si"] for g in s["separation"]))
        self.assertEqual(s["observability_status"], "unverified")
        with self.assertRaisesRegex(InfeasibleScenario, "target exceeds"):
            generate(
                "volume",
                "same_shape",
                "L3",
                712,
                requested_shape="box",
                measurements={"volume": {"value": 1000, "unit": "cm3"}},
            )
        with self.assertRaisesRegex(InfeasibleScenario, "excluded from autonomous"):
            generate("height", "same_shape", "L3", 712, requested_shape="cone")
        for value in (1, 30):
            with self.subTest(length_cm=value), self.assertRaisesRegex(InfeasibleScenario, "target exceeds"):
                generate(
                    "length",
                    "same_shape",
                    "L5",
                    712,
                    measurements={"length": {"value": value, "unit": "cm"}},
                )

    def test_same_scene_colors_do_not_encode_shape_or_target(self):
        from environments import settings
        from tasks.metric_spatial_understanding.metric_object_understanding.scenarios import object_specs

        s = generate("nested_dimension", "shape_conditioned", "L1", 712)
        colors = [tuple(o["color_rgb"]) for o in object_specs(s, settings)]
        self.assertLessEqual(float(np.ptp(np.asarray(colors), axis=0).max()), 0.05)

    def test_tampered_measurements_or_targets_are_rejected(self):
        s = generate("capacity", "shape_conditioned", "L5", 712)
        broken = deepcopy(s)
        broken["objects"][0]["physical_properties_si"]["capacity"] = 999
        with self.assertRaises(ValueError):
            validate(broken)
        broken = deepcopy(s)
        broken["target_id"] = "not_an_object"
        with self.assertRaises(ValueError):
            validate(broken)

    def test_v1_profile_balances_joint_cells_and_target_ranks(self):
        from collections import Counter

        import yaml

        profile = yaml.safe_load((Path(__file__).resolve().parents[1] / "configs/mou_pilot.yaml").read_text())
        manifest = create(18, 712, profile=profile)
        self.assertFalse(manifest["infeasible_cells"])
        cells = Counter((e["scenario"]["case"], e["scenario"]["level"]) for e in manifest["episodes"])
        self.assertEqual(len(cells), 9)
        self.assertEqual(set(cells.values()), {2})
        for case, level in cells:
            queries = {
                tuple(q["value_si"] for q in e["scenario"]["query"].values())
                for e in manifest["episodes"]
                if (e["scenario"]["case"], e["scenario"]["level"]) == (case, level)
            }
            self.assertEqual(len(queries), 2)
        ranks = Counter(
            (e["scenario"]["case"], next(iter(e["scenario"]["target_measurement_ranks"].values())))
            for e in manifest["episodes"]
        )
        self.assertEqual(len(ranks), 9)
        self.assertEqual(set(ranks.values()), {2})
        for entry in manifest["episodes"]:
            s = entry["scenario"]
            self.assertNotIn("matched_controls", s)
            self.assertEqual(len(s["objects"]), 3)
            self.assertTrue(all(describe(o["geometry"])["grasp_width_m"] <= 0.065 for o in s["objects"]))

    def test_diverse_profile_covers_every_shape_and_query_per_cell(self):
        import yaml

        profile = yaml.safe_load((Path(__file__).resolve().parents[1] / "configs/mou_diverse216.yaml").read_text())
        manifest = create(216, 712, profile=profile)
        self.assertFalse(manifest["infeasible_cells"])
        for case, spec in profile["cases"].items():
            for condition in profile["conditions"]:
                for level in profile["levels"]:
                    scenes = [
                        e["scenario"]
                        for e in manifest["episodes"]
                        if (e["scenario"]["case"], e["scenario"]["condition"], e["scenario"]["level"])
                        == (case, condition, level)
                    ]
                    actual = {(s["reference_shape"], tuple(q["value"] for q in s["query"].values())) for s in scenes}
                    expected = {
                        (shape, tuple(q["value"] for q in query.values()))
                        for shape in spec["shapes"]
                        for query in spec["queries"]
                    }
                    self.assertEqual(actual, expected)
                    self.assertTrue(all(o["category"] in spec["shapes"] for s in scenes for o in s["objects"]))

    def test_attribute_files_generate_all_requested_shapes_and_values_safely(self):
        from tasks.metric_spatial_understanding.metric_object_understanding.manifest import load_profile

        profile = load_profile(Path(__file__).resolve().parents[1] / "configs/mou.yaml")
        manifest = create(profile["count"], profile["seed"], profile=profile)
        self.assertFalse(manifest["infeasible_cells"])
        self.assertEqual(set(validate_manifest(manifest)["by_case"]), set(profile["cases"]))
        for name, spec in profile["cases"].items():
            for mode in profile["conditions"]:
                for level in profile["levels"]:
                    scenes = [
                        e["scenario"]
                        for e in manifest["episodes"]
                        if (e["scenario"]["case"], e["scenario"]["condition"], e["scenario"]["level"])
                        == (name, mode, level)
                    ]
                    pairs = {(s["reference_shape"], tuple(v["value"] for v in s["query"].values())) for s in scenes}
                    expected = {
                        (shape, tuple(v["value"] for v in query.values()))
                        for shape in spec["shapes"]
                        for query in spec["queries"]
                    }
                    self.assertEqual(pairs, expected)
                    self.assertTrue(
                        all(describe(o["geometry"])["grasp_width_m"] <= 0.065 for s in scenes for o in s["objects"])
                    )

    def test_manifest_balance_reproducibility_and_split_control(self):
        # Complete condition/difficulty cycles expose scale-dependent infeasible cells.
        m = create(300, 712)
        self.assertEqual(m, create(300, 712))
        report = validate_manifest(m)
        self.assertEqual(len(report["by_case"]), 11)
        capacity = {
            e["scenario"]["query"]["capacity"]["value"] for e in m["episodes"] if e["scenario"]["case"] == "capacity"
        }
        self.assertEqual(capacity, {250, 300, 350})
        self.assertEqual(len(report["by_condition"]), 3)
        self.assertEqual(report["by_level"], {f"L{i}": 60 for i in range(1, 6)})
        for level in report["by_level"]:
            splits = {e["scenario"]["split"] for e in m["episodes"] if e["scenario"]["level"] == level}
            self.assertEqual(splits, {"train", "eval"})
        for case in CASES:
            queries = {
                tuple(q["value_si"] for q in entry["scenario"]["query"].values())
                for entry in m["episodes"]
                if entry["scenario"]["case"] == case
            }
            self.assertGreaterEqual(len(queries), 2, f"{case}: target-value variation collapsed")
        from data_collection.manifest import load_manifest, write_manifest

        with TemporaryDirectory() as folder:
            write_manifest(m, folder, report)
            restored = load_manifest(Path(folder) / "plan.yaml")
            self.assertEqual([e["scenario"] for e in restored["episodes"]], [e["scenario"] for e in m["episodes"]])
            self.assertEqual(validate_manifest(restored), report)
            plan = Path(folder) / "plan.yaml"
            plan.write_text(plan.read_text().replace("case: length", "case: width", 1))
            with self.assertRaisesRegex(ValueError, "case differs"):
                load_manifest(plan)
        duplicate = deepcopy(m["episodes"][0])
        duplicate["slot"] = len(m["episodes"])
        duplicate["scenario"]["split"] = "eval" if duplicate["scenario"]["split"] == "train" else "train"
        m["episodes"].append(duplicate)
        with self.assertRaisesRegex(ValueError, "leaks"):
            validate_manifest(m)


class ResetRandomizationContracts(unittest.TestCase):
    def test_table_colors_are_seeded_validated_and_respect_switches(self):
        from data_collection.randomization import resolve_randomization, sample_table_style

        cfg = {
            "enabled": True,
            "table": {
                "enabled": True,
                "options": [
                    {"name": "wood", "color_rgb": [0.3, 0.2, 0.1], "source_texture": "wood.png"},
                    {"name": "marble", "color_rgb": [0.6, 0.6, 0.6], "source_texture": "marble.png"},
                ],
            },
        }
        draws = [sample_table_style(cfg, seed) for seed in range(30)]
        self.assertEqual({draw["name"] for draw in draws}, {"wood", "marble"})
        self.assertEqual(draws, [sample_table_style(cfg, seed) for seed in range(30)])
        cfg["light"] = {"enabled": True, "intensity_scale_range": [0.8, 1.2]}
        self.assertEqual(draws[0], sample_table_style(cfg, 0))
        cfg["table"]["enabled"] = False
        self.assertIsNone(sample_table_style(cfg, 0))
        cfg["table"]["enabled"] = True
        cfg["enabled"] = False
        self.assertIsNone(sample_table_style(cfg, 0))
        cfg["table"]["options"][0]["color_rgb"] = [2, 0.2, 0.1]
        with self.assertRaisesRegex(ValueError, "color_rgb"):
            resolve_randomization(cfg)


class EvaluationContracts(unittest.TestCase):
    def test_joint_shape_and_measurement_scores_are_separate(self):
        s = generate("length", "shape_conditioned", "L5", 712)
        wrong_shape = next(o for o in s["objects"] if not o["shape_match"] and o["measurement_match"])
        wrong_size = next(o for o in s["objects"] if o["shape_match"] and not o["measurement_match"])
        a = score(s, {"selected_object_id": wrong_shape["object_id"]})
        b = score(s, {"selected_object_id": wrong_size["object_id"]})
        self.assertFalse(a["identification_correct"])
        self.assertTrue(a["measurement_correct"])
        self.assertFalse(a["shape_correct"])
        self.assertFalse(b["measurement_correct"])
        self.assertTrue(b["shape_correct"])
        oracle = score(s, {"selected_object_id": s["target_id"], "source": "oracle", "manipulation_success": True})
        result = summarize([a, b, oracle])
        self.assertEqual(result["target_identification_accuracy"], 0.0)
        self.assertEqual(result["measurement_conditioned_identification_accuracy"], 0.5)
        self.assertIsNone(result["manipulation_success"])
        self.assertEqual(result["oracle_predictions"], 1)

    def test_generated_scenario_has_one_instruction(self):
        s = generate("radius", "shape_conditioned", "L5", 712)
        self.assertNotIn("matched_controls", s)
        self.assertTrue(score(s, {"selected_object_id": s["target_id"]})["identification_correct"])


class RecordingContracts(unittest.TestCase):
    def test_compact_recording_keeps_truth_and_calibration_without_repeating_sidecars(self):
        import h5py
        from data_collection.calibration import export_calibration
        from data_collection.metadata import episode_info, read_scene, recording_info, save_run_config
        from data_collection.recorder import EpisodeRecorder
        from data_collection.validate import validate_dataset
        from environments import settings
        from evaluation.review import statistics
        from tasks.metric_spatial_understanding.metric_object_understanding.task import MOUConfig

        scenario = generate("length", "shape_conditioned", "L3", 712)
        with TemporaryDirectory() as folder:
            root = Path(folder)
            config = MOUConfig()
            config.scenario = scenario
            resolved = save_run_config(root, config, settings, root)
            writer = EpisodeRecorder(
                root / "episode_0000",
                episode_info(config.task, scenario, 712, scenario["target_index"]),
                root / "dataset.hdf5",
                scene={"scenario": scenario, "objects": scenario["objects"], "randomization_samples": {"seed": 712}},
                recording=recording_info(settings, [f"joint_{i}" for i in range(9)], "training"),
                run_config=resolved,
            )
            sample = {
                "frame": 0,
                "timestamp_s": 0.0,
                "rgb": np.zeros((2, 2, 3), dtype=np.uint8),
                "intrinsics": np.eye(3),
                "position": np.zeros(3),
                "quaternion_ros_xyzw": np.array([0.0, 0.0, 0.0, 1.0]),
            }
            writer.before(
                np.zeros(9),
                "approach",
                0.0,
                {"joint_position": np.zeros(9)},
                {view: sample for view in settings.CAMERAS},
            )
            writer.after({"joint_position": np.zeros(9)}, False)
            writer.finish(False, "timeout")
            compact = json.loads((root / "episode_0000/metadata.json").read_text())
            self.assertLess((root / "episode_0000/metadata.json").stat().st_size, 2000)
            self.assertNotIn("query", compact)
            self.assertNotIn("objects", compact)
            self.assertAlmostEqual(compact["episode_duration_s"], 1 / 30)
            self.assertNotIn("scenario", resolved)
            self.assertFalse(any(key.startswith("_") for key in resolved["task_settings"]))
            self.assertEqual(validate_dataset(root / "dataset.hdf5")["transitions"], 1)
            self.assertEqual(statistics(root)["counts"]["failed_attempts"], 1)
            # The HDF5 is independently readable, without either configuration sidecar.
            (root / "run_config.yaml").unlink()
            self.assertEqual(export_calibration(root / "dataset.hdf5", root, pose_streams=False), 1)
            self.assertEqual({p.name for p in (root / "meta").iterdir()}, {"camera_calibration.json"})
            with h5py.File(root / "dataset.hdf5", "r+") as data:
                episode = data["data/episode_0000"]
                self.assertEqual(read_scene(episode)["scenario"], scenario)
                compact["target_id"] = "wrong_object"
                episode.attrs["metadata_json"] = json.dumps(compact)
            with self.assertRaisesRegex(ValueError, "target_id disagrees"):
                validate_dataset(root / "dataset.hdf5")

    def test_interrupted_action_is_not_mislabeled_a_complete_transition(self):
        from data_collection.recorder import EpisodeRecorder
        from data_collection.validate import validate_dataset

        s = generate("length", "shape_conditioned", "L3", 712)
        with TemporaryDirectory() as folder:
            root = Path(folder)
            metadata = {
                "benchmark": "Benchmark",
                "task": "metric_spatial_understanding/metric_object_understanding",
                "query": {"scenario": s},
                "observability_status": "unverified",
                "control_dt_s": 0.1,
                "camera_views": ["top"],
                "accepted_demonstration": False,
            }
            writer = EpisodeRecorder(root / "episode_0000", metadata, root / "dataset.hdf5")
            sample = {
                "frame": 0,
                "timestamp_s": 0.0,
                "rgb": np.zeros((2, 2, 3), dtype=np.uint8),
                "intrinsics": np.eye(3),
                "position": np.zeros(3),
                "quaternion_ros_xyzw": np.array([0.0, 0.0, 0.0, 1.0]),
            }
            writer.before(np.zeros(9), "approach", 0.0, {"joint_position": np.zeros(9)}, {"top": sample})
            writer.after({"joint_position": np.zeros(9)}, False)
            writer.before(np.zeros(9), "descend", 0.1, {"joint_position": np.zeros(9)}, {})
            writer.finish(False, "interrupted")
            self.assertEqual(validate_dataset(root / "dataset.hdf5"), {"episodes": 1, "successes": 0, "transitions": 1})
            self.assertFalse(json.loads((root / "episode_0000/metadata.json").read_text())["success"])


class TimingContracts(unittest.TestCase):
    def test_task_overrides_cannot_disable_success_or_make_collection_impossible(self):
        from tasks.metric_spatial_understanding.metric_object_understanding.task import MOUConfig, configure

        for override in (
            {"SUCCESS_HOLD_SECONDS": 0.0},
            {"LIFT_HEIGHT": 0.0},
            {"TIMEOUT_SECONDS": float("nan")},
            {"CONTROL_STEPS": 1.5},
            {"LIFT_DISTANCE": 0.04},
            {"GRIPPER_CLOSED_WIDTH": 0.08},
        ):
            with self.subTest(override=override), self.assertRaises(ValueError):
                configure(MOUConfig(task_settings=override))

    def test_twenty_fps_is_compatible_with_thirty_hz_actions(self):
        from core.timing import capture_stride

        self.assertEqual(capture_stride(1 / 120, 4, 20), 6)
        with self.assertRaisesRegex(ValueError, "cannot exceed"):
            capture_stride(1 / 120, 4, 60)
        with self.assertRaisesRegex(ValueError, "divide"):
            capture_stride(1 / 120, 4, 17)


if __name__ == "__main__":
    unittest.main(verbosity=2)
