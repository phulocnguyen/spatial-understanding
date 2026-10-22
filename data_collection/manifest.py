# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Task-independent manifest execution, logs, retry budgets, and resume."""

import argparse
import csv
import json
import subprocess
import sys
import time
from pathlib import Path

import yaml
from tasks.registry import get_task


def load_manifest(path):
    """Load an inline legacy manifest or a plan with paths relative to itself."""
    path = Path(path)
    if path.is_dir():
        path = path / "plan.yaml"
    manifest = yaml.safe_load(path.read_text())
    if manifest.get("schema_version") == 2:
        manifest["infeasible_cells"] = yaml.safe_load((path.parent / "reports" / "excluded_cells.yaml").read_text())
    for entry in manifest["episodes"]:
        if "scenario_file" in entry:
            scenario = json.loads((path.parent / entry["scenario_file"]).read_text())
            if scenario["fingerprint"] != entry["fingerprint"]:
                raise ValueError(f"Slot {entry['slot']}: scene fingerprint differs from the plan")
            for key in ("case", "condition", "level", "split"):
                if entry[key] != scenario[key]:
                    raise ValueError(f"Slot {entry['slot']}: {key} differs from the scene")
            if entry["prompt"] != scenario["instruction"]:
                raise ValueError(f"Slot {entry['slot']}: prompt differs from the scene")
            entry["scenario"] = scenario
    return manifest


def write_manifest(manifest, directory, report):
    """Separate the readable collection plan from exact generated scene records."""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    plan = {key: value for key, value in manifest.items() if key not in ("episodes", "infeasible_cells")}
    plan["schema_version"] = 2
    plan["episodes"] = []
    for entry in manifest["episodes"]:
        scenario = entry["scenario"]
        relative = Path("scenarios") / scenario["case"] / f"slot_{entry['slot']:04d}.json"
        target = directory / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(json.dumps(scenario, indent=2) + "\n")
        plan["episodes"].append(
            dict(
                slot=entry["slot"],
                case=scenario["case"],
                condition=scenario["condition"],
                level=scenario["level"],
                split=scenario["split"],
                prompt=scenario["instruction"],
                scenario_file=relative.as_posix(),
                fingerprint=scenario["fingerprint"],
            )
        )
    (directory / "plan.yaml").write_text(yaml.safe_dump(plan, sort_keys=False, allow_unicode=True))
    with (directory / "slots.csv").open("w", newline="") as stream:
        columns = ["slot", "case", "condition", "level", "split", "prompt", "scenario_file"]
        writer = csv.DictWriter(stream, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(plan["episodes"])
    reports = directory / "reports"
    reports.mkdir(exist_ok=True)
    (reports / "coverage.yaml").write_text(yaml.safe_dump(report, sort_keys=False))
    excluded = manifest.get("infeasible_cells", [])
    (reports / "excluded_cells.yaml").write_text(yaml.safe_dump(excluded, sort_keys=False))


def validate_manifest(manifest):
    slots = set()
    splits = {}
    for entry in manifest["episodes"]:
        s = entry["scenario"]
        info = get_task(f"{s['category']}/{s['task']}").validate_recorded_scenario(s)
        if entry["slot"] in slots:
            raise ValueError("Duplicate manifest slot")
        slots.add(entry["slot"])
        key = info.get("control_group_id", s["fingerprint"])
        if splits.setdefault(key, s["split"]) != s["split"]:
            raise ValueError("Matched geometry leaks across splits")


def collect_main():
    parser = argparse.ArgumentParser(
        description="Collect a Benchmark manifest with videos, saved logs, and bounded retries"
    )
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max_slots", type=int)
    parser.add_argument("--start_slot", type=int, default=0)
    parser.add_argument("--attempts_per_slot", type=int, default=3)
    parser.add_argument("--attempt_timeout_s", type=float, default=300.0)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--gpu", type=int, help="RTX renderer GPU index for this worker (as listed by nvidia-smi)")
    args = parser.parse_args()
    if (
        args.attempts_per_slot < 1
        or args.start_slot < 0
        or args.attempt_timeout_s <= 0
        or (args.gpu is not None and args.gpu < 0)
        or args.max_slots is not None
        and args.max_slots < 1
    ):
        parser.error("Invalid attempt/count/timeout limits")
    manifest = load_manifest(args.manifest)
    validate_manifest(manifest)
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    root = Path(__file__).resolve().parents[1]
    entries = [e for e in manifest["episodes"] if e["slot"] >= args.start_slot][: args.max_slots]
    for entry in entries:
        s = entry["scenario"]
        folder = output / f"slot_{entry['slot']:04d}"
        if folder.exists() and not args.resume:
            raise FileExistsError(f"{folder}: use a fresh output directory or --resume")
        folder.mkdir(exist_ok=True)
        source = folder / "scenario.json"
        if source.exists() and json.loads(source.read_text())["fingerprint"] != s["fingerprint"]:
            raise ValueError("Existing slot belongs to another scenario")
        source.write_text(json.dumps(s, indent=2) + "\n")
        history_path = folder / "attempts.json"
        history = json.loads(history_path.read_text()) if history_path.exists() else []
        if any(a["successful_rollout"] for a in history):
            print(f"Slot {entry['slot']}: successful attempt already exists; skipping", flush=True)
            continue
        for retry in range(len(history), args.attempts_per_slot):
            task_id = f"{s['category']}/{s['task']}"
            config = get_task(task_id).collection_settings(s)
            config.update(
                scenario_file=str(source),
                rollout_seed_offset=retry * 100003,
                num_episodes=1,
                max_attempts=1,
                success_target=None,
                video=True,
                capture_profile="full",
                output_dir=str(folder / "data"),
            )
            cfg = folder / f"collection_{retry:02d}.yaml"
            cfg.write_text(yaml.safe_dump(config, sort_keys=False))
            logpath = folder / f"attempt_{retry:02d}.log"
            before = set((folder / "data").glob("*/episode_*/metadata.json"))
            start = time.monotonic()
            timed_out = False
            print(
                f"Slot {entry['slot']}: {s['case']}/{s['condition']}/{s['level']}, attempt {retry + 1}; log={logpath}",
                flush=True,
            )
            with logpath.open("w") as log:
                command = [sys.executable, str(root / "collect_data.py"), "--config", str(cfg), "--visualizer", "none"]
                if args.gpu is not None:
                    command.extend(["--kit_args", f"--/renderer/multiGpu/activeCudaGpus={args.gpu},"])
                process = subprocess.Popen(
                    command,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                )
                try:
                    while process.poll() is None:
                        try:
                            remaining = args.attempt_timeout_s - (time.monotonic() - start)
                            process.wait(timeout=max(0.0, min(30.0, remaining)))
                        except subprocess.TimeoutExpired:
                            print(f"  slot {entry['slot']}: running {time.monotonic() - start:.0f}s", flush=True)
                        if process.poll() is None and time.monotonic() - start > args.attempt_timeout_s:
                            timed_out = True
                            print(
                                f"  slot {entry['slot']}: attempt exceeded {args.attempt_timeout_s:g}s; stopping",
                                flush=True,
                            )
                            process.terminate()
                            try:
                                process.wait(timeout=15)
                            except subprocess.TimeoutExpired:
                                process.kill()
                                process.wait()
                except KeyboardInterrupt:
                    process.send_signal(2)
                    try:
                        process.wait(timeout=30)
                    except subprocess.TimeoutExpired:
                        process.terminate()
                        process.wait(timeout=15)
                    raise
            new = set((folder / "data").glob("*/episode_*/metadata.json")) - before
            metas = [json.loads(p.read_text()) for p in new]
            success = any(m.get("finalized") and m.get("success") for m in metas)
            history.append(
                dict(
                    retry=retry,
                    returncode=process.returncode,
                    timed_out=timed_out,
                    successful_rollout=success,
                    wall_time_s=time.monotonic() - start,
                    metadata_files=[str(p) for p in new],
                )
            )
            history_path.write_text(json.dumps(history, indent=2) + "\n")
            print(
                f"  slot {entry['slot']}: {'SUCCESS' if success else 'FAILED'}; videos beside dataset.hdf5", flush=True
            )
            if success:
                break
