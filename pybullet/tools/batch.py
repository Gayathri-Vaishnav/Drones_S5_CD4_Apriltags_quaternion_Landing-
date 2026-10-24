"""Seeded process-isolated Monte Carlo runs with Wilson success intervals."""
from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor
import csv
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
from multiprocessing import get_context

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from evaluate import run_trial
from config import ADDRESSES
from metrics import summarize_trials


COMMON_SCENARIOS = ROOT.parent / "scenarios" / "common"


def load_scenarios():
    # JSON is a strict subset of YAML, so these files need no runtime YAML dependency.
    schema=json.loads((COMMON_SCENARIOS/"schema.yaml").read_text(encoding="utf-8-sig"))
    scenarios={}
    for path in sorted(COMMON_SCENARIOS.glob("*.yaml")):
        if path.name=="schema.yaml":
            continue
        item=json.loads(path.read_text(encoding="utf-8-sig"))
        missing=[key for key in schema["required_scenario_fields"] if key not in item]
        if missing:
            raise ValueError(f"{path.name} is missing required fields: {missing}")
        if item["schema_version"]!=schema["schema_version"]:
            raise ValueError(f"Unsupported scenario schema in {path.name}")
        _validate_supported_scenario(path.name,item)
        scenarios[path.stem]=item
    return schema,scenarios


def _validate_supported_scenario(name,item):
    address=int(item["target"]["address"])
    if address not in ADDRESSES:
        raise ValueError(f"{name}: address {address} is not configured in PyBullet")
    if item["target"]["type"]!=ADDRESSES[address].kind:
        raise ValueError(f"{name}: target type does not match configured address {address}")
    motion=ADDRESSES[address].motion
    path=item["target"].get("path",{})
    expected="stationary" if motion=="static" else ("circular" if motion=="circle" else "sinusoidal_line")
    if path.get("kind")!=expected:
        raise ValueError(f"{name}: path kind {path.get('kind')} does not match the configured target motion {expected}")
    if motion=="circle" and (abs(float(path.get("radius_m",-1))-1.0)>1e-9 or
                              abs(float(path.get("angular_rate_rad_s",-1))-ADDRESSES[address].speed)>1e-9):
        raise ValueError(f"{name}: moving path parameters differ from address {address} configuration")
    if item.get("events") or item.get("faults"):
        raise NotImplementedError(f"{name}: event/fault injection is not implemented; refusing to ignore requests")
    sensors=item.get("sensors",{})
    if sensors.get("noise_level")!="none" or float(sensors.get("latency_s",0))!=0:
        raise NotImplementedError(f"{name}: sensor noise/latency injection is not implemented")
    vision=item.get("vision",{})
    if (vision.get("lighting")!="nominal" or float(vision.get("blur_px",0))!=0 or
            float(vision.get("occlusion_fraction",0))!=0 or vision.get("tag_loss_events")):
        raise NotImplementedError(f"{name}: non-nominal vision effects are not implemented")
    wind=item.get("environment",{}).get("wind",{})
    if wind.get("profile") not in ("none","correlated_gust_proxy"):
        raise NotImplementedError(f"{name}: wind profile {wind.get('profile')} is not supported")
    if len(wind.get("mean_m_s",[0,0,0]))!=3:
        raise ValueError(f"{name}: wind mean must contain three components")


def run_job(job):
    address, seed, seconds, output_dir, start_position, wind_scale, wind_mean = job
    return run_trial(address, seed=seed, seconds=seconds, output_dir=output_dir,
                     start_position=start_position,wind_scale=wind_scale,wind_mean=wind_mean)


def main():
    parser = argparse.ArgumentParser()
    schema,available=load_scenarios()
    parser.add_argument("--scenario", choices=(*available,"all"), default="all")
    parser.add_argument("--trials", type=int, default=30)
    parser.add_argument("--seconds", type=float, default=50)
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument("--seed", type=int, default=1)
    args = parser.parse_args()
    if args.trials < 1 or args.seconds <= 0 or args.workers < 1:
        parser.error("trials, seconds, and workers must be positive")
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    batch_dir = ROOT / "results" / f"batch_{stamp}"
    batch_dir.mkdir(parents=True, exist_ok=False)
    selected = available if args.scenario == "all" else {args.scenario: available[args.scenario]}
    results = {"schema_version": 1, "created_utc": datetime.now(timezone.utc).isoformat(),
               "seed_start": args.seed, "duration_s": args.seconds, "worker_count": args.workers,
               "scenario_schema_version":schema["schema_version"],"scenarios": {}}
    jobs = []
    for name, scenario in selected.items():
        address=int(scenario["target"]["address"])
        start_position=scenario.get("start_pose",{}).get("position_m")
        profile=scenario.get("environment",{}).get("wind",{}).get("profile","none")
        wind_scale=0.0 if profile=="none" else 1.0
        wind_mean=scenario.get("environment",{}).get("wind",{}).get("mean_m_s",[0,0,0])
        scenario_dir = batch_dir / name
        scenario_dir.mkdir()
        (scenario_dir/"scenario.yaml").write_text(json.dumps(scenario,indent=2),encoding="utf-8")
        jobs.extend((address, args.seed+i, args.seconds, str(scenario_dir),start_position,wind_scale,wind_mean)
                    for i in range(args.trials))
    if args.workers == 1:
        trials_by_scenario = {name: [] for name in selected}
        job_index=0
        for name, scenario in selected.items():
            scenario_dir = str(batch_dir/name)
            address=int(scenario["target"]["address"])
            start_position=scenario.get("start_pose",{}).get("position_m")
            profile=scenario.get("environment",{}).get("wind",{}).get("profile","none")
            wind_scale=0.0 if profile=="none" else 1.0
            wind_mean=scenario.get("environment",{}).get("wind",{}).get("mean_m_s",[0,0,0])
            for i in range(args.trials):
                trials_by_scenario[name].append(run_job((address,args.seed+i,args.seconds,scenario_dir,
                                                         start_position,wind_scale,wind_mean)))
    else:
        # Each spawned worker creates and owns a separate p.DIRECT client.
        with ProcessPoolExecutor(max_workers=args.workers, mp_context=get_context("spawn")) as pool:
            all_trials = list(pool.map(run_job, jobs))
        trials_by_scenario = {name: [t for t in all_trials if t["address"]==int(scenario["target"]["address"])]
                              for name,scenario in selected.items()}
    for name, trials in trials_by_scenario.items():
        summary = summarize_trials(trials)
        results["scenarios"][name] = summary
        (batch_dir/f"{name}_summary.json").write_text(json.dumps(summary,indent=2),encoding="utf-8")
        with (batch_dir/f"{name}_metrics.csv").open("w",newline="",encoding="utf-8") as handle:
            fields=("address","seed","phase","success","landed","final_horizontal_error_m","time_to_acquire_s",
                    "time_to_land_s","touchdown_speed_m_s","max_tilt_deg","path_length_m","energy_j",
                    "failsafe_events","camera_mean_s","controller_mean_s","physics_step_mean_s",
                    "decoded_frames","simulated_seconds")
            writer=csv.DictWriter(handle,fieldnames=fields,extrasaction="ignore")
            writer.writeheader(); writer.writerows(trials)
    results_path=batch_dir/"batch_summary.json"
    results_path.write_text(json.dumps(results,indent=2),encoding="utf-8")
    print(json.dumps({"output":str(batch_dir),"summary":results},indent=2))


if __name__ == "__main__":
    main()
