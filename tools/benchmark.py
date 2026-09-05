from __future__ import annotations

import argparse
import json
import statistics
import sys
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from robo_control.config import load_config
from robo_control.models import MissionStatus
from robo_control.simulation import SimulationEngine


def main() -> int:
    parser = argparse.ArgumentParser(description="Repeat the deterministic demo mission.")
    parser.add_argument("--runs", type=int, default=100)
    parser.add_argument("--step", type=float, default=0.05)
    args = parser.parse_args()
    if args.runs <= 0 or args.step <= 0:
        parser.error("--runs and --step must be positive")

    config = load_config()
    planning_times: list[float] = []
    completed = 0
    collision_free = 0
    for _ in range(args.runs):
        engine = SimulationEngine(config)
        planning_times.append(engine.planning_ms)
        engine.start()
        for _ in range(int(config.mission.duration_s / args.step) + 2):
            engine.step(args.step)
            if engine.status != MissionStatus.RUNNING:
                break
        completed += engine.status == MissionStatus.COMPLETED
        collision_free += engine.collision_count == 0

    ordered = sorted(planning_times)
    percentile_index = min(len(ordered) - 1, int(len(ordered) * 0.95))
    result = {
        "runs": args.runs,
        "completed": completed,
        "collision_free": collision_free,
        "planning_mean_ms": round(statistics.mean(planning_times), 3),
        "planning_p95_ms": round(ordered[percentile_index], 3),
        "planning_max_ms": round(max(planning_times), 3),
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if completed == collision_free == args.runs else 2


if __name__ == "__main__":
    raise SystemExit(main())
