from __future__ import annotations

import argparse
import json
import math
import sys
import webbrowser
from pathlib import Path

from .config import load_config
from .simulation import SimulationEngine
from .server import serve


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="robo-control",
        description="Run the hardware-independent six-robot control simulator.",
    )
    parser.add_argument("--config", type=Path, help="JSON configuration file")
    parser.add_argument("--scenario", type=Path, help="scenario JSON file")
    parser.add_argument("--host", help="dashboard bind host (default: config value)")
    parser.add_argument("--port", type=int, help="dashboard port (default: config value)")
    parser.add_argument(
        "--no-browser", action="store_true", help="do not open the dashboard automatically"
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="run a fast deterministic mission and print the final JSON state",
    )
    parser.add_argument(
        "--headless-step",
        type=float,
        default=0.05,
        help="simulation step in headless mode (default: 0.05s)",
    )
    return parser


def run_headless(engine: SimulationEngine, step_s: float) -> int:
    if not math.isfinite(step_s) or step_s <= 0:
        raise ValueError("--headless-step must be finite and positive")
    effective_step_s = min(step_s, 0.25) * engine.config.mission.simulation_speed
    if not math.isfinite(effective_step_s) or effective_step_s <= 0:
        raise ValueError("effective headless step must be finite and positive")
    engine.start()
    # The engine clamps wall-time steps and applies simulation_speed. Follow
    # its actual status/time instead of assuming duration / requested_step.
    # A paused, unreachable mission cannot resume without external input.
    while engine.status.value == "running":
        previous_elapsed_s = engine.elapsed_s
        engine.step(step_s)
        if engine.status.value == "running" and engine.elapsed_s <= previous_elapsed_s:
            raise RuntimeError("headless simulation time did not advance")
    snapshot = engine.snapshot()
    print(json.dumps(snapshot, ensure_ascii=False, indent=2))
    return 0 if snapshot["status"] == "completed" and snapshot["metrics"]["collision_count"] == 0 else 2


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        config = load_config(args.config)
        engine = SimulationEngine(config, scenario_path=args.scenario)
        if args.headless:
            return run_headless(engine, args.headless_step)

        host = args.host or config.network.dashboard_host
        port = args.port or config.network.dashboard_port
        url = f"http://{host}:{port}/"
        print(f"Robo Control dashboard: {url}")
        print("실제 하드웨어 출력은 비활성화되어 있습니다. 종료: Ctrl+C")
        if not args.no_browser:
            webbrowser.open(url)
        serve(engine, host, port)
    except KeyboardInterrupt:
        print("\nStopped safely.")
        return 0
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
