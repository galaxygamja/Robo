from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True, slots=True)
class FieldConfig:
    width_m: float
    height_m: float
    grid_cell_m: float
    boundary_margin_m: float


@dataclass(frozen=True, slots=True)
class MissionConfig:
    duration_s: float
    robot_count: int
    control_hz: float
    planner_hz: float
    simulation_speed: float


@dataclass(frozen=True, slots=True)
class RobotConfig:
    radius_m: float
    linear_speed_mps: float
    goal_tolerance_m: float
    tracking_timeout_s: float
    command_timeout_s: float


@dataclass(frozen=True, slots=True)
class NetworkConfig:
    dashboard_host: str
    dashboard_port: int
    robot_udp_port: int


@dataclass(frozen=True, slots=True)
class SafetyConfig:
    minimum_separation_m: float
    stop_on_tracking_loss: bool
    stop_on_predicted_collision: bool


@dataclass(frozen=True, slots=True)
class AppConfig:
    field: FieldConfig
    mission: MissionConfig
    robot: RobotConfig
    network: NetworkConfig
    safety: SafetyConfig


def default_config_path() -> Path:
    return default_data_path("default.json")


def default_data_path(filename: str) -> Path:
    """Resolve editable source data first, then wheel-installed package data.

    The repository-level ``config`` directory stays convenient for teams that
    run or edit a source checkout.  A normal wheel/sdist installation does not
    have that directory next to site-packages, so distributable copies live in
    ``robo_control/data`` and are included as package data.
    """

    if Path(filename).name != filename:
        raise ValueError(f"data filename must not contain a path: {filename!r}")
    package_dir = Path(__file__).resolve().parent
    source_root = package_dir.parent
    source_candidate = source_root / "config" / filename
    if (source_root / "pyproject.toml").is_file() and source_candidate.is_file():
        return source_candidate

    packaged_candidate = package_dir / "data" / filename
    if packaged_candidate.is_file():
        return packaged_candidate
    raise FileNotFoundError(f"required Robo data file is missing: {filename}")


def _is_finite_number(value: Any) -> bool:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


def _require_positive(section: str, values: dict[str, Any], keys: tuple[str, ...]) -> None:
    for key in keys:
        value = values[key]
        if not _is_finite_number(value) or value <= 0:
            raise ValueError(f"{section}.{key} must be finite and positive, got {value!r}")


def load_config(path: str | Path | None = None) -> AppConfig:
    config_path = Path(path) if path else default_config_path()
    with config_path.open("r", encoding="utf-8") as handle:
        raw = json.load(handle)

    for section in ("field", "mission", "robot", "network", "safety"):
        if section not in raw:
            raise ValueError(f"missing configuration section: {section}")

    _require_positive("field", raw["field"], ("width_m", "height_m", "grid_cell_m"))
    _require_positive(
        "mission",
        raw["mission"],
        ("duration_s", "robot_count", "control_hz", "planner_hz", "simulation_speed"),
    )
    _require_positive(
        "robot",
        raw["robot"],
        (
            "radius_m",
            "linear_speed_mps",
            "goal_tolerance_m",
            "tracking_timeout_s",
            "command_timeout_s",
        ),
    )
    _require_positive("network", raw["network"], ("dashboard_port", "robot_udp_port"))
    _require_positive("safety", raw["safety"], ("minimum_separation_m",))

    if isinstance(raw["mission"]["robot_count"], bool) or not isinstance(
        raw["mission"]["robot_count"], int
    ):
        raise ValueError("mission.robot_count must be an integer")
    if not 1 <= raw["mission"]["robot_count"] <= 8:
        raise ValueError("mission.robot_count must be between 1 and 8 in v0.1")
    if raw["field"]["grid_cell_m"] >= min(raw["field"]["width_m"], raw["field"]["height_m"]):
        raise ValueError("field.grid_cell_m is too large for the field")
    margin = raw["field"]["boundary_margin_m"]
    if not _is_finite_number(margin) or margin < 0:
        raise ValueError("field.boundary_margin_m must be finite and not negative")
    for key in ("stop_on_tracking_loss", "stop_on_predicted_collision"):
        if not isinstance(raw["safety"][key], bool):
            raise ValueError(f"safety.{key} must be a boolean")
    for key in ("dashboard_port", "robot_udp_port"):
        port = raw["network"][key]
        if isinstance(port, bool) or not isinstance(port, int):
            raise ValueError(f"network.{key} must be an integer")
        if port > 65535:
            raise ValueError(f"network.{key} must be at most 65535")
    if raw["safety"]["minimum_separation_m"] < 2 * raw["robot"]["radius_m"]:
        raise ValueError("safety.minimum_separation_m must cover two robot radii")

    return AppConfig(
        field=FieldConfig(**raw["field"]),
        mission=MissionConfig(**raw["mission"]),
        robot=RobotConfig(**raw["robot"]),
        network=NetworkConfig(**raw["network"]),
        safety=SafetyConfig(**raw["safety"]),
    )
