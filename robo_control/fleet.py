"""Robot identity is independent of its role, display name and tag number."""
from __future__ import annotations

import re
from collections.abc import Mapping

ROBOT_ID = re.compile(r"^[A-Za-z][A-Za-z0-9_-]{0,31}$")
DEFAULT_ROLES = {"H1": "hamster", "H2": "beaver", "B1": "beaver", "B2": "beaver"}
DEFAULT_NAMES = {"H1": "햄스터", "H2": "세 번째 비버 (B3)", "B1": "한가한 비버", "B2": "바쁜 비버"}


def robot_id_valid(value: object) -> bool:
    return isinstance(value, str) and ROBOT_ID.fullmatch(value) is not None


def validate_roles(roles: Mapping[str, str] | None = None) -> dict[str, str]:
    result = dict(DEFAULT_ROLES if roles is None else roles)
    if not result or any(not robot_id_valid(k) or v not in {"hamster", "beaver"}
                         for k, v in result.items()):
        raise ValueError("Fleet needs unique robot IDs with explicit hamster/beaver roles")
    return result


def roles_from_scenario(data: dict) -> dict[str, str]:
    robots = data["ground_robots"]
    result = {r["id"]: r["role"] for r in robots}
    if len(result) != len(robots):
        raise ValueError("Duplicate robot ID in ground_robots")
    return validate_roles(result)


def validate_tag_registry(data: dict, tag_to_robot: Mapping[int, str]) -> dict[str, str]:
    """Fail startup if mission identities and the printed-tag registry disagree."""
    roles = roles_from_scenario(data)
    mapping = {}
    for robot in data["ground_robots"]:
        tag = robot.get("tag_id")
        if type(tag) is not int or tag < 0 or tag in mapping:
            raise ValueError("Each fleet entry needs a unique nonnegative tag_id")
        mapping[tag] = robot["id"]
    if mapping != dict(tag_to_robot):
        raise ValueError("Mission fleet and robot tag registry disagree; update both configurations")
    return roles
