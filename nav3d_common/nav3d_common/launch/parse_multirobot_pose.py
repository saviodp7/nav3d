from typing import Any

import launch
import yaml
from launch.launch_context import LaunchContext


class ParseMultiRobotPose(launch.Substitution):  # type: ignore[misc]
    def __init__(self, robots_argument: launch.SomeSubstitutionsType) -> None:
        super().__init__()
        from launch.utilities import normalize_to_list_of_substitutions

        self._robots_argument = normalize_to_list_of_substitutions(robots_argument)

    def describe(self) -> str:
        return ""

    def perform(self, context: LaunchContext) -> str:
        robots_str = launch.utilities.perform_substitutions(context, self._robots_argument).strip()
        if not robots_str:
            return "{}"

        multirobots: dict[str, dict[str, float]] = {}

        for robot_entry in robots_str.split(";"):
            entry = robot_entry.strip()
            if not entry:
                continue

            key_val = entry.split("=", 1)
            if len(key_val) != 2:
                continue

            robot_name = key_val[0].strip()
            pose_str = key_val[1].strip()

            pose: Any = yaml.safe_load(pose_str)
            if not isinstance(pose, dict):
                pose = {}

            for k in ("x", "y", "z", "roll", "pitch", "yaw"):
                v = pose.get(k, 0.0)
                try:
                    pose[k] = float(v)
                except (TypeError, ValueError):
                    pose[k] = 0.0

            multirobots[robot_name] = {
                "x": pose["x"],
                "y": pose["y"],
                "z": pose["z"],
                "roll": pose["roll"],
                "pitch": pose["pitch"],
                "yaw": pose["yaw"],
            }

        return yaml.safe_dump(multirobots, default_flow_style=True).strip()
