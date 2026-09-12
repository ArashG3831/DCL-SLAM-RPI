#!/usr/bin/env python3
"""Helpers for the opt-in live ON/OFF Slam Toolbox comparison.

The production mapper is left untouched.  The comparison mapper receives a
copied parameter file with only the output frame/topic and scan-matching flag
changed, so both mappers consume the same live /scan and /odom streams.
"""

import copy
import os
from pathlib import Path

import yaml


OFF_NODE_NAME = "slam_toolbox_off"
OFF_MAP_FRAME = "map_off"
OFF_MAP_TOPIC = "/map_off"


def write_live_off_parameters(source_path, destination_path):
    """Write a non-production OFF config and return its absolute path."""
    source = Path(source_path).expanduser().resolve()
    destination = Path(destination_path).expanduser().resolve()
    with source.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError(f"SLAM parameter file is not a YAML mapping: {source}")

    branch = copy.deepcopy(config)
    # ROS parameter files are keyed by the fully resolved node name.  The
    # production file is keyed as "slam_toolbox"; the second node is named
    # "slam_toolbox_off", so leaving the old key would silently make ROS use
    # defaults for the OFF mapper.
    production_node = branch.pop("slam_toolbox", None)
    if not isinstance(production_node, dict):
        raise ValueError("SLAM parameter file lacks slam_toolbox parameters")
    branch[OFF_NODE_NAME] = production_node
    params = branch[OFF_NODE_NAME].setdefault("ros__parameters", {})
    params["use_scan_matching"] = False
    params["map_frame"] = OFF_MAP_FRAME
    params["map_name"] = OFF_MAP_TOPIC
    # The validated production matcher remains represented in the copied file;
    # it is simply unused by this OFF branch.
    params["do_loop_closing"] = False

    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(branch, stream, sort_keys=False)
    return os.fspath(destination)


def live_off_launch_command(params_path):
    """Return the command for the second, uniquely named mapper."""
    return [
        "python3",
        os.path.join(os.path.dirname(__file__), "live_slam_off_launcher.py"),
        params_path,
    ]
