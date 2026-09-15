"""Versioned raw-evidence contract for the thin Condition C recorder.

This module deliberately contains no ROS subscriptions and no derived metric
logic.  It is the single place that defines which raw streams the native
recorder owns and which streams are required to be non-empty for a valid
complete-evidence run.
"""

from __future__ import annotations

from typing import Iterable


SCHEMA_VERSION = "thin_raw_evidence_contract_1.4"


def _topic(specs, topic, message_type, semantic, required=False,
           expected_presence="runtime"):
    specs[topic] = {
        "topic": topic,
        "message_type": message_type,
        "semantic": semantic,
        "required": bool(required),
        "expected_presence": expected_presence,
    }


def topic_spec(robots: Iterable[str] = ("robot1", "robot2")) -> dict:
    """Return the raw topic contract without importing ROS message classes."""
    robots = tuple(str(robot) for robot in robots)
    specs = {}
    _topic(specs, "/clock", "rosgraph_msgs/msg/Clock", "simulation_clock",
           required=True)
    _topic(specs, "/tf", "tf2_msgs/msg/TFMessage", "dynamic_tf",
           required=True)
    _topic(specs, "/tf_static", "tf2_msgs/msg/TFMessage", "static_tf",
           required=True)
    _topic(specs, "/rosout", "rcl_interfaces/msg/Log", "rosout",
           expected_presence="if_published")
    _topic(specs, "/cslam/unknown_pose/start_release", "std_msgs/msg/String",
           "common_start_release", required=True,
           expected_presence="before_exploration")
    for robot in robots:
        prefix = f"/{robot}"
        _topic(specs, f"{prefix}/odom", "nav_msgs/msg/Odometry", "odometry",
               required=True)
        _topic(specs, f"{prefix}/cmd_vel_nav", "geometry_msgs/msg/Twist",
               "navigation_command", required=True)
        _topic(specs, f"{prefix}/cmd_vel", "geometry_msgs/msg/TwistStamped",
               "command", required=True)
        # The thin contract deliberately has one authoritative motion source
        # (odom/commands) and does not duplicate the three derived LaserScan
        # streams or joint_states. Those streams remain available to the
        # legacy diagnostic recorder; they are not inputs to the thesis
        # metrics reconstructed below.
        for stream in ("map", "shared_map"):
            _topic(specs, f"{prefix}/{stream}", "nav_msgs/msg/OccupancyGrid",
                   stream, required=stream in ("map", "shared_map"))
        # Canonical Condition C uses the unknown-pose exchange namespace.
        # The known-pose peer-map namespace is a separate launch mode.
        _topic(specs, f"/cslam/unknown_pose/{robot}/local_map",
               "my_epuck_interfaces/msg/PeerMap", "peer_map",
               required=True)
        _topic(specs, f"{prefix}/frontier_candidates",
               "my_epuck_interfaces/msg/FrontierCandidateArray",
               "frontier_candidates", required=True)
        _topic(specs, f"/cslam/unknown_pose/{robot}/exploration_status",
               "my_epuck_interfaces/msg/ExplorationStatus",
               "exploration_status", expected_presence="if_published")
        _topic(specs, f"/cslam/unknown_pose/{robot}/exploration_event",
               "my_epuck_interfaces/msg/ExplorationEvent",
               "exploration_event", expected_presence="if_published")
        for name, message_type, semantic in (
                ("task_snapshot", "my_epuck_interfaces/msg/TaskSnapshot", "task_snapshot"),
                ("task_bids", "my_epuck_interfaces/msg/TaskBidArray", "task_bids"),
                ("pair_decision", "my_epuck_interfaces/msg/PairDecision", "pair_decision"),
                ("distributed_status", "my_epuck_interfaces/msg/DistributedExplorationStatus", "distributed_status"),
                ("distributed_event", "my_epuck_interfaces/msg/DistributedExplorationEvent", "distributed_event"),
                ("exploration_failure", "my_epuck_interfaces/msg/ExplorationFailure", "exploration_failure"),
        ):
            _topic(specs, f"{prefix}/{name}", message_type, semantic,
                   expected_presence="if_published")
        for suffix, message_type, semantic in (
                ("navigate_to_pose/_action/status",
                 "action_msgs/msg/GoalStatusArray", "nav_status"),
                ("navigate_to_pose/_action/feedback",
                 "nav2_msgs/action/NavigateToPose_FeedbackMessage",
                 "nav_feedback"),
                ("follow_path/_action/status",
                 "action_msgs/msg/GoalStatusArray", "controller_status"),
                ("compute_path_to_pose/_action/status",
                 "action_msgs/msg/GoalStatusArray", "planner_status"),
                ("plan", "nav_msgs/msg/Path", "planner_path"),
        ):
            _topic(specs, f"{prefix}/{suffix}", message_type, semantic,
                   expected_presence="if_published")
    return specs


def required_nonempty_topics(robots: Iterable[str] = ("robot1", "robot2")) -> tuple[str, ...]:
    """Streams whose absence makes a C raw-evidence run invalid."""
    specs = topic_spec(robots)
    return tuple(topic for topic, value in specs.items() if value["required"])


def condition_semantic_requirements(condition: str | None,
                                    robots: Iterable[str] = ("robot1", "robot2"),
                                    unknown_initial_pose: bool = False,
                                    assignment_strategy: str | None = None) -> dict:
    """Return semantic, condition-aware evidence requirements.

    Topic presence is not enough for a cooperative run: a bag can contain
    maps and odometry while containing no evidence that the distributed
    protocol actually ran.  These requirements are deliberately expressed as
    small count predicates so the raw recorder remains lossless and the
    evaluator remains the owner of derived interpretation.

    ``None`` keeps the historical topic-only contract for callers that are
    validating a generic bag.  In particular, this preserves compatibility
    with old A/B fixtures while making C fail closed when its cooperation
    evidence is absent.
    """
    if condition is None:
        return {}
    condition = str(condition).upper()
    robots = tuple(str(robot) for robot in robots)
    requirements = {
        "navigation_outcomes": {
            "all_nonempty": [
                f"/{robot}/navigate_to_pose/_action/status"
                for robot in robots
            ],
        },
    }
    if condition == "C":
        requirements.update({
            "cooperation_status": {
                "all_nonempty": [
                    f"/{robot}/distributed_status" for robot in robots
                ],
            },
            "cooperation_events": {
                "all_nonempty": [
                    f"/{robot}/distributed_event" for robot in robots
                ],
            },
            "task_snapshots": {
                "all_nonempty": [
                    f"/{robot}/task_snapshot" for robot in robots
                ],
            },
            # The replicated allocator may publish bids or a pair decision;
            # distributed events are the fallback authoritative decision
            # stream in the current C implementation.
            "assignment_decisions": {
                "any_nonempty": [
                    *[f"/{robot}/task_bids" for robot in robots],
                    *[f"/{robot}/pair_decision" for robot in robots],
                    *[f"/{robot}/distributed_event" for robot in robots],
                ],
            },
        })
        if str(assignment_strategy or '').strip().lower() == 'frontier_cost_only':
            # This is deliberately only the raw event-stream requirement.
            # Payload-level certificate observations are validated by the
            # offline protocol replay, which distinguishes OBSERVED from
            # NOT_INVOKED and MISSING_OBSERVATIONS.
            requirements["certificate_event_stream"] = {
                "any_nonempty": [
                    f"/{robot}/distributed_event" for robot in robots
                ],
            }
        if unknown_initial_pose:
            requirements["structured_handoff"] = {
                "artifact": "handoff_structured"
            }
    elif condition == "B":
        requirements["independent_robot_activity"] = {
            "all_nonempty": [f"/{robot}/odom" for robot in robots]
        }
    return requirements


def semantic_contract_status(counts: dict | None, condition: str | None,
                             robots: Iterable[str] = ("robot1", "robot2"),
                             unknown_initial_pose: bool = False,
                             semantic_artifacts: dict | None = None,
                             assignment_strategy: str | None = None) -> dict:
    """Evaluate semantic requirements without interpreting message payloads."""
    counts = counts or {}
    semantic_artifacts = semantic_artifacts or {}
    requirements = condition_semantic_requirements(
        condition, robots, unknown_initial_pose, assignment_strategy)
    failed = []
    checks = {}
    for name, requirement in requirements.items():
        if "artifact" in requirement:
            value = bool(semantic_artifacts.get(requirement["artifact"], False))
            checks[name] = {"complete": value, "artifact": requirement["artifact"]}
        else:
            all_topics = requirement.get("all_nonempty", [])
            any_topics = requirement.get("any_nonempty", [])
            all_ok = all(int(counts.get(topic, 0)) > 0 for topic in all_topics)
            any_ok = (not any_topics or any(int(counts.get(topic, 0)) > 0
                                            for topic in any_topics))
            value = all_ok and any_ok
            checks[name] = {
                "complete": value,
                "all_nonempty": all_topics,
                "any_nonempty": any_topics,
                "missing": [topic for topic in all_topics
                            if int(counts.get(topic, 0)) <= 0],
            }
        if not checks[name]["complete"]:
            failed.append(name)
    return {
        "condition": condition,
        "complete": not failed,
        "failed_requirements": failed,
        "checks": checks,
    }


def contract_metadata(robots: Iterable[str] = ("robot1", "robot2"),
                      condition: str | None = None,
                      unknown_initial_pose: bool = False,
                      assignment_strategy: str | None = None) -> dict:
    specs = topic_spec(robots)
    return {
        "schema_version": SCHEMA_VERSION,
        "robots": list(robots),
        "topics": specs,
        "required_nonempty_topics": list(required_nonempty_topics(robots)),
        "condition": condition,
        "assignment_strategy": assignment_strategy,
        "unknown_initial_pose": bool(unknown_initial_pose),
        "semantic_requirements": condition_semantic_requirements(
            condition, robots, unknown_initial_pose, assignment_strategy),
        "timestamp_policy": {
            "authoritative": "simulation/header time",
            "bag_order": "native rosbag2 storage order",
            "wall_receive_time": "not a scientific metric",
        },
        "derived_metrics": "offline_only",
    }
