#!/usr/bin/env python3
"""Capture a focused Nav2 debug bundle for Concert planner/costmap issues."""

from __future__ import annotations

import argparse
import datetime as _dt
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import time
from typing import Iterable


NAV2_NODES = [
    "/planner_server",
    "/controller_server",
    "/bt_navigator",
    "/recoveries_server",
    "/velocity_smoother",
    "/collision_monitor",
    "/lifecycle_manager_navigation",
    "/global_costmap/global_costmap",
    "/local_costmap/local_costmap",
]

LOCALIZATION_NODES = [
    "/amcl",
    "/map_server",
    "/lifecycle_manager_localization",
    "/slam_toolbox",
]

PERCEPTION_NODES = [
    "/pointcloud_to_laserscan_front",
    "/pointcloud_to_laserscan_back",
    "/laserscan_multi_merger",
    "/pointcloud_merger",
]

TOPICS_FOR_INFO = [
    "/clock",
    "/tf",
    "/tf_static",
    "/map",
    "/map_updates",
    "/scan",
    "/VLP16_lidar_front/scan",
    "/VLP16_lidar_back/scan",
    "/base_link/odom",
    "/amcl_pose",
    "/particle_cloud",
    "/plan",
    "/unsmoothed_plan",
    "/transformed_global_plan",
    "/optimal_trajectory",
    "/trajectories",
    "/behavior_tree_log",
    "/rosout",
    "/diagnostics",
    "/local_costmap/costmap",
    "/local_costmap/costmap_raw",
    "/local_costmap/costmap_updates",
    "/local_costmap/costmap_raw_updates",
    "/local_costmap/footprint",
    "/local_costmap/obstacle_layer",
    "/local_costmap/published_footprint",
    "/global_costmap/costmap",
    "/global_costmap/costmap_raw",
    "/global_costmap/costmap_updates",
    "/global_costmap/costmap_raw_updates",
    "/global_costmap/footprint",
    "/global_costmap/obstacle_layer",
    "/global_costmap/static_layer",
    "/global_costmap/published_footprint",
    "/cmd_vel_nav",
    "/cmd_vel_smoothed",
    "/omnisteering/cmd_vel",
    "/collision_monitor_state",
    "/polygon_stop",
    "/polygon_slow",
    "/goal_pose",
    "/initialpose",
    "/speed_limit",
    "/navigate_to_pose/_action/status",
    "/navigate_to_pose/_action/feedback",
    "/compute_path_to_pose/_action/status",
    "/compute_path_to_pose/_action/feedback",
    "/follow_path/_action/status",
    "/follow_path/_action/feedback",
]

TOPICS_FOR_HZ = [
    "/clock",
    "/scan",
    "/VLP16_lidar_front/scan",
    "/VLP16_lidar_back/scan",
    "/base_link/odom",
    "/amcl_pose",
    "/local_costmap/costmap",
    "/global_costmap/costmap",
    "/plan",
    "/transformed_global_plan",
    "/optimal_trajectory",
    "/cmd_vel_nav",
    "/cmd_vel_smoothed",
    "/omnisteering/cmd_vel",
]

TOPICS_FOR_ECHO = [
    "/clock",
    "/scan",
    "/base_link/odom",
    "/amcl_pose",
    "/local_costmap/published_footprint",
    "/global_costmap/published_footprint",
    "/cmd_vel_nav",
    "/cmd_vel_smoothed",
    "/omnisteering/cmd_vel",
    "/collision_monitor_state",
]

TOPICS_FOR_FAST_INFO = [
    "/clock",
    "/tf",
    "/tf_static",
    "/map",
    "/scan",
    "/base_link/odom",
    "/amcl_pose",
    "/plan",
    "/transformed_global_plan",
    "/optimal_trajectory",
    "/local_costmap/costmap",
    "/local_costmap/published_footprint",
    "/global_costmap/costmap",
    "/global_costmap/published_footprint",
    "/cmd_vel_nav",
    "/cmd_vel_smoothed",
    "/omnisteering/cmd_vel",
    "/collision_monitor_state",
]

DEFAULT_BAG_TOPICS = [
    "/clock",
    "/tf",
    "/tf_static",
    "/map",
    "/map_updates",
    "/scan",
    "/VLP16_lidar_front/scan",
    "/VLP16_lidar_back/scan",
    "/base_link/odom",
    "/amcl_pose",
    "/particle_cloud",
    "/plan",
    "/unsmoothed_plan",
    "/transformed_global_plan",
    "/optimal_trajectory",
    "/trajectories",
    "/behavior_tree_log",
    "/rosout",
    "/diagnostics",
    "/local_costmap/costmap",
    "/local_costmap/costmap_raw",
    "/local_costmap/costmap_updates",
    "/local_costmap/costmap_raw_updates",
    "/local_costmap/footprint",
    "/local_costmap/obstacle_layer",
    "/local_costmap/published_footprint",
    "/global_costmap/costmap",
    "/global_costmap/costmap_raw",
    "/global_costmap/costmap_updates",
    "/global_costmap/costmap_raw_updates",
    "/global_costmap/footprint",
    "/global_costmap/obstacle_layer",
    "/global_costmap/static_layer",
    "/global_costmap/published_footprint",
    "/cmd_vel_nav",
    "/cmd_vel_smoothed",
    "/omnisteering/cmd_vel",
    "/collision_monitor_state",
    "/polygon_stop",
    "/polygon_slow",
    "/goal_pose",
    "/initialpose",
    "/speed_limit",
    "/navigate_to_pose/_action/status",
    "/navigate_to_pose/_action/feedback",
    "/compute_path_to_pose/_action/status",
    "/compute_path_to_pose/_action/feedback",
    "/follow_path/_action/status",
    "/follow_path/_action/feedback",
]

POINTCLOUD_TOPICS = [
    "/VLP16_lidar_front/points",
    "/VLP16_lidar_back/points",
    "/merged_cloud",
]

TF_PAIRS = [
    ("map", "odom"),
    ("map", "base_link_projected"),
    ("odom", "base_link"),
    ("odom", "base_link_projected"),
    ("base_link", "base_link_projected"),
    ("base_link_projected", "VLP16_lidar_front"),
    ("base_link_projected", "VLP16_lidar_back"),
    ("base_link_projected", "VLP16_lidar_front_base_link"),
    ("base_link_projected", "VLP16_lidar_back_base_link"),
]

ENV_KEYS = [
    "ROS_VERSION",
    "ROS_DISTRO",
    "ROS_DOMAIN_ID",
    "RMW_IMPLEMENTATION",
    "USE_SIM_TIME",
    "CYCLONEDDS_URI",
    "FASTRTPS_DEFAULT_PROFILES_FILE",
    "AMENT_PREFIX_PATH",
    "COLCON_PREFIX_PATH",
    "LD_LIBRARY_PATH",
    "PATH",
]


def sanitize(value: str) -> str:
    cleaned = value.strip().strip("/")
    if not cleaned:
        return "root"
    return "".join(c if c.isalnum() or c in "._-" else "_" for c in cleaned.replace("/", "__"))


def timestamp() -> str:
    return _dt.datetime.now().strftime("%Y%m%d_%H%M%S")


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8", errors="replace")


class Capture:
    def __init__(self, root: Path, dry_run: bool = False) -> None:
        self.root = root
        self.dry_run = dry_run
        self.counter = 0
        self.root.mkdir(parents=True, exist_ok=True)

    def path(self, group: str, name: str, suffix: str = ".txt") -> Path:
        self.counter += 1
        return self.root / group / f"{self.counter:03d}_{sanitize(name)}{suffix}"

    def run(
        self,
        group: str,
        name: str,
        cmd: list[str],
        timeout_s: float = 10.0,
        cwd: Path | None = None,
        env: dict[str, str] | None = None,
    ) -> subprocess.CompletedProcess[str] | None:
        out_path = self.path(group, name)
        header = [
            f"$ {' '.join(cmd)}",
            f"cwd: {cwd or Path.cwd()}",
            f"started_at: {_dt.datetime.now().isoformat(timespec='seconds')}",
            "",
        ]

        if self.dry_run:
            write_text(out_path, "\n".join(header + ["DRY RUN\n"]))
            return None

        started = time.monotonic()
        try:
            result = subprocess.run(
                cmd,
                cwd=str(cwd) if cwd else None,
                env=env,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                timeout=timeout_s,
                errors="replace",
            )
            elapsed = time.monotonic() - started
            body = [
                f"returncode: {result.returncode}",
                f"elapsed_s: {elapsed:.2f}",
                "",
                result.stdout or "",
            ]
            write_text(out_path, "\n".join(header + body))
            return result
        except subprocess.TimeoutExpired as exc:
            elapsed = time.monotonic() - started
            output = exc.stdout or ""
            if isinstance(output, bytes):
                output = output.decode("utf-8", errors="replace")
            body = [
                f"returncode: TIMEOUT",
                f"elapsed_s: {elapsed:.2f}",
                "",
                output,
            ]
            write_text(out_path, "\n".join(header + body))
            return None


def command_output(cmd: list[str], timeout_s: float = 10.0) -> tuple[int, str]:
    try:
        proc = subprocess.run(
            cmd,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=timeout_s,
            errors="replace",
        )
        return proc.returncode, proc.stdout or ""
    except subprocess.TimeoutExpired as exc:
        out = exc.stdout or ""
        if isinstance(out, bytes):
            out = out.decode("utf-8", errors="replace")
        return 124, out


def ros2_topic_list(include_hidden: bool = True) -> list[str]:
    cmd = ["ros2", "topic", "list"]
    if include_hidden:
        cmd.append("--include-hidden-topics")
    code, out = command_output(cmd, timeout_s=10)
    if code != 0 and include_hidden:
        code, out = command_output(["ros2", "topic", "list"], timeout_s=10)
    if code != 0:
        return []
    return sorted({line.strip() for line in out.splitlines() if line.strip().startswith("/")})


def ros2_node_list() -> list[str]:
    code, out = command_output(["ros2", "node", "list"], timeout_s=10)
    if code != 0:
        return []
    return sorted({line.strip() for line in out.splitlines() if line.strip().startswith("/")})


def ros2_pkg_share(package: str) -> Path | None:
    code, out = command_output(["ros2", "pkg", "prefix", "--share", package], timeout_s=10)
    if code != 0:
        return None
    path = Path(out.strip())
    return path if path.exists() else None


def copy_path(src: Path, dst: Path) -> None:
    if not src.exists():
        return
    dst.parent.mkdir(parents=True, exist_ok=True)
    if src.is_dir():
        shutil.copytree(src, dst, dirs_exist_ok=True, ignore=shutil.ignore_patterns("__pycache__"))
    else:
        shutil.copy2(src, dst)


def capture_installed_files(root: Path) -> None:
    files_root = root / "installed_files"
    packages = {
        "concert_navigation": ["launch", "config", "behavior_tree", "real_sim", "package.xml"],
        "concert_localization": ["launch", "config", "package.xml"],
        "concert_mapping": ["launch", "config", "maps", "package.xml"],
    }
    copied: list[str] = []
    missing: list[str] = []
    for package, rel_paths in packages.items():
        share = ros2_pkg_share(package)
        if share is None:
            missing.append(package)
            continue
        for rel in rel_paths:
            src = share / rel
            if src.exists():
                dst = files_root / package / rel
                copy_path(src, dst)
                copied.append(f"{src} -> {dst}")
    write_text(files_root / "COPY_SUMMARY.txt", "\n".join(["Copied:"] + copied + ["", "Missing packages:"] + missing))


def capture_git_state(capture: Capture) -> None:
    candidates = [
        Path("/home/user/xbot2_ws/src/concert_navigation"),
        Path("/home/user/xbot2_ws/src/concert_localization"),
        Path("/home/user/xbot2_ws/src/concert_mapping"),
    ]
    for repo in candidates:
        if not (repo / ".git").exists():
            continue
        label = repo.name
        capture.run("git", f"{label}_status", ["git", "status", "--short"], timeout_s=10, cwd=repo)
        capture.run("git", f"{label}_diff_stat", ["git", "diff", "--stat"], timeout_s=10, cwd=repo)
        capture.run("git", f"{label}_diff", ["git", "diff", "--", "."], timeout_s=20, cwd=repo)


def capture_ros_snapshot(
    capture: Capture,
    args: argparse.Namespace,
    topics: list[str] | None = None,
    nodes: list[str] | None = None,
) -> tuple[list[str], list[str]]:
    if topics is None:
        print("[nav2_debug_capture] snapshot: discovering topics", flush=True)
        topics = ros2_topic_list(include_hidden=True)
    if nodes is None:
        print("[nav2_debug_capture] snapshot: discovering nodes", flush=True)
        nodes = ros2_node_list()

    metadata = {
        "created_at": _dt.datetime.now().isoformat(timespec="seconds"),
        "hostname": socket.gethostname(),
        "cwd": str(Path.cwd()),
        "argv": sys.argv,
        "bag_enabled": args.bag,
        "bag_duration_s": args.duration,
        "env": {key: os.environ.get(key, "") for key in ENV_KEYS},
        "topics_seen": topics,
        "nodes_seen": nodes,
    }
    write_text(capture.root / "metadata.json", json.dumps(metadata, indent=2, sort_keys=True))

    print("[nav2_debug_capture] snapshot: ROS graph", flush=True)
    if args.quick:
        write_text(capture.root / "ros_graph" / "DOCTOR_SKIPPED.txt", "Skipped by --quick.\n")
    else:
        capture.run("ros_graph", "ros2_doctor_report", ["ros2", "doctor", "--report"], timeout_s=20)
    capture.run("ros_graph", "node_list", ["ros2", "node", "list"], timeout_s=10)
    capture.run("ros_graph", "topic_list_t", ["ros2", "topic", "list", "-t", "--include-hidden-topics"], timeout_s=10)
    capture.run("ros_graph", "service_list_t", ["ros2", "service", "list", "-t"], timeout_s=10)
    capture.run("ros_graph", "action_list_t", ["ros2", "action", "list", "-t"], timeout_s=10)

    if args.quick:
        write_text(capture.root / "actions" / "ACTION_INFO_SKIPPED.txt", "Skipped by --quick. Action status/feedback topics are recorded in the bag.\n")
    else:
        print("[nav2_debug_capture] snapshot: actions", flush=True)
        for action in ["/navigate_to_pose", "/compute_path_to_pose", "/follow_path"]:
            capture.run("actions", f"action_info_{action}", ["ros2", "action", "info", action], timeout_s=10)

    interesting_topics = sorted(set(TOPICS_FOR_INFO + args.extra_topic))
    existing_topics = set(topics)
    missing_topics = [topic for topic in interesting_topics if topic not in existing_topics]
    write_text(capture.root / "topics" / "MISSING_TOPICS.txt", "\n".join(missing_topics) + "\n")

    if args.quick:
        write_text(capture.root / "topics" / "TOPIC_INFO_SKIPPED.txt", "Skipped by --quick. See ros_graph/topic_list_t for topic types.\n")
    elif args.full_topic_info:
        print("[nav2_debug_capture] snapshot: full topic info", flush=True)
        for topic in interesting_topics:
            if topic in existing_topics:
                capture.run("topics", f"info_{topic}", ["ros2", "topic", "info", "-v", topic], timeout_s=5)
    else:
        print("[nav2_debug_capture] snapshot: fast topic info", flush=True)
        for topic in TOPICS_FOR_FAST_INFO:
            if topic in existing_topics:
                capture.run("topics", f"info_{topic}", ["ros2", "topic", "info", topic], timeout_s=3)

    hz_window = max(2.0, float(args.hz_window))
    timeout_cmd = shutil.which("timeout")
    if args.quick:
        write_text(capture.root / "topic_hz" / "SKIPPED.txt", "Skipped by --quick.\n")
        write_text(capture.root / "topic_echo" / "SKIPPED.txt", "Skipped by --quick.\n")
    else:
        print(f"[nav2_debug_capture] snapshot: topic hz ({hz_window:.1f}s each)", flush=True)
        for topic in TOPICS_FOR_HZ:
            if topic not in existing_topics:
                continue
            if timeout_cmd:
                cmd = [timeout_cmd, "--signal=INT", f"{hz_window}s", "ros2", "topic", "hz", topic]
                capture.run("topic_hz", f"hz_{topic}", cmd, timeout_s=hz_window + 4)
            else:
                capture.run("topic_hz", f"hz_{topic}", ["ros2", "topic", "hz", topic], timeout_s=hz_window + 2)

        print("[nav2_debug_capture] snapshot: one-shot topic samples", flush=True)
        for topic in TOPICS_FOR_ECHO:
            if topic not in existing_topics:
                continue
            cmd = ["ros2", "topic", "echo", "--once", topic]
            if timeout_cmd:
                cmd = [timeout_cmd, "--signal=INT", "6s"] + cmd
            capture.run("topic_echo", f"echo_{topic}", cmd, timeout_s=8)

    param_nodes = sorted(set(NAV2_NODES + LOCALIZATION_NODES + PERCEPTION_NODES + args.extra_node))
    existing_nodes = set(nodes)
    missing_nodes = [node for node in param_nodes if node not in existing_nodes]
    write_text(capture.root / "params" / "MISSING_NODES.txt", "\n".join(missing_nodes) + "\n")

    if args.quick:
        write_text(capture.root / "params" / "PARAM_DUMPS_SKIPPED.txt", "Skipped by --quick. Installed YAML files are still copied.\n")
        write_text(capture.root / "lifecycle" / "LIFECYCLE_SKIPPED.txt", "Skipped by --quick.\n")
    else:
        print("[nav2_debug_capture] snapshot: params and lifecycle", flush=True)
        for node in param_nodes:
            if node not in existing_nodes:
                continue
            capture.run("params", f"param_dump_{node}", ["ros2", "param", "dump", node], timeout_s=5)
            capture.run("params", f"param_list_{node}", ["ros2", "param", "list", node], timeout_s=5)
            capture.run("lifecycle", f"lifecycle_get_{node}", ["ros2", "lifecycle", "get", node], timeout_s=3)

    tf_root = capture.root / "tf"
    tf_root.mkdir(parents=True, exist_ok=True)
    if args.quick:
        write_text(capture.root / "tf" / "TF_SNAPSHOT_SKIPPED.txt", "view_frames and tf2_echo samples skipped by --quick. The bag still records /tf and /tf_static.\n")
    else:
        print("[nav2_debug_capture] snapshot: TF", flush=True)
        capture.run("tf", "view_frames", ["ros2", "run", "tf2_tools", "view_frames"], timeout_s=12, cwd=tf_root)
    if not args.quick and timeout_cmd:
        for parent, child in TF_PAIRS:
            cmd = [timeout_cmd, "--signal=INT", "3s", "ros2", "run", "tf2_ros", "tf2_echo", parent, child]
            capture.run("tf", f"tf2_echo_{parent}_to_{child}", cmd, timeout_s=6)

    print("[nav2_debug_capture] snapshot: installed files", flush=True)
    capture_installed_files(capture.root)
    if args.quick:
        write_text(capture.root / "git" / "GIT_DIFF_SKIPPED.txt", "Skipped by --quick.\n")
    else:
        print("[nav2_debug_capture] snapshot: git state", flush=True)
        capture_git_state(capture)
    return topics, nodes


def build_bag_command(
    capture: Capture,
    args: argparse.Namespace,
    topics_seen: Iterable[str],
) -> tuple[Path, Path, list[str] | None]:
    bag_root = capture.root / "bag"
    bag_root.mkdir(parents=True, exist_ok=True)
    log_path = bag_root / "rosbag_record.log"
    topics_seen_set = set(topics_seen)

    if args.bag_all_topics:
        selected_topics: list[str] = []
        cmd = ["ros2", "bag", "record", "-a", "--include-hidden-topics", "-o", str(bag_root / "nav2_debug_bag")]
    else:
        requested = list(DEFAULT_BAG_TOPICS) + list(args.extra_topic)
        if args.with_pointclouds:
            requested += POINTCLOUD_TOPICS
        selected_topics = sorted({topic for topic in requested if topic in topics_seen_set})
        skipped = sorted({topic for topic in requested if topic not in topics_seen_set})
        write_text(
            bag_root / "BAG_TOPICS.txt",
            "\n".join(["Recorded topics:"] + selected_topics + ["", "Skipped missing topics:"] + skipped) + "\n",
        )
        if not selected_topics:
            write_text(log_path, "No requested bag topics are currently available. Bag was not recorded.\n")
            return bag_root, log_path, None
        cmd = [
            "ros2",
            "bag",
            "record",
            "--include-hidden-topics",
            "-o",
            str(bag_root / "nav2_debug_bag"),
        ] + selected_topics

    return bag_root, log_path, cmd


def start_bag_recording(
    capture: Capture,
    args: argparse.Namespace,
    topics_seen: Iterable[str],
) -> tuple[subprocess.Popen[str] | None, Path, float | None]:
    bag_root, log_path, cmd = build_bag_command(capture, args, topics_seen)
    if cmd is None:
        print("[nav2_debug_capture] bag: no matching topics; skipping bag", flush=True)
        return None, bag_root, None

    if args.dry_run:
        write_text(log_path, "$ " + " ".join(cmd) + "\nDRY RUN\n")
        print("[nav2_debug_capture] bag: dry-run command written", flush=True)
        return None, bag_root, None

    print(
        f"[nav2_debug_capture] bag: recording started for at least {args.duration:.1f}s -> {bag_root / 'nav2_debug_bag'}",
        flush=True,
    )
    log = log_path.open("w", encoding="utf-8", errors="replace")
    log.write("$ " + " ".join(cmd) + "\n\n")
    log.flush()
    proc = subprocess.Popen(
        cmd,
        stdout=log,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    # Keep the file handle alive by attaching it to the process object. It is
    # closed in stop_bag_recording after rosbag has flushed its metadata.
    proc._concert_log_handle = log  # type: ignore[attr-defined]
    return proc, bag_root, time.monotonic()


def stop_bag_recording(capture: Capture, proc: subprocess.Popen[str] | None, bag_root: Path) -> None:
    if proc is None:
        return

    previous_sigint = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    try:
        if proc.poll() is None:
            print("[nav2_debug_capture] bag: stopping rosbag and flushing metadata", flush=True)
            try:
                os.killpg(proc.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                proc.wait(timeout=20)
            except subprocess.TimeoutExpired:
                print("[nav2_debug_capture] bag: SIGINT timed out, sending SIGTERM", flush=True)
                try:
                    os.killpg(proc.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                try:
                    proc.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    print("[nav2_debug_capture] bag: SIGTERM timed out, sending SIGKILL", flush=True)
                    try:
                        os.killpg(proc.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    proc.wait(timeout=5)
    finally:
        signal.signal(signal.SIGINT, previous_sigint)

    log = getattr(proc, "_concert_log_handle", None)
    if log is not None:
        log.close()

    capture.run("bag", "rosbag_info", ["ros2", "bag", "info", str(bag_root / "nav2_debug_bag")], timeout_s=20)


def write_usage_notes(root: Path) -> None:
    notes = """Concert Nav2 debug capture

Recommended workflow:
1. Start Gazebo/odometry/lidar/localization/Nav2.
2. Run this capture script before sending a Nav2 goal.
3. Send the goal that shows the bad planner behavior while the bag is recording.
4. Share the whole output directory.

The bundle contains:
- ROS graph, topic/service/action lists.
- Topic rates and one-shot samples for scan, odom, commands, footprints, AMCL.
- Nav2/localization/perception parameter dumps.
- Lifecycle states.
- TF echoes and a tf2_tools view_frames artifact when available.
- Installed launch/config/map/BT files used by the current environment.
- A targeted rosbag unless --no-bag was passed.

The default bag intentionally skips raw pointclouds to stay small. Use
--with-pointclouds if the failure looks like a perception/scan conversion issue.
"""
    write_text(root / "README_CAPTURE.txt", notes)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture logs, params, TF, costmaps and a targeted rosbag for Concert Nav2 debugging."
    )
    parser.add_argument(
        "-o",
        "--output-dir",
        default=str(Path.home() / "xbot2_ws" / "nav2_debug_captures"),
        help="Parent directory where a timestamped capture directory is created.",
    )
    parser.add_argument(
        "--name",
        default="",
        help="Optional suffix for the capture directory name, e.g. start_occupied or bad_turning.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=60.0,
        help="Bag recording duration in seconds. Default: 60.",
    )
    parser.add_argument(
        "--hz-window",
        type=float,
        default=2.0,
        help="Seconds to sample each ros2 topic hz command. Default: 2.",
    )
    parser.add_argument(
        "--quick",
        action="store_true",
        help="Skip slower topic hz, topic echo, param/lifecycle dumps, and tf2_echo snapshots; the bag still records normally.",
    )
    parser.add_argument(
        "--full-topic-info",
        action="store_true",
        help="Run ros2 topic info -v for all debug topics. Slower; default captures fast info for core topics only.",
    )
    parser.add_argument(
        "--bag",
        dest="bag",
        action="store_true",
        default=True,
        help="Record a targeted rosbag. This is the default.",
    )
    parser.add_argument(
        "--no-bag",
        dest="bag",
        action="store_false",
        help="Only collect snapshot logs; do not record a bag.",
    )
    parser.add_argument(
        "--bag-all-topics",
        action="store_true",
        help="Record every currently available topic instead of the focused Nav2 set.",
    )
    parser.add_argument(
        "--with-pointclouds",
        action="store_true",
        help="Add raw/merged pointcloud topics to the default bag topic set.",
    )
    parser.add_argument(
        "--extra-topic",
        action="append",
        default=[],
        help="Extra topic to inspect and record. Can be used multiple times.",
    )
    parser.add_argument(
        "--extra-node",
        action="append",
        default=[],
        help="Extra node to dump params/lifecycle for. Can be used multiple times.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Create the output structure and command files without running ROS commands.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if shutil.which("ros2") is None:
        print("ros2 was not found in PATH. Source /opt/ros/jazzy/setup.bash and ~/xbot2_ws/setup.bash first.", file=sys.stderr)
        return 2

    suffix = f"_{sanitize(args.name)}" if args.name else ""
    root = Path(args.output_dir).expanduser().resolve() / f"nav2_debug_{timestamp()}{suffix}"
    capture = Capture(root=root, dry_run=args.dry_run)

    print(f"[nav2_debug_capture] writing bundle to {root}", flush=True)
    write_usage_notes(root)

    print("[nav2_debug_capture] discovering current ROS graph", flush=True)
    topics = ros2_topic_list(include_hidden=True)
    nodes = ros2_node_list()

    bag_proc: subprocess.Popen[str] | None = None
    bag_root = capture.root / "bag"
    bag_started_at: float | None = None
    if args.bag:
        bag_proc, bag_root, bag_started_at = start_bag_recording(capture, args, topics)
    else:
        write_text(root / "bag" / "SKIPPED.txt", "Bag recording disabled with --no-bag.\n")

    interrupted = False
    try:
        capture_ros_snapshot(capture, args, topics=topics, nodes=nodes)

        if bag_proc is not None and bag_started_at is not None:
            elapsed = time.monotonic() - bag_started_at
            remaining = max(0.0, float(args.duration) - elapsed)
            if remaining > 0:
                print(f"[nav2_debug_capture] bag: snapshot done, recording {remaining:.1f}s more", flush=True)
                end_time = time.monotonic() + remaining
                while True:
                    wait_s = end_time - time.monotonic()
                    if wait_s <= 0:
                        break
                    sleep_s = min(5.0, wait_s)
                    time.sleep(sleep_s)
                    if wait_s > 5.0:
                        left = max(0.0, end_time - time.monotonic())
                        print(f"[nav2_debug_capture] bag: {left:.0f}s remaining", flush=True)
            stop_bag_recording(capture, bag_proc, bag_root)
    except KeyboardInterrupt:
        interrupted = True
        print("[nav2_debug_capture] interrupted: closing bag cleanly and keeping partial bundle", flush=True)
        stop_bag_recording(capture, bag_proc, bag_root)

    if interrupted:
        print(f"[nav2_debug_capture] partial bundle saved: {root}", flush=True)
        return 130

    print(f"[nav2_debug_capture] done: {root}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
