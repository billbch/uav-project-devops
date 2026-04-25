import argparse
import csv
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import List, Optional, Tuple

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from px4_msgs.msg import VehicleOdometry

from formation_flight import WaypointProgressor, follower_targets_from_offsets, formation_errors
from llm_agent import FormationNode
from metrics import (
    MetricThresholds,
    POSE_CSV_FIELDS,
    analyze_pose_csv,
    write_summary,
)
from scenario import (
    CORRIDOR_WAYPOINTS,
    LINE_FOLLOWER_OFFSETS,
    PX4_NAMESPACES,
    Vec3,
    all_finite,
    distance,
    is_in_corridor_safe_region,
    vehicle_local_to_alliance,
)


class EvaluationNode(Node):
    def __init__(
        self,
        *,
        mode: str,
        pose_csv_path: str,
        central_node: Optional[FormationNode],
        thresholds: MetricThresholds,
        stop_after_success_sec: float,
    ):
        super().__init__("uav_evaluation_node")
        self.mode = mode
        self.pose_csv_path = pose_csv_path
        self.central_node = central_node
        self.thresholds = thresholds
        self.stop_after_success_sec = float(stop_after_success_sec)
        self.positions: List[Optional[Vec3]] = [None, None, None]
        self.start_time = time.monotonic()
        self.success_time: Optional[float] = None
        self._progressor = WaypointProgressor(
            CORRIDOR_WAYPOINTS,
            loop=False,
            arrival_radius_m=thresholds.goal_tolerance_m,
            fallback_ticks_per_waypoint=1_000_000,
        )
        self._csv_fh = open(self.pose_csv_path, "w", encoding="utf-8", newline="")
        self._writer = csv.DictWriter(self._csv_fh, fieldnames=POSE_CSV_FIELDS)
        self._writer.writeheader()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        for i, ns in enumerate(PX4_NAMESPACES):
            self.create_subscription(
                VehicleOdometry,
                f"{ns}/fmu/out/vehicle_odometry",
                lambda msg, idx=i: self._on_odom(idx, msg),
                qos,
            )
        self.timer = self.create_timer(0.5, self._record_sample)

    def close(self) -> None:
        try:
            self._csv_fh.flush()
            self._csv_fh.close()
        except Exception:
            pass

    def _on_odom(self, vehicle_index: int, msg: VehicleOdometry) -> None:
        try:
            loc = (float(msg.position[0]), float(msg.position[1]), float(msg.position[2]))
            self.positions[vehicle_index] = vehicle_local_to_alliance(vehicle_index, loc)
        except Exception:
            return

    def _current_target(self) -> Tuple[int, Vec3]:
        if self.central_node is not None and getattr(self.central_node, "_progressor", None) is not None:
            progressor = self.central_node._progressor
            return int(progressor.index), progressor.current_target

        leader = self.positions[0]
        if leader is not None:
            self._progressor.tick(leader)
        return int(self._progressor.index), self._progressor.current_target

    def _record_sample(self) -> None:
        wp_idx, leader_target = self._current_target()
        follower_nominals = follower_targets_from_offsets(leader_target, LINE_FOLLOWER_OFFSETS)
        targets = [leader_target, *follower_nominals]
        t_sec = time.monotonic() - self.start_time

        def vals(pos: Optional[Vec3]) -> Tuple[float, float, float]:
            return pos if pos is not None else (float("nan"), float("nan"), float("nan"))

        row = {
            "t_sec": f"{t_sec:.6f}",
            "motion": "1",
            "wp_idx": str(wp_idx),
            "avoid": "0",
        }
        for prefix, pos in (
            ("L_cmd", targets[0]),
            ("F1_cmd", targets[1]),
            ("F2_cmd", targets[2]),
            ("F1_nom", follower_nominals[0]),
            ("F2_nom", follower_nominals[1]),
            ("L_odom", vals(self.positions[0])),
            ("F1_odom", vals(self.positions[1])),
            ("F2_odom", vals(self.positions[2])),
        ):
            row[f"{prefix}_x"] = f"{pos[0]:.6f}"
            row[f"{prefix}_y"] = f"{pos[1]:.6f}"
            row[f"{prefix}_z"] = f"{pos[2]:.6f}"
        self._writer.writerow(row)
        self._csv_fh.flush()

        self._update_success_time()

    def _update_success_time(self) -> None:
        if self.success_time is not None:
            return
        leader, f1, f2 = self.positions
        if leader is None or f1 is None or f2 is None:
            return
        if not all_finite((*leader, *f1, *f2)):
            return
        errs = formation_errors(leader, (f1, f2), LINE_FOLLOWER_OFFSETS)
        safe = all(is_in_corridor_safe_region(pos) for pos in (leader, f1, f2))
        if (
            safe
            and max(errs) <= self.thresholds.max_formation_error_m
            and distance(leader, CORRIDOR_WAYPOINTS[-1]) <= self.thresholds.goal_tolerance_m
        ):
            self.success_time = time.monotonic()

    def should_stop(self, duration_limit_sec: float) -> bool:
        now = time.monotonic()
        if now - self.start_time >= duration_limit_sec:
            return True
        if self.success_time is not None and now - self.success_time >= self.stop_after_success_sec:
            return True
        return False


def _start_decentralized_processes() -> List[subprocess.Popen]:
    script = Path(__file__).with_name("decentralized_agent.py")
    procs: List[subprocess.Popen] = []
    for uav_id in (0, 1, 2):
        procs.append(
            subprocess.Popen(
                [sys.executable, str(script), "--uav-id", str(uav_id), "--scenario", "corridor"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.STDOUT,
            )
        )
    return procs


def _stop_processes(procs: List[subprocess.Popen]) -> None:
    for proc in procs:
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
    deadline = time.monotonic() + 5.0
    for proc in procs:
        remaining = max(0.1, deadline - time.monotonic())
        try:
            proc.wait(timeout=remaining)
        except subprocess.TimeoutExpired:
            proc.kill()


def main() -> int:
    parser = argparse.ArgumentParser(description="Run a centralized/decentralized corridor experiment.")
    parser.add_argument("--mode", choices=("centralized", "decentralized"), required=True)
    parser.add_argument("--scenario", choices=("corridor",), default="corridor")
    parser.add_argument("--duration-limit-sec", type=float, default=90.0)
    parser.add_argument("--log-dir", required=True)
    parser.add_argument("--goal-tolerance-m", type=float, default=2.5)
    parser.add_argument("--max-formation-error-m", type=float, default=2.0)
    parser.add_argument("--stale-timeout-sec", type=float, default=2.0)
    parser.add_argument("--stop-after-success-sec", type=float, default=3.0)
    parser.add_argument("--no-llm", action="store_true", help="Accepted for reproducibility; corridor trials do not call the LLM.")
    args = parser.parse_args()

    os.makedirs(args.log_dir, exist_ok=True)
    pose_csv = os.path.join(args.log_dir, "pose.csv")
    summary_json = os.path.join(args.log_dir, "summary.json")
    thresholds = MetricThresholds(
        goal_tolerance_m=args.goal_tolerance_m,
        max_formation_error_m=args.max_formation_error_m,
        stale_timeout_sec=args.stale_timeout_sec,
    )

    rclpy.init()
    central_node: Optional[FormationNode] = None
    eval_node: Optional[EvaluationNode] = None
    executor = MultiThreadedExecutor()
    procs: List[subprocess.Popen] = []

    try:
        if args.mode == "centralized":
            central_node = FormationNode()
            central_node.update_waypoints(
                list(CORRIDOR_WAYPOINTS),
                enable_motion=True,
                loop_waypoints=False,
                arrival_radius_m=args.goal_tolerance_m,
                fallback_ticks_per_waypoint=1_000_000,
            )
            executor.add_node(central_node)
        else:
            procs = _start_decentralized_processes()

        eval_node = EvaluationNode(
            mode=args.mode,
            pose_csv_path=pose_csv,
            central_node=central_node,
            thresholds=thresholds,
            stop_after_success_sec=args.stop_after_success_sec,
        )
        executor.add_node(eval_node)

        while rclpy.ok() and not eval_node.should_stop(args.duration_limit_sec):
            executor.spin_once(timeout_sec=0.1)

    finally:
        if eval_node is not None:
            eval_node.close()
            try:
                executor.remove_node(eval_node)
            except Exception:
                pass
            eval_node.destroy_node()
        if central_node is not None:
            try:
                executor.remove_node(central_node)
            except Exception:
                pass
            central_node.destroy_node()
        _stop_processes(procs)
        if rclpy.ok():
            rclpy.shutdown()

    summary = analyze_pose_csv(
        pose_csv,
        mode=args.mode,
        scenario=args.scenario,
        thresholds=thresholds,
    )
    write_summary(summary, summary_json)
    print(f"Wrote {pose_csv}")
    print(f"Wrote {summary_json}")
    print(f"mission_success={summary.mission_success}")
    return 0 if summary.mission_success else 2


if __name__ == "__main__":
    raise SystemExit(main())
