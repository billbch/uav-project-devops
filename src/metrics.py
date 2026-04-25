import csv
import json
import math
import os
from dataclasses import dataclass, asdict
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from formation_flight import formation_errors
from scenario import (
    CORRIDOR_WAYPOINTS,
    DEFAULT_CRUISE_Z_ENU,
    LINE_FOLLOWER_OFFSETS,
    Vec3,
    all_finite,
    distance,
    is_in_corridor_safe_region,
)


POSE_CSV_FIELDS: Tuple[str, ...] = (
    "t_sec",
    "motion",
    "wp_idx",
    "avoid",
    "L_cmd_x",
    "L_cmd_y",
    "L_cmd_z",
    "F1_cmd_x",
    "F1_cmd_y",
    "F1_cmd_z",
    "F2_cmd_x",
    "F2_cmd_y",
    "F2_cmd_z",
    "F1_nom_x",
    "F1_nom_y",
    "F1_nom_z",
    "F2_nom_x",
    "F2_nom_y",
    "F2_nom_z",
    "L_odom_x",
    "L_odom_y",
    "L_odom_z",
    "F1_odom_x",
    "F1_odom_y",
    "F1_odom_z",
    "F2_odom_x",
    "F2_odom_y",
    "F2_odom_z",
)


@dataclass(frozen=True)
class MetricThresholds:
    goal_tolerance_m: float = 2.5
    max_formation_error_m: float = 2.0
    stale_timeout_sec: float = 2.0


@dataclass
class RunSummary:
    mode: str
    scenario: str
    mission_success: bool
    mission_time_sec: Optional[float]
    formation_error_avg_m: Optional[float]
    formation_error_max_m: Optional[float]
    samples: int
    motion_samples: int
    goal_reached: bool
    corridor_violation: bool
    stale_odometry: bool
    oscillation_detected: bool
    thresholds: Dict[str, float]
    failure_reasons: List[str]


def _wp_idx(row: Dict[str, str]) -> int:
    try:
        return int(float(row.get("wp_idx", "-1")))
    except ValueError:
        return -1


def _parse_float(row: Dict[str, str], key: str) -> float:
    try:
        return float(row.get(key, "nan"))
    except (TypeError, ValueError):
        return float("nan")


def _vec(row: Dict[str, str], prefix: str) -> Vec3:
    return (
        _parse_float(row, f"{prefix}_x"),
        _parse_float(row, f"{prefix}_y"),
        _parse_float(row, f"{prefix}_z"),
    )


def _is_motion(row: Dict[str, str]) -> bool:
    try:
        return int(float(row.get("motion", "0"))) == 1
    except ValueError:
        return False


def _row_positions(row: Dict[str, str]) -> Tuple[Vec3, Vec3, Vec3]:
    return _vec(row, "L_odom"), _vec(row, "F1_odom"), _vec(row, "F2_odom")


def row_formation_errors(row: Dict[str, str]) -> List[float]:
    leader, f1, f2 = _row_positions(row)
    if not all_finite((*leader, *f1, *f2)):
        return []
    return formation_errors(leader, (f1, f2), LINE_FOLLOWER_OFFSETS)


def row_corridor_violation(row: Dict[str, str]) -> bool:
    leader, f1, f2 = _row_positions(row)
    if not all_finite((*leader, *f1, *f2)):
        return False
    return not all(is_in_corridor_safe_region(pos) for pos in (leader, f1, f2))


def row_ready_for_evaluation(row: Dict[str, str]) -> bool:
    leader, f1, f2 = _row_positions(row)
    if not all_finite((*leader, *f1, *f2)):
        return False
    if not all(abs(pos[2] - DEFAULT_CRUISE_Z_ENU) <= 0.75 for pos in (leader, f1, f2)):
        return False
    errors = formation_errors(leader, (f1, f2), LINE_FOLLOWER_OFFSETS)
    return max(errors) <= 3.0


def _detect_stale_odometry(rows: Sequence[Dict[str, str]], stale_timeout_sec: float) -> bool:
    last_t: Optional[float] = None
    seen_valid_sample = False
    for row in rows:
        if not _is_motion(row):
            continue
        t = _parse_float(row, "t_sec")
        leader, f1, f2 = _row_positions(row)
        valid = math.isfinite(t) and all_finite((*leader, *f1, *f2))
        if not valid:
            if seen_valid_sample:
                return True
            continue
        seen_valid_sample = True
        if last_t is not None and t - last_t > stale_timeout_sec:
            return True
        last_t = t
    return not seen_valid_sample


def _detect_oscillation(rows: Sequence[Dict[str, str]]) -> bool:
    lateral_values: List[float] = []
    for row in rows:
        if not _is_motion(row):
            continue
        leader, _f1, _f2 = _row_positions(row)
        if math.isfinite(leader[1]):
            lateral_values.append(leader[1])
    if len(lateral_values) < 8:
        return False
    sign_changes = 0
    prev_sign = 0
    for value in lateral_values:
        if abs(value) < 0.15:
            continue
        sign = 1 if value > 0.0 else -1
        if prev_sign and sign != prev_sign:
            sign_changes += 1
        prev_sign = sign
    return sign_changes >= 6 and max(abs(v) for v in lateral_values) > 0.8


def _read_rows(csv_path: str) -> List[Dict[str, str]]:
    with open(csv_path, "r", encoding="utf-8", newline="") as fh:
        return list(csv.DictReader(fh))


def analyze_pose_csv(
    csv_path: str,
    *,
    mode: str,
    scenario: str = "corridor",
    thresholds: MetricThresholds = MetricThresholds(),
) -> RunSummary:
    rows = _read_rows(csv_path) if os.path.exists(csv_path) else []
    motion_rows = [row for row in rows if _is_motion(row)]
    mission_start_idx: Optional[int] = None
    for idx, row in enumerate(motion_rows):
        if _wp_idx(row) > 0:
            mission_start_idx = idx
            break

    eval_start_idx: Optional[int] = mission_start_idx
    for idx, row in enumerate(motion_rows):
        if mission_start_idx is not None and idx < mission_start_idx:
            continue
        if row_ready_for_evaluation(row):
            eval_start_idx = idx
            break
    evaluated_rows = motion_rows[eval_start_idx:] if eval_start_idx is not None else []
    errors: List[float] = []
    goal_reached_at: Optional[float] = None
    mission_start_t: Optional[float] = None
    corridor_violation = False

    if mission_start_idx is not None and mission_start_idx < len(motion_rows):
        t0 = _parse_float(motion_rows[mission_start_idx], "t_sec")
        if math.isfinite(t0):
            mission_start_t = t0

    final_goal = CORRIDOR_WAYPOINTS[-1]
    for row in evaluated_rows:
        t = _parse_float(row, "t_sec")

        row_errors = row_formation_errors(row)
        errors.extend(row_errors)

        if row_corridor_violation(row):
            corridor_violation = True

        leader, _f1, _f2 = _row_positions(row)
        if (
            goal_reached_at is None
            and all_finite(leader)
            and distance(leader, final_goal) <= thresholds.goal_tolerance_m
            and row_errors
            and max(row_errors) <= thresholds.max_formation_error_m
            and not corridor_violation
            and math.isfinite(t)
        ):
            goal_reached_at = t

    stale = _detect_stale_odometry(evaluated_rows, thresholds.stale_timeout_sec)
    oscillation = _detect_oscillation(evaluated_rows)
    avg_error = (sum(errors) / len(errors)) if errors else None
    max_error = max(errors) if errors else None
    goal_reached = goal_reached_at is not None

    failure_reasons: List[str] = []
    if mission_start_t is None:
        failure_reasons.append("mission_never_left_initial_waypoint")
    if not evaluated_rows:
        failure_reasons.append("no_stable_airborne_evaluation_window")
    if not goal_reached:
        failure_reasons.append("leader_did_not_reach_final_waypoint_with_valid_formation")
    if corridor_violation:
        failure_reasons.append("corridor_or_obstacle_safety_violation")
    if stale:
        failure_reasons.append("missing_or_stale_odometry")
    if max_error is None:
        failure_reasons.append("formation_error_unavailable")
    elif max_error > thresholds.max_formation_error_m:
        failure_reasons.append("formation_error_threshold_exceeded")
    if oscillation:
        failure_reasons.append("oscillation_detected")

    mission_success = not failure_reasons
    mission_time = (
        goal_reached_at - mission_start_t
        if goal_reached_at is not None and mission_start_t is not None
        else None
    )

    return RunSummary(
        mode=mode,
        scenario=scenario,
        mission_success=mission_success,
        mission_time_sec=mission_time,
        formation_error_avg_m=avg_error,
        formation_error_max_m=max_error,
        samples=len(rows),
        motion_samples=len(motion_rows),
        goal_reached=goal_reached,
        corridor_violation=corridor_violation,
        stale_odometry=stale,
        oscillation_detected=oscillation,
        thresholds=asdict(thresholds),
        failure_reasons=failure_reasons,
    )


def write_summary(summary: RunSummary, path: str) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(asdict(summary), fh, indent=2, sort_keys=True)
        fh.write("\n")
