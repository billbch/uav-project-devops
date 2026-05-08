import csv
import os
import sys
import tempfile
import unittest


sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from metrics import MetricThresholds, POSE_CSV_FIELDS, analyze_pose_csv  # noqa: E402


def _row(t, leader_x, f1_x, f2_x, *, y=0.0, z=6.0):
    row = {field: "0" for field in POSE_CSV_FIELDS}
    row.update(
        {
            "t_sec": f"{t:.3f}",
            "motion": "1",
            "wp_idx": "1",
            "avoid": "0",
            "L_cmd_x": "40",
            "L_cmd_y": "0",
            "L_cmd_z": "6",
            "F1_cmd_x": "42",
            "F1_cmd_y": "0",
            "F1_cmd_z": "6",
            "F2_cmd_x": "44",
            "F2_cmd_y": "0",
            "F2_cmd_z": "6",
            "F1_nom_x": "42",
            "F1_nom_y": "0",
            "F1_nom_z": "6",
            "F2_nom_x": "44",
            "F2_nom_y": "0",
            "F2_nom_z": "6",
            "L_odom_x": f"{leader_x:.3f}",
            "L_odom_y": f"{y:.3f}",
            "L_odom_z": f"{z:.3f}",
            "F1_odom_x": f"{f1_x:.3f}",
            "F1_odom_y": f"{y:.3f}",
            "F1_odom_z": f"{z:.3f}",
            "F2_odom_x": f"{f2_x:.3f}",
            "F2_odom_y": f"{y:.3f}",
            "F2_odom_z": f"{z:.3f}",
        }
    )
    return row


class MetricsTests(unittest.TestCase):
    def _write_csv(self, rows):
        tmp = tempfile.NamedTemporaryFile("w", delete=False, newline="", encoding="utf-8")
        with tmp:
            writer = csv.DictWriter(tmp, fieldnames=POSE_CSV_FIELDS)
            writer.writeheader()
            writer.writerows(rows)
        self.addCleanup(lambda: os.path.exists(tmp.name) and os.unlink(tmp.name))
        return tmp.name

    def test_successful_corridor_summary(self):
        path = self._write_csv(
            [
                _row(0.0, 4.0, 6.0, 8.0),
                _row(1.0, 20.0, 22.0, 24.0),
                _row(2.0, 39.0, 41.0, 43.0),
            ]
        )

        summary = analyze_pose_csv(path, mode="centralized", thresholds=MetricThresholds())

        self.assertTrue(summary.mission_success)
        self.assertAlmostEqual(summary.mission_time_sec, 2.0)
        self.assertAlmostEqual(summary.formation_error_max_m, 0.0)

    def test_corridor_violation_fails(self):
        path = self._write_csv(
            [
                _row(0.0, 4.0, 6.0, 8.0),
                _row(1.0, 20.0, 22.0, 24.0, y=3.5),
                _row(2.0, 39.0, 41.0, 43.0),
            ]
        )

        summary = analyze_pose_csv(path, mode="decentralized", thresholds=MetricThresholds())

        self.assertFalse(summary.mission_success)
        self.assertTrue(summary.corridor_violation)
        self.assertIn("corridor_or_obstacle_safety_violation", summary.failure_reasons)


if __name__ == "__main__":
    unittest.main()
