import os
import sys
import unittest


sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from formation_flight import (  # noqa: E402
    FormationPlan,
    WaypointProgressor,
    follower_targets_from_offsets,
    formation_errors,
    leader_reached_goal,
    validate_plan,
)


class FormationFlightTests(unittest.TestCase):
    def test_follower_targets_and_errors(self):
        leader = (10.0, 0.0, 6.0)
        offsets = [(2.0, 0.0, 0.0), (4.0, 0.0, 0.0)]
        targets = follower_targets_from_offsets(leader, offsets)

        self.assertEqual(targets, [(12.0, 0.0, 6.0), (14.0, 0.0, 6.0)])
        self.assertEqual(formation_errors(leader, targets, offsets), [0.0, 0.0])

    def test_validate_plan_rejects_tight_offsets(self):
        plan = FormationPlan(
            formation_name="bad",
            follower_offsets=[(0.1, 0.0, 0.0), (2.0, 0.0, 0.0)],
            leader_waypoints=[(0.0, 0.0, 6.0)],
        )

        with self.assertRaises(ValueError):
            validate_plan(plan, min_separation_m=0.9)

    def test_non_looping_progressor_holds_final_waypoint(self):
        progressor = WaypointProgressor(
            [(0.0, 0.0, 6.0), (4.0, 0.0, 6.0)],
            loop=False,
            arrival_radius_m=0.5,
            fallback_ticks_per_waypoint=100,
        )

        progressor.tick((0.0, 0.0, 6.0))
        self.assertEqual(progressor.index, 1)
        progressor.tick((4.0, 0.0, 6.0))
        self.assertEqual(progressor.index, 1)

    def test_leader_reached_goal(self):
        self.assertTrue(
            leader_reached_goal((39.0, 0.0, 6.0), (40.0, 0.0, 6.0), tolerance_m=2.5)
        )
        self.assertFalse(
            leader_reached_goal((34.0, 0.0, 6.0), (40.0, 0.0, 6.0), tolerance_m=2.5)
        )


if __name__ == "__main__":
    unittest.main()
