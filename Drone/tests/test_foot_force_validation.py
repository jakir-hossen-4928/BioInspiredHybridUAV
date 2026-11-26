"""Replay the retained Webots force trace through the live acceptance checks."""

import json
from pathlib import Path
import sys
import unittest


DRONE = Path(__file__).resolve().parents[1]
EVIDENCE = DRONE / "docs" / "evidence" / "foot-force"
sys.path.insert(0, str(DRONE / "validation" / "controllers" / "foot_force_validation"))

from foot_force_validation import measure, read_samples


class FootForceValidationTests(unittest.TestCase):
    def test_recorded_force_trace_matches_live_results(self):
        samples = read_samples(EVIDENCE / "foot_force.csv.gz")
        expected = json.loads((EVIDENCE / "foot_force_summary.json").read_text(encoding="utf-8"))
        self.assertTrue(expected["passed"])
        self.assertEqual(measure(samples), expected)

    def test_missing_stationary_foot_load_fails(self):
        samples = read_samples(EVIDENCE / "foot_force.csv.gz")
        for row in samples:
            if 1.0 <= row["time_s"] < 1.9:
                row["FL_world_z_n"] = 0.0
        result = measure(samples)
        self.assertFalse(result["passed"])
        self.assertFalse(result["checks"]["FL_standing_mean_n"]["pass"])
        self.assertFalse(result["checks"]["standing_total_mean_n"]["pass"])
