import unittest

from tools.regression_guard import compare


def batch(success=0.5, median=0.4, p95=0.9, decoded=0.8):
    return {"scenarios": {"static_ground": {"trial_count": 4, "success_rate": success,
            "median_final_horizontal_error_m": median, "p95_final_horizontal_error_m": p95,
            "trial_results": [{"decoded_fraction": decoded}]}}}


class RegressionGuardTests(unittest.TestCase):
    def test_candidate_within_tolerances_passes(self):
        result = compare(batch(), batch(success=0.48, median=0.5, p95=1.1, decoded=0.75), "static_ground")
        self.assertTrue(result["passed"])

    def test_success_drop_beyond_tolerance_fails(self):
        result = compare(batch(), batch(success=0.4), "static_ground")
        self.assertFalse(result["passed"])

    def test_missing_scenario_is_rejected(self):
        with self.assertRaises(ValueError):
            compare(batch(), batch(), "moving_medium")


if __name__ == "__main__":
    unittest.main()
