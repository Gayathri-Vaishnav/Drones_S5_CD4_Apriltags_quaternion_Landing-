import unittest

from metrics import summarize_trials, wilson_interval


class MetricsTests(unittest.TestCase):
    def test_wilson_interval_boundaries(self):
        self.assertEqual(wilson_interval(0,0),[None,None])
        lo,hi=wilson_interval(10,10)
        self.assertGreaterEqual(lo,0.0)
        self.assertLessEqual(hi,1.0)
        self.assertLess(lo,1.0)

    def test_summary_reports_success_and_error_distribution(self):
        rows=[{"success":True,"final_horizontal_error_m":.02},
              {"success":False,"final_horizontal_error_m":.4}]
        result=summarize_trials(rows)
        self.assertEqual(result["trial_count"],2)
        self.assertEqual(result["landed_count"],1)
        self.assertEqual(result["median_landed_error_m"],.02)
        self.assertEqual(result["p95_final_horizontal_error_m"],.4)


if __name__=="__main__":
    unittest.main()
