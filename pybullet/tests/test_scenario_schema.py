import json
from pathlib import Path
import unittest


COMMON = Path(__file__).resolve().parents[2] / "scenarios" / "common"


class ScenarioSchemaTests(unittest.TestCase):
    def test_scenario_fixtures_match_shared_schema(self):
        schema=json.loads((COMMON/"schema.yaml").read_text(encoding="utf-8-sig"))
        files=[path for path in COMMON.glob("*.yaml") if path.name!="schema.yaml"]
        self.assertGreaterEqual(len(files),2)
        for path in files:
            with self.subTest(scenario=path.name):
                scenario=json.loads(path.read_text(encoding="utf-8-sig"))
                self.assertEqual(scenario["schema_version"],schema["schema_version"])
                self.assertFalse(set(schema["required_scenario_fields"])-scenario.keys())
                self.assertIn(scenario["target"]["type"],("ground","vehicle","roof"))
                self.assertEqual(len(scenario["start_pose"]["position_m"]),3)
                from tools.batch import _validate_supported_scenario
                _validate_supported_scenario(path.name,scenario)

    def test_schema_lists_common_trial_metrics(self):
        schema=json.loads((COMMON/"schema.yaml").read_text(encoding="utf-8-sig"))
        self.assertIn("success",schema["shared_trial_metrics"])
        self.assertIn("energy_j",schema["shared_trial_metrics"])
        self.assertIn("failsafe_events",schema["shared_trial_metrics"])


if __name__=="__main__":
    unittest.main()
