import csv
import tempfile
import unittest
from pathlib import Path

from simulation import TELEMETRY_FIELDS
from tools.validate_telemetry import validate_csv


class TelemetrySchemaTests(unittest.TestCase):
    def write_rows(self, rows):
        temp = tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parents[1] / "results")
        self.addCleanup(temp.cleanup)
        path = Path(temp.name) / "telemetry.csv"
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(TELEMETRY_FIELDS)
            writer.writerows(rows)
        return path

    def valid_row(self, time_s=0.1):
        row = [0] * len(TELEMETRY_FIELDS)
        values = {"time_s": time_s, "address": 1, "phase": "APPROACH", "tag_id": -1,
                  "pad_contact": 0, "tracker_update_accepted": 0}
        for name, value in values.items():
            row[TELEMETRY_FIELDS.index(name)] = value
        return row

    def test_valid_finite_monotonic_telemetry_passes(self):
        report = validate_csv(self.write_rows([self.valid_row(0.1), self.valid_row(0.2)]))
        self.assertTrue(report["valid"], report["errors"])
        self.assertEqual(report["rows"], 2)

    def test_nan_and_bad_boolean_fail(self):
        row = self.valid_row()
        row[TELEMETRY_FIELDS.index("x_m")] = "nan"
        row[TELEMETRY_FIELDS.index("address")] = "inf"
        row[TELEMETRY_FIELDS.index("pad_contact")] = 2
        report = validate_csv(self.write_rows([row]))
        self.assertFalse(report["valid"])
        self.assertTrue(any("not finite" in error for error in report["errors"]))
        self.assertTrue(any("must be 0 or 1" in error for error in report["errors"]))

    def test_backwards_time_fails(self):
        report = validate_csv(self.write_rows([self.valid_row(0.2), self.valid_row(0.1)]))
        self.assertFalse(report["valid"])
        self.assertTrue(any("moved backwards" in error for error in report["errors"]))


if __name__ == "__main__":
    unittest.main()
