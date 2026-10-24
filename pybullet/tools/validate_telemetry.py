"""Validate PyBullet mission telemetry CSVs against the runtime schema."""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from simulation import TELEMETRY_FIELDS


INTEGER_FIELDS = {"address", "tag_id", "pad_contact", "tracker_update_accepted"}
TEXT_FIELDS = {"phase"}
BOOLEAN_FIELDS = {"pad_contact", "tracker_update_accepted"}


def validate_csv(path: str | Path) -> dict:
    path = Path(path)
    errors: list[str] = []
    row_count = 0
    last_time = -math.inf
    addresses: set[int] = set()
    try:
        with path.open("r", newline="", encoding="utf-8-sig") as handle:
            reader = csv.reader(handle)
            header = next(reader, None)
            if header != list(TELEMETRY_FIELDS):
                errors.append("header does not exactly match simulation.TELEMETRY_FIELDS")
                return {"path": str(path), "valid": False, "rows": 0, "errors": errors}
            for line_number, row in enumerate(reader, start=2):
                row_count += 1
                if len(row) != len(TELEMETRY_FIELDS):
                    errors.append(f"line {line_number}: expected {len(TELEMETRY_FIELDS)} fields, got {len(row)}")
                    continue
                values = dict(zip(TELEMETRY_FIELDS, row))
                parsed: dict[str, float] = {}
                for name, value in values.items():
                    if name in TEXT_FIELDS:
                        if not value:
                            errors.append(f"line {line_number}: {name} is empty")
                        continue
                    try:
                        number = float(value)
                    except ValueError:
                        errors.append(f"line {line_number}: {name} is not numeric")
                        continue
                    if not math.isfinite(number):
                        errors.append(f"line {line_number}: {name} is not finite")
                        continue
                    if name in INTEGER_FIELDS and number != int(number):
                        errors.append(f"line {line_number}: {name} must be an integer")
                    if name in BOOLEAN_FIELDS and number not in (0, 1):
                        errors.append(f"line {line_number}: {name} must be 0 or 1")
                    parsed[name] = number
                current_time = parsed.get("time_s")
                if current_time is not None:
                    if current_time < last_time:
                        errors.append(f"line {line_number}: time_s moved backwards")
                    last_time = current_time
                address = parsed.get("address")
                if address is not None:
                    addresses.add(int(address))
                tag_id = parsed.get("tag_id")
                if tag_id is not None and tag_id < -1:
                    errors.append(f"line {line_number}: tag_id must be -1 or non-negative")
            if len(addresses) > 1:
                errors.append("CSV contains more than one mission address")
    except (OSError, csv.Error) as exc:
        errors.append(f"could not read CSV: {exc}")
    return {"path": str(path), "valid": not errors, "rows": row_count, "errors": errors}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", nargs="+", type=Path, help="mission telemetry CSV file(s)")
    parser.add_argument("--json", type=Path, help="also write a JSON validation report")
    args = parser.parse_args()
    results = [validate_csv(path) for path in args.csv]
    report = {"schema_version": 1, "valid": all(item["valid"] for item in results), "files": results}
    rendered = json.dumps(report, indent=2)
    print(rendered)
    if args.json:
        args.json.write_text(rendered + "\n", encoding="utf-8")
    return 0 if report["valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
