"""Compare a candidate batch summary with an explicitly recorded baseline."""
from __future__ import annotations

import argparse
import json
from pathlib import Path


DEFAULT_TOLERANCES = {
    "success_rate_drop": 0.05,
    "median_error_increase_m": 0.20,
    "p95_error_increase_m": 0.50,
    "decoded_fraction_drop": 0.10,
}


def _scenario(summary: dict, name: str) -> dict:
    if "scenarios" in summary:
        scenarios = summary["scenarios"]
    elif "scenario" in summary:
        scenarios = {summary["scenario"]: summary}
    else:
        scenarios = {k.removesuffix("_summary"): v for k, v in summary.items()
                     if isinstance(v, dict) and "trial_results" in v}
    if name not in scenarios:
        raise ValueError(f"scenario {name!r} not found in summary")
    return scenarios[name]


def _decoded_fraction(summary: dict) -> float | None:
    trials = summary.get("trial_results", [])
    vals = [float(t["decoded_fraction"]) for t in trials if t.get("decoded_fraction") is not None]
    return sum(vals) / len(vals) if vals else None


def compare(baseline: dict, candidate: dict, scenario: str,
            tolerances: dict | None = None) -> dict:
    limits = {**DEFAULT_TOLERANCES, **(tolerances or {})}
    base, cand = _scenario(baseline, scenario), _scenario(candidate, scenario)
    checks = []

    def check(label: str, b: float | None, c: float | None, allowed: float, *, direction: str):
        if b is None or c is None:
            checks.append({"metric": label, "status": "SKIP", "baseline": b, "candidate": c,
                           "reason": "metric unavailable in one or both summaries"})
            return
        if direction == "lower":
            passed = c >= b - allowed
            change = c - b
        else:
            passed = c <= b + allowed
            change = c - b
        checks.append({"metric": label, "status": "PASS" if passed else "FAIL",
                       "baseline": b, "candidate": c, "change": change,
                       "allowed_degradation": allowed})

    check("success_rate", base.get("success_rate"), cand.get("success_rate"),
          limits["success_rate_drop"], direction="lower")
    check("median_final_horizontal_error_m", base.get("median_final_horizontal_error_m"),
          cand.get("median_final_horizontal_error_m"), limits["median_error_increase_m"], direction="higher")
    check("p95_final_horizontal_error_m", base.get("p95_final_horizontal_error_m"),
          cand.get("p95_final_horizontal_error_m"), limits["p95_error_increase_m"], direction="higher")
    check("decoded_fraction", _decoded_fraction(base), _decoded_fraction(cand),
          limits["decoded_fraction_drop"], direction="lower")
    if not any(item["status"] != "SKIP" for item in checks):
        raise ValueError("no comparable metrics were available")
    return {"schema_version": 1, "scenario": scenario,
            "passed": all(item["status"] != "FAIL" for item in checks),
            "baseline_trial_count": base.get("trial_count"),
            "candidate_trial_count": cand.get("trial_count"),
            "tolerances": limits, "checks": checks}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", required=True, type=Path, help="frozen batch_summary.json")
    parser.add_argument("--candidate", required=True, type=Path, help="new batch_summary.json")
    parser.add_argument("--scenario", required=True, help="scenario key in both summaries")
    parser.add_argument("--tolerances", type=Path, help="optional JSON tolerance overrides")
    parser.add_argument("--json", type=Path, help="write comparison report")
    args = parser.parse_args()
    try:
        tolerance_data = json.loads(args.tolerances.read_text(encoding="utf-8")) if args.tolerances else None
        report = compare(json.loads(args.baseline.read_text(encoding="utf-8")),
                         json.loads(args.candidate.read_text(encoding="utf-8")),
                         args.scenario, tolerance_data)
    except (OSError, json.JSONDecodeError, ValueError) as exc:
        parser.error(str(exc))
    rendered = json.dumps(report, indent=2)
    print(rendered)
    if args.json:
        args.json.write_text(rendered + "\n", encoding="utf-8")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
