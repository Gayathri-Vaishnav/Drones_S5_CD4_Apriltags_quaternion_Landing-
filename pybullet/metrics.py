"""Pure metric aggregation helpers used by DIRECT batch runs."""
from __future__ import annotations

import math
import statistics


def wilson_interval(successes: int, trials: int, z: float = 1.959963984540054):
    if trials <= 0:
        return [None, None]
    if not 0 <= successes <= trials:
        raise ValueError("successes must be between zero and trials")
    proportion = successes / trials
    denom = 1 + z*z/trials
    center = (proportion + z*z/(2*trials)) / denom
    radius = z * math.sqrt(proportion*(1-proportion)/trials + z*z/(4*trials*trials)) / denom
    return [max(0.0, center-radius), min(1.0, center+radius)]


def summarize_trials(trials):
    successes = [trial for trial in trials if trial.get("success", trial.get("landed", False))]
    errors = [float(trial["final_horizontal_error_m"]) for trial in trials
              if trial.get("final_horizontal_error_m") is not None]
    landed_errors = [float(trial["final_horizontal_error_m"]) for trial in successes
                     if trial.get("final_horizontal_error_m") is not None]
    p95=sorted(errors)[max(0,math.ceil(.95*len(errors))-1)] if errors else None
    return {"trial_count": len(trials), "landed_count": len(successes),
            "success_rate": len(successes)/len(trials) if trials else None,
            "success_rate_wilson_95": wilson_interval(len(successes),len(trials)),
            "median_final_horizontal_error_m": statistics.median(errors) if errors else None,
            "p95_final_horizontal_error_m": p95,
            "median_landed_error_m": statistics.median(landed_errors) if landed_errors else None,
            "trial_results": trials}
