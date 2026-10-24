"""Compare two same-seed DIRECT runs within one installed software stack."""
from __future__ import annotations

import numpy as np

from simulation import LandingSimulation


def run(seed: int = 2026, steps: int = 36) -> tuple[np.ndarray, list[list[int]]]:
    sim = LandingSimulation(gui=False, seed=seed)
    sim.select(1)
    states = []
    tags = []
    try:
        for _ in range(steps):
            snap = sim.step()
            states.append(np.concatenate((snap["position"], snap["orientation"], snap["velocity"])))
            tags.append([int(d.tag_id) for d in snap["detections"]])
        return np.asarray(states), tags
    finally:
        sim.close()


def main() -> int:
    first, first_tags = run()
    second, second_tags = run()
    max_difference = float(np.max(np.abs(first - second)))
    identical = np.array_equal(first, second) and first_tags == second_tags
    print(f"same_seed_bitwise_match={identical}")
    print(f"max_state_difference={max_difference:.17g}")
    print(f"samples={len(first)} camera_tag_samples={sum(bool(tags) for tags in first_tags)}")
    return 0 if identical else 1


if __name__ == "__main__":
    raise SystemExit(main())
