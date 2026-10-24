# Changelog

## 2026-10-23 — telemetry validation and smoke regression guard

- Added a CSV telemetry validator for header/order, widths, finite values,
  integer/boolean constraints, monotonic timestamps, and single-address runs.
- Added unit coverage for valid/invalid telemetry and baseline comparisons.
- Added a batch-summary regression guard with explicit tolerances for success
  rate, median/p95 horizontal error, and decoded-frame fraction.
- Captured a two-trial, 5-second static-ground smoke baseline and repeated it
  with the same seeds; both ended in APPROACH. This is a short-run plumbing
  baseline, not a landing-performance benchmark.
- Verification: 20 unit tests pass; telemetry validation and same-seed smoke
  comparison recorded in `docs/VALIDATION.md`.

## 2026-10-22 — audit and upgrade in progress

- Added the baseline system audit and ordered upgrade/acceptance plan.
- Pinned runtime and development dependencies; added Windows setup and environment doctor commands.
- Added per-run manifests, seeded simulation RNG, fixed-step DIRECT setup, and repeat-run determinism check.
- Fixed telemetry field indexing in evaluation and plotting; added baseline math regression tests.
- Replaced the heavy single-thrust approximation with a Crazyflie-class rigid body, rotor-level thrust/reaction torque, first-order motor lag, mixer allocation, saturation handling, and documented provisional parameters.
- Tuned visual scale and contact settings for the small vehicle; preserved pre-existing result files and redirected smoke output to unique timestamped files.
- Added per-step physics/control/camera timing summaries, configurable camera size/rate, selected-tag altitude checks, shared scenario fixtures, parallel seeded batches with Wilson intervals, trial metrics, and HTML/PDF report output.
- Added a covariance-aware Kalman target tracker with Mahalanobis gating, bounded moving-target intercept, and innovation/covariance telemetry.
- Replaced procedural black-cell tag meshes with flat UV-mapped tag36h11 textures bound through PyBullet `changeVisualShape`; all ten selected IDs decode in the fixed 4 m DIRECT check.
- Verification so far: 14 unit/integration tests pass; `selftest.py`, doctor (DIRECT and GUI clients), compileall, determinism (36 samples match bitwise on this host), ten-address 4 m tag check, address 1 altitude checks (2–10 m), 600-step telemetry smoke with tracker diagnostics, step-response plots, serial and multiprocessing batch smoke runs, and HTML/PDF report generation. The full acceptance benchmark and controller targets remain unmet/unverified; see `AUDIT.md` and `docs/VALIDATION.md`.
