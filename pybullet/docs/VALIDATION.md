# Validation record

This page distinguishes executable checks from mission-performance evidence. Current reproducible commands from the project directory:

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
.\.venv\Scripts\python.exe selftest.py
.\.venv\Scripts\python.exe doctor.py
.\.venv\Scripts\python.exe determinism_test.py
.\.venv\Scripts\python.exe smoke_test.py
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
.\.venv\Scripts\python.exe tools\validate_telemetry.py results\smoke_tests\smoke_20261022T184509Z.csv
```

Observed on 2026-10-22: 14 unit/integration tests pass; selftest passes package, tag decode, rendered tag/PnP and finite-state checks; doctor passes Python 3.12.14, PyBullet API/DIRECT, NumPy, OpenCV AprilTag, pupil-apriltags and generated asset checks. GUI and DIRECT clients both open. Hardware OpenGL captured a small test frame, but full-scene GUI tag validation stalled; this is not a full camera pass. Same-seed DIRECT runs match bitwise for 36 samples; flat textured tags decode at all ten selected pads from 4 m overhead. Address 1's tag also decodes from 2 through 10 m. The latest 600-step smoke produces 128 control telemetry rows at 5.0 simulated seconds and ends in APPROACH with 2.72 m horizontal error; it is not a landing test. Its peak speed is 2.43 m/s (2.04 m/s horizontal), still above the 2.0 m/s horizontal command limit. Tracker innovation/gate diagnostics are finite in the CSV. See `results/smoke_tests/smoke_20261022T184509Z.csv`.

A separate kinematic truck contact test maintained drone-pad contact over 2 simulated seconds while the truck traveled 0.966 m; relative horizontal drift was 0.000515 m. This validates this fixed-state carry check only, not a full moving-target landing.

`tools/step_response.py` generated CSV and plot artifacts under `results/dynamics/`. At this revision the X step has 1.08 s 10–90% rise, 17.0% overshoot and no 2% settling within the 9 s post-step window. The Z step has 1.07 s rise, 15.0% overshoot and no 2% settling within the 9 s window. These results show that the controller still needs tuning.

Camera performance from `results/runs/run_20261022T184454.100224Z_a01_seed4.manifest.json` at 640×480 and 10 Hz: render mean 0.2579 s (p95 0.3295 s); tag detection mean 0.00418 s (p95 0.00566 s); controller mean 0.00074 s; physics-step mean 0.00140 s (p95 0.00185 s). No physics-step budget overruns occurred in that 600-step run. This timing is from this Windows renderer environment and will vary by machine.

On 2026-10-23, the expanded unit suite passed **20 tests**. A two-trial, 5-second static-ground check was captured and repeated with the same seeds. Both runs reached APPROACH without landing; the baseline median final horizontal error was 2.674 m, and decoded fraction was 0.219. The frozen baseline is `results/baselines/static_ground_smoke_20261023.json`; the repeated candidate is `results/batch_20261023T080648.445836Z/batch_summary.json`. The regression guard passed with zero measured change on all four compared metrics; its report is `results/baselines/static_ground_smoke_repeat_check_20261023.json`. Both candidate telemetry CSVs (128 rows each) passed schema validation. These runs exercise acquisition/approach reproducibility only and are not evidence of landing performance.

To repeat the comparison after creating a new batch, run:

```powershell
.\.venv\Scripts\python.exe tools\regression_guard.py --baseline results\baselines\static_ground_smoke_20261023.json --candidate results\batch_<timestamp>\batch_summary.json --scenario static_ground
```

NOT RUN BY ME: full-scene GUI OpenGL tag check, 30-seed scenario benchmarks, time-step convergence, camera angle robustness, lighting/occlusion/latency tests, estimator/battery/failsafe studies, lint/mypy/pytest toolchain, and numeric Webots/MuJoCo calibration. The acceptance success rates and median error thresholds are therefore unverified.
