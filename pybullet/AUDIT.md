# PyBullet Simulation Audit

**Audit baseline date:** 22 October 2026 (before the changes listed below)  
**Scope:** This audit covers `pybullet/`, the PyBullet project named in the current request. The sibling `webots/` and `3D simulation/` directories are separate projects and were not changed or numerically compared.

## Current architecture

The application entry points are `demo.py` (dashboard or single/multiple addressed missions), `panel.py` (Tkinter dashboard), `run_with_plots.py` (mission plus Matplotlib plots), `record.py` (GIF/CSV), and `evaluate.py` (headless experiments). `simulation.py` owns the common mission loop. `world.py` builds the procedural city, landing pads, tags, and moving vehicles. `physics.py` implements the quadrotor body and controller. Perception is in `perception/vision.py` and `perception/beacon.py`; guidance, attitude math, quaternions, and target tracking are in `control/`.

## Vehicle model and actuation

- There is **no URDF, SDF, Gym-PyBullet-Drones model, or articulated rotor dynamics**. `Quadrotor` creates a single PyBullet rigid body with a box collision shape, procedural mesh visual, and mass `1.25 kg` (`config.py`). PyBullet derives the default inertia from the collision geometry; the project does not supply or log a Crazyflie inertia tensor.
- Four arms, motors, propellers, landing legs, camera, and lens are separate mass-zero visual bodies. They are not rotor links and do not generate lift.
- `Quadrotor.control()` is a position-error/velocity-damping controller that maps desired acceleration to a target quaternion, applies a quaternion attitude error torque, and computes one scalar total thrust. It applies lift with `p.applyExternalForce()` and attitude torque with `p.applyExternalTorque()`. There is no X-frame motor mixer, per-rotor thrust or reaction torque, body-rate loop, rotor RPM, or motor failure model.
- The scalar thrust has a first-order lag with a nominal 0.04 s time constant and a total-thrust clamp. A small altitude-based ground-cushion multiplier is applied. There is no aerodynamic drag model, battery voltage model, or physically parameterized rotor/airframe model. The wind term is a clipped seeded random-walk force, not a Dryden airflow model or force relative to airspeed.
- The seed initializes NumPy generators in the simulation and vehicle. Beacon error and wind use those generators. No repeated-run determinism test exists; renderer, platform, and dependency versions are not captured in a manifest.

## Physics configuration and moving objects

- Baseline: `world.py` set gravity to `(0, 0, -9.81)` and the fixed step to `1/120 s`. Mission/guidance updates occur at `1/30 s`; the low-level motor controller is called every physics step (120 Hz). The baseline camera was 960×720 at 20 Hz. Current defaults are 640×480 at 10 Hz; all camera settings are environment-configurable.
- The application does not call `p.resetSimulation()`, explicitly disable real-time mode, configure solver iterations/substeps, or run a time-step convergence study. The project-specific time step is set during world construction. Physics-engine/contact parameters beyond drone lateral friction and damping are left at PyBullet defaults.
- Moving address 3, 5, and 7 bodies are mass-zero kinematic scene objects. `UrbanWorld.update()` resets their poses, child parts, and now their analytic linear velocities. A two-second truck-deck contact check verified 0.000515 m relative drift while the platform traveled 0.966 m; see `docs/VALIDATION.md`.
- Pads are physical collision boxes and touchdown reads `p.getContactPoints()`. The controller checks horizontal/vertical bounds, target speed, relative speed, and 45 consecutive stable control samples. Friction, restitution, contact stiffness, and damping now have explicit values, but still need sensitivity studies.

## Camera and AprilTag pipeline

- At baseline, `OnboardCamera.render()` used TinyRenderer for both DIRECT and GUI at 960×720/20 Hz. It now uses TinyRenderer in DIRECT and requests hardware OpenGL in GUI, with 640×480/10 Hz defaults configurable by environment variables. GUI hardware rendering succeeded on a small test frame; the full-scene GUI tag check stalled.
- Camera intrinsics are derived from a 65 degree field of view and zero distortion. Exposure, motion blur, distortion, frame latency and calibration remain absent. Camera, detector, controller, and physics timing are now summarized in run manifests. The eye was moved 0.018 m below the body center for the small quadrotor.
- At audit baseline, the world built black-cell meshes over white plates without binding textures. It now uses flat UV-mapped marker surfaces and applies generated `tag36h11` textures via `changeVisualShape`. `pupil_apriltags` decodes rendered images; OpenCV `solvePnP` estimates pose, followed by fixed empirical size/position offsets. Multi-tag fusion, view-angle/pixel-size confidence, and injected lighting/occlusion/loss scenarios remain absent.
- The detector is configured with one thread. Wrong-address detections can appear in frames, but mission control selects only the requested tag ID (or its paired precision tag). A selected tag's decoded pose is used in the tracker when reprojection error is under 4 px.

## Estimation, guidance, mission, and safety

- At baseline, `ABGTracker` estimated tag-derived pad position. It has now been replaced in the mission path by a constant-velocity Kalman target tracker with covariance propagation and Mahalanobis outlier gating. It is not a vehicle EKF/ESKF and does not fuse IMU, GPS, or altimeter data. The approximate beacon (`approximate_fix`) remains a noisy address coordinate used to begin the search; simulated GPS is absent.
- `Guidance` schedules waypoints by mission phase and adds estimated target velocity. Moving targets now use a bounded 0.2–1.25 s linear intercept prediction during approach/high descent. There is no time-to-contact descent profile, route planner, obstacle-aware RTH, geofence, or configurable failsafe matrix.
- `Mission` is a compact enum/state machine with acquisition, approach, descent, flare, touchdown, landed, and abort states. It applies a 180 s timeout and moving-target lock/speed guards. Go-around and abort/retry behavior is not implemented. The dashboard selects one of ten sites; binary mission UI is absent. Shared scenario fixtures are consumed by the batch runner.
- The battery, motor-degradation, estimator-divergence, radio/link-loss, and battery failsafe models requested in the research brief are absent. The current simulation has seeded wind only; it is not user-configurable through the PyBullet dashboard.

## Logging, experiments, and reproducibility

- `simulation.py` stores control-sample rows and writes one telemetry CSV schema through `save_csv()`. `record.py`, `panel.py`, and `run_with_plots.py` create additional CSV/GIF outputs. Existing files under `results/` were preserved.
- `evaluate.py` runs single-seed address/perception/static/dynamic trials, a six-seed feed-forward ablation, or a Monte Carlo loop. It does not compute Wilson intervals, median/p95 landing-error aggregates, distributions, energy, touchdown speed, path length, failsafe counts, or timing statistics. Its decoded-frame and pose-error fields currently index the wrong telemetry columns and need correction before the evaluation output is relied upon.
- There are two executable checks, `selftest.py` and `smoke_test.py`; no pytest suite, static lint/type check, baseline regression guard, scenario catalog, per-run manifest, batch directory, or CI workflow is present. `plot_results.py` still reads velocity column names that do not match the current CSV schema.
- At audit time, requirements were not fully pinned and specified `pybullet-arm64==3.2.8`. Despite the distribution name, its PyPI release includes Windows x86-64 wheels; this is the currently tested package. `run.bat` checks for that exact distribution before dispatching most commands; a one-command setup path exists in the launcher text but `setup.bat` is missing. No doctor script exists.

## Existing verification and evidence

The following results were observed in this session before this audit:

- `selftest.py` passed its package import, generated-tag decode, rendered-scene tag detection, finite-state controller step, and cleanup checks after the camera-eye fix.
- A direct camera check placed the camera 5 m above each of the ten selected address targets; it detected the selected acquisition or precision tag for all ten. Moving targets were evaluated at simulated time 12 s. This is a visibility check, not a closed-loop landing trial.
- `smoke_test.py` passed 600 headless physics steps, wrote telemetry, and found the current velocity columns after its stale column check was fixed. The run ended at 5 simulated seconds in `DESCEND_LOW`; it did not verify touchdown.
- A full ten-address mission evaluation was started but interrupted because it was too slow for the available execution window; it produced no complete aggregate and is not evidence of landing success.
- Existing `results/mission_3_check.json` contains a pre-fix report of a blank live view. It is historical and stale with respect to the corrected camera placement; it has not been edited. Existing result files were not removed.

## Claims that are not established by current evidence

Past successful CSVs and a historical check file do not establish current repeatable landing rates. No confidence intervals, repeated seeded trial statistics, time-step convergence, DIRECT-vs-GUI render comparison, contact-carry test, obstacle/failsafe scenario coverage, motor step responses, or real-time budget measurements are currently available. No cross-simulator numeric equivalence is claimed.

## Source inventory reviewed

Reviewed the project README, requirements, launcher, and Python source in the root, `control/`, `perception/`, and `world/`: `analysis.py`, `config.py`, `demo.py`, `evaluate.py`, `live_plotter.py`, `make_diagrams.py`, `mission.py`, `panel.py`, `physics.py`, `plot_results.py`, `record.py`, `run_with_plots.py`, `selftest.py`, `simulation.py`, `smoke_test.py`, `world.py`, `control/{attitude,guidance,quaternion,tracker}.py`, `perception/{beacon,vision}.py`, and `world/build_world.py`.

## Upgrade progress (updated after the baseline audit)

The reproduction setup, run manifest, seeded RNG, deterministic DIRECT settings, regression tests, telemetry indexing, parameterized Crazyflie-class body, per-rotor thrust/reaction torque, motor lag, correlated-wind proxy, relative-air-speed drag proxy, textured flat tags, target Kalman tracker, predictive intercept, and contact parameters have since been implemented. A Windows launcher argument-forwarding defect was fixed. See `CHANGELOG.md`, `PLAN.md`, and `docs/` for staged work and gaps. Current check evidence: fourteen unit/integration tests pass; `selftest.py`, `doctor.py` (DIRECT and GUI client availability), `determinism_test.py`, compileall, launcher checks, and ten-address selected-tag DIRECT camera check pass. Textured address-1 tags detect from 2–10 m. Hardware OpenGL renders a small test frame, while full-scene GUI tag validation stalled. A 600-step smoke confirms telemetry columns but does **not** establish landing performance. Latest run ends in APPROACH at 5.0 s with 2.72 m horizontal error and peak speeds 2.43 m/s total, 2.04 m/s horizontal; the horizontal peak remains slightly above the 2.0 m/s setpoint. At 640×480/10 Hz, render averaged 0.258 s (p95 0.330 s); controller and physics-step means were 0.00074 s and 0.00140 s. The current isolated response suite reports X rise 1.08 s, 17.0% overshoot, no 2% settling within 9 s; Z rise 1.07 s, 15.0% overshoot, no 2% settling. Serial and multiprocessing batch smoke paths were exercised, but the 30-trial acceptance campaigns were not run. GUI full-scene tag behavior, convergence, vehicle EKF/sensor models, battery, full Dryden validation, failsafe matrix, and landing success thresholds remain unverified.
