# PyBullet Research-Grade Upgrade Plan

**Plan date:** 22 October 2026  
**Scope:** `pybullet/` is the active implementation. The requested Webots and MuJoCo parity work will use a simulator-neutral schema and qualitative comparisons only; no numeric parity will be claimed without a calibration study.

## Architecture target

Keep `simulation.py` as a small orchestrator and preserve the existing dashboard and entry points while moving physics, sensor models, estimation, guidance, mission logic, safety, logging, and configuration behind explicit module interfaces. Keep a headless DIRECT path for batch tests and reserve GUI rendering for interactive use. Store scenario inputs and physical parameters as versioned configuration, and make each run self-describing with a manifest and common metric schema.

## Ordered workstreams

Each workstream is implemented and verified before the next one changes mission behavior. Long benchmark work is deferred until unit and short-run regressions make it affordable and interpretable.

1. **Audit and baseline** — completed by `AUDIT.md`; preserve all existing results. Initial telemetry index defects are corrected, and new tests cover math, mission gates, rotor allocation, and moving-platform contact.
2. **Reproducibility and setup** — implemented pinned dependencies, setup scripts, doctor, fixed-step DIRECT/GUI initialization, per-run manifests, seeded RNG, and determinism check. GUI and DIRECT availability were both observed on this host; cross-host and cross-renderer determinism is not claimed.
3. **Baseline tests, schemas, and metrics** — substantially complete: telemetry schema documented and validated for exact headers, numeric finiteness, integer/boolean fields, monotonic time, and single-mission address; evaluator/plot readers fixed; shared scenario schema checked; Wilson/error aggregation and a configurable batch regression guard added. A short-run static-ground smoke baseline exists. Full mission-performance regression thresholds still require a completed landing campaign.
4. **Physical parameters and vehicle dynamics** — partially complete: sourced provisional parameters, rotor thrust/reaction torque, mixer, lag, saturation, a cascaded controller, conditional anti-windup, relative-air-speed body drag, a seeded correlated-wind proxy, and ground-effect proxy are implemented. X/Z step-response CSV and plots exist; roll/pitch/yaw response sweeps, voltage sag, and a validated Dryden model remain.
5. **Sensors and state estimation** — not started: configurable IMU/gyro, altimeter/GPS, and camera noise, bias, and latency; camera exposure/blur/distortion/resolution/FPS controls; calibration and undistortion; an EKF/ESKF with covariance, innovation, Mahalanobis gating, and tag-loss coasting; a power-to-current/voltage/state-of-charge battery model. Label remaining proxies in logs and documentation.
6. **Vision, target tracking, and guidance** — partially verified: flat UV-mapped tag36h11 textures decode at ten selected pads from a fixed 4 m overhead view and at address 1 from 2–10 m; target tracking uses a covariance-aware Kalman filter with Mahalanobis gating; moving bodies publish analytic velocities and truck contact is integration-tested; moving-target intercept is bounded and predictive. View-angle/lighting/occlusion tests, multi-tag fusion, and time-to-contact descent remain.
7. **Mission safety and planning** — partially documented; timeout and tag-lock/contact gates exist. Height-aware route/RTH, geofence, go-around/retry, motor degradation, and scenario-tested battery/geofence/tag/estimator/motor/link failsafes remain.
8. **Scenario engine and benchmark** — partially complete: shared JSON-compatible YAML fixtures/schema, process-isolated DIRECT batch runner, per-trial metrics, Wilson intervals, and median/p95 summaries exist. The 30-trial acceptance campaigns, complete ablations, regression thresholds, and Webots/MuJoCo fixture consumers remain.
9. **Dashboard, reports, and quality** — partially complete: data schema and architecture/control/estimation/failsafe/validation/limitation docs, HTML/PDF report generation, timing summaries, scenario-schema and telemetry-schema tests, and a regression guard exist. UI overlays, CSV replay, lint/type checks, and CI remain.
10. **Documentation and release check** — partial: requested documents and changelog exist; README cleanup and acceptance evidence collection remain.

## Defaults and decision policy

- Use the existing PyBullet implementation as the active target. Do not modify the sibling projects while building the common schema.
- Use `p.DIRECT` for tests and batch trials. Keep an interactive GUI mode for the dashboard.
- Select a fixed physics rate by convergence evidence; retain the current 120 Hz as the baseline until comparative runs justify a different rate. Set controller and camera rates independently and measure camera cost.
- Use a single seeded NumPy `Generator` per run and pass it to stochastic subsystems. Record the seed and package/platform versions.
- Treat Crazyflie 2.1 published parameters as sourced nominal values; flag any values that require calibration or lack a public source.
- Keep current result files immutable. New benchmarks go into uniquely named `results/batch_<timestamp>/` directories.
- Cross-simulator comparison is qualitative (trends, ranking, failure modes) unless independently calibrated.

## Main risks and mitigations

- **Controller retuning risk:** changing mass from 1.25 kg to a Crazyflie-class mass invalidates existing gains. Introduce dynamics and controller tests separately, and keep a known baseline configuration during transition.
- **Slow camera/batch runs:** lower and configure camera FPS, time camera operations, and benchmark one short scenario before scaling to 30 trials. Parallel runs must use one independent PyBullet client per process.
- **Renderer differences:** use the same scene/camera setup for tag tests, report DIRECT TinyRenderer and GUI OpenGL behavior separately, and avoid interpreting detector differences as vehicle dynamics changes.
- **Model identifiability:** motor constants, inertia, contact, battery, and turbulence parameters may be approximate. Record source, units, uncertainty, and calibration status.
- **Scope size:** acceptance criteria require multiple long simulations and GUI checks. Run only measured workloads that fit; mark unrun Windows GUI tests and incomplete benchmark counts explicitly rather than filling in estimates.
- **Legacy results:** old results may reflect a different model or stale camera placement. Preserve them, label their provenance, and never combine them with new-run statistics without a documented compatibility check.

## Acceptance checklist

- [ ] Clean Windows setup and first run in no more than two commands.
- [ ] Doctor passes; seeded determinism is measured and documented.
- [ ] Unit tests and lint pass; PyBullet tests are clearly marked.
- [ ] Convergence study supports the selected physics rate.
- [ ] At least 30 seeded trials for each core scenario, with Wilson intervals and aggregate error metrics.
- [ ] Static landing success >=95% and median landing error <=0.05 m; moving-target success >=85% at medium speed, or report measured misses and causes.
- [ ] All failsafe scenarios trigger and log the expected response.
- [ ] Timing measurements report mean/max/std and cycle-budget overruns.
- [ ] Every published metric traces to generated result files; cross-simulator comparisons remain qualitative absent calibration.
