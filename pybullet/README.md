# Vision guided drone landing on moving, addressed pads — PyBullet edition

This is a PyBullet implementation of the project described by the supplied README. It keeps the ten address sites and the core workflow while replacing MuJoCo with PyBullet: an approximate beacon fix brings the quadrotor into the search area, the simulated onboard camera renders the scene, `pupil-apriltags` decodes genuine `tag36h11` images, OpenCV `solvePnP` estimates marker pose, and a gated position/velocity Kalman tracker feeds predictive moving-target guidance.

The three vehicle/rover sites move; three addresses are roof pads at 3.2, 5.0 and 7.5 metres. Each pad has an approach marker (ID equal to its address, 0.42 m) and centered terminal marker (ID + 10, 0.12 m). The controller has altitude-relative descent gates, a moving-pad velocity feed-forward term, attitude stabilization, and a motor thrust lag model. All dynamics and camera pixels come from PyBullet.

## Install and run (Windows)

Install Python 3.11, 3.12, or 3.13. From this directory:

```bat
run.bat install
run.bat
```

The first run creates `.venv` and installs the PyBullet 3.2.8 compatible binary distribution (`pybullet-arm64`, imported in code as `pybullet`), OpenCV contrib (for tag generation), pupil-apriltags, NumPy, Matplotlib, and Pillow. The dashboard uses a dark flight-operations layout with a large 3D city view, a 640×480 onboard camera feed at 10 Hz, ten color-coded landing-site buttons, and live flight telemetry. Set `PYBULLET_CAMERA_WIDTH`, `PYBULLET_CAMERA_HEIGHT`, and `PYBULLET_CAMERA_FPS` before launch to adjust the camera; defaults are recorded in each run manifest. The status separates visual tag lock from actual pad contact; a landing is confirmed only after stable physical contact with the selected pad. Drag in the city view to orbit, scroll to zoom, or choose **RESET VIEW** to restore the default angle. Its expanded procedural urban environment has a long asphalt avenue, sidewalks, lane and crossing markings, palms, street lights, glass-window high-rises, and rooftop landing buildings. Address B-03 uses a moving tractor-trailer with a marked landing deck; the quadrotor uses a tapered body shell, cross arms, motor pods, landing legs, a front camera/lens, and animated counter-rotating propeller blades. Press 1–9 or 0 for address 10; selecting another address mid-flight diverts to it and highlights its pad.

```bat
run.bat 5
run.bat paper 5
run.bat plots 5
```

Direct Python entry points:

```bat
python demo.py 3
python demo.py 5 9 1
python demo.py 5 --paper
python record.py 3
python evaluate.py --only dynamic
python evaluate.py --only mc --missions 40
python tools/batch.py --scenario static_ground --trials 30 --seconds 50 --workers 2
python tools/batch.py --scenario moving_medium --trials 30 --seconds 50 --workers 2
python tools/make_report.py results/batch_<timestamp>/batch_summary.json --pdf
python analysis.py
python world\build_world.py
python selftest.py
```

For a live telemetry window alongside a mission, run `run.bat plots 5` or
`python run_with_plots.py 5`.
It displays the rolling position, altitude, smoothed velocity components, speed,
and pad-offset traces. The velocity display uses a short exponential smoother
for readability; CSV files retain the raw simulation measurements. Each mission
saves a timestamped CSV under `results/`. The dashboard also saves its active
mission when its window closes.

Telemetry includes position and target position, horizontal landing error, tag
pose error and ID, raw north/east/up velocity, speed, roll/pitch/yaw, angular
velocity, thrust, commanded waypoint and target velocity, and pad-contact state.
These are simulator measurements, not flight-controller telemetry.

`run.bat check`, `doctor`, `determinism`, `analyse`, `record 3`, `evaluate`, `world`, and `install` provide shortcuts. Results and mission telemetry are written to `results/`; marker textures are generated under `assets/generated_tags/`. See `AUDIT.md`, `PLAN.md`, and `docs/` for the audit, validation evidence, limitations, data schema, and upgrade status. `tools/step_response.py` writes controller response artifacts under `results/dynamics/`.

## Modules

| Path | Responsibility |
|---|---|
| `config.py` | All tunables and ten-address book |
| `world.py` / `world/build_world.py` | PyBullet urban scene, pads, vehicles, rooftop geometry, tag textures |
| `physics.py` | Rigid-body quadrotor, body-frame thrust, attitude torque, first-order motor lag |
| `perception/vision.py`, `perception/beacon.py` | Rendered camera, AprilTag decode, PnP pose, and approximate address fix |
| `control/tracker.py` | Covariance-aware Kalman target position/velocity tracker with Mahalanobis gating |
| `control/guidance.py`, `control/attitude.py`, `control/quaternion.py` | Altitude-relative gates, velocity feed-forward, acceleration-to-tilt, quaternion math |
| `mission.py` | Search, approach, descent, touchdown, divert and timeout states |
| `panel.py`, `demo.py` | Interactive dashboard and command-line missions |
| `record.py`, `evaluate.py`, `analysis.py`, `make_diagrams.py` | GIF/CSV recording, experiment summaries, control analysis, and methodology figure |

The control/data path is **addressed beacon → rendered downward camera → AprilTag ID and solvePnP pose → gated Kalman target tracker → velocity feed-forward and moving-target intercept guidance → cascaded attitude/body-rate control and rotor allocation → PyBullet rigid-body contacts**. The dashboard also supports a new address selection while a flight is in progress.

`evaluate.py` writes fresh PyBullet measurements to `results/pybullet_*_summary.json` and figures to `figures/`. These are local runs; the MuJoCo benchmark values in the supplied README are not copied or claimed as PyBullet results.

## Addresses

| # | Address | Surface |
|---:|---|---|
| 1 | A-01 Street drop point | ground |
| 2 | A-02 Loading bay | ground |
| 3 | B-03 Moving freight truck | moving semi-truck, landing deck, up to 0.53 m/s |
| 4 | B-04 Clinic entrance | ground |
| 5 | C-05 Service rover | circling rover |
| 6 | C-06 Apartment roof, 2F | roof, 3.2 m |
| 7 | D-07 Utility cart | moving vehicle, 0.7 m/s nominal |
| 8 | D-08 Apartment roof, 3F | roof, 5.0 m |
| 9 | E-09 Tower roof, 5F | roof, 7.5 m |
| 10 | E-10 Kerbside locker | ground |

## Scope and limitations

This is an engineering recreation, not a byte-for-byte copy of the referenced MuJoCo implementation. It uses a parameterized rigid-body quadrotor with provisional parameters, approximate beacon initialization, procedurally modeled urban geometry, and no radio or sensor latency model. Headless camera frames use PyBullet TinyRenderer; the scene is a stylized 3D model rather than a photorealistic asset pack. The evaluator computes new results from the local simulation; README numbers from the MuJoCo project are not asserted as PyBullet results. PyBullet does not provide MuJoCo's native multibody integrator or its original assets.
