# Architecture

`simulation.py` owns one fixed-step PyBullet client, one per-run NumPy generator, the world, vehicle, camera, mission, tracker, guidance and telemetry. `world.py` builds static geometry and kinematic targets; `physics.py` owns rotor-level actuation and the cascaded controller. `perception/vision.py` renders the downward view, decodes tag36h11 and performs solvePnP. `mission.py` gates descent and touchdown. `evaluate.py` uses the same `LandingSimulation` in DIRECT mode.

The current vehicle is a procedural rigid body. It is not loaded from URDF or gym-pybullet-drones. The thrust model and controller are described in `CONTROL_DESIGN.md`; observed gaps are in `LIMITATIONS.md` and staged in `PLAN.md`.

## Reproducibility boundary

Each mission writes a JSON manifest under `results/runs/` with scenario, seed, configuration, package and platform versions, physics settings, renderer selection, and a telemetry path once saved. DIRECT uses TinyRenderer; GUI onboard capture requests Bullet hardware OpenGL. Determinism has been checked only for repeated DIRECT runs on this host with this PyBullet build and renderer. It is not a cross-platform guarantee.

## Simulator comparison

The sibling Webots project and MuJoCo reference are separate implementations. The shared scenario/metrics schema is planned; no numeric parity is claimed without a calibration study.
