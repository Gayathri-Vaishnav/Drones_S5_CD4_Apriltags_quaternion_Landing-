# Vision-guided autonomous landing on addressed, moving AirTag targets

A MuJoCo simulation of the system described in the base paper
*"UAV Autonomous Landing Technology Based on AprilTags Vision Positioning Algorithm"*
(Zhou Li, Yang Chen, Hao Lu, Huaiyu Wu, Lei Cheng — 38th Chinese Control
Conference, 2019), extended with an **addressing layer**: ten landing pads
numbered 1–10 — four on the street, three carried by moving vehicles and three on
building roofs. Press an address, and the drone flies to it, tracks the pad while
it moves, and lands on it.

Everything in the perception path is real. The camera image is rendered through
the drone's own downward camera in MuJoCo; the markers are genuine `tag36h11`
AprilTags painted onto the pads; they are decoded by `pupil_apriltags` and the
pose is recovered with `cv2.solvePnP`. Nothing is faked or projected
analytically, so when the drone tilts, the tag really does swing across the
image.

## Run it

On Windows just double-click **`run.bat`**. It checks the environment, installs
anything missing the first time, then opens the simulation straight away — no
prompts.

One window: the world on the left, the drone's own camera top right, and the ten
landing sites as numbered buttons underneath it. **Press a number key, or click a
button, and the drone flies there and lands** — including mid-flight, which makes
it a divert rather than a restart.

```bash
run.bat
```

```bash
run.bat 5
```


Other things `run.bat` can do:

| command | what it does |
|---------|--------------|
| `run.bat check` | verify the environment and prove the detector decodes a marker |
| `run.bat analyse` | controllability, observability and the lerp/slerp analysis |
| `run.bat paper 5` | fly address 5 with the base paper's fixed-gain PID alone |
| `run.bat record 3` | record a mission to an animated GIF |
| `run.bat evaluate` | run every experiment and rebuild all figures |
| `run.bat world` | regenerate the pad textures and the MuJoCo world |
| `run.bat doc` | rebuild both review documents |
| `run.bat install` | install the Python packages |

Everything below is the same thing without the batch file.

Live demo — asks which address to fly to, opens the 3D viewer and the onboard
camera window:

```bash
python drone_sim/demo.py
```

Fly straight to one address, or several in sequence:

```bash
python drone_sim/demo.py 3
```

```bash
python drone_sim/demo.py 5 9 1
```

Fly with the base paper's fixed-gain PID alone, to see the moving-pad lag it
leaves behind:

```bash
python drone_sim/demo.py 5 --paper
```

Regenerate the world and the pad textures after editing `config.py`:

```bash
python drone_sim/world/build_world.py
```

Run every experiment and rebuild all figures (this takes about half an hour —
every frame is really rendered and really decoded):

```bash
python drone_sim/evaluate.py
```

Run one experiment on its own — `scene`, `addressing`, `perception`, `static`,
`dynamic`, `ablation` or `mc`:

```bash
python drone_sim/evaluate.py --only dynamic
```

Record a mission as a GIF, chase view beside the onboard camera:

```bash
python drone_sim/record.py 3
```

Rebuild the review documents:

```bash
node docs/make_docx.js
```

```bash
node docs/make_intro_method_docx.js
```

## The ten addresses

| # | Address | Kind |
|---|---------|------|
| 1 | A-01 Street drop point | ground |
| 2 | A-02 Loading bay | ground |
| 3 | B-03 Courier van | **vehicle**, 0.9 m/s |
| 4 | B-04 Clinic entrance | ground |
| 5 | C-05 Service rover | **vehicle**, circling |
| 6 | C-06 Apartment roof, 2F | **roof**, 3.2 m up |
| 7 | D-07 Utility cart | **vehicle**, 0.7 m/s |
| 8 | D-08 Apartment roof, 3F | **roof**, 5.0 m up |
| 9 | E-09 Tower roof, 5F | **roof**, 7.5 m up |
| 10 | E-10 Kerbside locker | ground |

Three kinds of site: on the street, on the deck of a **moving vehicle**, and on
an **apartment roof**. A roof landing is the same manoeuvre performed higher up —
the search altitude, the descent gates and the touchdown test are all measured
relative to the surface the pad sits on, not to the ground.

Each pad carries two AprilTags: a 0.42 m marker with id = the pad number for the
approach, and a 0.12 m marker with id = pad number + 10 at the pad centre for the
last two metres. Distinct ids mean the drone can be told *which* pad to land on
and will ignore the others — at search altitude two or three pads are in frame
at once, every time.

## Layout

```
drone_sim/
  config.py            every tunable number, and the address book, in one place
  world/build_world.py generates the pad textures and the MuJoCo world from config
  physics/quad.py      MuJoCo env: body-frame thrust, wind, moving pads, sensors
  control/attitude.py  acceleration -> tilt -> torque (a real quadrotor must tilt)
  control/guidance.py  PID of eq.(9), alpha-beta-gamma pad tracker, gain schedule
  perception/vision.py real camera frames -> AprilTag decode -> pad-centre offset
  perception/beacon.py the AirTag radio fix that makes an address flyable
  mission.py           the state machine and the landing interlocks
  panel.py             the interactive control panel (this is what run.bat opens)
  demo.py              the older two-window demo, kept for reference
  control/quaternion.py quaternion algebra, SLERP, and the orientation filter
  analysis.py          controllability, observability, lerp/slerp analysis
  selftest.py          health check: imports, world, camera, detector
  record.py            records a mission to an animated GIF
  evaluate.py          all experiments -> figures/ and results/
  make_diagrams.py     the explanatory diagrams used in the methodology document
run.bat                one-click launcher for all of the above (Windows)
figures/               every figure used in the review document
results/               mujoco_summary.json, the numbers the document quotes
docs/                  the two review documents and the scripts that build them
friend_version/        the original MuJoCo project this was built on
simulation/            an earlier kinematic prototype (matplotlib, no physics)
```

## Requirements

Python 3.11+, `mujoco`, `pupil-apriltags`, `opencv-python`, `numpy`,
`matplotlib`. Node with the `docx` package to rebuild the document.


## Three platforms, one model

The same methodology is implemented three times, independently:

| folder | platform | what is real, what is modelled |
|---|---|---|
| `drone_sim/` | Python + MuJoCo | everything real: rendered camera, real AprilTag detector, contact physics |
| `matlab/` | MATLAB R2024a | same controller and state machine; the detector is modelled geometrically |
| `netlogo/` | NetLogo 6.4 | same controller and state machine; ray-cast camera panel; detector modelled |

Run them with `run.bat` (MuJoCo), `matlab/run_matlab.bat` and `netlogo/run2.bat`.
All three agree on the result to within about a centimetre, and all three find
the same address hardest. `docs/Mathematics_of_the_Landing.pptx` walks through
the mathematics they share.
