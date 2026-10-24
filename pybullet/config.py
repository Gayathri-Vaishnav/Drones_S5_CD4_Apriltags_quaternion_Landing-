"""Simulation, sensor, and controller tunables."""
from dataclasses import dataclass
import json
import os
from pathlib import Path

VEHICLE_PARAMS = json.loads((Path(__file__).resolve().parent/"params"/"crazyflie_2_1.json").read_text(encoding="utf-8"))

DT = 1 / 120
CTRL_PERIOD = 1 / 30
CAMERA_WIDTH = int(os.environ.get("PYBULLET_CAMERA_WIDTH", "640"))
CAMERA_HEIGHT = int(os.environ.get("PYBULLET_CAMERA_HEIGHT", "480"))
CAMERA_FPS = int(os.environ.get("PYBULLET_CAMERA_FPS", "10"))
CAMERA_PERIOD = 1 / CAMERA_FPS
if CAMERA_WIDTH < 160 or CAMERA_HEIGHT < 120 or not 1 <= CAMERA_FPS <= 120:
    raise ValueError("Camera settings require width >=160, height >=120, and FPS between 1 and 120")
CAMERA_FOV_DEG = 65
CAMERA_NEAR, CAMERA_FAR = 0.02, 60.0
SOLVER_ITERATIONS = 100
NUM_SUBSTEPS = 0  # Fixed-step integration: simulation advances exactly once per DT.
MASS_KG, GRAVITY = float(VEHICLE_PARAMS["mass_kg"]), 9.81
MAX_TILT_DEG = 24
MAX_HORIZONTAL_SPEED_M_S = 2.0
MAX_VERTICAL_SPEED_M_S = 1.2
MAX_LANDING_SPEED_M_S = .8
AIR_DENSITY_KG_M3 = 1.225
DRAG_CD_A_XY_M2 = .0035
DRAG_CD_A_Z_M2 = .0020
WIND_MEAN_M_S = (0.0, 0.0, 0.0)
WIND_GUST_STD_M_S = .05
WIND_CORRELATION_TIME_S = .8
MAX_WIND_GUST_M_S = .3
PAD_SIZE_M = 1.25
LARGE_TAG_M, SMALL_TAG_M = 0.42, 0.12
ALTITUDE_SWITCH_M = 2.0
APPROACH_ALTITUDE_M = 5.0
TOUCHDOWN_HEIGHT_M = 0.08

@dataclass(frozen=True)
class Address:
    number: int
    code: str
    name: str
    kind: str
    xyz: tuple[float, float, float]
    motion: str = "static"
    speed: float = 0.0

ADDRESSES = {
    1: Address(1, "A-01", "Street drop point", "ground", (-8.0, -1.0, 0.0)),
    2: Address(2, "A-02", "Loading bay", "ground", (-4.0, 3.0, 0.0)),
    3: Address(3, "B-03", "Moving freight truck", "vehicle", (0.0, 7.0, 0.35), "line", 0.35),
    4: Address(4, "B-04", "Clinic entrance", "ground", (5.0, 8.0, 0.0)),
    5: Address(5, "C-05", "Service rover", "vehicle", (9.0, 1.0, 0.35), "circle", 0.65),
    6: Address(6, "C-06", "Apartment roof, 2F", "roof", (-8.0, 12.0, 3.2)),
    7: Address(7, "D-07", "Utility cart", "vehicle", (0.0, 15.0, 0.35), "line", 0.7),
    8: Address(8, "D-08", "Apartment roof, 3F", "roof", (7.0, 15.0, 5.0)),
    9: Address(9, "E-09", "Tower roof, 5F", "roof", (12.0, 13.0, 7.5)),
    10: Address(10, "E-10", "Kerbside locker", "ground", (13.0, -2.0, 0.0)),
}

