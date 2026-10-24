"""Check runtime packages and simulation assets; optionally probe the GUI."""
from __future__ import annotations

import argparse
import importlib.metadata as metadata
import json
import platform
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def check(name: str, passed: bool, detail: str) -> dict[str, str | bool]:
    return {"check": name, "status": "PASS" if passed else "FAIL", "passed": passed, "detail": detail}


def version(name: str) -> str | None:
    try:
        return metadata.version(name)
    except metadata.PackageNotFoundError:
        return None


def probe_gui() -> tuple[bool, str]:
    code = "import pybullet as p; c=p.connect(p.GUI); print('GUI_CLIENT',c); p.disconnect(c) if c>=0 else None; raise SystemExit(0 if c>=0 else 2)"
    try:
        result = subprocess.run([sys.executable, "-c", code], cwd=ROOT, capture_output=True,
                                text=True, timeout=12)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return False, str(exc)
    detail = (result.stdout + result.stderr).strip().replace("\n", " | ")[-500:]
    return result.returncode == 0, detail or f"exit code {result.returncode}"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--probe-gui", action="store_true", help="Open and close a brief PyBullet GUI client probe")
    args = parser.parse_args()
    rows: list[dict[str, str | bool]] = []

    try:
        import cv2
        import numpy
        import pybullet as p
        from pupil_apriltags import Detector
        from world import tag_texture
        rows.append(check("Python", sys.version_info >= (3, 11) and sys.version_info < (3, 14), sys.version.split()[0]))
        rows.append(check("PyBullet", bool(p.getAPIVersion()), f"API {p.getAPIVersion()}, distribution {version('pybullet') or version('pybullet-arm64') or 'unknown'}"))
        rows.append(check("NumPy", numpy.__version__ == "2.5.4", numpy.__version__))
        aruco_ok = hasattr(cv2, "aruco") and hasattr(cv2.aruco, "DICT_APRILTAG_36h11")
        rows.append(check("OpenCV AprilTag dictionary", aruco_ok, cv2.__version__))
        rows.append(check("pupil-apriltags", version("pupil-apriltags") == "1.0.4.post11", version("pupil-apriltags") or "not installed"))
        if aruco_ok:
            marker = cv2.imread(tag_texture(1), cv2.IMREAD_GRAYSCALE)
            ids = [int(item.tag_id) for item in Detector(families="tag36h11", nthreads=1).detect(marker)] if marker is not None else []
            rows.append(check("Generated AprilTag asset", 1 in ids, f"ID 1 decode; shape {None if marker is None else marker.shape}"))
        else:
            rows.append(check("Generated AprilTag asset", False, "OpenCV AprilTag support unavailable"))
        direct = p.connect(p.DIRECT)
        if direct >= 0:
            p.resetSimulation(physicsClientId=direct)
            p.setRealTimeSimulation(0, physicsClientId=direct)
            p.disconnect(direct)
        rows.append(check("PyBullet DIRECT", direct >= 0, f"client {direct}"))
        if args.probe_gui:
            ok, detail = probe_gui()
            rows.append(check("PyBullet GUI", ok, detail))
        else:
            rows.append({"check": "PyBullet GUI", "status": "SKIP", "passed": True, "detail": "not probed; rerun with --probe-gui"})
        required = [ROOT / "config.py", ROOT / "physics.py", ROOT / "simulation.py", ROOT / "world.py"]
        missing = [str(path.relative_to(ROOT)) for path in required if not path.is_file()]
        rows.append(check("Model and scene source paths", not missing,
                          "procedural PyBullet model; no URDF expected" if not missing else f"missing: {missing}"))
        tag_path = ROOT / "assets" / "generated_tags"
        rows.append(check("Generated tag path", tag_path.is_dir(), str(tag_path)))
    except Exception as exc:
        rows.append(check("Doctor execution", False, f"{type(exc).__name__}: {exc}"))

    print("CHECK                         STATUS  DETAIL")
    print("-" * 100)
    for row in rows:
        print(f"{row['check']:<29} {row['status']:<7} {row['detail']}")
    out_dir = ROOT / "results" / "doctor"
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    report = {"created_utc": datetime.now(timezone.utc).isoformat(), "python": platform.python_version(), "checks": rows}
    (out_dir / f"doctor_{stamp}.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    return 0 if all(row["passed"] for row in rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
