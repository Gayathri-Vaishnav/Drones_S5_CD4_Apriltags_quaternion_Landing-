# Limitations and evidence boundary

This project is a software simulation and has not been validated against a real Crazyflie flight log. The mass is a nominal 2.1 value; inertia and motor coefficients are approximations from public 2.x models. The RPM/thrust runtime model omits the fitted curve's linear term. Drag coefficients are proxies; battery sag and full Dryden turbulence are not implemented. Realistic IMU/GPS/altimeter noise and latency, and an onboard vehicle-state estimator are also absent.

The current camera uses a world-stabilized downward gimbal and zero lens distortion. DIRECT uses TinyRenderer; GUI mode requests hardware OpenGL. Ten addressed tags were detected in a fixed 4 m overhead DIRECT check; address 1 was detected from 2–10 m. Hardware OpenGL rendered a small test frame on this host, but a full-scene GUI tag check stalled and remains unverified. View-angle, lighting, blur and occlusion robustness are untested. The target Kalman filter is not a vehicle-state EKF. The latest smoke run ends in APPROACH and is not a landing result.

Cross-simulator comparisons are limited to qualitative trends, rankings, and failure modes unless a calibration study demonstrates numeric equivalence. Existing result files predate some model changes and must not be pooled with new runs without provenance review.

See `AUDIT.md` for observed checks and `PLAN.md` for pending acceptance work. All performance thresholds remain unclaimed until generated batch results support them.
