# Control design

## Current model

Four motor speeds track commanded RPM with a symmetric first-order lag. Rotor thrust is `min(k_f * omega^2, T_max)`, applied in the body +Z direction at the four X-frame arms. Rotor reaction torque is `spin_sign * k_m * omega^2` about body Z. PyBullet integrates forces, torque and contacts at 120 Hz. Mission/guidance updates occur at 30 Hz; the low-level controller runs at 120 Hz.

The position loop generates bounded horizontal/vertical velocity setpoints with target-velocity feed-forward, then requests bounded acceleration with scheduled gains and integral action. Acceleration maps to a bounded desired attitude; a quaternion error generates body-rate requests; a rate loop maps to desired body torque; an X-frame mixer maps thrust and torque to four motor commands. Position and rate integrators use saturation-aware conditional integration. The velocity limits are setpoints and can be exceeded transiently due to actuation lag.

## Parameters and assumptions

`params/crazyflie_2_1.json` contains values, units, sources and confidence notes. Mass is nominal. Inertia is transferred from a published 2.0 estimate. Motor coefficients are converted from the dominant quadratic term of a fitted 2.x RPM curve, so its linear term and asymmetric up/down motor response are omitted. Battery voltage sag and rotor aerodynamic interference are not yet modeled. Body drag uses relative airspeed with proxy drag-area coefficients; wind uses a seeded first-order correlated gust proxy, not a full Dryden shaping-filter implementation. Ground effect is a small synthetic near-surface multiplier. `config.py` exposes wind mean, gust scale/correlation, and drag parameters.

## Verification status

The rigid-body self-test verifies finite state after a short run. Determinism is measured in `determinism_test.py`. The current 600-step landing smoke reaches APPROACH but does not land; its peak total speed is 2.43 m/s. A generated 1 m step-response suite measures X rise 1.08 s, 17.0% overshoot and no 2% settling in the 9 s window; Z rise 1.07 s, 15.0% overshoot and no 2% settling in the 9 s window. The controller is not yet tuned to acceptance targets.
