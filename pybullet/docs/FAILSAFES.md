# Failsafe coverage

The current mission has address validation, a 180 s timeout abort, moving-target visual-lock gating during descent, and contact-based touchdown confirmation. It does not yet implement return-to-home, geofence, battery, estimator-divergence, motor/link-loss, or retry policies. No trigger scenario has been validated for those unimplemented failsafes.

The full failsafe matrix and triggering scenarios are pending per `PLAN.md`. The timeout behavior has a unit test; physical go-around and abort behavior have not been exercised in a PyBullet run.
