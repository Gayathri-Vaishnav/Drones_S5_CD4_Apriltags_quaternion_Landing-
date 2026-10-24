"""Pure-Python baseline regression tests; runnable with unittest or pytest."""
import unittest

import numpy as np

from control.quaternion import normalize, multiply, slerp, OrientationFilter
from control.tracker import ABGTracker, KalmanTargetTracker
from control.guidance import Guidance
from mission import Mission, Phase


class QuaternionTests(unittest.TestCase):
    def test_normalize_and_multiply_preserve_unit_norm(self):
        q = normalize((0.0, 0.0, 0.0, 2.0))
        self.assertTrue(np.allclose(q, (0, 0, 0, 1)))
        self.assertAlmostEqual(float(np.linalg.norm(multiply(q, q))), 1.0)

    def test_slerp_endpoints_and_filter(self):
        start = np.array((0.0, 0.0, 0.0, 1.0))
        end = np.array((0.0, 0.0, 1.0, 0.0))
        self.assertTrue(np.allclose(slerp(start, end, 0.0), start))
        self.assertTrue(np.allclose(slerp(start, end, 1.0), end))
        result = OrientationFilter().update(end)
        self.assertTrue(np.allclose(result, end))


class TrackerTests(unittest.TestCase):
    def test_constant_velocity_measurements_converge(self):
        tracker = ABGTracker()
        for index in range(20):
            position = np.array((0.1 * index, -0.05 * index, 2.0))
            estimate, velocity = tracker.update(position, 0.1)
        self.assertTrue(np.all(np.isfinite(estimate)))
        self.assertTrue(np.all(np.isfinite(velocity)))
        self.assertLess(float(np.linalg.norm(estimate - position)), 0.2)

    def test_kalman_tracker_converges_and_covariance_is_valid(self):
        tracker = KalmanTargetTracker()
        for index in range(40):
            estimate, velocity = tracker.update((index * .05, 0, 1), .1, measurement_std=.02)
        self.assertAlmostEqual(velocity[0], .5, delta=.08)
        self.assertAlmostEqual(estimate[2], 1., delta=.03)
        self.assertTrue(np.allclose(tracker.covariance, tracker.covariance.T, atol=1e-10))
        self.assertGreaterEqual(float(np.linalg.eigvalsh(tracker.covariance).min()), -1e-10)

    def test_kalman_tracker_gates_pose_outlier(self):
        tracker = KalmanTargetTracker()
        tracker.update((0, 0, 0), .1, measurement_std=.05)
        estimate, _ = tracker.update((20, 0, 0), .1, measurement_std=.05)
        self.assertFalse(tracker.last_measurement_accepted)
        self.assertGreater(tracker.last_mahalanobis, tracker.gate_threshold)
        self.assertLess(estimate[0], 1.)

    def test_guidance_predicts_moving_target_intercept(self):
        guidance = Guidance()
        current = np.array((0., 0., 5.))
        target = np.array((1., 0., 0.))
        velocity = np.array((1., 0., 0.))
        moving = guidance.waypoint(current,target,velocity,"APPROACH",moving=True)
        stationary = guidance.waypoint(current,target,velocity,"APPROACH",moving=False)
        self.assertGreater(moving[0],stationary[0])
        self.assertAlmostEqual(moving[2],5.)


class MissionTests(unittest.TestCase):
    def test_address_validation_and_takeoff_gate(self):
        mission = Mission()
        with self.assertRaises(ValueError):
            mission.select(11, 0.0)
        mission.select(1, 0.0)
        phase = mission.update(0.1, (0, 0, 4), (0, 0, 0))
        self.assertEqual(phase, Phase.SEARCH)

    def test_timeout_aborts(self):
        mission = Mission()
        mission.select(1, 0.0)
        phase = mission.update(181.0, (0, 0, 1), (0, 0, 0))
        self.assertEqual(phase, Phase.ABORT)


if __name__ == "__main__":
    unittest.main()
