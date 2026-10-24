"""Small PyBullet integration checks; run separately from the pure math suite."""
import unittest

import numpy as np
import pybullet as p

from config import GRAVITY
from simulation import LandingSimulation


class RotorAllocationTests(unittest.TestCase):
    def test_hover_request_allocates_equal_motors(self):
        sim = LandingSimulation(gui=False, seed=17)
        try:
            rpm, saturated = sim.drone._allocate(sim.drone.mass * GRAVITY, np.zeros(3))
            np.testing.assert_allclose(rpm, np.full(4, rpm[0]), rtol=0, atol=1e-8)
            self.assertFalse(saturated)
        finally:
            sim.close()

    def test_truck_contact_carries_kinematic_platform_velocity(self):
        sim = LandingSimulation(gui=False, seed=7)
        try:
            number = 3
            t = 0.0
            sim.world.update(t)
            target = sim.world.target_position(number, t)
            velocity = sim.world.target_velocity(number, t)
            p.resetBasePositionAndOrientation(
                sim.drone.body, (target[0], target[1], target[2] + .08), (0, 0, 0, 1),
                physicsClientId=sim.client,
            )
            p.resetBaseVelocity(sim.drone.body, velocity.tolist(), (0, 0, 0), physicsClientId=sim.client)
            for _ in range(240):
                t += 1 / 120
                sim.world.update(t)
                p.stepSimulation(physicsClientId=sim.client)
            position = np.asarray(p.getBasePositionAndOrientation(sim.drone.body, physicsClientId=sim.client)[0])
            target = sim.world.target_position(number, t)
            contact = p.getContactPoints(bodyA=sim.drone.body, bodyB=sim.world.pad_ids[number], physicsClientId=sim.client)
            self.assertTrue(contact, "vehicle lost deck contact")
            self.assertLess(np.linalg.norm(position[:2] - target[:2]), .02)
        finally:
            sim.close()


if __name__ == "__main__":
    unittest.main()
