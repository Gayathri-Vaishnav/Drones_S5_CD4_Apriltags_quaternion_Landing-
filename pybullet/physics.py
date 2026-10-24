"""Crazyflie-class rigid body with rotor-level actuation and cascaded control."""
from __future__ import annotations

import math

import numpy as np
import pybullet as p

from config import (DT, GRAVITY, MASS_KG, MAX_TILT_DEG, MAX_HORIZONTAL_SPEED_M_S,
                    MAX_VERTICAL_SPEED_M_S, MAX_LANDING_SPEED_M_S, VEHICLE_PARAMS)
from config import (AIR_DENSITY_KG_M3, DRAG_CD_A_XY_M2, DRAG_CD_A_Z_M2,
                    WIND_MEAN_M_S, WIND_GUST_STD_M_S, WIND_CORRELATION_TIME_S,
                    MAX_WIND_GUST_M_S)
from control.attitude import acceleration_to_quaternion, quaternion_error

# Keep the flight model unchanged while making the drone easier to see in the
# rendered PyBullet scene.
VISUAL_SCALE = 8.0


class Quadrotor:
    """Rigid-body quadrotor with independent motors and X-frame allocation.

    PyBullet integrates the body rigidly. Each motor contributes a quadratic
    thrust force at its body-frame arm location and a signed reaction torque.
    The controller is position/velocity -> attitude -> body-rate -> mixer.
    """

    def __init__(self, client, start=(0, 0, 4), seed=4, rng=None):
        self.client = client
        self.params = VEHICLE_PARAMS
        self.mass = MASS_KG
        self.kf = float(self.params["thrust_coefficient_N_per_rad_s2"])
        self.km = float(self.params["reaction_torque_coefficient_Nm_per_rad_s2"])
        self.max_motor_thrust = float(self.params["maximum_thrust_per_rotor_N"])
        self.max_rpm = float(self.params["maximum_rotor_speed_rpm"])
        self.motor_tau = float(self.params["motor_time_constant_s"])
        arm = float(self.params["rotor_axis_offset_m"])
        # Motor order: front-right, rear-right, front-left, rear-left.
        self.motor_offsets = np.array(((arm, arm), (arm, -arm), (-arm, arm), (-arm, -arm)), dtype=float)
        self.spin_directions = np.array((1.0, -1.0, -1.0, 1.0))
        self.inertia = np.asarray(self.params["inertia_diagonal_kg_m2"], dtype=float)

        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=(.022, .022, .010), physicsClientId=client)
        shell_vertices = []
        shell_indices = []
        segments = 16
        for y, rx, rz in ((.010, .008, .004), (.005, .025, .009), (-.012, .022, .008), (-.027, .012, .006)):
            for k in range(segments):
                angle = 2 * math.pi * k / segments
                shell_vertices.append((VISUAL_SCALE * rx * math.cos(angle), VISUAL_SCALE * y,
                                       VISUAL_SCALE * (.0012 + rz * math.sin(angle))))
        for ring in range(3):
            for k in range(segments):
                a = ring * segments + k
                b = ring * segments + (k + 1) % segments
                c = (ring + 1) * segments + (k + 1) % segments
                d = (ring + 1) * segments + k
                shell_indices.extend((a, b, c, a, c, d))
        shell = p.createVisualShape(p.GEOM_MESH, vertices=shell_vertices, indices=shell_indices,
                                    rgbaColor=(.89, .90, .88, 1), physicsClientId=client)
        self.body = p.createMultiBody(self.mass, col, shell, start, physicsClientId=client)
        p.changeDynamics(self.body, -1, localInertiaDiagonal=self.inertia.tolist(),
                          lateralFriction=.8, spinningFriction=.01, rollingFriction=.001,
                          restitution=0.0, linearDamping=0.0, angularDamping=0.0,
                          contactStiffness=20000.0, contactDamping=40.0, physicsClientId=client)

        self.visual_parts = []
        self.rotors = []
        motor_span = VISUAL_SCALE * math.sqrt(2.0) * 2.0 * arm
        arm_x = p.createVisualShape(p.GEOM_CYLINDER, radius=.0025 * VISUAL_SCALE, length=motor_span,
                                    rgbaColor=(.78, .80, .81, 1), physicsClientId=client)
        arm_y = p.createVisualShape(p.GEOM_CYLINDER, radius=.0025 * VISUAL_SCALE, length=motor_span,
                                    rgbaColor=(.78, .80, .81, 1), physicsClientId=client)
        motor = p.createVisualShape(p.GEOM_CYLINDER, radius=.006 * VISUAL_SCALE, length=.006 * VISUAL_SCALE,
                                    rgbaColor=(.87, .88, .86, 1), physicsClientId=client)
        prop = p.createVisualShape(p.GEOM_BOX, halfExtents=tuple(VISUAL_SCALE * v for v in (.023, .0015, .0005)),
                                   rgbaColor=(.075, .083, .095, 1), physicsClientId=client)
        leg = p.createVisualShape(p.GEOM_CYLINDER, radius=.0015 * VISUAL_SCALE, length=.018 * VISUAL_SCALE,
                                  rgbaColor=(.74, .77, .79, 1), physicsClientId=client)
        camera = p.createVisualShape(p.GEOM_BOX, halfExtents=tuple(VISUAL_SCALE * v for v in (.006, .008, .004)),
                                     rgbaColor=(.19, .22, .25, 1), physicsClientId=client)
        lens = p.createVisualShape(p.GEOM_CYLINDER, radius=.0025 * VISUAL_SCALE, length=.0035 * VISUAL_SCALE,
                                  rgbaColor=(.04, .07, .10, 1), physicsClientId=client)
        self._make_part(arm_x, start, (0, 0, .001 * VISUAL_SCALE), p.getQuaternionFromEuler((0, math.pi / 2, 0)))
        self._make_part(arm_y, start, (0, 0, .001 * VISUAL_SCALE), p.getQuaternionFromEuler((math.pi / 2, 0, 0)))
        for x, y in self.motor_offsets:
            visual_offset = np.array((x, y, 0.0)) * VISUAL_SCALE
            self._make_part(motor, start, (*visual_offset[:2], .002 * VISUAL_SCALE))
            self._make_part(leg, start, (x * .78 * VISUAL_SCALE, y * .78 * VISUAL_SCALE, -.011 * VISUAL_SCALE))
            for phase in (0, math.pi / 2):
                blade = self._make_part(prop, start, (*visual_offset[:2], .006 * VISUAL_SCALE), p.getQuaternionFromEuler((0, 0, phase)))
                self.rotors.append((blade, phase, int(self.spin_directions[len(self.rotors) // 2])))
        self._make_part(camera, start, (0, -.022 * VISUAL_SCALE, -.006 * VISUAL_SCALE))
        self._make_part(lens, start, (0, -.030 * VISUAL_SCALE, -.006 * VISUAL_SCALE), p.getQuaternionFromEuler((math.pi / 2, 0, 0)))

        self._prop_angle = 0.0
        hover_force = self.mass * GRAVITY / 4.0
        self.motor_rpm = np.full(4, self._rpm_for_thrust(hover_force), dtype=float)
        self.motor_command_rpm = self.motor_rpm.copy()
        self.thrust = self.mass * GRAVITY
        self.integral = np.zeros(3)
        self.rate_integral = np.zeros(3)
        self.prev_error = np.zeros(3)
        self.have_prev = False
        self.enable_gain_schedule = True
        self.enable_feedforward = True
        self.surface_height = 0.0
        self.wind_scale = 1.0
        self.rng = rng if rng is not None else np.random.default_rng(seed)
        self.wind_mean = np.asarray(WIND_MEAN_M_S,dtype=float)
        self.wind = self.wind_mean.copy()
        self.motor_saturated = False
        self.max_tilt = 0.0

    def _make_part(self, shape, position, offset, orientation=(0, 0, 0, 1)):
        body = p.createMultiBody(0, -1, shape, position, orientation, physicsClientId=self.client)
        self.visual_parts.append((body, np.asarray(offset, dtype=float), np.asarray(orientation, dtype=float)))
        return body

    def state(self):
        pos, orn = p.getBasePositionAndOrientation(self.body, physicsClientId=self.client)
        rot = np.asarray(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
        rotor_state = {body: (phase, direction) for body, phase, direction in self.rotors}
        for body, offset, local_orn in self.visual_parts:
            point = np.asarray(pos) + rot @ offset
            part_orn = p.multiplyTransforms((0, 0, 0), orn, (0, 0, 0), local_orn)[1]
            if body in rotor_state:
                phase, direction = rotor_state[body]
                spin = p.getQuaternionFromEuler((0, 0, phase + direction * self._prop_angle))
                part_orn = p.multiplyTransforms((0, 0, 0), part_orn, (0, 0, 0), spin)[1]
            p.resetBasePositionAndOrientation(body, point, part_orn, physicsClientId=self.client)
        vel, omega = p.getBaseVelocity(self.body, physicsClientId=self.client)
        return np.asarray(pos), np.asarray(orn), np.asarray(vel), np.asarray(omega)

    @staticmethod
    def quat_matrix(q):
        return np.asarray(p.getMatrixFromQuaternion(q)).reshape(3, 3)

    def _rpm_for_thrust(self, thrust):
        omega = math.sqrt(max(0.0, float(thrust)) / self.kf)
        rpm = omega * 60.0 / (2.0 * math.pi)
        return float(np.clip(rpm, 0.0, self.max_rpm))

    def _allocate(self, total_thrust, torque_body):
        arm = float(self.params["rotor_axis_offset_m"])
        yaw_arm = self.km / self.kf
        mixer = np.array((
            (1.0, 1.0, 1.0, 1.0),
            (arm, -arm, arm, -arm),
            (-arm, -arm, arm, arm),
            self.spin_directions * yaw_arm,
        ))
        requested = np.array((total_thrust, *torque_body), dtype=float)
        motor_thrust = np.linalg.solve(mixer, requested)
        saturated = bool(np.any(motor_thrust < 0.0) or np.any(motor_thrust > self.max_motor_thrust))
        motor_thrust = np.clip(motor_thrust, 0.0, self.max_motor_thrust)
        rpm = np.array([self._rpm_for_thrust(value) for value in motor_thrust])
        return rpm, saturated

    def control(self, target_xyz, target_velocity=(0, 0, 0), descend=False):
        self._prop_angle = (self._prop_angle + DT * 42.0) % (2.0 * math.pi)
        pos, orn, vel, omega_world = self.state()
        rot = self.quat_matrix(orn)
        error = np.asarray(target_xyz, dtype=float) - pos
        vel_ref = np.asarray(target_velocity, dtype=float) if self.enable_feedforward else np.zeros(3)

        low_altitude = (pos[2] - self.surface_height) < 2.0
        speed_limit = MAX_LANDING_SPEED_M_S if low_altitude else MAX_HORIZONTAL_SPEED_M_S
        kp_pos_xy = 1.3 if self.enable_gain_schedule and low_altitude else 1.0
        kp_vel_xy = 2.0 if self.enable_gain_schedule and low_altitude else 1.8
        ki_xy = .18 if self.enable_gain_schedule and low_altitude else .12
        position_velocity_xy = kp_pos_xy * error[:2]
        desired_speed_xy = float(np.linalg.norm(position_velocity_xy))
        stopping_speed_xy = math.sqrt(2.0 * 1.0 * float(np.linalg.norm(error[:2])))
        allowed_speed_xy = min(speed_limit, stopping_speed_xy)
        if desired_speed_xy > allowed_speed_xy:
            position_velocity_xy *= allowed_speed_xy / desired_speed_xy
        desired_velocity_xy = position_velocity_xy + vel_ref[:2]
        total_speed_xy = float(np.linalg.norm(desired_velocity_xy))
        if total_speed_xy > speed_limit:
            desired_velocity_xy *= speed_limit / total_speed_xy
        stopping_speed_z = math.sqrt(2.0 * 1.0 * abs(float(error[2])))
        allowed_speed_z = min(MAX_VERTICAL_SPEED_M_S, stopping_speed_z)
        desired_vertical_speed = float(np.clip(1.2 * error[2], -allowed_speed_z, allowed_speed_z) + vel_ref[2])
        desired_vertical_speed = float(np.clip(desired_vertical_speed,
                                               -MAX_VERTICAL_SPEED_M_S, MAX_VERTICAL_SPEED_M_S))
        candidate_integral = np.clip(self.integral + error * DT, -1.5, 1.5)
        desired_acc = np.array((kp_vel_xy * (desired_velocity_xy[0] - vel[0]) + ki_xy * candidate_integral[0],
                                kp_vel_xy * (desired_velocity_xy[1] - vel[1]) + ki_xy * candidate_integral[1],
                                2.1 * (desired_vertical_speed - vel[2]) + .25 * candidate_integral[2]))
        clipped_acc = np.clip(desired_acc, (-4.0, -4.0, -3.0), (4.0, 4.0, 3.0))
        position_saturated = not np.allclose(desired_acc, clipped_acc)
        if not position_saturated:
            self.integral = candidate_integral

        yaw = p.getEulerFromQuaternion(orn)[2]
        target_orn = acceleration_to_quaternion(clipped_acc, yaw)
        roll, pitch, _ = p.getEulerFromQuaternion(target_orn)
        self.max_tilt = max(self.max_tilt, abs(float(roll)), abs(float(pitch)))
        angle_error_world = quaternion_error(orn, target_orn)
        angle_error_body = rot.T @ angle_error_world
        rate_command = np.clip(5.0 * angle_error_body, -8.0, 8.0)
        rate_body = rot.T @ omega_world
        rate_error = rate_command - rate_body
        prior_rate_integral = self.rate_integral.copy()
        candidate_rate_integral = np.clip(self.rate_integral + rate_error * DT, -1.0, 1.0)
        torque_body = .00022 * rate_error + .000015 * candidate_rate_integral
        torque_body = np.clip(torque_body, (-.003, -.003, -.0008), (.003, .003, .0008))

        desired_total_thrust = self.mass * (GRAVITY + clipped_acc[2]) / max(math.cos(roll) * math.cos(pitch), .65)
        desired_total_thrust = float(np.clip(desired_total_thrust, 0.0, 4.0 * self.max_motor_thrust))
        rpm_command, saturated = self._allocate(desired_total_thrust, torque_body)
        self.motor_saturated = saturated or position_saturated
        if self.motor_saturated:
            self.rate_integral = prior_rate_integral
            # Conditional integration: preserve the position integrator when
            # motor authority or acceleration limits are active.
            if not position_saturated:
                self.integral = np.clip(self.integral - error * DT, -1.5, 1.5)
        else:
            self.rate_integral = candidate_rate_integral
        self.motor_command_rpm = rpm_command
        alpha = min(1.0, DT / max(self.motor_tau, DT))
        self.motor_rpm += alpha * (self.motor_command_rpm - self.motor_rpm)

        rpm_rad_s = self.motor_rpm * (2.0 * math.pi / 60.0)
        motor_thrust = np.minimum(self.kf * rpm_rad_s ** 2, self.max_motor_thrust)
        self.thrust = float(np.sum(motor_thrust))
        height = max(float(pos[2] - self.surface_height), .08)
        ground_effect = 1.0 + .06 * math.exp(-height / .28)
        wind_alpha=math.exp(-DT/max(WIND_CORRELATION_TIME_S,DT))
        wind_noise_std=WIND_GUST_STD_M_S*math.sqrt(max(0.,1.-wind_alpha**2))*self.wind_scale
        self.wind=self.wind_mean+wind_alpha*(self.wind-self.wind_mean)+self.rng.normal(0.0,wind_noise_std,3)
        self.wind=np.clip(self.wind,self.wind_mean-MAX_WIND_GUST_M_S,self.wind_mean+MAX_WIND_GUST_M_S)
        relative_air_velocity=vel-self.wind
        drag_coefficients=.5*AIR_DENSITY_KG_M3*np.array((DRAG_CD_A_XY_M2,DRAG_CD_A_XY_M2,DRAG_CD_A_Z_M2))
        aerodynamic_drag=-drag_coefficients*np.abs(relative_air_velocity)*relative_air_velocity

        for index, ((x, y), thrust, speed, spin) in enumerate(
                zip(self.motor_offsets, motor_thrust, rpm_rad_s, self.spin_directions)):
            offset_world = rot @ np.array((x, y, 0.0))
            force_world = rot @ np.array((0.0, 0.0, float(thrust * ground_effect)))
            p.applyExternalForce(self.body, -1, force_world.tolist(),
                                 (pos + offset_world).tolist(), p.WORLD_FRAME, physicsClientId=self.client)
            reaction_body = np.array((0.0, 0.0, spin * self.km * float(speed * speed)))
            p.applyExternalTorque(self.body, -1, (rot @ reaction_body).tolist(),
                                  p.WORLD_FRAME, physicsClientId=self.client)
        p.applyExternalForce(self.body, -1, aerodynamic_drag.tolist(), pos.tolist(), p.WORLD_FRAME, physicsClientId=self.client)
