"""ROS-independent route fixture for isolating control from localization."""

import numpy as np
from scipy.spatial.transform import Rotation


class RouteController:
    """Apply the same route law to an explicitly selected feedback source."""

    def __init__(self, forward_m, desired_speed_limit=.2, enabled=True):
        if not np.isfinite(desired_speed_limit) or desired_speed_limit <= 0:
            raise ValueError('Desired speed limit must be finite and positive')
        self.forward_m = forward_m
        self.desired_speed_limit = desired_speed_limit
        self.enabled = enabled
        self.sample = None
        self.origin = None
        self.route_rotation = None
        self.status = 'inactive'
        self.return_hover = False

    def observe(self, pose, body_velocity):
        """Retain finite, ordered poses and body-frame velocity from one source."""
        sample = np.concatenate((pose, body_velocity)).astype(float)
        if (sample.shape != (11,) or not np.isfinite(sample).all()
                or np.linalg.norm(sample[4:8]) < 1e-6
                or (self.sample is not None and sample[0] <= self.sample[0])):
            return False
        self.sample = sample
        if self.origin is None and sample[0] >= 15:
            self.origin = sample[1:4].copy()
            self.route_rotation = Rotation.from_quat(sample[4:8])
        return True

    def command(self, now):
        """Return pitch/roll disturbances; stale feedback cancels route commands."""
        if not self.enabled or now < 20:
            self.status = 'hover' if not self.enabled else 'inactive'
            return 0., 0.
        if self.sample is None or self.origin is None:
            self.status = 'waiting_for_feedback'
            return 0., 0.
        if not np.isfinite(now) or not 0 <= now - self.sample[0] < .5:
            self.status = 'stale_feedback'
            return 0., 0.
        self.status = 'tracking'
        forward = self.forward_m if now < 70 else 0.
        goal = self.origin + self.route_rotation.apply([forward, 0, 0])
        rotation = Rotation.from_quat(self.sample[4:8])
        error = rotation.inv().apply(goal - self.sample[1:4])
        # Once the return waypoint is reached, stop feeding ICP velocity
        # noise through the attitude loop. Hysteresis resumes tracking if the
        # vehicle actually drifts away; this is not a timed stop at trial end.
        distance = np.linalg.norm(error[:2])
        speed = np.linalg.norm(self.sample[8:10])
        if now >= 70:
            if self.return_hover:
                self.return_hover = distance <= .20
            elif distance <= .10 and speed <= .12:
                self.return_hover = True
            if self.return_hover:
                self.status = 'settled'
                return 0., 0.
        desired_velocity = np.clip(.5 * error[:2],
                                    -self.desired_speed_limit,
                                    self.desired_speed_limit)
        # This limits the requested speed, not the measured vehicle speed.
        acceleration = np.clip(1.2 * (desired_velocity - self.sample[8:10]),
                               -.45, .45)
        pitch, roll = acceleration[0] / 9.81, -acceleration[1] / 9.81
        return float(-180 * pitch), float(-180 * roll)
