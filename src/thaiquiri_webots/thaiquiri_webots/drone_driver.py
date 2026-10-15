"""ROS 2 driver for the Thaiquiri quadrotor simulated in Webots."""

import math

from geometry_msgs.msg import Twist

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Imu, JointState

from webots_ros2_msgs.msg import FloatStamped


def quaternion_to_roll_pitch(q):
    """Return body roll/pitch in radians, or None for an invalid attitude."""
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if not math.isfinite(norm) or norm < 1e-9:
        return None
    x, y, z, w = (v / norm for v in (q.x, q.y, q.z, q.w))
    return (math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y)),
            math.asin(max(-1.0, min(1.0, 2*(w*y - z*x)))))


def mix_motor_speeds(thrust, roll, pitch, yaw, limits):
    """Mix FL/FR/RL/RR thrust corrections, preserving propeller spin directions."""
    magnitudes = (thrust - roll + pitch - yaw,
                  thrust + roll + pitch + yaw,
                  thrust - roll - pitch + yaw,
                  thrust + roll - pitch - yaw)
    return [sign * max(0.0, min(value, limit))
            for sign, value, limit in zip((-1, 1, 1, -1), magnitudes, limits)]


class DroneDriver:
    """Translate ROS 2 flight commands into Webots motor velocities."""

    def init(self, webots_node, properties):
        """Initialize Webots devices, controller gains, and ROS interfaces.

        Args:
            webots_node: Webots ROS 2 node that owns the simulated robot.
            properties: Controller properties supplied by the Webots launcher.
                The current driver does not require any custom properties.
        """
        self.__robot = webots_node.robot

        # Configure the altimeter and use the robot simulation step for sampling.
        self.__timestep = int(self.__robot.getBasicTimeStep())
        self.__devices = {
            'altimeter': self.__robot.getDevice('altimeter_cube'),
        }
        self.__devices['altimeter'].enable(self.__timestep)

        # Initialize the four propeller motors used by the quadrotor.
        self.__motors = {
            'front_left':  self.__robot.getDevice('motor_propeller_front_left'),
            'front_right': self.__robot.getDevice('motor_propeller_front_right'),
            'rear_left':   self.__robot.getDevice('motor_propeller_rear_left'),
            'rear_right':  self.__robot.getDevice('motor_propeller_rear_right')
        }

        # Wait for a valid attitude before spinning the propellers.
        for motor in self.__motors.values():
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)

        # Controller gains and the initial altitude setpoint.
        self.k_vertical_thrust = 469.0
        self.k_vertical_p = 60.0
        self.k_vertical_d = 35.0
        self.k_vertical_i = 10.0
        self.k_roll_p = 180.0
        self.k_pitch_p = 180.0
        self.k_attitude_d = 40.0
        self.k_attitude_i = 30.0
        self.k_yaw_d = 10.0
        self.target_altitude = 0.5
        self.__last_altitude = None
        self.__altitude_integral = 0.0
        self.__roll_integral = 0.0
        self.__pitch_integral = 0.0
        # Feed-forward trim for the PROTO's forward-heavy payload (~14 mm CoM offset).
        # Integral feedback corrects residual imbalance; these are motor-speed offsets.
        self.roll_trim = -1.0
        self.pitch_trim = 11.0

        # Latest command and sensor messages used by the control loop.
        self.__target_twist = Twist()
        self.__imu_data = None
        self.altimeter_msg = FloatStamped()
        self.velocity_motor_msg = JointState()

        # ROS 2 node, command/sensor subscriptions, and telemetry publishers.
        if not rclpy.ok():
            rclpy.init(args=None)
        self.__node = rclpy.create_node('thaiquiri_driver', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.__node.create_subscription(
            Twist, 'cmd_pos', self.__cmd_vel_callback, 1)
        self.__node.create_subscription(
            Imu, '/imu/data', self.__imu_callback, qos_profile_sensor_data)
        self.__altimeter_pub = self.__node.create_publisher(
            FloatStamped, '/drone/altimeter_cube', 10)
        self.__velocity_pub = self.__node.create_publisher(
            JointState, '/drone/velocity', 10)

    def __cmd_vel_callback(self, twist):
        """Store the latest desired translational and yaw commands."""
        self.__target_twist = twist

    def __imu_callback(self, imu_msg):
        """Store the latest inertial measurement used for attitude control."""
        self.__imu_data = imu_msg

    def clamp(self, n, min_n, max_n):
        """Constrain ``n`` to the inclusive interval ``[min_n, max_n]``."""
        return max(min_n, min(n, max_n))

    def step(self):
        """Process ROS messages, update control outputs, and publish telemetry.

        The attitude and altitude errors are converted into four motor commands
        using proportional control and the quadrotor motor-mixing equations.
        """
        # Service clock, attitude, and command callbacks without accumulating lag.
        try:
            for _ in range(4):
                if not rclpy.ok():
                    return
                rclpy.spin_once(self.__node, timeout_sec=0)
        except ExternalShutdownException:
            return

        attitude = (quaternion_to_roll_pitch(self.__imu_data.orientation)
                    if self.__imu_data is not None else None)
        if attitude is None:
            for motor in self.__motors.values():
                motor.setVelocity(0.0)
            return
        roll, pitch = attitude
        altitude = self.__devices['altimeter'].getValue()
        roll_rate = self.__imu_data.angular_velocity.x
        pitch_rate = self.__imu_data.angular_velocity.y

        self.altimeter_msg.header.stamp = self.__node.get_clock().now().to_msg()
        self.altimeter_msg.header.frame_id = 'altimeter_cube'
        self.altimeter_msg.data = altitude

        self.__altimeter_pub.publish(self.altimeter_msg)

        pitch_disturbance = self.__target_twist.linear.x
        roll_disturbance = self.__target_twist.linear.y
        self.target_altitude += self.__target_twist.linear.z
        yaw_disturbance = self.__target_twist.angular.z
        self.__target_twist.linear.z = 0.0
        self.__target_twist.angular.z = self.__target_twist.angular.z * \
            0.7 if abs(self.__target_twist.angular.z) > 0 else 0

        dt = self.__timestep * 0.001
        roll_error = roll + roll_disturbance / self.k_roll_p
        pitch_error = pitch + pitch_disturbance / self.k_pitch_p
        self.__roll_integral = self.clamp(self.__roll_integral + roll_error * dt, -0.5, 0.5)
        self.__pitch_integral = self.clamp(self.__pitch_integral + pitch_error * dt, -0.5, 0.5)
        roll_input = self.k_roll_p * \
            self.clamp(roll, -1.0, 1.0) + self.k_attitude_d * roll_rate + roll_disturbance \
            + self.k_attitude_i * self.__roll_integral + self.roll_trim
        pitch_input = self.k_pitch_p * \
            self.clamp(pitch, -1.0, 1.0) + \
            self.k_attitude_d * pitch_rate + pitch_disturbance \
            + self.k_attitude_i * self.__pitch_integral + self.pitch_trim
        yaw_input = yaw_disturbance + self.k_yaw_d * self.__imu_data.angular_velocity.z
        vertical_rate = (0.0 if self.__last_altitude is None else
                         (altitude - self.__last_altitude) / dt)
        self.__last_altitude = altitude
        altitude_error = self.clamp(self.target_altitude - altitude, -1.0, 1.0)
        self.__altitude_integral = self.clamp(
            self.__altitude_integral + altitude_error * dt, -2.0, 2.0)
        vertical_input = (self.k_vertical_p * altitude_error
                          - self.k_vertical_d * vertical_rate
                          + self.k_vertical_i * self.__altitude_integral)

        speeds = mix_motor_speeds(
            self.k_vertical_thrust + vertical_input, roll_input, pitch_input, yaw_input,
            [motor.getMaxVelocity() for motor in self.__motors.values()])
        for motor, speed in zip(self.__motors.values(), speeds):
            motor.setVelocity(speed)

        self.velocity_motor_msg.header.stamp = self.__node.get_clock().now().to_msg()
        self.velocity_motor_msg.name = [
            'front_left', 'front_right', 'rear_left', 'rear_right']
        self.velocity_motor_msg.velocity = speeds
        self.velocity_motor_msg.effort = [0.0, 0.0, 0.0, 0.0]

        self.__velocity_pub.publish(self.velocity_motor_msg)
