"""Message-shaped test doubles for pure driver tests when ROS is unavailable.

No transport, clock, executor or synchronization implementation is provided.
These tests check controller/callback logic, not ROS integration.
"""

from dataclasses import dataclass, field
import sys
from types import ModuleType


@dataclass
class Time:
    sec: int = 0
    nanosec: int = 0


@dataclass
class Header:
    stamp: Time = field(default_factory=Time)
    frame_id: str = ''


@dataclass
class Vector3:
    x: float = 0.
    y: float = 0.
    z: float = 0.


@dataclass
class Quaternion:
    x: float = 0.
    y: float = 0.
    z: float = 0.
    w: float = 1.


@dataclass
class Twist:
    linear: Vector3 = field(default_factory=Vector3)
    angular: Vector3 = field(default_factory=Vector3)


@dataclass
class Imu:
    header: Header = field(default_factory=Header)
    orientation: Quaternion = field(default_factory=Quaternion)
    angular_velocity: Vector3 = field(default_factory=Vector3)
    linear_acceleration: Vector3 = field(default_factory=Vector3)
    orientation_covariance: list = field(default_factory=lambda: [0.]*9)
    angular_velocity_covariance: list = field(default_factory=lambda: [0.]*9)
    linear_acceleration_covariance: list = field(default_factory=lambda: [0.]*9)


@dataclass
class JointState:
    header: Header = field(default_factory=Header)
    name: list = field(default_factory=list)
    velocity: list = field(default_factory=list)
    effort: list = field(default_factory=list)


@dataclass
class FloatStamped:
    header: Header = field(default_factory=Header)
    data: float = 0.


class Parameter:
    def __init__(self, name, value):
        self.name, self.value = name, value


def install():
    def module(name, **attributes):
        result = ModuleType(name)
        result.__dict__.update(attributes)
        sys.modules[name] = result
        return result

    module('builtin_interfaces', msg=module('builtin_interfaces.msg', Time=Time))
    module('geometry_msgs', msg=module('geometry_msgs.msg', Quaternion=Quaternion, Twist=Twist))
    module('sensor_msgs', msg=module('sensor_msgs.msg', Imu=Imu, JointState=JointState))
    module('webots_ros2_msgs', msg=module('webots_ros2_msgs.msg', FloatStamped=FloatStamped))
    ros = module('rclpy')
    ros.executors = module('rclpy.executors', ExternalShutdownException=type('ExternalShutdownException', (Exception,), {}))
    ros.node = module('rclpy.node', Node=object)
    ros.parameter = module('rclpy.parameter', Parameter=Parameter)
    ros.qos = module('rclpy.qos', qos_profile_sensor_data=object())
    module('message_filters')
