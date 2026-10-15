import rclpy
from geometry_msgs.msg import Twist
from webots_ros2_msgs.msg import FloatStamped
from scipy.spatial.transform import Rotation as R
import numpy as np
from controller import Supervisor
from time import sleep
from tf2_ros import TransformBroadcaster, TransformStamped
from thaiquiri_webots.webots_vehicle import WebotsArduVehicle


class DroneDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.motors = ['motor_propeller_front_left', 'motor_propeller_front_right',
                       'motor_propeller_rear_left', 'motor_propeller_rear_right']

        # Devices
        self.__timestep = int(self.__robot.getBasicTimeStep())
        self.__devices = {

            'altimeter': self.__robot.getDevice('altimeter_cube'),
        }
        self.__devices['altimeter'].enable(self.__timestep)

        # # 1. Inicializar los 4 motores
        # self.__motors = {
        #     'front_left':  self.__robot.getDevice('motor_propeller_front_left'),
        #     # *-1
        #     'front_right': self.__robot.getDevice('motor_propeller_front_right'),
        #     'rear_left':   self.__robot.getDevice('motor_propeller_rear_left'),
        #     # *-1
        #     'rear_right':  self.__robot.getDevice('motor_propeller_rear_right')
        # }

        # Configurar motores para control por velocidad
        # for motor in self.__motors.values():
        #     motor.setPosition(float('inf'))
        #     motor.setVelocity(1.0)

        self.vehicle = WebotsArduVehicle(motor_names=self.motors,
                                         reversed_motors=None,
                                         accel_name='accelerometer_cube',
                                         imu_name=None,
                                         gyro_name='gyro_cube',
                                         gps_name=None,
                                         instance=0,
                                         bidirectional_motors=False,
                                         uses_propellers=True,
                                         sitl_address='172.18.0.3',
                                         robot=self.__robot)

        self.altimeter_msg = FloatStamped()

        # 6. Nodo ROS 2
        rclpy.init(args=None)
        self.__node = rclpy.create_node('thaiquiri_driver')
        self.__altimeter_pub = self.__node.create_publisher(
            FloatStamped, '/drone/altimeter_cube', 10)

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
        self.altimeter_msg.header.stamp = self.__node.get_clock().now().to_msg()
        self.altimeter_msg.header.frame_id = 'altimeter_cube'
        self.altimeter_msg.data = self.__devices['altimeter'].getValue()
        self.__altimeter_pub.publish(self.altimeter_msg)
