"""Merge native Cube inertial samples into a ROS forward-left-up IMU."""

import math

import message_filters

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Imu


class ImuMergerNode(Node):
    """Synchronize and rotate the co-located Cube accelerometer and gyro."""

    def __init__(self):
        """Create sensor subscriptions and startup-settling state."""
        super().__init__('imu_merger_node')
        self._settled_since = None
        self._initialized = False
        self._last_stamp_ns = None

        # Ajusta estos nombres según los tópicos reales que expone tu robot en Webots
        accel_topic = '/drone/cube/acc'
        gyro_topic = '/drone/cube/gyro'
        imu_out_topic = '/imu/data_raw'

        # Suscriptores usando message_filters para la sincronización
        self.accel_sub = message_filters.Subscriber(
            self, Imu, accel_topic, qos_profile=qos_profile_sensor_data)
        self.gyro_sub = message_filters.Subscriber(
            self, Imu, gyro_topic, qos_profile=qos_profile_sensor_data)

        # Both co-located Webots devices publish at 125 Hz on the same clock.
        # Approximate matching allowed 50 ms of gyro/acceleration skew, which
        # combines different attitudes during flight and mis-stamps the gyro.
        self.ts = message_filters.TimeSynchronizer(
            [self.accel_sub, self.gyro_sub], queue_size=50)
        self.ts.registerCallback(self.sync_callback)

        # Publicador del tópico combinado
        self.imu_pub = self.create_publisher(Imu, imu_out_topic, 10)
        self.get_logger().info('IMU Merger Node iniciado. Esperando datos sincronizados...')

    def sync_callback(self, accel_msg: Imu, gyro_msg: Imu):
        """Publish synchronized body-frame samples after initial settling."""
        stamp_ns = accel_msg.header.stamp.sec * 1000000000 + accel_msg.header.stamp.nanosec
        gyro_stamp_ns = gyro_msg.header.stamp.sec * 1000000000 + gyro_msg.header.stamp.nanosec
        a, w = accel_msg.linear_acceleration, gyro_msg.angular_velocity
        if (stamp_ns != gyro_stamp_ns
                or (self._last_stamp_ns is not None and stamp_ns <= self._last_stamp_ns)
                or not all(math.isfinite(v) for v in (a.x, a.y, a.z, w.x, w.y, w.z))):
            return
        self._last_stamp_ns = stamp_ns
        # Initialize Madgwick only after the spawn drop/contact impulse settles.
        # Once initialized, forward all dynamics (including free fall).
        if not self._initialized:
            stamp = accel_msg.header.stamp.sec + accel_msg.header.stamp.nanosec * 1e-9
            if (abs(math.sqrt(a.x*a.x + a.y*a.y + a.z*a.z) - 9.81) > 0.5
                    or math.sqrt(w.x*w.x + w.y*w.y + w.z*w.z) > 0.1):
                self._settled_since = None
                return
            if self._settled_since is None:
                self._settled_since = stamp
            if stamp - self._settled_since < 0.2:
                return
            self._initialized = True
        # Crear el mensaje Imu consolidado
        merged_imu = Imu()

        # 1. Asignar el Header (usamos el del acelerómetro como base)
        merged_imu.header.stamp = accel_msg.header.stamp
        merged_imu.header.frame_id = 'imu_link'

        # 2. Extraer Aceleración Lineal del tópico del acelerómetro
        # Webots publica la aceleración en el campo linear_acceleration
        # PROTO axes: x=left, y=up, z=forward. ROS body: x=forward, y=left, z=up.
        merged_imu.linear_acceleration.x = accel_msg.linear_acceleration.z
        merged_imu.linear_acceleration.y = accel_msg.linear_acceleration.x
        merged_imu.linear_acceleration.z = accel_msg.linear_acceleration.y

        # 3. Extraer Velocidad Angular del tópico del giroscopio
        # Webots publica la velocidad angular en el campo angular_velocity
        merged_imu.angular_velocity.x = gyro_msg.angular_velocity.z
        merged_imu.angular_velocity.y = gyro_msg.angular_velocity.x
        merged_imu.angular_velocity.z = gyro_msg.angular_velocity.y
        order = (2, 0, 1)
        merged_imu.linear_acceleration_covariance = [
            accel_msg.linear_acceleration_covariance[3 * i + j]
            for i in order for j in order]
        merged_imu.angular_velocity_covariance = [
            gyro_msg.angular_velocity_covariance[3 * i + j]
            for i in order for j in order]

        # 4. Matrices de covarianza (Opcional, pero recomendado)
        # Si tienes ruidos conocidos de tus sensores simulados, puedes añadirlos aquí.
        # Dejaremos -1 en el primer elemento de la covarianza de orientación para indicar
        # que este mensaje no provee orientación (eso lo hará Madgwick después).
        merged_imu.orientation_covariance[0] = -1.0

        # Publicar el mensaje unificado
        self.imu_pub.publish(merged_imu)


def main(args=None):
    """Run the IMU merger until launch shuts down."""
    rclpy.init(args=args)
    node = ImuMergerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
