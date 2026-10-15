import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Importación del mensaje específico de ArduPilot
from ardupilot_msgs.msg import GlobalPosition


class GpsPosePublisher(Node):
    def __init__(self):
        super().__init__('gps_pose_publisher')

        # Perfil QoS BEST_EFFORT requerido por ArduPilot para sensores externos
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.publisher_ = self.create_publisher(
            GlobalPosition,
            '/ap/cmd_gps_pose',
            qos_profile
        )

        # Publicar a 5 Hz para inicializar rápidamente el origen global en el EKF3
        self.timer = self.create_timer(0.2, self.timer_callback)
        self.get_logger().info('Publicando origen global (Lat/Lon) en /ap/cmd_gps_pose...')

    def timer_callback(self):
        msg = GlobalPosition()

        # CRÍTICO: Heredar el tiempo de simulación de Webots
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        msg.coordiante_frame = GlobalPosition.FRAME_GLOBAL_REL_ALT

        msg.type_mask = (
            GlobalPosition.IGNORE_LATITUDE |
            GlobalPosition.IGNORE_LONGITUDE |
            GlobalPosition.IGNORE_ALTITUDE |
            GlobalPosition.IGNORE_VX |
            GlobalPosition.IGNORE_VY |
            GlobalPosition.IGNORE_VZ |
            GlobalPosition.IGNORE_AFX |
            GlobalPosition.IGNORE_AFY |
            GlobalPosition.IGNORE_AFZ |
            GlobalPosition.IGNORE_YAW |
            GlobalPosition.IGNORE_YAW_RATE
        )
        # Coordenadas geográficas estáticas de origen (Sucre, Bolivia)
        msg.latitude = 0.
        msg.longitude = 0.
        msg.altitude = 0.  # Altitud en metros sobre el nivel del mar
        msg.yaw = 0.  # Yaw en radianes (0 rad = Norte)

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GpsPosePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
