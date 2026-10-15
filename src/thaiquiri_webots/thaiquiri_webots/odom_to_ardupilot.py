import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from scipy.spatial.transform import Rotation as R


class OdomToArdupilotTF(Node):
    """
    Reenvía la pose de nav_msgs/Odometry como TF dinámico en /ap/tf.

    Por defecto:
      entrada: /odom
      salida:  /ap/tf
      parent:  odom
      child:   base_link

    Este nodo copia posición y orientación; no transforma ejes ni frames.
    Asegúrate de que la odometría de entrada usa las convenciones correctas.
    """

    def __init__(self):
        super().__init__('odom_to_ardupilot_tf')

        self.declare_parameter('odom_topic', '/drone/odom')
        self.declare_parameter('tf_topic', '/ap/tf')
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame', 'base_footprint')

        odom_topic = self.get_parameter('odom_topic').value
        tf_topic = self.get_parameter('tf_topic').value

        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value

        self.publisher = self.create_publisher(
            TFMessage,
            tf_topic,
            # QoSProfile(
            #     reliability=ReliabilityPolicy.BEST_EFFORT,
            #     history=HistoryPolicy.KEEP_LAST,
            #     depth=10
            # )
            10
        )

        self.subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
        )

        self.get_logger().info(
            f'Escuchando {odom_topic}; publicando TF en {tf_topic} '
            f'({self.parent_frame} -> {self.child_frame})'
        )

    def odom_callback(self, msg: Odometry):
        # Validar los frames para evitar publicar una transformación
        # incorrecta silenciosamente.
        if msg.header.frame_id and msg.header.frame_id != self.parent_frame:
            self.get_logger().warning(
                f'header.frame_id="{msg.header.frame_id}", '
                f'pero se espera "{self.parent_frame}". '
                'Verifica el árbol TF y la configuración.'
            )
            return

        if msg.child_frame_id and msg.child_frame_id != self.child_frame:
            self.get_logger().warning(
                f'child_frame_id="{msg.child_frame_id}", '
                f'pero se espera "{self.child_frame}". '
                'Verifica el árbol TF y la configuración.'
            )
            return

        q_orig = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]

        # 2. Tu rotación estática del URDF (rpy="1.5708 0 1.5708")
        # Convertido a cuaternión [x, y, z, w], esto equivale exactamente a:
        rot_urdf = R.from_euler('XYZ', [1.5708, 0, 1.5708], degrees=False)
        rot_orig = R.from_quat(q_orig)
        rot_aligned = rot_orig * rot_urdf.inv()

        # 3. PRE-VOLTEAR 180 grados para compensar la pasarela DDS de ArduPilot
        rot_flip = R.from_euler('XYZ', [math.pi, 0, 0], degrees=False)
        rot_final = rot_aligned * rot_flip
        q_final = rot_final.as_quat()  # Devuelve el nuevo [x, y, z, w]

        # 4. Construir el mensaje TF
        t = TransformStamped()
        # CRÍTICO: Usar tiempo de simulación
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        # CRÍTICO: Engañamos a ArduPilot con el nombre correcto
        t.child_frame_id = 'base_link'

        # La posición XYZ se pasa intacta
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # Inyectar el nuevo cuaternión rotado
        t.transform.rotation.x = q_final[0]
        t.transform.rotation.y = q_final[1]
        t.transform.rotation.z = q_final[2]
        t.transform.rotation.w = q_final[3]

        # Finalmente publicas 't' en tu publisher de /ap/tf
        tf_msg = TFMessage()
        tf_msg.transforms = [t]
        self.publisher.publish(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToArdupilotTF()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
