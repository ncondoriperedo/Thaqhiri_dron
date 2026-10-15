import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import CameraInfo


class FixCameraInfo(Node):
    def __init__(self):
        super().__init__('fix_camera_info')
        # Escuchamos los mensajes defectuosos de Webots
        self.sub_left = self.create_subscription(
            CameraInfo, '/drone/camera_left/camera_info', self.left_cb, 10)
        self.sub_right = self.create_subscription(
            CameraInfo, '/drone/camera_right/camera_info', self.right_cb, 10)

        # Publicamos los mensajes corregidos
        self.pub_left = self.create_publisher(
            CameraInfo, '/drone/left/camera_info', 10)
        self.pub_right = self.create_publisher(
            CameraInfo, '/drone/right/camera_info', 10)

    def left_cb(self, msg):
        # La cámara izquierda es el punto de origen (Tx = 0), solo la re-publicamos
        self.pub_left.publish(msg)

    def right_cb(self, msg):
        # Corregimos el cuarto valor de la matriz P (el índice 3 en el arreglo lineal)
        # Tx = -fx * baseline (0.05m)
        tx_val = -msg.k[0] * 0.05

        # Las tuplas de ROS 2 son inmutables, así que convertimos a lista, cambiamos y devolvemos
        p_list = list(msg.p)
        p_list[3] = tx_val
        msg.p = p_list

        self.pub_right.publish(msg)


def main():
    rclpy.init()
    node = FixCameraInfo()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
