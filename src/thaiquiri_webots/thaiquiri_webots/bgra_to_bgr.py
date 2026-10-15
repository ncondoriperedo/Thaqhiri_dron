import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class BgraToBgrNode(Node):
    def __init__(self):
        super().__init__('bgra_converter')
        self.bridge = CvBridge()
        # Escucha la imagen original de Webots
        self.sub_left = self.create_subscription(
            Image, '/drone/camera_left/image_color', self.left_callback, 10)
        self.sub_right = self.create_subscription(
            Image, '/drone/camera_right/image_color', self.right_callback, 10)
        # Publica la imagen corregida para el nodo estéreo
        self.pub_left = self.create_publisher(
            Image, '/drone/left/image_raw', 10)
        self.pub_right = self.create_publisher(
            Image, '/drone/right/image_raw', 10)

    def left_callback(self, msg):
        # Convierte el mensaje ROS a matriz de OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgra8')
        # Elimina el canal Alpha (Pasa de 4 canales a 3 canales)
        bgr_image = cv2.cvtColor(cv_image, cv2.COLOR_BGRA2BGR)
        # Vuelve a convertir a mensaje ROS y lo publica
        out_msg = self.bridge.cv2_to_imgmsg(bgr_image, encoding='bgr8')
        out_msg.header = msg.header
        self.pub_left.publish(out_msg)

    def right_callback(self, msg):
        # Convierte el mensaje ROS a matriz de OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgra8')
        # Elimina el canal Alpha (Pasa de 4 canales a 3 canales)
        bgr_image = cv2.cvtColor(cv_image, cv2.COLOR_BGRA2BGR)
        # Vuelve a convertir a mensaje ROS y lo publica
        out_msg = self.bridge.cv2_to_imgmsg(bgr_image, encoding='bgr8')
        out_msg.header = msg.header
        self.pub_right.publish(out_msg)


def main():
    rclpy.init()
    node = BgraToBgrNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
