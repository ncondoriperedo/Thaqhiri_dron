"""Independent Webots ground-truth instrument; never feeds the flight estimator."""

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation
from std_msgs.msg import Int32
from webots_ros2_msgs.srv import SpawnNodeFromString


def main(args=None):
    """Spawn an observer, publish true pose/contact counts, and step alongside ROS."""
    rclpy.init(args=args)
    node = Node('sil_observer')
    lights_on = node.declare_parameter('lights_on', True).value
    headlight_on = node.declare_parameter('headlight_on', True).value
    client = node.create_client(SpawnNodeFromString, '/Ros2Supervisor/spawn_node_from_string')
    if not client.wait_for_service(timeout_sec=300):
        raise RuntimeError('Webots supervisor spawn service unavailable')
    request = SpawnNodeFromString.Request()
    request.data = 'DEF SILObserver Robot { name "SILObserver" controller "<extern>" supervisor TRUE }'
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=20)
    if not future.done() or not future.result().success:
        raise RuntimeError('Could not spawn SILObserver')

    # The runner supplies the ROS driver's matching controller library and URL.
    from controller import Supervisor
    robot = Supervisor()
    drone = robot.getFromDef('drone')
    mine = robot.getFromDef('MINE')
    if drone is None or mine is None:
        raise RuntimeError('SIL requires a world with DEF drone and DEF MINE')
    mine.getField('lightsOn').setSFBool(lights_on)
    lamp = drone.getField('headlightOn')
    if lamp is None:
        raise RuntimeError('SIL requires a switchable onboard headlight')
    lamp.setSFBool(headlight_on)
    pub = node.create_publisher(Odometry, '/sil/ground_truth', 10)
    contacts_pub = node.create_publisher(Int32, '/sil/contact_count', 10)
    native_from_body = Rotation.from_euler('xyz', [np.pi / 2, 0, np.pi / 2]).inv()
    timestep = int(robot.getBasicTimeStep())
    counter = 0
    try:
        while rclpy.ok() and robot.step(timestep) != -1:
            rclpy.spin_once(node, timeout_sec=0)
            counter += 1
            if counter % 6:
                continue
            orientation = Rotation.from_matrix(np.array(drone.getOrientation()).reshape(3, 3))
            body = orientation * native_from_body
            pose = Odometry()
            pose.header.stamp = Time(seconds=robot.getTime()).to_msg()
            pose.header.frame_id = 'world'
            pose.child_frame_id = 'base_footprint'
            pose.pose.pose.position.x, pose.pose.pose.position.y, pose.pose.pose.position.z = drone.getPosition()
            q = body.as_quat()
            (pose.pose.pose.orientation.x, pose.pose.pose.orientation.y,
             pose.pose.pose.orientation.z, pose.pose.pose.orientation.w) = q
            velocity = drone.getVelocity()
            linear, angular = body.inv().apply(velocity[:3]), body.inv().apply(velocity[3:])
            (pose.twist.twist.linear.x, pose.twist.twist.linear.y,
             pose.twist.twist.linear.z) = linear
            (pose.twist.twist.angular.x, pose.twist.twist.angular.y,
             pose.twist.twist.angular.z) = angular
            pub.publish(pose)
            contacts_pub.publish(Int32(data=len(drone.getContactPoints(True))))
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
