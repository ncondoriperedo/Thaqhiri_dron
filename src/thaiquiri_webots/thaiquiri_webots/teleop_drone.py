import rclpy
import rcl_interfaces.msg
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import threading
# import msvcrt

msg = """
This node takes keypresses from the keyboard and publishes them
as Twist/TwistStamped messages. It works best with a US keyboard layout.
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

t : up (+z)
b : down (-z)
y : yaw left (+z)
n : yaw right (-z)

anything else : stop

CTRL-C to quit
"""


moveBindings = {  # (x, y, z, th)
    'i': (-1.5, 0, 0, 0),  # x == 2
    'o': (-1.5, -1, 0, 0),
    'j': (0, 1, 0, 0),
    'l': (0, -1, 0, 0),
    'u': (-1.5, 1, 0, 0),
    ',': (1.5, 0, 0, 0),
    '.': (1.5, -1, 0, 0),
    'm': (1.5, 1, 0, 0),
    't': (0, 0, 0.05, 0),
    'b': (0, 0, -0.05, 0),
    'y': (0, 0, 0, 1.3),
    'n': (0, 0, 0, -1.3),
}


def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(speed, turn):
    return 'currently:\tspeed %.2f\tturn %.2f ' % (speed, turn)


def main(args=None):
    settings = saveTerminalSettings()
    rclpy.init(args=args)
    node = rclpy.create_node('teleop_drone')

    read_only_descriptor = rcl_interfaces.msg.ParameterDescriptor(
        read_only=True)
    stamped = node.declare_parameter(
        'stamped', False, read_only_descriptor).value
    frame_id = node.declare_parameter(
        'frame_id', '', read_only_descriptor).value

    publisher = node.create_publisher(Twist, 'cmd_pos', 10)

    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    try:
        print(msg)
        while True:
            key = getKey(settings)
            if key in moveBindings.keys():
                twist = Twist()
                twist.linear.x = float(moveBindings[key][0])
                twist.linear.y = float(moveBindings[key][1])
                twist.linear.z = float(moveBindings[key][2])
                twist.angular.z = float(moveBindings[key][3])
                publisher.publish(twist)
            else:
                if key == '\x03':
                    break
                twist = Twist()
                publisher.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.z = 0
        publisher.publish(twist)
        rclpy.shutdown()
        spinner.join()
        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()
