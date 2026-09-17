#!/usr/bin/env python3
"""
Interactive Teleoperation Node for Boston Dynamics Spot.
Publishes geometry_msgs/Twist to /cmd_vel using keyboard inputs.
"""

import sys
import select
import termios
import tty
import argparse

usage_msg = """
Spot Keyboard Teleoperation
---------------------------
Moving around:
        w
   a    s    d
        x

q/e : Turn counter-clockwise / clockwise
w/x : Forward / Backward (+vx / -vx)
a/d : Strafe Left / Right (+vy / -vy)
s/SPACE : Force Stop

Speed adjustments:
u/j : Increase / decrease max linear speed by 10%
i/k : Increase / decrease max angular speed by 10%

CTRL-C to quit
"""

move_bindings = {
    'w': (1.0, 0.0, 0.0),
    'x': (-1.0, 0.0, 0.0),
    'a': (0.0, 1.0, 0.0),
    'd': (0.0, -1.0, 0.0),
    'q': (0.0, 0.0, 1.0),
    'e': (0.0, 0.0, -1.0),
    's': (0.0, 0.0, 0.0),
    ' ': (0.0, 0.0, 0.0),
}

speed_bindings = {
    'u': (1.1, 1.0),
    'j': (0.9, 1.0),
    'i': (1.0, 1.1),
    'k': (1.0, 0.9),
}

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    parser = argparse.ArgumentParser(description='Spot Teleop Node')
    parser.add_argument('--topic', type=str, default='/cmd_vel', help='Command velocity topic')
    parser.add_argument('--ros', type=int, default=1, choices=[1, 2], help='ROS version (1 or 2)')
    parser.add_argument('--speed', type=float, default=0.6, help='Base linear velocity (m/s)')
    parser.add_argument('--turn', type=float, default=0.8, help='Base angular velocity (rad/s)')
    parser.add_argument('--rate', type=float, default=10.0, help='Publish rate (Hz)')
    args = parser.parse_args()

    settings = termios.tcgetattr(sys.stdin)

    linear_speed = args.speed
    turn_speed = args.turn
    vx, vy, wz = 0.0, 0.0, 0.0

    print(usage_msg)
    print(f'Current Speeds -> Linear: {linear_speed:.2f} m/s | Angular: {turn_speed:.2f} rad/s')

    if args.ros == 1:
        import rospy
        from geometry_msgs.msg import Twist

        rospy.init_node('spot_teleop_keyboard', anonymous=True)
        pub = rospy.Publisher(args.topic, Twist, queue_size=1)
        rate = rospy.Rate(args.rate)

        try:
            while not rospy.is_shutdown():
                key = get_key(settings)
                if key in move_bindings:
                    vx = move_bindings[key][0] * linear_speed
                    vy = move_bindings[key][1] * linear_speed
                    wz = move_bindings[key][2] * turn_speed
                elif key in speed_bindings:
                    linear_speed *= speed_bindings[key][0]
                    turn_speed *= speed_bindings[key][1]
                    print(f'Speed updated -> Linear: {linear_speed:.2f} m/s | Angular: {turn_speed:.2f} rad/s')
                elif key == chr(3):  # Ctrl-C
                    break

                twist = Twist()
                twist.linear.x = vx
                twist.linear.y = vy
                twist.linear.z = 0.0
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = wz
                pub.publish(twist)
                rate.sleep()
        finally:
            twist = Twist()
            pub.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    else:
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import Twist

        rclpy.init()
        node = Node('spot_teleop_keyboard')
        pub = node.create_publisher(Twist, args.topic, 1)

        try:
            while rclpy.ok():
                key = get_key(settings)
                if key in move_bindings:
                    vx = move_bindings[key][0] * linear_speed
                    vy = move_bindings[key][1] * linear_speed
                    wz = move_bindings[key][2] * turn_speed
                elif key in speed_bindings:
                    linear_speed *= speed_bindings[key][0]
                    turn_speed *= speed_bindings[key][1]
                    print(f'Speed updated -> Linear: {linear_speed:.2f} m/s | Angular: {turn_speed:.2f} rad/s')
                elif key == chr(3):  # Ctrl-C
                    break

                twist = Twist()
                twist.linear.x = vx
                twist.linear.y = vy
                twist.linear.z = 0.0
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = wz
                pub.publish(twist)
                rclpy.spin_once(node, timeout_sec=1.0 / args.rate)
        finally:
            twist = Twist()
            pub.publish(twist)
            node.destroy_node()
            rclpy.shutdown()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
